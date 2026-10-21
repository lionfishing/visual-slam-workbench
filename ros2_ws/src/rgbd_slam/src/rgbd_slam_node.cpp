#include <chrono>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/u_int64.hpp"

#include "System.h"

namespace
{
using Image = sensor_msgs::msg::Image;
using SyncPolicy = message_filters::sync_policies::ApproximateTime<Image, Image>;
using Synchronizer = message_filters::Synchronizer<SyncPolicy>;
constexpr double kMaxPairIntervalSec = 0.02;

struct SlamParameters
{
    double finish_timeout_sec;
    std::string vocabulary_path;
    std::string settings_path;
};

struct ProcessingStats
{
    std::size_t rgb_received = 0;
    std::size_t depth_received = 0;
    // Completed TrackRGBD calls include frames whose tracking state is not OK.
    std::size_t paired_count = 0;
    std::size_t tail_pairs = 0;
};

struct FinishState
{
    std::size_t expected_pairs = 0;
    bool playback_finished = false;
    bool finalization_started = false;
    std::chrono::steady_clock::time_point finish_received_at{};
};

struct FrameCache
{
    Image::ConstSharedPtr last_rgb;
    Image::ConstSharedPtr last_depth;
    Image::ConstSharedPtr last_paired_rgb;
    Image::ConstSharedPtr last_paired_depth;
};

bool is_regular_file(const std::string &path)
{
    std::error_code error;
    return std::filesystem::is_regular_file(path, error) && !error;
}

bool validate_slam_parameters(const SlamParameters &parameters, const rclcpp::Logger &logger)
{
    if (!std::isfinite(parameters.finish_timeout_sec) || parameters.finish_timeout_sec <= 0.0) {
        RCLCPP_ERROR(logger, "finish_timeout_sec must be positive and finite");
        return false;
    }
    if (!is_regular_file(parameters.vocabulary_path) || !is_regular_file(parameters.settings_path)) {
        RCLCPP_ERROR(logger, "Vocabulary or settings file does not exist: '%s', '%s'",
                     parameters.vocabulary_path.c_str(), parameters.settings_path.c_str());
        return false;
    }
    return true;
}

bool is_valid_rgbd_pair(const cv::Mat &rgb, const cv::Mat &depth)
{
    return !rgb.empty() && !depth.empty() && rgb.size() == depth.size() &&
           rgb.type() == CV_8UC3 && depth.type() == CV_16UC1;
}

// T_wc maps camera optical coordinates into the current SLAM map coordinates.
geometry_msgs::msg::PoseStamped make_pose_message(
    const Sophus::SE3f &T_wc, const std_msgs::msg::Header &header)
{
    const auto position = T_wc.translation();
    const auto orientation = T_wc.unit_quaternion();
    geometry_msgs::msg::PoseStamped message;
    message.header = header;
    message.pose.position.x = position.x();
    message.pose.position.y = position.y();
    message.pose.position.z = position.z();
    message.pose.orientation.x = orientation.x();
    message.pose.orientation.y = orientation.y();
    message.pose.orientation.z = orientation.z();
    message.pose.orientation.w = orientation.w();
    return message;
}

class RgbdSlamNode : public rclcpp::Node
{
public:
    RgbdSlamNode() : Node("rgbd_slam")
    {
        parameters_ = {
            declare_parameter<double>("finish_timeout_sec", 5.0),
            declare_parameter<std::string>("vocabulary_path", ""),
            declare_parameter<std::string>("settings_path", "")};
        if (!validate_slam_parameters(parameters_, get_logger())) {
            throw std::invalid_argument("Invalid SLAM parameters");
        }
        slam_ = std::make_unique<ORB_SLAM3::System>(
            parameters_.vocabulary_path, parameters_.settings_path, ORB_SLAM3::System::RGBD, false);
        pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/slam/pose", 10);
        configure_subscriptions();
        finish_timer_ = create_wall_timer(
            std::chrono::milliseconds(100), [this]() { check_finish(); });
    }

    int exit_code() const { return exit_code_; }

    // Called after spin returns, for normal completion, failure and Ctrl+C.
    void shutdown_slam()
    {
        if (slam_ && !slam_shutdown_) {
            slam_->Shutdown();
            slam_shutdown_ = true;
        }
        // The pinned upstream Shutdown requests finish but does not join all workers.
        // Do not export or claim a final optimized trajectory at this point.
    }

    void print_summary() const
    {
        RCLCPP_INFO(get_logger(), "Summary: RGB=%zu, depth=%zu, pairs=%zu, tail_pairs=%zu",
                    stats_.rgb_received, stats_.depth_received, stats_.paired_count, stats_.tail_pairs);
    }

private:
    void configure_subscriptions()
    {
        rgb_sub_.subscribe(this, "/camera/color/image_raw");
        depth_sub_.subscribe(this, "/camera/depth/image_raw");
        // Register counters before the synchronizer, so a paired callback sees current input counts.
        rgb_sub_.registerCallback([this](const Image::ConstSharedPtr &message) {
            ++stats_.rgb_received;
            frames_.last_rgb = message;
        });
        depth_sub_.registerCallback([this](const Image::ConstSharedPtr &message) {
            ++stats_.depth_received;
            frames_.last_depth = message;
        });
        sync_ = std::make_unique<Synchronizer>(SyncPolicy(10), rgb_sub_, depth_sub_);
        sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(kMaxPairIntervalSec));
        sync_->registerCallback(std::bind(
            &RgbdSlamNode::process_pair, this, std::placeholders::_1, std::placeholders::_2));
        finished_sub_ = create_subscription<std_msgs::msg::UInt64>(
            "/dataset/playback_finished", 1,
            [this](std_msgs::msg::UInt64::ConstSharedPtr message) {
                handle_playback_finished(*message);
            });
    }

    void fail(const std::string &reason)
    {
        RCLCPP_ERROR(get_logger(), "%s", reason.c_str());
        exit_code_ = 1;
        stop();
    }

    void stop()
    {
        stopping_ = true;
        rclcpp::shutdown();
    }

    void publish_pose(const Sophus::SE3f &T_cw, const std_msgs::msg::Header &rgb_header)
    {
        auto header = rgb_header;
        header.frame_id = "map";  // A frame name does not apply an optical-to-robot axis transform.
        const auto T_wc = T_cw.inverse();
        pose_pub_->publish(make_pose_message(T_wc, header));
        const auto position = T_wc.translation();
        RCLCPP_INFO(get_logger(), "Camera position: x=%.3f, y=%.3f, z=%.3f m",
                    position.x(), position.y(), position.z());
    }

    void log_image_layout(const cv::Mat &rgb, const cv::Mat &depth) const
    {
        RCLCPP_INFO(get_logger(),
                    "RGB Mat: %dx%d, channels=%d, bytes/channel=%zu; "
                    "Depth Mat: %dx%d, channels=%d, bytes/channel=%zu",
                    rgb.cols, rgb.rows, rgb.channels(), rgb.elemSize1(),
                    depth.cols, depth.rows, depth.channels(), depth.elemSize1());
    }

    bool process_pair(const Image::ConstSharedPtr &rgb, const Image::ConstSharedPtr &depth)
    {
        if (stopping_) {
            return false;
        }
        try {
            // Keep these owners alive while cv::Mat refers to shared ROS message storage.
            const auto rgb_cv = cv_bridge::toCvShare(rgb, "bgr8");
            const auto depth_cv = cv_bridge::toCvShare(depth, "16UC1");
            const auto &rgb_image = rgb_cv->image;
            const auto &depth_image = depth_cv->image;
            if (!is_valid_rgbd_pair(rgb_image, depth_image)) {
                fail("Invalid RGB-D image shape or type");
                return false;
            }
            if (stats_.paired_count == 0) {
                log_image_layout(rgb_image, depth_image);
            }
            // The pose uses the RGB acquisition time; depth remains in dataset units.
            const auto T_cw = slam_->TrackRGBD(
                rgb_image, depth_image, rclcpp::Time(rgb->header.stamp).seconds());
            const int tracking_state = slam_->GetTrackingState();
            RCLCPP_INFO(get_logger(), "Tracking state: %d", tracking_state);
            if (tracking_state == ORB_SLAM3::Tracking::OK) {
                if (!T_cw.matrix().allFinite()) {
                    fail("SLAM returned a non-finite pose");
                    return false;
                }
                publish_pose(T_cw, rgb->header);
            }
            ++stats_.paired_count;
            frames_.last_paired_rgb = rgb;
            frames_.last_paired_depth = depth;
            const double dt_ms = (rclcpp::Time(rgb->header.stamp) -
                                  rclcpp::Time(depth->header.stamp)).seconds() * 1000.0;
            RCLCPP_INFO(get_logger(),
                        "Pair: rgb=%d.%09u, depth=%d.%09u, dt(rgb-depth)=%.3f ms",
                        rgb->header.stamp.sec, rgb->header.stamp.nanosec,
                        depth->header.stamp.sec, depth->header.stamp.nanosec, dt_ms);
            return true;
        } catch (const cv_bridge::Exception &error) {
            fail(std::string("Image conversion failed: ") + error.what());
        } catch (const std::exception &error) {
            fail(std::string("RGB-D processing failed: ") + error.what());
        }
        return false;
    }

    void handle_playback_finished(const std_msgs::msg::UInt64 &message)
    {
        if (finish_.playback_finished) {
            RCLCPP_WARN(get_logger(), "Ignoring duplicate playback-finished notification");
            return;
        }
        // Freeze both the count and the deadline on the first notification.
        finish_.expected_pairs = static_cast<std::size_t>(message.data);
        finish_.finish_received_at = std::chrono::steady_clock::now();
        finish_.playback_finished = true;
        RCLCPP_INFO(get_logger(), "Playback finished notification: expected_pairs=%zu",
                    finish_.expected_pairs);
    }

    bool inputs_complete()
    {
        if (stats_.rgb_received > finish_.expected_pairs ||
            stats_.depth_received > finish_.expected_pairs) {
            RCLCPP_ERROR(get_logger(), "Received too many images: expected=%zu, RGB=%zu, depth=%zu",
                        finish_.expected_pairs, stats_.rgb_received, stats_.depth_received);
            exit_code_ = 1;
            stop();
            return false;
        }
        if (stats_.rgb_received == finish_.expected_pairs &&
            stats_.depth_received == finish_.expected_pairs) {
            return true;
        }
        const double waited = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - finish_.finish_received_at).count();
        if (waited >= parameters_.finish_timeout_sec) {
            RCLCPP_ERROR(get_logger(), "Timed out after %.3f s: expected=%zu, RGB=%zu, depth=%zu",
                        waited, finish_.expected_pairs, stats_.rgb_received, stats_.depth_received);
            exit_code_ = 1;
            stop();
        }
        return false;
    }

    bool tail_is_valid() const
    {
        if (stats_.paired_count > finish_.expected_pairs ||
            finish_.expected_pairs - stats_.paired_count != 1 ||
            !frames_.last_rgb || !frames_.last_depth) {
            return false;
        }
        const auto rgb_time = rclcpp::Time(frames_.last_rgb->header.stamp);
        const auto depth_time = rclcpp::Time(frames_.last_depth->header.stamp);
        const bool rgb_unused = !frames_.last_paired_rgb ||
            rgb_time > rclcpp::Time(frames_.last_paired_rgb->header.stamp);
        const bool depth_unused = !frames_.last_paired_depth ||
            depth_time > rclcpp::Time(frames_.last_paired_depth->header.stamp);
        return rgb_unused && depth_unused &&
            std::abs((rgb_time - depth_time).seconds()) <= kMaxPairIntervalSec;
    }

    void check_finish()
    {
        if (stopping_ || !finish_.playback_finished || finish_.finalization_started ||
            !inputs_complete()) {
            return;
        }
        finish_.finalization_started = true;
        if (stats_.paired_count != finish_.expected_pairs) {
            // ApproximateTime can retain one final candidate without a following message.
            // Only a fresh, unique tail inside the same 20 ms gate may bypass the synchronizer.
            if (!tail_is_valid()) {
                fail("Unexpected pending-frame state or invalid tail pair");
                return;
            }
            RCLCPP_INFO(get_logger(), "Processing one pending tail pair");
            if (!process_pair(frames_.last_rgb, frames_.last_depth)) {
                return;
            }
            ++stats_.tail_pairs;
        }
        RCLCPP_INFO(get_logger(), "End check: expected=%zu, RGB=%zu, depth=%zu, pairs=%zu",
                    finish_.expected_pairs, stats_.rgb_received, stats_.depth_received, stats_.paired_count);
        stop();
    }

    SlamParameters parameters_{};
    ProcessingStats stats_;
    FinishState finish_;
    FrameCache frames_;
    int exit_code_ = 0;
    bool stopping_ = false;
    bool slam_shutdown_ = false;
    std::unique_ptr<ORB_SLAM3::System> slam_;
    message_filters::Subscriber<Image> rgb_sub_;
    message_filters::Subscriber<Image> depth_sub_;
    std::unique_ptr<Synchronizer> sync_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr finished_sub_;
    rclcpp::TimerBase::SharedPtr finish_timer_;
};
}  // namespace

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<RgbdSlamNode> node;
    int exit_code = 1;
    try {
        node = std::make_shared<RgbdSlamNode>();
        rclcpp::spin(node);
        exit_code = node->exit_code();
    } catch (const std::exception &error) {
        RCLCPP_ERROR(rclcpp::get_logger("rgbd_slam"), "%s", error.what());
    }
    if (node) {
        try {
            node->shutdown_slam();
        } catch (const std::exception &error) {
            RCLCPP_ERROR(node->get_logger(), "SLAM shutdown failed: %s", error.what());
            exit_code = 1;
        }
        node->print_summary();
    }
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    return exit_code;
}
