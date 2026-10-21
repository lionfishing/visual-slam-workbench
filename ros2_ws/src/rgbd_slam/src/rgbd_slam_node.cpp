#include <memory>
#include <functional>
#include <cstddef>
#include <chrono>
#include <cmath>
#include <string>
#include <filesystem>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "std_msgs/msg/u_int64.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/header.hpp"

#include "cv_bridge/cv_bridge.h"

#include "System.h"

struct ProcessingStats
{
    std::size_t rgb_received = 0;
    std::size_t depth_received = 0;
    std::size_t paired_count = 0;
    std::size_t tail_pairs = 0;
};

// 将相机到地图的变换 T_wc 转换为位姿消息，保留调用者提供的消息头。
geometry_msgs::msg::PoseStamped make_pose_message(
    const Sophus::SE3f &T_wc,
    const std_msgs::msg::Header &header)
{
    const auto position = T_wc.translation();
    const auto orientation = T_wc.unit_quaternion();

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header = header;

    pose_msg.pose.position.x = position.x();
    pose_msg.pose.position.y = position.y();
    pose_msg.pose.position.z = position.z();
    pose_msg.pose.orientation.x = orientation.x();
    pose_msg.pose.orientation.y = orientation.y();
    pose_msg.pose.orientation.z = orientation.z();
    pose_msg.pose.orientation.w = orientation.w();

    return pose_msg;
}

bool is_valid_rgbd_pair(
    const cv::Mat &rgb_image,
    const cv::Mat &depth_image)
{
    if (rgb_image.empty() || depth_image.empty() ||
    rgb_image.size() != depth_image.size() ||
    rgb_image.type() != CV_8UC3 ||
    depth_image.type() != CV_16UC1){
        return false;
    }
    return true;
}

bool validate_slam_parameters(
    double finish_timeout_sec,
    const std::string &vocabulary_path,
    const std::string &settings_path,
    const rclcpp::Logger &logger)
{
    if(!std::isfinite(finish_timeout_sec) ||
    finish_timeout_sec <= 0){
        RCLCPP_ERROR(
        logger,
        "finish_timeout_sec must be positive and finite"
        );
        return false;
    }
    if (!std::filesystem::is_regular_file(vocabulary_path) ||
        !std::filesystem::is_regular_file(settings_path)) {
        RCLCPP_ERROR(
            logger,
            "Vocabulary or settings file does not exist: '%s', '%s'",
            vocabulary_path.c_str(),
            settings_path.c_str()
        );
        return false;
    }
    return true;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("rgbd_slam");

    const auto logger = node->get_logger();
    //! 用于超时校验
    const double finish_timeout_sec =
        node->declare_parameter<double>("finish_timeout_sec", 5.0);
    const auto vocabulary_path =
        node->declare_parameter<std::string>("vocabulary_path", "");
    const auto settings_path =
        node->declare_parameter<std::string>("settings_path", "");

    if(!validate_slam_parameters(
        finish_timeout_sec,
        vocabulary_path,
        settings_path,
        logger))
    {
        rclcpp::shutdown();
        return 1;
    }

    ORB_SLAM3::System slam(
        vocabulary_path,
        settings_path,
        ORB_SLAM3::System::RGBD,
        false
    );

    ProcessingStats stats;
    std::size_t expected_pairs = 0;
    int exit_code = 0;
    // 是否收到播放完毕信号
    bool playback_finished = false;
    // 是否已进行首尾检查
    bool finalization_started = false;

    std::chrono::steady_clock::time_point finish_received_at{};

    using Image = sensor_msgs::msg::Image;
    //! 用于尾帧校验和处理
    Image::ConstSharedPtr last_rgb;
    Image::ConstSharedPtr last_depth;
    Image::ConstSharedPtr last_paired_rgb;
    Image::ConstSharedPtr last_paired_depth;

    message_filters::Subscriber<Image> rgb_sub(
        node, "/camera/color/image_raw"
    );
    message_filters::Subscriber<Image> depth_sub(
        node, "/camera/depth/image_raw"
    );
    rgb_sub.registerCallback(
        [&stats, &last_rgb](const Image::ConstSharedPtr &msg){
            ++stats.rgb_received;
            last_rgb = msg;
        }
    );
    depth_sub.registerCallback(
        [&stats, &last_depth](const Image::ConstSharedPtr &msg){
            ++stats.depth_received;
            last_depth = msg;
        }
    );
    //! 配对规则 根据采集时间进行近似配对的策略
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<Image, Image>;
    //! 同步器缓存队列大小
    message_filters::Synchronizer<SyncPolicy> sync(
        SyncPolicy(10), rgb_sub, depth_sub
    );
    sync.setMaxIntervalDuration(
        rclcpp::Duration::from_seconds(0.02)
    );

    auto pose_pub =
        node->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/slam/pose",
            10
        );

    auto on_rgbd = [logger, &stats, &last_paired_rgb, &last_paired_depth, &exit_code, &slam, pose_pub](
    const Image::ConstSharedPtr &rgb,
    const Image::ConstSharedPtr &depth) -> bool
    {
        try{
            //! toCvShare返回只读对象，编码等条件允许时可共享原信息内存
            const auto rgb_cv = cv_bridge::toCvShare(rgb, "bgr8");
            const auto depth_cv = cv_bridge::toCvShare(depth, "16UC1");

            const cv::Mat &rgb_image = rgb_cv->image;
            const cv::Mat &depth_image = depth_cv->image;

            if (!is_valid_rgbd_pair(rgb_image, depth_image)) {
                RCLCPP_ERROR(logger, "Invalid RGB-D image shape or type");
                exit_code = 1;
                rclcpp::shutdown();
                return false;
            }

            if (stats.paired_count == 0) {
                RCLCPP_INFO(
                    logger,
                    "RGB Mat: %dx%d, channels=%d, bytes/channel=%zu; "
                    "Depth Mat: %dx%d, channels=%d, bytes/channel=%zu",
                    rgb_image.cols, rgb_image.rows,
                    rgb_image.channels(), rgb_image.elemSize1(),
                    depth_image.cols, depth_image.rows,
                    depth_image.channels(), depth_image.elemSize1()
                );
            }
            const double timestamp =
                rclcpp::Time(rgb->header.stamp).seconds();
            const auto T_cw = slam.TrackRGBD(rgb_image, depth_image, timestamp);
            const int tracking_state = slam.GetTrackingState();
            RCLCPP_INFO(logger, "Tracking state: %d", tracking_state);
            if(tracking_state == ORB_SLAM3::Tracking::OK){
                const auto T_wc = T_cw.inverse();
                const auto position = T_wc.translation();

                auto pose_header = rgb->header;
                pose_header.frame_id = "map";

                pose_pub->publish(make_pose_message(T_wc, pose_header));

                RCLCPP_INFO(
                    logger,
                    "Camera position: x=%.3f, y=%.3f, z=%.3f m",
                    position.x(), position.y(), position.z()
                );
            }

            ++stats.paired_count;
            last_paired_rgb = rgb;
            last_paired_depth = depth;
            // 处理这一对图像
            const double dt_ms =
            (rclcpp::Time(rgb->header.stamp) -
            rclcpp::Time(depth->header.stamp)).seconds() * 1000.0;

            RCLCPP_INFO(
                logger,
                "Pair: rgb=%d.%09u, depth=%d.%09u, dt(rgb-depth)=%.3f ms",
                rgb->header.stamp.sec,
                rgb->header.stamp.nanosec,
                depth->header.stamp.sec,
                depth->header.stamp.nanosec,
                dt_ms
            );
            return true;
        } catch(const cv_bridge::Exception &e){
            RCLCPP_ERROR(logger, "Image conversion failed: %s", e.what());
            exit_code = 1;
            rclcpp::shutdown();
            return false;
        }

    };
    //! 将同步器输出的第一条，第二条信息传给on_rgbd
    sync.registerCallback(
    std::bind(on_rgbd, std::placeholders::_1, std::placeholders::_2));

    auto finished_sub =
        node->create_subscription<std_msgs::msg::UInt64>(
            "/dataset/playback_finished",
            1,
            [logger, &expected_pairs, &playback_finished, &finish_received_at](
                std_msgs::msg::UInt64::ConstSharedPtr msg){
                    if (playback_finished) {
                        RCLCPP_WARN(
                            logger,
                            "Ignoring duplicate playback-finished notification"
                        );
                        return;
                    }
                    expected_pairs = static_cast<std::size_t>(msg->data);
                    finish_received_at = std::chrono::steady_clock::now();
                    playback_finished = true;

                    RCLCPP_INFO(
                        logger,
                        "Playback finished notification: expected_pairs=%zu",
                        expected_pairs
                    );
                }
            );

    auto finish_timer =
        node->create_wall_timer(
            std::chrono::milliseconds(100),
            [&](){
                // ! 未受到发送完毕信号或已完成尾帧校验
                if(!playback_finished || finalization_started){
                    return;
                }
                //! 已收到发送完毕信号但未完成接收
                // 数量超额：无需等待超时
                if (stats.rgb_received > expected_pairs ||
                    stats.depth_received > expected_pairs) {
                    RCLCPP_ERROR(
                        logger,
                        "Received too many images: expected=%zu, RGB=%zu, depth=%zu",
                        expected_pairs,
                        stats.rgb_received,
                        stats.depth_received
                    );
                    exit_code = 1;
                    rclcpp::shutdown();
                    return;
                }

                // 数量不足：限时等待
                if (stats.rgb_received < expected_pairs ||
                    stats.depth_received < expected_pairs) {
                    const double waited_sec =
                        std::chrono::duration<double>(
                            std::chrono::steady_clock::now() - finish_received_at
                        ).count();

                    if (waited_sec >= finish_timeout_sec) {
                        RCLCPP_ERROR(
                            logger,
                            "Timed out after %.3f s: expected=%zu, RGB=%zu, depth=%zu",
                            waited_sec,
                            expected_pairs,
                            stats.rgb_received,
                            stats.depth_received
                        );
                        exit_code = 1;
                        rclcpp::shutdown();
                    }

                    return;
                }
                //! 已完成接收但未完成尾帧校验
                finalization_started = true;
                //! 已经全部处理，无需补尾帧
                if(stats.paired_count == expected_pairs){
                    rclcpp::shutdown();
                    return;
                }
                //! 在这里手动处理最后一帧（目前仅支持一帧）
                //! 确保只缺一对，缓存中的对象存在，采集时间符合20ms限制
                if (stats.paired_count > expected_pairs ||
                    expected_pairs - stats.paired_count != 1 ||
                    !last_rgb || !last_depth) {
                    RCLCPP_ERROR(logger, "Unexpected pending-frame state");
                    exit_code = 1;
                    rclcpp::shutdown();
                    return;
                }

                const auto rgb_time = rclcpp::Time(last_rgb->header.stamp);
                const auto depth_time = rclcpp::Time(last_depth->header.stamp);
                //! 两张图像都必须晚于各自最后处理的图像 这里确保两张图像都是最新的
                const bool rgb_unused =
                    !last_paired_rgb ||
                    rgb_time > rclcpp::Time(last_paired_rgb->header.stamp);
                const bool depth_unused =
                    !last_paired_depth ||
                    depth_time > rclcpp::Time(last_paired_depth->header.stamp);
                const double tail_dt_ms =
                    (rgb_time - depth_time).seconds() * 1000.0;

                if (!rgb_unused || !depth_unused ||
                    std::abs(tail_dt_ms) > 20.0) {
                    RCLCPP_ERROR(
                        logger,
                        "Tail pair is reused or outside time limit"
                    );
                    exit_code = 1;
                    rclcpp::shutdown();
                    return;
                }
                RCLCPP_INFO(logger, "Processing one pending tail pair");

                if(!on_rgbd(last_rgb, last_depth)){
                    return;
                }
                ++stats.tail_pairs;

                RCLCPP_INFO(
                    logger,
                    "End check: expected=%zu, RGB=%zu, depth=%zu, pairs=%zu",
                    expected_pairs,
                    stats.rgb_received,
                    stats.depth_received,
                    stats.paired_count
                );
                rclcpp::shutdown();
            }
        );

    rclcpp::spin(node);
    slam.Shutdown();

    RCLCPP_INFO(
        logger,
        "Summary: RGB=%zu, depth=%zu, pairs=%zu, tail_pairs=%zu",
        stats.rgb_received,
        stats.depth_received,
        stats.paired_count,
        stats.tail_pairs
    );

    rclcpp::shutdown();
    return exit_code;
}
