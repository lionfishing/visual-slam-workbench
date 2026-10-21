#include <algorithm>
#include <charconv>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/u_int64.hpp"

namespace
{
struct PlayerParameters
{
    std::string dataset_root;
    std::string association_path;
    bool wait_for_subscribers;
    std::int64_t max_frames;
    std::int64_t publish_period_ms;
};

struct FramePair
{
    std::string rgb_time;
    std::string rgb_relative_path;
    std::string depth_relative_path;
    std_msgs::msg::Header rgb_header;
    std_msgs::msg::Header depth_header;
};

// Parse decimal microseconds as integers; avoid precision loss from a double timestamp.
std_msgs::msg::Header make_header(const std::string &timestamp)
{
    const auto dot = timestamp.find('.');
    const auto digits = [](char value) { return value >= '0' && value <= '9'; };
    if (dot == std::string::npos || dot == 0 || timestamp.size() - dot - 1 != 6 ||
        !std::all_of(timestamp.begin(), timestamp.begin() + dot, digits) ||
        !std::all_of(timestamp.begin() + dot + 1, timestamp.end(), digits)) {
        throw std::invalid_argument("Invalid timestamp '" + timestamp + "': expected seconds.microseconds");
    }
    std::int32_t seconds = 0;
    std::uint32_t microseconds = 0;
    const auto sec_result = std::from_chars(timestamp.data(), timestamp.data() + dot, seconds);
    const auto us_result = std::from_chars(
        timestamp.data() + dot + 1, timestamp.data() + timestamp.size(), microseconds);
    if (sec_result.ec != std::errc{} || us_result.ec != std::errc{}) {
        throw std::invalid_argument("Timestamp outside ROS time range: " + timestamp);
    }
    std_msgs::msg::Header header;
    header.stamp.sec = seconds;
    header.stamp.nanosec = microseconds * 1000;
    header.frame_id = "camera_color_optical_frame";
    return header;
}

std::vector<FramePair> load_associations(const std::string &path)
{
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("Cannot open association file " + path);
    }
    std::vector<FramePair> frames;
    std::string line;
    std::size_t line_number = 0;
    while (std::getline(input, line)) {
        ++line_number;
        const auto first = line.find_first_not_of(" \t\r");
        if (first == std::string::npos || line[first] == '#') {
            continue;
        }
        FramePair frame;
        std::string depth_time;
        std::istringstream fields(line);
        if (!(fields >> frame.rgb_time >> frame.rgb_relative_path >> depth_time >> frame.depth_relative_path)) {
            throw std::runtime_error("Fields read error at association line " + std::to_string(line_number));
        }
        frame.rgb_header = make_header(frame.rgb_time);
        frame.depth_header = make_header(depth_time);
        frames.push_back(std::move(frame));
    }
    if (!input.eof()) {
        throw std::runtime_error("Cannot read association file " + path);
    }
    if (frames.empty()) {
        throw std::runtime_error("Association file is empty");
    }
    return frames;
}

sensor_msgs::msg::Image make_image_message(
    const cv::Mat &image, const std::string &encoding, const std_msgs::msg::Header &header)
{
    const cv::Mat contiguous = image.isContinuous() ? image : image.clone();
    sensor_msgs::msg::Image message;
    message.header = header;
    message.height = contiguous.rows;
    message.width = contiguous.cols;
    message.encoding = encoding;
    const std::uint16_t endian_probe = 1;
    message.is_bigendian = *reinterpret_cast<const std::uint8_t *>(&endian_probe) == 0;
    message.step = static_cast<std::uint32_t>(contiguous.step);
    message.data.assign(contiguous.data, contiguous.data + contiguous.total() * contiguous.elemSize());
    return message;
}

sensor_msgs::msg::CameraInfo make_camera_info(const std_msgs::msg::Header &header)
{
    // This player currently uses the TUM Freiburg 1 calibration, at its original resolution.
    sensor_msgs::msg::CameraInfo message;
    message.header = header;
    message.height = 480;
    message.width = 640;
    message.distortion_model = "plumb_bob";
    message.d = {0.262383, -0.953104, -0.005358, 0.002628, 1.163314};
    message.k = {517.306408, 0.0, 318.643040, 0.0, 516.469215, 255.313989, 0.0, 0.0, 1.0};
    message.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    // P describes the selected virtual rectified camera; filling it does not rectify pixels.
    message.p = {517.306408, 0.0, 318.643040, 0.0,
                 0.0, 516.469215, 255.313989, 0.0,
                 0.0, 0.0, 1.0, 0.0};
    return message;
}

class DatasetPlayerNode : public rclcpp::Node
{
public:
    DatasetPlayerNode() : Node("rgbd_dataset_player")
    {
        parameters_ = {
            declare_parameter<std::string>("dataset_root", ""),
            declare_parameter<std::string>("association_path", ""),
            declare_parameter<bool>("wait_for_subscribers", true),
            declare_parameter<std::int64_t>("max_frames", 3),
            declare_parameter<std::int64_t>("publish_period_ms", 1000)};
        validate_parameters();
        frames_ = load_associations(parameters_.association_path);
        RCLCPP_INFO(get_logger(), "frame.size: %zu, last_rgb_stamp: %s",
                    frames_.size(), frames_.back().rgb_time.c_str());
        rgb_pub_ = create_publisher<sensor_msgs::msg::Image>("/camera/color/image_raw", 10);
        depth_pub_ = create_publisher<sensor_msgs::msg::Image>("/camera/depth/image_raw", 10);
        camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("/camera/color/camera_info", 10);
        finished_pub_ = create_publisher<std_msgs::msg::UInt64>("/dataset/playback_finished", 1);
        timer_ = create_wall_timer(
            std::chrono::milliseconds(parameters_.publish_period_ms), [this]() { tick(); });
    }

    int exit_code() const { return exit_code_; }

private:
    void validate_parameters() const
    {
        if (parameters_.max_frames < 0 || parameters_.publish_period_ms <= 0) {
            throw std::invalid_argument("max_frames must be >= 0; publish_period_ms must be > 0");
        }
        std::error_code error;
        if (!std::filesystem::is_directory(parameters_.dataset_root, error) || error) {
            throw std::invalid_argument("dataset_root must be an existing directory");
        }
    }

    bool subscribers_ready() const
    {
        return rgb_pub_->get_subscription_count() > 0 &&
               depth_pub_->get_subscription_count() > 0 &&
               finished_pub_->get_subscription_count() > 0;
    }

    bool playback_complete() const
    {
        return frame_index_ >= frames_.size() ||
            (parameters_.max_frames > 0 && frame_index_ >= static_cast<std::size_t>(parameters_.max_frames));
    }

    void stop()
    {
        stopped_ = true;
        timer_->cancel();
        rclcpp::shutdown();
    }

    void finish_playback()
    {
        std_msgs::msg::UInt64 message;
        message.data = static_cast<std::uint64_t>(frame_index_);
        finished_pub_->publish(message);
        // Reliable transport acknowledgement does not imply completion of subscriber business logic.
        const bool rgb_acked = rgb_pub_->wait_for_all_acked(std::chrono::seconds(2));
        const bool depth_acked = depth_pub_->wait_for_all_acked(std::chrono::seconds(2));
        const bool finish_acked = finished_pub_->wait_for_all_acked(std::chrono::seconds(2));
        if (!rgb_acked || !depth_acked || !finish_acked) {
            RCLCPP_ERROR(get_logger(), "Timed out waiting for message acknowledgements");
            exit_code_ = 1;
        }
        RCLCPP_INFO(get_logger(), "Playback finished: %zu pairs published", frame_index_);
        stop();
    }

    void publish_frame(const FramePair &frame)
    {
        const auto rgb_path = (std::filesystem::path(parameters_.dataset_root) / frame.rgb_relative_path).string();
        const auto depth_path = (std::filesystem::path(parameters_.dataset_root) / frame.depth_relative_path).string();
        const cv::Mat rgb = cv::imread(rgb_path, cv::IMREAD_COLOR);
        const cv::Mat depth = cv::imread(depth_path, cv::IMREAD_UNCHANGED);
        if (rgb.empty()) {
            throw std::runtime_error("Cannot read RGB image: " + rgb_path);
        }
        if (depth.empty()) {
            throw std::runtime_error("Cannot read DEPTH image: " + depth_path);
        }
        if (depth.type() != CV_16UC1) {
            throw std::runtime_error("Unexpected depth type: " + std::to_string(depth.type()));
        }
        if (rgb.cols != 640 || rgb.rows != 480 || depth.size() != rgb.size()) {
            throw std::runtime_error("Image size must match the 640x480 Freiburg 1 calibration");
        }
        rgb_pub_->publish(make_image_message(rgb, "bgr8", frame.rgb_header));
        depth_pub_->publish(make_image_message(depth, "16UC1", frame.depth_header));
        camera_info_pub_->publish(make_camera_info(frame.rgb_header));
        RCLCPP_INFO(get_logger(), "index=%zu, rgb_time=%s", frame_index_, frame.rgb_time.c_str());
        ++frame_index_;
    }

    void tick()
    {
        if (stopped_) {
            return;
        }
        try {
            if (playback_complete()) {
                finish_playback();
            } else if (parameters_.wait_for_subscribers && frame_index_ == 0 && !subscribers_ready()) {
                RCLCPP_INFO(get_logger(), "Waiting for RGB, depth and finish subscribers...");
            } else {
                publish_frame(frames_[frame_index_]);
            }
        } catch (const std::exception &error) {
            RCLCPP_ERROR(get_logger(), "%s", error.what());
            exit_code_ = 1;
            stop();
        }
    }

    PlayerParameters parameters_{};
    std::vector<FramePair> frames_;
    std::size_t frame_index_ = 0;
    int exit_code_ = 0;
    bool stopped_ = false;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr finished_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    int exit_code = 1;
    try {
        const auto node = std::make_shared<DatasetPlayerNode>();
        rclcpp::spin(node);
        exit_code = node->exit_code();
    } catch (const std::exception &error) {
        RCLCPP_ERROR(rclcpp::get_logger("rgbd_dataset_player"), "%s", error.what());
    }
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    return exit_code;
}
