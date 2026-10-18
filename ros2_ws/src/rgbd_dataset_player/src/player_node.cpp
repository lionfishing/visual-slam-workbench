#include <string>
#include <fstream>
#include <sstream>
#include <filesystem>
#include <vector>
#include <cstdint>

#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

struct FramePair {
    std::string rgb_time;
    std::string rgb_relative_path;
    std::string depth_time;
    std::string depth_relative_path;
};

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("rgbd_dataset_player");

    const std::string dataset_root =
        node->declare_parameter<std::string>("dataset_root","");
    const std::string association_path =
        node->declare_parameter<std::string>("association_path", "");
    const bool wait_for_subscribers =
        node->declare_parameter<bool>("wait_for_subscribers", true);
    const auto max_frames =
        node->declare_parameter<std::int64_t>("max_frames", 3);
    const auto publish_period_ms =
        node->declare_parameter<std::int64_t>("publish_period_ms", 1000);

    //! bianjiejiaoyan
    if(max_frames < 0 || publish_period_ms <= 0){
        RCLCPP_ERROR(node->get_logger(),
                    "max_frames must be >= 0; publish_period_ms must be > 0");
        rclcpp::shutdown();
        return 1;
    }
    std::ifstream association_file(association_path);
    if(!association_file.is_open()){
        RCLCPP_ERROR(node->get_logger(),
                    "Cannot open association file %s",
                    association_path.c_str());
        rclcpp::shutdown();
        return 1;
    }
    std::string first_line;
    if (!std::getline(association_file, first_line)) {
        RCLCPP_ERROR(node->get_logger(), "Association file is empty");
        rclcpp::shutdown();
        return 1;
    }
    std::istringstream fields(first_line);
    std::string rgb_time, rgb_relative_path, depth_time, depth_relative_path;
    if (!(fields >> rgb_time >> rgb_relative_path
            >> depth_time >> depth_relative_path)) {
        RCLCPP_ERROR(node->get_logger(), "Fields read error");
        rclcpp::shutdown();
        return 1;
    }
    //!在首行解析成功后push
    std::vector<FramePair> frames;
    FramePair first_frame = {rgb_time, rgb_relative_path, depth_time, depth_relative_path};
    frames.push_back(first_frame);
    //!循环读入后续frame
    std::string line;
    while(std::getline(association_file, line)){

        FramePair frame;
        if (line.empty() || line[0] == '#') {
            continue;
        }
        std::istringstream fields(line);
        if (!(fields >> frame.rgb_time >> frame.rgb_relative_path
                >> frame.depth_time >> frame.depth_relative_path)) {
            RCLCPP_ERROR(node->get_logger(), "Fields read error");
            rclcpp::shutdown();
            return 1;
        }
        frames.push_back(frame);
    }
    RCLCPP_INFO(node->get_logger(),
        "frame.size: %zu, last_rgb_stamp: %s",
        frames.size(), frames.back().rgb_time.c_str());


    auto image_pub =
    node->create_publisher<sensor_msgs::msg::Image>(
        "/camera/color/image_raw", 10);
    auto depth_pub =
    node->create_publisher<sensor_msgs::msg::Image>(
        "/camera/depth/image_raw", 10);

    std::size_t frame_index = 0;
    const auto logger = node->get_logger();
    //!
    int exit_code = 0;

    auto timer = node->create_wall_timer(
        std::chrono::milliseconds(publish_period_ms),
        [&frames, &frame_index, logger, dataset_root, image_pub, depth_pub, wait_for_subscribers, max_frames, &exit_code]() {
            if (frame_index >= frames.size() || (max_frames > 0 &&
                                            frame_index >= static_cast<std::size_t>(max_frames))) {
                RCLCPP_INFO(logger, "Playback finished: %zu pairs published",
                frame_index);
                rclcpp::shutdown();
                return;
            }

            if (wait_for_subscribers && frame_index == 0 &&
            (image_pub->get_subscription_count() == 0 ||
                depth_pub->get_subscription_count() == 0)) {
                RCLCPP_INFO(logger, "Waiting for RGB and depth subscribers...");
            return;
            }
            const FramePair &pair = frames[frame_index];
            //! 路径拼接
            const std::string rgb_path =
                (std::filesystem::path(dataset_root) / pair.rgb_relative_path).string();
            const std::string depth_path =
                (std::filesystem::path(dataset_root) / pair.depth_relative_path).string();
            //! 读图检查
            const cv::Mat image = cv::imread(rgb_path, cv::IMREAD_COLOR);
            if (image.empty()) {
                RCLCPP_ERROR(logger, "Cannot read RGB image: %s",rgb_path.c_str());
                exit_code = 1;
                rclcpp::shutdown();
                return;
            }
            const cv::Mat depth_image = cv::imread(depth_path, cv::IMREAD_UNCHANGED);
            if(depth_image.empty()){
                RCLCPP_ERROR(logger, "Cannot read DEPTH image: %s", depth_path.c_str());
                exit_code = 1;
                rclcpp::shutdown();
                return;
            }
            if (depth_image.type() != CV_16UC1) {
                RCLCPP_ERROR(logger,
                "Unexpected depth type: %d", depth_image.type());
                exit_code = 1;
                rclcpp::shutdown();
                return;
            }
            //! 构造消息
            sensor_msgs::msg::Image msg;
            sensor_msgs::msg::Image depth_msg;
            //rgb
            msg.height = image.rows;
            msg.width = image.cols;
            msg.encoding = "bgr8";
            msg.is_bigendian = false;
            msg.step = static_cast<uint32_t>(image.step);
            //! 起点 终点
            msg.data.assign(
                image.data,
                image.data + image.total() * image.elemSize()
            );
            const auto dot = pair.rgb_time.find('.');
            if (dot == std::string::npos || pair.rgb_time.size() - dot - 1 != 6) {
                RCLCPP_ERROR(logger,
                    "Error rgb stamp type");
                    exit_code = 1;
                    rclcpp::shutdown();
                    return;
            }
            msg.header.stamp.sec = std::stoi(pair.rgb_time.substr(0, dot));
            msg.header.stamp.nanosec =
            std::stoul(pair.rgb_time.substr(dot + 1)) * 1000;
            msg.header.frame_id = "camera_color_optical_frame";
            //depth
            depth_msg.height = depth_image.rows;
            depth_msg.width = depth_image.cols;
            depth_msg.encoding = "16UC1";
            depth_msg.is_bigendian = false;
            depth_msg.step = static_cast<uint32_t>(depth_image.step);
            //! 起点 终点
            depth_msg.data.assign(
                depth_image.data,
                depth_image.data + depth_image.total() * depth_image.elemSize()
            );
            const auto depth_dot = pair.depth_time.find('.');
            if (depth_dot == std::string::npos || pair.depth_time.size() - depth_dot - 1 != 6) {
                RCLCPP_ERROR(logger,
                    "Error deptt stamp type");
                    exit_code = 1;
                    rclcpp::shutdown();
                    return;
            }
            depth_msg.header.stamp.sec = std::stoi(pair.depth_time.substr(0, depth_dot));
            depth_msg.header.stamp.nanosec =
                std::stoul(pair.depth_time.substr(depth_dot + 1)) * 1000;
            depth_msg.header.frame_id = "camera_color_optical_frame";

            RCLCPP_INFO(logger, "index=%zu, rgb_time=%s",
                        frame_index, pair.rgb_time.c_str());

            image_pub->publish(msg);
            depth_pub->publish(depth_msg);

            ++frame_index;
        }
    );

    rclcpp::spin(node);
    rclcpp::shutdown();
    return exit_code;
}