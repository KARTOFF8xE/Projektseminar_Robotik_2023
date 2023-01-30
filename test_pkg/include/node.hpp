#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "opencv2/opencv.hpp"

#include <optional>

//when callbacks have more then one argument this cast will hide that fact from ros
#define MAKE_SINGLE_ARGUMENT(msg_type, function_bind) static_cast<std::function<void(const msg_type::SharedPtr)>>(function_bind)

class Node: public rclcpp::Node {
    public:
        Node();
    private:
        void callback_camera_info(const sensor_msgs::msg::CameraInfo::SharedPtr msg, rclcpp::Logger& logger);
        void callback_imu(const sensor_msgs::msg::Imu::SharedPtr msg, rclcpp::Logger& logger);
        void callback_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg, rclcpp::Logger& logger);
        void callback_rgb_image(const sensor_msgs::msg::Image::SharedPtr msg, rclcpp::Logger& logger);

        struct custom_parameters_t {
            std::string camera_info_topic;
            std::string imu_topic;
            std::string pointcloud_topic;
            std::string rgb_image_topic;
        } custom_parameters;

        //rclcpp
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr   rgb_camera_sub;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr          imu_sub;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr  pointcloud2_sub;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr        rgb_image_sub;

        //misc
        cv::Mat point_cloud,
                K;
        std::optional<cv::Vec3d> euler_angles = std::nullopt;
};