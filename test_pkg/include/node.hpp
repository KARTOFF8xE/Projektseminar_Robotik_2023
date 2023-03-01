#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
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

        static void callback_mouse_image(int event, int x, int y, int flags, void *userdata);
        static void callback_mouse_warped_image(int event, int x, int y, int flags, void *userdata);

        struct custom_parameters_t {
            std::string camera_info_topic;
            std::string imu_topic;
            std::string pointcloud_topic;
            std::string odometry_topic;
            std::string rgb_image_topic;
        } custom_parameters;

        //rclcpp
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr   rgb_camera_sub;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr          imu_sub;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr  pointcloud2_sub;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr        rgb_image_sub;

        //misc
        cv::Mat point_cloud,
                warped,
                image;
        std::array<double, 9ul> K;
        std::optional<cv::Vec3d> euler_angles = std::nullopt;

        //debug
        bool continue_updating = true;
        bool A_is_set = true,
             B_is_set = true;
        //[666, 450]      [580, 710]      [638, 12]       [625, 715]
        cv::Point2i A = cv::Point2i(666, 450),
                    B = cv::Point2i(580, 710),
                    A_warped = cv::Point2i(638, 12),
                    B_warped = cv::Point2i(625, 715);
        const int radius = 3;
        const double scaling = .6;
        cv::Mat transformation_matrix;
};