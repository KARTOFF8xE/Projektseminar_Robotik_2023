#pragma once

//#include <sl/Camera.hpp>

#include <opencv2/opencv.hpp>

#include <string>
#include <memory>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "zed_interfaces/msg/depth_info_stamped.hpp"
#include "custom_msgs/msg/distance.hpp"

//when callbacks have more then one argument this cast will hide that fact from ros
#define MAKE_SINGLE_ARGUMENT(msg_type, function_bind) static_cast<std::function<void(const msg_type::SharedPtr)>>(function_bind)

class Node: public rclcpp::Node {
    public:
        Node();
        // ~Node();
    private:
        void callback_camera_info(const sensor_msgs::msg::CameraInfo::SharedPtr msg, rclcpp::Logger& logger);
        void callback_imu(const sensor_msgs::msg::Imu::SharedPtr msg, rclcpp::Logger& logger);
        void callback_depth_info(const zed_interfaces::msg::DepthInfoStamped::SharedPtr msg, rclcpp::Logger& logger);
        void callback_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg, rclcpp::Logger& logger);
        void callback_depth_image(const sensor_msgs::msg::Image::SharedPtr msg, rclcpp::Logger& logger);
        void callback_rgb_image(const sensor_msgs::msg::Image::SharedPtr msg, rclcpp::Logger& logger);

        struct custom_parameters_t {
            //topics
            std::string camera_info_topic;
            std::string imu_topic;
            std::string depth_info_topic;
            std::string pointcloud_topic;
            std::string rgb_image_topic;
            std::string depth_image_topic;

            std::string pub_topic;

            //other
            float filter_distance;
        } custom_parameters;

        //rclcpp
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr           camera_info_sub;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr                  imu_sub;
        rclcpp::Subscription<zed_interfaces::msg::DepthInfoStamped>::SharedPtr  depth_info_sub;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr          pointcloud2_sub;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr                depth_image_sub;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr                rgb_image_sub;
        rclcpp::Publisher<custom_msgs::msg::Distance>::SharedPtr                publisher;

        //misc
        bool is_run_in_debugger;
        cv::Mat point_cloud,    //point cloud for retrieval of real world coordinates
                depth_image,    //depth image for filtering
                K;              //intrinsic camera  calibration matrix
        std::optional<cv::Vec3d> euler_angles = std::nullopt; //pitch, yaw, roll
        struct range_limits_t { //depth image range limits
            float min = -1; //default -1 for checking wether or not it is already set
            float max;
        } range_limits; 
};