#pragma once

//#include <sl/Camera.hpp>

#include <opencv2/opencv.hpp>

#include <string>
#include <memory>
#include <optional>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "zed_interfaces/msg/depth_info_stamped.hpp"
#include "custom_msgs/msg/distance.hpp"

#include "filters/filters.hpp"

//when callbacks have more then one argument this cast will hide that fact from ros
#define MAKE_SINGLE_ARGUMENT(msg_type, function_bind) static_cast<std::function<void(const msg_type::SharedPtr)>>(function_bind)

#define DEBUG //define to show image output

class Node: public rclcpp::Node {
    public:
        Node(bool visualize);
        #ifdef DEBUG
        ~Node();
        #endif //DEBUG
    private:
        void callback_camera_info(const sensor_msgs::msg::CameraInfo::SharedPtr msg, rclcpp::Logger& logger);
        void callback_imu(const sensor_msgs::msg::Imu::SharedPtr msg, rclcpp::Logger& logger);
        void callback_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg, rclcpp::Logger& logger);
        void callback_rgb_image(const sensor_msgs::msg::Image::SharedPtr msg, rclcpp::Logger& logger);

        struct custom_parameters_t {
            //topics
            std::string camera_info_topic;
            std::string imu_topic;
            std::string pointcloud_topic;
            std::string rgb_image_topic;

            std::string pub_topic;

            //other
            double angle_filter;
            double optimal_line_angle;
            double relative_scan_line_height;

            double distance_thr;
            int quantity_check_for_runaways;
            int quantity_thr;
            int quantity_check_for_avg_dist;
            int counter_thr_for_avg;
            double avg_dist_thr;
            int quantity_check_for_island;
            int counter_thr_for_island;
        } custom_parameters;

        //rclcpp
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr           camera_info_sub;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr                  imu_sub;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr          pointcloud2_sub;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr                rgb_image_sub;
        rclcpp::Publisher<custom_msgs::msg::Distance>::SharedPtr                publisher;

        //misc
        std::array<double, 9ul> K;                              //intrinsic camera  calibration matrix as array
        cv::Mat pointcloud;                                     //point cloud for retrieval of real world coordinates
        std::optional<cv::Vec3d> euler_angles = std::nullopt;   //pitch, yaw, roll
        std::vector<filters::limit> limits_buffer;              //filtering buffer for detected limits
        size_t high_deviations_min_buffer_size,
               runaways_min_buffer_size,
               islands_min_buffer_size;

        //visualization
        bool do_visualize;
        bool is_run_in_debugger;
        const cv::Scalar blue   = cv::Scalar(255, 0, 0),
                         green  = cv::Scalar(0, 255, 0),
                         red    = cv::Scalar(0, 0, 255);
                         

        #ifdef DEBUG
        //metric testing
        std::ofstream csv_file;
        #endif //DEBUG
};