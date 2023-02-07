#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "custom_msgs/msg/distance.hpp"

#include "lidar_curb_detection.hpp"

#include <string>

class Node: public rclcpp::Node {
    public:
        Node(bool do_visualize = false);
    private:
        void callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    struct custom_parameters_t {
        std::string sub_topic;
        std::string pub_topic;

        double mounting_angle;
        double wheel_inside;
        double wheel_width;
        double advanced_ray_check_thr;
        double distance_thr;
        double height_diff;
        double avg_dist_thr;
        u_int max_check_length;
        u_int quantity_check_for_runaways;
        u_int quantity_thr;
        u_int angle_thr;
        u_int quantity_thr_for_smoother;
        u_int repetitions;
        u_int counter_thr;
        u_int quantity_check_for_avg_dist;
        u_int quantity_check_for_island;
        u_int counter_thr_for_island;
    } custom_parameters;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr    sub;
    rclcpp::Publisher<custom_msgs::msg::Distance>::SharedPtr        pub;

    bool do_visualize;
    std::vector<filters::limit> limits_vec;
};