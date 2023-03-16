#pragma once

#include "rclcpp/rclcpp.hpp"
#include "custom_msgs/msg/distance.hpp"

#include "filters/post_filters.hpp"

#include "decider.hpp"

class Node: public rclcpp::Node {
    public:
        Node();
    private:
        void callback_camera(const custom_msgs::msg::Distance::SharedPtr msg);
        void callback_lidar(const custom_msgs::msg::Distance::SharedPtr msg);
        void callback_merge_timer();

    struct custom_parameters_t {
        std::string sub_topic_camera;
        std::string sub_topic_lidar;
        std::string pub_topic;

        int timer_delay;
        double time_diff_thr;
        double disgard_time_thr;
    } custom_parameters;

    rclcpp::Subscription<custom_msgs::msg::Distance>::SharedPtr sub_camera,
                                                                sub_lidar;
    rclcpp::Publisher<custom_msgs::msg::Distance>::SharedPtr    pub;
    rclcpp::TimerBase::SharedPtr                                timer;
    
    std::vector<filters::limit> camera_buffer, lidar_buffer;
    bool stop_buffer_write = false;
};