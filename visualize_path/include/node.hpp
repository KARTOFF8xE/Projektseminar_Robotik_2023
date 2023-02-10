#pragma once

#include "rclcpp/rclcpp.hpp"
#include "custom_msgs/msg/distance.hpp"

#include "visualize_path.hpp"

#include <string>

class Node: public rclcpp::Node {
    public:
        Node(std::string sub_topic);
    private:
        void callback(const custom_msgs::msg::Distance::SharedPtr msg);

    struct custom_parameters_t {
        double wheel_inside;
        double wheel_width;
    } custom_parameters;

    rclcpp::Subscription<custom_msgs::msg::Distance>::SharedPtr    sub;

    std::vector<filters::limit> limits_vec;
};