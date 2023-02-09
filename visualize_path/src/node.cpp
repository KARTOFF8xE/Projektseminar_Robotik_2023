#include "node.hpp"

#include "visualize_path.hpp"
#include "filters/post_filters.hpp"

#include "rclcpp/qos.hpp"

#include <cmath>

using std::placeholders::_1;

Node::Node(): rclcpp::Node("visualize_path") {
    this->declare_parameter<std::string>("topics.sub_topic",                "/lidar_path_width");
    this->declare_parameter<double> ("robot_specific.wheel_inside",         .2854);             // Distance of the vertical Plane in the center of the Robot to the vertical inside Plane of the Wheels
    this->declare_parameter<double> ("robot_specific.wheel_width",          .1143);             // Width of the Wheels
    
    this->get_parameter("topics.sub_topic",                     custom_parameters.sub_topic);
    this->get_parameter("robot_specific.wheel_inside",          custom_parameters.wheel_inside);
    this->get_parameter("robot_specific.wheel_width",           custom_parameters.wheel_width);

    this->sub = this->create_subscription<custom_msgs::msg::Distance>(custom_parameters.sub_topic, rclcpp::SensorDataQoS(), std::bind(&Node::callback, this, _1));
}

void Node::callback(const custom_msgs::msg::Distance::SharedPtr msg) {
    rclcpp::Logger logger = this->get_logger();

    filters::limit limit {
        msg->header.stamp,
        msg->left,
        msg->right
    };

    limits_vec.push_back(limit);
    visualize_path::visualize_street_view(limits_vec, custom_parameters.wheel_inside, custom_parameters.wheel_width);
}