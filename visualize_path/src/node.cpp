#include "node.hpp"

#include "visualize_path.hpp"
#include "filters/post_filters.hpp"

#include "rclcpp/qos.hpp"

#include <cmath>

using std::placeholders::_1;

Node::Node(): rclcpp::Node("visualize_path") {
    this->declare_parameter<std::string>("topics.dmc.laserscan",            "/sick/scan");
    this->declare_parameter<double> ("robot_specific.wheel_inside",         .2854);             // Distance of the vertical Plane in the center of the Robot to the verical inside Plane of the Wheels
    this->declare_parameter<double> ("robot_specific.wheel_width",          .1143);             // Width of the Wheels
    
    this->get_parameter("topics.dmc.laserscan",                 custom_parameters.sub_topic);
    this->get_parameter("robot_specific.wheel_inside",          custom_parameters.wheel_inside);
    this->get_parameter("robot_specific.wheel_width",           custom_parameters.wheel_width);

    this->sub = this->create_subscription<custom_msgs::msg::Distance>(custom_parameters.sub_topic, rclcpp::SensorDataQoS(), std::bind(&Node::callback, this, _1));
}

void Node::callback(const custom_msgs::msg::Distance::SharedPtr msg) {
    rclcpp::Logger logger = this->get_logger();

    visualize_path::visualize_street_view(limits_vec, custom_parameters.wheel_inside, custom_parameters.wheel_width);

}