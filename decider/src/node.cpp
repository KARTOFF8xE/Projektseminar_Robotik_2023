#include "node.hpp"
#include "decider.hpp"

#include "rclcpp/qos.hpp"

using std::placeholders::_1;

Node::Node(): rclcpp::Node("decider") {

    this->sub = this->create_subscription<custom_msgs::msg::Distance>(custom_parameters.sub_topic, rclcpp::SystemDefaultsQoS(), std::bind(&Node::callback, this, _1));
    this->pub = this->create_publisher<custom_msgs::msg::Width>(custom_parameters.pub_topic, rclcpp::SystemDefaultsQoS());

}
void Node::callback(const custom_msgs::msg::Distance msg) {
    rclcpp::Logger logger = this->get_logger();

}