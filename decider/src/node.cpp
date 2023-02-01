#include "node.hpp"
#include "decider.hpp"

#include "rclcpp/qos.hpp"

using std::placeholders::_1;

Node::Node(): rclcpp::Node("decider") {
    this->declare_parameter<std::string>("sub_topic_1", "/path_information");
    this->declare_parameter<std::string>("sub_topic_2", "/sick/scan");
    this->declare_parameter<std::string>("publish_topic", "/path_information");


    this->sub_1 = this->create_subscription<custom_msgs::msg::Distance>(custom_parameters.sub_topic_1, rclcpp::SystemDefaultsQoS(), std::bind(&Node::callback, this, _1));
    this->sub_2 = this->create_subscription<custom_msgs::msg::Distance>(custom_parameters.sub_topic_2, rclcpp::SystemDefaultsQoS(), std::bind(&Node::callback, this, _1));
    this->pub = this->create_publisher<custom_msgs::msg::Width>(custom_parameters.pub_topic, rclcpp::SystemDefaultsQoS());

}
void Node::callback(const custom_msgs::msg::Distance msg) {
    rclcpp::Logger logger = this->get_logger();

    this->val_buf_sub_top_1.pushback(decider::received_msg{
        msg->left,
        msg->right,
        msg->timestamp
    })


}