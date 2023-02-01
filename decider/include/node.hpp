#pragma once

#include "rclcpp/rclcpp.hpp"
#include "custom_msgs/msg/distance.hpp"
#include "custom_msgs/msg/width.hpp"

#include "decider.hpp"

class Node: public rclcpp::Node {
    private:
        void callback(const custom_msgs::msg::Distance msg);

    struct custom_parameters_t {
        std::string sub_topic_1;
        std::string sub_topic_2;
        std::string pub_topic;
    } custom_parameters;

    rclcpp::Subscription<custom_msgs::msg::Distance>::SharedPtr sub_1;
    rclcpp::Subscription<custom_msgs::msg::Distance>::SharedPtr sub_2;
    rclcpp::Publisher<custom_msgs::msg::Distance>::SharedPtr    pub;
    

    std::vector<decider::received_msg> val_buf_sub_top_1;
    std::vector<decider::received_msg> val_buf_sub_top_2;
    std::vector<decider::received_msg> limits;
    //TODO: Vektor in Vektor in welchem alle Rechten Limits pro Topic gespeichert werden
    //TODO: Vektor in welchem alle validen Limits gespeichert werden
    //TODO: Vektor in welchem alle Distanzen gespeichert werden
};