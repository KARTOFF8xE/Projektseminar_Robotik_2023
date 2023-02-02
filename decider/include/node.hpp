#pragma once

#include "rclcpp/rclcpp.hpp"
#include "custom_msgs/msg/distance.hpp"

#include "decider.hpp"

class Node: public rclcpp::Node {
    public:
        Node();
    private:
        void callback_1(const custom_msgs::msg::Distance::SharedPtr msg);
        void callback_2(const custom_msgs::msg::Distance::SharedPtr msg);

    struct custom_parameters_t {
        std::string sub_topic_1;
        std::string sub_topic_2;
        std::string pub_topic;
    } custom_parameters;

    rclcpp::Subscription<custom_msgs::msg::Distance>::SharedPtr sub_1;
    rclcpp::Subscription<custom_msgs::msg::Distance>::SharedPtr sub_2;
    rclcpp::Publisher<custom_msgs::msg::Distance>::SharedPtr    pub;
    

    std::vector<decider::limit> val_buf_sub_top_1_left;
    std::vector<decider::limit> val_buf_sub_top_1_right;
    std::vector<decider::limit> val_buf_sub_top_2_left;
    std::vector<decider::limit> val_buf_sub_top_2_right;
    std::vector<decider::limit> limits;
    //TODO: Vektor in Vektor in welchem alle Rechten Limits pro Topic gespeichert werden
    //TODO: Vektor in welchem alle validen Limits gespeichert werden
    //TODO: Vektor in welchem alle Distanzen gespeichert werden
};