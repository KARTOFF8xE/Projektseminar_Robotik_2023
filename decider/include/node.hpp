#pragma once

#include "rclcpp/rclcpp.hpp"
#include "custom_msgs/msg/distance.hpp"

#include "decider.hpp"

class Node: public rclcpp::Node {
    public:
        Node();
    private:
        void callback_lf(const custom_msgs::msg::Distance::SharedPtr msg);
        void callback_hf(const custom_msgs::msg::Distance::SharedPtr msg);
        void timer_callback();

    struct custom_parameters_t {
        std::string sub_topic_lf;
        std::string sub_topic_hf;
        std::string pub_topic;
    } custom_parameters;

    rclcpp::Subscription<custom_msgs::msg::Distance>::SharedPtr sub_lf;
    rclcpp::Subscription<custom_msgs::msg::Distance>::SharedPtr sub_hf;
    rclcpp::Publisher<custom_msgs::msg::Distance>::SharedPtr    pub;
    rclcpp::TimerBase::SharedPtr                                timer_;
    

    std::vector<decider::limit> val_buf_sub_top_lf_left;
    std::vector<decider::limit> val_buf_sub_top_lf_right;
    std::vector<decider::limit> val_buf_sub_top_hf_left;
    std::vector<decider::limit> val_buf_sub_top_hf_right;
    std::vector<decider::limit> limits;
    bool left_limit_exists = false;
    bool right_limit_exists = false;
};