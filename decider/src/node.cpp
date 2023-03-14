#include "node.hpp"
#include "decider.hpp"
#include <chrono>

#include "rclcpp/qos.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

Node::Node(): rclcpp::Node("decider") {
    this->declare_parameter<std::string>("sub_topic_hf",        "/lidar_path_width");  // hf stands for "high frequency"
    this->declare_parameter<std::string>("sub_topic_lf",        "/camera_path_width"); // lf stands for "low frequency"
    this->declare_parameter<std::string>("publish_topic",       "/path_width");

    this->get_parameter("sub_topic_lf",                         custom_parameters.sub_topic_lf);
    this->get_parameter("sub_topic_hf",                         custom_parameters.sub_topic_hf);
    this->get_parameter("publish_topic",                        custom_parameters.pub_topic);

    this->sub_lf = this->create_subscription<custom_msgs::msg::Distance>(custom_parameters.sub_topic_lf, rclcpp::SystemDefaultsQoS(), std::bind(&Node::callback_lf, this, _1));
    this->sub_hf = this->create_subscription<custom_msgs::msg::Distance>(custom_parameters.sub_topic_hf, rclcpp::SystemDefaultsQoS(), std::bind(&Node::callback_hf, this, _1));
    this->pub   = this->create_publisher<custom_msgs::msg::Distance>(custom_parameters.pub_topic, rclcpp::SystemDefaultsQoS());
    this->timer_= this->create_wall_timer(5ms, std::bind(&Node::timer_callback, this));
}

void Node::callback_lf(const custom_msgs::msg::Distance::SharedPtr msg) {
    rclcpp::Logger logger = this->get_logger();
    /**
     * Lege die Limits in die jeweiligen Buffer
    */
    this->val_buf_sub_top_lf_left.push_back(decider::limit{
        msg->left,
        msg->header.stamp
    });
    this->val_buf_sub_top_lf_right.push_back(decider::limit{
        msg->left,
        msg->header.stamp
    });

    return;
}

void Node::callback_hf(const custom_msgs::msg::Distance::SharedPtr msg) {
    rclcpp::Logger logger = this->get_logger();
    /**
     * Wenn das erhaltene Limit jeweils existiert, wird es in den Buffer geladen, andernfalls nicht
    */
    if (msg->left > 0) {    
        this->val_buf_sub_top_hf_left.push_back(decider::limit{
            msg->left,
            msg->header.stamp
        });
    }
    if (msg->right > 0) {
        this->val_buf_sub_top_hf_right.push_back(decider::limit{
            msg->left,
            msg->header.stamp
        });
    }

    return;
}

void Node::timer_callback() {
    decider::limit left_limit;
    decider::limit right_limit;
    if ((val_buf_sub_top_lf_left.size() > 3) && (val_buf_sub_top_hf_left.size() > 10) &&
        (val_buf_sub_top_lf_right.size()) > 3 && (val_buf_sub_top_hf_right.size() > 10)) {
        left_limit = decider::get_limits(val_buf_sub_top_lf_left[0], val_buf_sub_top_hf_left);
            val_buf_sub_top_hf_left.erase(val_buf_sub_top_lf_left.begin());
        right_limit = decider::get_limits(val_buf_sub_top_lf_right[0], val_buf_sub_top_hf_right);
            val_buf_sub_top_hf_right.erase(val_buf_sub_top_lf_right.begin());
        
        auto pub_msg = custom_msgs::msg::Distance();
        pub_msg.header.stamp = left_limit.timestamp;
        pub_msg.left = left_limit.limit;
        pub_msg.right = right_limit.limit;
        if (left_limit.limit > 0 && right_limit.limit > 0) {
            pub_msg.width = left_limit.limit + right_limit.limit;
        }
        pub->publish(pub_msg);
    }

    return;
}