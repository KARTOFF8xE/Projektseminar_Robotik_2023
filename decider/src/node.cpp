#include "node.hpp"
#include "decider.hpp"

#include "rclcpp/qos.hpp"

using std::placeholders::_1;

Node::Node(): rclcpp::Node("decider") {
    this->declare_parameter<std::string>("sub_topic_1",         "/path_information");
    this->declare_parameter<std::string>("sub_topic_2",         "/sick/scan");
    this->declare_parameter<std::string>("publish_topic",       "/path_information");

    this->get_parameter("sub_topic_1",                          custom_parameters.sub_topic_1);
    this->get_parameter("sub_topic_2",                          custom_parameters.sub_topic_2);
    this->get_parameter("publish_topic",                        custom_parameters.pub_topic);

    this->sub_1 = this->create_subscription<custom_msgs::msg::Distance>(custom_parameters.sub_topic_1, rclcpp::SystemDefaultsQoS(), std::bind(&Node::callback_1, this, _1));
    this->sub_2 = this->create_subscription<custom_msgs::msg::Distance>(custom_parameters.sub_topic_2, rclcpp::SystemDefaultsQoS(), std::bind(&Node::callback_2, this, _1));
    this->pub   = this->create_publisher<custom_msgs::msg::Distance>(custom_parameters.pub_topic, rclcpp::SystemDefaultsQoS());
}

void Node::callback_1(const custom_msgs::msg::Distance::SharedPtr msg) {
    rclcpp::Logger logger = this->get_logger();
    /**
     * Lege die Limits in die jeweiligen Buffer
    */
    this->val_buf_sub_top_1_left.push_back(decider::limit{
        msg->left,
        msg->header.stamp
    });
    this->val_buf_sub_top_1_right.push_back(decider::limit{
        msg->left,
        msg->header.stamp
    });

    
    decider::limit left_limit;
    decider::limit right_limit;
    if ((val_buf_sub_top_1_left.size() > 3) && (val_buf_sub_top_2_left.size() > 10) &&
        (val_buf_sub_top_1_right.size()) > 3 && (val_buf_sub_top_2_right.size() > 10)) {
        left_limit = decider::get_limits(val_buf_sub_top_1_left[0], val_buf_sub_top_2_left);
            val_buf_sub_top_1_left.erase(val_buf_sub_top_1_left.begin());
        right_limit = decider::get_limits(val_buf_sub_top_1_right[0], val_buf_sub_top_2_right);
            val_buf_sub_top_1_right.erase(val_buf_sub_top_1_right.begin());
        
        auto pub_msg = custom_msgs::msg::Distance();
        pub_msg.header.stamp = left_limit.timestamp;
        pub_msg.left = left_limit.limit;
        pub_msg.right = right_limit.limit;
        pub->publish(pub_msg);
    }
}

void Node::callback_2(const custom_msgs::msg::Distance::SharedPtr msg) {
    rclcpp::Logger logger = this->get_logger();
    /**
     * Wenn das erhaltene Limit jeweils existiert, wird es in den Buffer geladen, andernfalls nicht
    */
    if (msg->left > 0) {    
        this->val_buf_sub_top_2_left.push_back(decider::limit{
            msg->left,
            msg->header.stamp
        });
    }
    if (msg->right > 0) {
        this->val_buf_sub_top_2_right.push_back(decider::limit{
            msg->left,
            msg->header.stamp
        });
    }
}