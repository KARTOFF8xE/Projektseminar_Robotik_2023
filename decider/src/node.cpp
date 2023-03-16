#include "node.hpp"
#include "decider.hpp"

#include <chrono>
#include <utility>
#include <optional>

#include "rclcpp/qos.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;


Node::Node(): rclcpp::Node("decider") {
    this->declare_parameter<std::string>("topics.sub.camera",   "/lidar_path_width");
    this->declare_parameter<std::string>("topics.sub.lidar",    "/camera_path_width");
    this->declare_parameter<std::string>("topics.pub",          "/path_width");
    this->declare_parameter<int>   ("params.timer_delay",       200); //[ms]
    this->declare_parameter<double>("params.time_diff_thr",     1.0); //[s]

    this->get_parameter("topics.sub.camera",        custom_parameters.sub_topic_camera);
    this->get_parameter("topics.sub.lidar",         custom_parameters.sub_topic_lidar);
    this->get_parameter("topics.pub",               custom_parameters.pub_topic);
    this->get_parameter("params.timer_delay",       custom_parameters.timer_delay);
    this->get_parameter("params.time_diff_thr",     custom_parameters.time_diff_thr);

    this->sub_camera    = this->create_subscription<custom_msgs::msg::Distance>(custom_parameters.sub_topic_camera, rclcpp::SystemDefaultsQoS(), std::bind(&Node::callback_camera, this, _1));
    this->sub_lidar     = this->create_subscription<custom_msgs::msg::Distance>(custom_parameters.sub_topic_lidar,  rclcpp::SystemDefaultsQoS(), std::bind(&Node::callback_lidar,  this, _1));
    this->pub           = this->create_publisher<custom_msgs::msg::Distance>(custom_parameters.pub_topic, rclcpp::SystemDefaultsQoS());
    this->timer         = this->create_wall_timer(std::chrono::milliseconds(custom_parameters.timer_delay), std::bind(&Node::callback_merge_timer, this)); //5Hz
}

void Node::callback_camera(const custom_msgs::msg::Distance::SharedPtr msg) {
    while (this->stop_buffer_write) {} //TODO: is there a bettter way to do this?
    
    this->camera_buffer.push_back(filters::limit{rclcpp::Time(msg->header.stamp), msg->left, msg->right});
}

void Node::callback_lidar(const custom_msgs::msg::Distance::SharedPtr msg) {
    while (this->stop_buffer_write) {} //TODO: is there a bettter way to do this?
    
    this->lidar_buffer.push_back(filters::limit{rclcpp::Time(msg->header.stamp), msg->left, msg->right});
}

void Node::callback_merge_timer() {
    rclcpp::Logger logger = this->get_logger();

    this->stop_buffer_write = true;
    //to ensure std::{min|max}_element iterators will never be .end()
    if (camera_buffer.empty() || lidar_buffer.empty()) {
        RCLCPP_DEBUG_STREAM(logger, "[timer callback merge] Some buffers are empty.");
        this->stop_buffer_write = false;
        return;
    } else {
        RCLCPP_DEBUG_STREAM(logger, "[timer callback merge] camera buffer size: " << camera_buffer.size() << " lidar buffer size: " << lidar_buffer.size());
    }

    //get pair of camera/lidar values to merge
    std::optional<std::pair<filters::limit, filters::limit>> pair = decider::getTimedPair(camera_buffer, lidar_buffer, custom_parameters.time_diff_thr, logger);
    this->stop_buffer_write = false;

    //exit here if no pair was found
    if (!pair.has_value()) {
        return;
    }
    //else merge pair into one limit
    filters::limit merged_limit = decider::mergeTimedPair(pair.value());
    this->left_output_buffer[this->output_buffer_position]  = merged_limit.left;
    this->right_output_buffer[this->output_buffer_position] = merged_limit.right;
    this->output_buffer_position = (this->output_buffer_position + 1) / OUTPUT_BUFFER_SIZE;

    double left  = std::accumulate(this->left_output_buffer.begin(),  this->left_output_buffer.end(),  0.0) / OUTPUT_BUFFER_SIZE,
           right = std::accumulate(this->right_output_buffer.begin(), this->right_output_buffer.end(), 0.0) / OUTPUT_BUFFER_SIZE;

    custom_msgs::msg::Distance msg;
    msg.header.set__stamp(merged_limit.timestamp);
    msg.set__left(left);
    msg.set__right(right);
    msg.set__width(left + right);

    this->pub->publish(msg);
}