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
    this->declare_parameter<int>   ("params.timer_delay",       50); //[ms]
    this->declare_parameter<double>("params.time_diff_thr",     1.0); //[s]
    this->declare_parameter<double>("params.disgard_time_thr",  25.0); //[s]

    this->get_parameter("topics.sub.camera",        custom_parameters.sub_topic_camera);
    this->get_parameter("topics.sub.lidar",         custom_parameters.sub_topic_lidar);
    this->get_parameter("topics.pub",               custom_parameters.pub_topic);
    this->get_parameter("params.timer_delay",       custom_parameters.timer_delay);
    this->get_parameter("params.time_diff_thr",     custom_parameters.time_diff_thr);
    this->get_parameter("params.disgard_time_thr",  custom_parameters.disgard_time_thr);

    this->sub_camera    = this->create_subscription<custom_msgs::msg::Distance>(custom_parameters.sub_topic_camera, rclcpp::SystemDefaultsQoS(), std::bind(&Node::callback_camera, this, _1));
    this->sub_lidar     = this->create_subscription<custom_msgs::msg::Distance>(custom_parameters.sub_topic_lidar,  rclcpp::SystemDefaultsQoS(), std::bind(&Node::callback_lidar,  this, _1));
    this->pub           = this->create_publisher<custom_msgs::msg::Distance>(custom_parameters.pub_topic, rclcpp::SystemDefaultsQoS());
    this->timer         = this->create_wall_timer(std::chrono::milliseconds(custom_parameters.timer_delay), std::bind(&Node::callback_merge_timer, this)); //5Hz
}

void Node::callback_camera(const custom_msgs::msg::Distance::SharedPtr msg) {
    while (this->stop_buffer_write) {} //TODO: is there a bettter way to do this?
    
    if (msg->left > 0 || msg->right > 0) {
        this->camera_buffer.push_back(filters::limit{rclcpp::Time(msg->header.stamp), msg->left, msg->right});
    }
}

void Node::callback_lidar(const custom_msgs::msg::Distance::SharedPtr msg) {
    while (this->stop_buffer_write) {} //TODO: is there a bettter way to do this?
    
    if (msg->left > 0 || msg->right > 0) {
        this->lidar_buffer.push_back(filters::limit{rclcpp::Time(msg->header.stamp), msg->left, msg->right});
    }
}

void Node::callback_merge_timer() {
    rclcpp::Logger logger = this->get_logger();

    this->stop_buffer_write = true;
    //to ensure std::{min|max}_element iterators will never be .end()
    if (camera_buffer.empty() || lidar_buffer.empty()) {
        RCLCPP_INFO_STREAM(logger, "[timer callback merge] Some buffers are empty.");
        this->stop_buffer_write = false;
        return;
    } else {
        RCLCPP_INFO_STREAM(logger, "[timer callback merge] camera buffer size: " << camera_buffer.size() << " lidar buffer size: " << lidar_buffer.size());
    }

    rclcpp::Time most_recent_timestamp = decider::getMostRecentLimitTimestamp(this->camera_buffer, this->lidar_buffer);
    RCLCPP_INFO_STREAM(logger, "[timer callback merge] most recent timestamp: " << most_recent_timestamp);
    this->camera_buffer = std::move(decider::removeTooOldLimits(this->camera_buffer, most_recent_timestamp, custom_parameters.disgard_time_thr));
    this->lidar_buffer  = std::move(decider::removeTooOldLimits(this->lidar_buffer, most_recent_timestamp, custom_parameters.disgard_time_thr));

    if (camera_buffer.empty() || lidar_buffer.empty()) {
        RCLCPP_INFO_STREAM(logger, "[timer callback merge] Some buffers are empty after cleanup.");
        this->stop_buffer_write = false;
        return;
    } else {
        RCLCPP_INFO_STREAM(logger, "[timer callback merge] camera buffer size: " << camera_buffer.size() << " lidar buffer size: " << lidar_buffer.size() << " after cleanup.");
    }

    std::optional<std::pair<filters::limit, filters::limit>> pair = decider::getTimedPair(camera_buffer, lidar_buffer, custom_parameters.time_diff_thr, logger);
    this->stop_buffer_write = false;

    if (!pair.has_value()) {
        return;
    }
    filters::limit merged_limit = decider::mergeTimedPair(pair.value());

    custom_msgs::msg::Distance msg;
    // uint64_t time_since_epoch_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    // msg.header.set__stamp(rclcpp::Time(time_since_epoch_ns, RCL_SYSTEM_TIME));
    msg.header.set__stamp(merged_limit.timestamp);
    msg.set__left(merged_limit.left);
    msg.set__right(merged_limit.right);
    msg.set__width(merged_limit.left + merged_limit.right);

    this->pub->publish(msg);
}