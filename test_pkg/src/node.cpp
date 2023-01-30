#include "node.hpp"

#include "opencv2/opencv.hpp"
#include "opencv2/core/quaternion.hpp"

#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/region_of_interest.hpp"

#include "filters/filters.hpp"
#include "src.hpp"

#include <string>
#include <utility>
#include <memory>
#include <cmath>

using std::placeholders::_1;

Node::Node(): rclcpp::Node("test_pkg") {
    this->declare_parameter<std::string>("topics.flux.camera_info", "");
    this->declare_parameter<std::string>("topics.flux.imu",         "");
    this->declare_parameter<std::string>("topics.flux.pointcloud",  "");
    this->declare_parameter<std::string>("topics.flux.rgb_image",   "");

    this->get_parameter("topics.flux.camera_info",  custom_parameters.camera_info_topic);
    this->get_parameter("topics.flux.imu",          custom_parameters.imu_topic);
    this->get_parameter("topics.flux.pointcloud",   custom_parameters.pointcloud_topic);
    this->get_parameter("topics.flux.rgb_image",    custom_parameters.rgb_image_topic);

    rclcpp::Logger logger = this->get_logger();

    RCLCPP_ERROR_EXPRESSION(logger, custom_parameters.camera_info_topic == "", "Empty camera info topic.");
    RCLCPP_ERROR_EXPRESSION(logger, custom_parameters.imu_topic         == "", "Empty imu topic.");
    RCLCPP_ERROR_EXPRESSION(logger, custom_parameters.pointcloud_topic  == "", "Empty pointcloud topic.");
    RCLCPP_ERROR_EXPRESSION(logger, custom_parameters.rgb_image_topic   == "", "Empty rgb image topic.");

    this->rgb_camera_sub    = this->create_subscription<sensor_msgs::msg::CameraInfo>(custom_parameters.camera_info_topic,  rclcpp::SensorDataQoS(), MAKE_SINGLE_ARGUMENT(sensor_msgs::msg::CameraInfo, std::bind(&Node::callback_camera_info, this, _1, logger)));
    this->imu_sub           = this->create_subscription<sensor_msgs::msg::Imu>(custom_parameters.imu_topic,                 rclcpp::SensorDataQoS(), MAKE_SINGLE_ARGUMENT(sensor_msgs::msg::Imu, std::bind(&Node::callback_imu, this, _1, logger)));
    this->pointcloud2_sub   = this->create_subscription<sensor_msgs::msg::PointCloud2>(custom_parameters.pointcloud_topic,  rclcpp::SensorDataQoS(), MAKE_SINGLE_ARGUMENT(sensor_msgs::msg::PointCloud2, std::bind(&Node::callback_pointcloud, this, _1, logger)));
    this->rgb_image_sub     = this->create_subscription<sensor_msgs::msg::Image>(custom_parameters.rgb_image_topic,         rclcpp::SensorDataQoS(), MAKE_SINGLE_ARGUMENT(sensor_msgs::msg::Image, std::bind(&Node::callback_rgb_image, this, _1, logger)));
}

void Node::callback_camera_info(const sensor_msgs::msg::CameraInfo::SharedPtr msg, rclcpp::Logger& logger) {
    RCLCPP_INFO_ONCE(logger, "[camera info] Got first callback.");

    this->K = cv::Mat(3, 3, CV_64FC1, &msg->k[0]);
}

void Node::callback_imu(const sensor_msgs::msg::Imu::SharedPtr msg, rclcpp::Logger& logger) {
    RCLCPP_INFO_ONCE(logger, "[imu] Got first callback.");

    cv::Quat quaternion(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    this->euler_angles = quaternion.toEulerAngles(cv::QuatEnum::INT_YXZ);
}

void Node::callback_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg, rclcpp::Logger& logger) {
    RCLCPP_INFO_ONCE(logger, "[pointcloud] Got first callback.");

    this->point_cloud = cv::Mat(msg->height, msg->width, CV_32FC4, &msg->data[0], msg->row_step);
}

void Node::callback_rgb_image(const sensor_msgs::msg::Image::SharedPtr msg, rclcpp::Logger& logger) { //BGRA 8-bit
    if (this->point_cloud.empty()) {
        RCLCPP_DEBUG(logger, "Attempted to get rgb image before pointcloud was set.");
        return;
    } else if (this->K.empty()) {
        RCLCPP_DEBUG(logger, "Attempted to get rgb image before K was set.");
        return;
    } else if (!this->euler_angles.has_value()) {
        RCLCPP_DEBUG(logger, "Attempted to get rgb image before camera orientation was set.");
        return;
    }
    RCLCPP_INFO_ONCE(logger, "[rgb image] Got first callback.");

    filters::encoding_info_t image_encoding = filters::get_encoding_info(msg->encoding);
    cv::Mat image(msg->height, msg->width, image_encoding.type, &(msg->data[0]), msg->step);
    if (image_encoding.conversion != cv::COLOR_COLORCVT_MAX) { //convert to BGR/BGRA if not already so
        cv::cvtColor(image, image, image_encoding.conversion);
    }

    cv::Mat warped, image_copy;
    image.copyTo(image_copy);
    cv::Size size = image.size();

    cv::Vec3d euler_angles = this->euler_angles.value();
    cv::Point2d vanishing_point = src::get_vanishing_point(this->K, euler_angles[0] - M_PI, euler_angles[1] + M_PI_2);
    std::vector<cv::Point2i> trapeze = src::get_transformation_points(size, vanishing_point);
    src::draw_vanishing_lines(image_copy, vanishing_point, trapeze);
    src::transform_to_top_down(image, warped, trapeze, size);

    cv::imshow("image", image_copy);
    cv::imshow("warped", warped);
    if (cv::waitKey(10) == 27) { //ESC
        RCLCPP_DEBUG(logger, "Exiting node upon user request.");
        exit(0);
    }
}