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

cv::Vec3f getPointFromPointcloud(const cv::Mat& pointcloud, const cv::Point2i& pt) {
    cv::Vec4f pt_4D = pointcloud.at<cv::Vec4f>(pt);
    return cv::Vec3f(pt_4D[0], pt_4D[1], pt_4D[2]);
}

Node::Node(): rclcpp::Node("test_pkg") {
    this->declare_parameter<std::string>("topics.flux.camera_info", "");
    this->declare_parameter<std::string>("topics.flux.imu",         "");
    this->declare_parameter<std::string>("topics.flux.pointcloud",  "");
    this->declare_parameter<std::string>("topics.flux.odometry",    "");
    this->declare_parameter<std::string>("topics.flux.rgb_image",   "");

    this->get_parameter("topics.flux.camera_info",  custom_parameters.camera_info_topic);
    this->get_parameter("topics.flux.imu",          custom_parameters.imu_topic);
    this->get_parameter("topics.flux.pointcloud",   custom_parameters.pointcloud_topic);
    this->get_parameter("topics.flux.odometry",     custom_parameters.odometry_topic);
    this->get_parameter("topics.flux.rgb_image",    custom_parameters.rgb_image_topic);

    rclcpp::Logger logger = this->get_logger();

    cv::namedWindow("image", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("warped", cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback("image", Node::callback_mouse_image, static_cast<void*>(this));
    cv::setMouseCallback("warped", Node::callback_mouse_warped_image, static_cast<void*>(this));

    RCLCPP_ERROR_EXPRESSION(logger, custom_parameters.camera_info_topic == "", "Empty camera info topic.");
    RCLCPP_ERROR_EXPRESSION(logger, custom_parameters.imu_topic         == "", "Empty imu topic.");
    RCLCPP_ERROR_EXPRESSION(logger, custom_parameters.pointcloud_topic  == "", "Empty pointcloud topic.");
    RCLCPP_ERROR_EXPRESSION(logger, custom_parameters.odometry_topic    == "", "Empty odometry topic.");
    RCLCPP_ERROR_EXPRESSION(logger, custom_parameters.rgb_image_topic   == "", "Empty rgb image topic.");

    this->rgb_camera_sub    = this->create_subscription<sensor_msgs::msg::CameraInfo>(custom_parameters.camera_info_topic,  rclcpp::SensorDataQoS(), MAKE_SINGLE_ARGUMENT(sensor_msgs::msg::CameraInfo, std::bind(&Node::callback_camera_info, this, _1, logger)));
    this->imu_sub           = this->create_subscription<sensor_msgs::msg::Imu>(custom_parameters.imu_topic,                 rclcpp::SensorDataQoS(), MAKE_SINGLE_ARGUMENT(sensor_msgs::msg::Imu, std::bind(&Node::callback_imu, this, _1, logger)));
    this->pointcloud2_sub   = this->create_subscription<sensor_msgs::msg::PointCloud2>(custom_parameters.pointcloud_topic,  rclcpp::SensorDataQoS(), MAKE_SINGLE_ARGUMENT(sensor_msgs::msg::PointCloud2, std::bind(&Node::callback_pointcloud, this, _1, logger)));
    this->rgb_image_sub     = this->create_subscription<sensor_msgs::msg::Image>(custom_parameters.rgb_image_topic,         rclcpp::SensorDataQoS(), MAKE_SINGLE_ARGUMENT(sensor_msgs::msg::Image, std::bind(&Node::callback_rgb_image, this, _1, logger)));
}

void Node::callback_mouse_image(int event, int x, int y, int, void *userdata) {
    if (event == cv::EVENT_LBUTTONUP) {
        Node* node = static_cast<Node*>(userdata);

        node->A = cv::Point2i(x, y) / node->scaling;
        node->A_warped = src::warpPoint(node->transformation_matrix, node->A);
        node->A_is_set = true;

    } else if (event == cv::EVENT_RBUTTONUP) {
        Node* node = static_cast<Node*>(userdata);

        node->B = cv::Point2i(x, y) / node->scaling;
        node->B_warped = src::warpPoint(node->transformation_matrix, node->B);
        node->B_is_set = true;
    }
}

void Node::callback_mouse_warped_image(int event, int x, int y, int, void *userdata) {
    if (event == cv::EVENT_LBUTTONUP) {
        Node* node = static_cast<Node*>(userdata);

        node->A_warped = cv::Point2i(x, y) / node->scaling;
        node->A = src::unwarpPoint(node->transformation_matrix, node->A_warped);
        node->A_is_set = true;

    } else if (event == cv::EVENT_RBUTTONUP) {
        Node* node = static_cast<Node*>(userdata);

        node->B_warped = cv::Point2i(x, y) / node->scaling;
        node->B = src::unwarpPoint(node->transformation_matrix, node->B_warped);
        node->B_is_set = true;
    }
}

void Node::callback_camera_info(const sensor_msgs::msg::CameraInfo::SharedPtr msg, rclcpp::Logger& logger) {
    RCLCPP_INFO_ONCE(logger, "[camera info] Got first callback.");

    this->K = msg->k;
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
        RCLCPP_DEBUG(logger, "[rgb image] Attempted to get rgb image before pointcloud was set.");
        return;
    } else if (this->K.empty()) {
        RCLCPP_DEBUG(logger, "[rgb image] Attempted to get rgb image before K was set.");
        return;
    } else if (!this->euler_angles.has_value()) {
        RCLCPP_DEBUG(logger, "[rgb image] Attempted to get rgb image before camera orientation was set.");
        return;
    }
    RCLCPP_INFO_ONCE(logger, "[rgb image] Got first callback.");

    if (this->continue_updating) {
        //get image from msg
        filters::encoding_info_t image_encoding = filters::get_encoding_info(msg->encoding);
        cv::Mat image(msg->height, msg->width, image_encoding.type, &(msg->data[0]), msg->step);
        if (image_encoding.conversion != cv::COLOR_COLORCVT_MAX) { //convert to BGR/BGRA if not already so
            cv::cvtColor(image, image, image_encoding.conversion);
        }

        //convert camera view to top down
        //cv::Mat transformation_matrix;
        std::vector<cv::Point2i> trapeze;

        cv::Size image_size = image.size();
        cv::Vec3d euler_angles = this->euler_angles.value();
        //cv::Mat K(3, 3, CV_64FC1, &this->K[0]);
        cv::Mat K = cv::Mat(this->K, true).reshape(1, 3);

        cv::Point2d vanishing_point = src::getVanishingPoint(K, euler_angles[0] - M_PI, euler_angles[1] + M_PI_2);
        if (!src::getTransformation(this->transformation_matrix, trapeze, image_size, vanishing_point)) {
            RCLCPP_WARN(logger, "Could not find transformation matrix.");
            return;
        }
        
        cv::Mat warped;
        src::transformToTopDown(image, warped, this->transformation_matrix, image_size);

        src::drawVanishingLines(image, vanishing_point, trapeze);

        image.copyTo(this->image);
        warped.copyTo(this->warped);
    }

    //display
    cv::Mat draw_image, draw_warped,
            resize_image, resize_warped;

    this->image.copyTo(draw_image);
    this->warped.copyTo(draw_warped);

    if (this->A_is_set) {
        cv::circle(draw_image, this->A, this->radius, cv::Scalar(0, 0, 255), 2);
        cv::circle(draw_warped, this->A_warped, this->radius, cv::Scalar(0, 0, 255), 2);
    }
    if (this->B_is_set) {
        cv::circle(draw_image, this->B, this->radius, cv::Scalar(0, 255, 0), 2);
        cv::circle(draw_warped, this->B_warped, this->radius, cv::Scalar(0, 255, 0), 2);
    }
    if (this->A_is_set && this->B_is_set) {
        RCLCPP_INFO_STREAM_ONCE(logger, "[rgb image] pts: " << this->A << '\t' << this->B << '\t' << this->A_warped << '\t' << this->B_warped);

        cv::line(draw_image, this->A, this->B, cv::Scalar(255, 0, 0));
        cv::line(draw_warped, this->A_warped, this->B_warped, cv::Scalar(255, 0, 0));

        cv::Vec3f A_3D = getPointFromPointcloud(this->point_cloud, this->A),
                  B_3D = getPointFromPointcloud(this->point_cloud, this->B);
        RCLCPP_INFO_STREAM(logger, "[rgb image] " << A_3D << " -> " << B_3D << " = " << cv::norm(B_3D - A_3D));
    }

    cv::resize(draw_image, resize_image, cv::Size(), this->scaling, this->scaling);
    cv::resize(draw_warped, resize_warped, cv::Size(), this->scaling, this->scaling);

    cv::imshow("image", resize_image);
    cv::imshow("warped", resize_warped);

    char key = cv::waitKey(10);
    if (key == 27) { //ESC
        this->A_is_set = false;
        this->B_is_set = false;
    } else if (key == 32) {
        this->continue_updating = !this->continue_updating;
        RCLCPP_INFO_STREAM(logger, "[rgb image] updated data retrieval bool to: " << std::boolalpha << this->continue_updating);
    }
}