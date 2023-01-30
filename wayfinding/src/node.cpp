#include "node.hpp"

#include "filters/filters.hpp"
#include "wayfinding/wayfinding.hpp"

#include "rclcpp/qos.hpp"
#include "sensor_msgs/image_encodings.hpp"

#include "opencv2/core/quaternion.hpp"

#include <chrono>
#include <iostream>
#include <fstream>
//#include <unistd.h>

using namespace std::chrono_literals;
using namespace std::placeholders;

#define DEBUG

/**
 * Checks /proc/self/status for a TracerPid to determine if process is tracked.
 * 
 * @return true if process is tracked, false else
*/
bool isRunByDebugger() {
    const std::string delimiter_name = "TracerPid:";

    std::ifstream status_stream;
    //status_stream.open("/proc/" + std::to_string(::getpid()) + "/status", std::ios_base::in);
    status_stream.open("/proc/self/status", std::ios_base::in);
    
    std::string element;
    while (!status_stream.eof()) {
        status_stream >> element;
        
        if (element == delimiter_name) {
            status_stream >> element;

            return std::stoi(element) != 0;
        }
    }

    status_stream.close();

    //technically this should mean nothing but this assumption should be justifiable
    return false;
}

Node::Node(): rclcpp::Node("wayfinding") {
    this->declare_parameter<std::string>("topics.flux.camera_info", "/zed2i_c1/zed_node_c1/rgb/camera_info");
    this->declare_parameter<std::string>("topics.flux.imu",         "/imu/data");
    this->declare_parameter<std::string>("topics.flux.depth_info",  "/zed2i_c1/zed_node_c1/depth/depth_info");
    this->declare_parameter<std::string>("topics.flux.pointcloud",  "/zed2i_c1/zed_node_c1/point_cloud/cloud_registered");
    this->declare_parameter<std::string>("topics.flux.rgb_image",   "/zed2i_c1/zed_node_c1/rgb/image_rect_color");
    this->declare_parameter<std::string>("topics.flux.depth_image", "/zed2i_c1/zed_node_c1/depth/depth_registered");
    this->declare_parameter<std::string>("topics.pub",              "/path_width");
    this->declare_parameter<float>("params.filter_distance",        15); //meters

    this->get_parameter("topics.flux.camera_info",  custom_parameters.camera_info_topic);
    this->get_parameter("topics.flux.imu",          custom_parameters.imu_topic);
    this->get_parameter("topics.flux.depth_info",   custom_parameters.depth_info_topic);
    this->get_parameter("topics.flux.pointcloud",   custom_parameters.pointcloud_topic);
    this->get_parameter("topics.flux.rgb_image",    custom_parameters.rgb_image_topic);
    this->get_parameter("topics.flux.depth_image",  custom_parameters.depth_image_topic);
    this->get_parameter("topics.pub",               custom_parameters.pub_topic);
    this->get_parameter("params.filter_distance",   custom_parameters.filter_distance);


    //setup misc
    rclcpp::Logger logger = this->get_logger();
    this->is_run_in_debugger = isRunByDebugger();


    RCLCPP_ERROR_EXPRESSION(logger, custom_parameters.camera_info_topic == "", "Empty camera info topic.");
    RCLCPP_ERROR_EXPRESSION(logger, custom_parameters.imu_topic         == "", "Empty imu topic.");
    RCLCPP_ERROR_EXPRESSION(logger, custom_parameters.depth_info_topic  == "", "Empty depth info topic.");
    RCLCPP_ERROR_EXPRESSION(logger, custom_parameters.pointcloud_topic  == "", "Empty pointcloud topic.");
    RCLCPP_ERROR_EXPRESSION(logger, custom_parameters.rgb_image_topic   == "", "Empty rgb image topic.");
    RCLCPP_ERROR_EXPRESSION(logger, custom_parameters.depth_image_topic == "", "Empty depth image topic.");
    RCLCPP_ERROR_EXPRESSION(logger, custom_parameters.pub_topic         == "", "Empty publishing topic.");

    //setup publishers/subscribers
    this->camera_info_sub   = this->create_subscription<sensor_msgs::msg::CameraInfo>(custom_parameters.camera_info_topic,          rclcpp::SensorDataQoS(), MAKE_SINGLE_ARGUMENT(sensor_msgs::msg::CameraInfo, std::bind(&Node::callback_camera_info, this, _1, logger)));
    this->imu_sub           = this->create_subscription<sensor_msgs::msg::Imu>(custom_parameters.imu_topic,                         rclcpp::SensorDataQoS(), MAKE_SINGLE_ARGUMENT(sensor_msgs::msg::Imu, std::bind(&Node::callback_imu, this, _1, logger)));
    this->depth_info_sub    = this->create_subscription<zed_interfaces::msg::DepthInfoStamped>(custom_parameters.depth_info_topic,  rclcpp::ParametersQoS(), MAKE_SINGLE_ARGUMENT(zed_interfaces::msg::DepthInfoStamped, std::bind(&Node::callback_depth_info, this, _1, logger)));
    this->pointcloud2_sub   = this->create_subscription<sensor_msgs::msg::PointCloud2>(custom_parameters.pointcloud_topic,          rclcpp::SensorDataQoS(), MAKE_SINGLE_ARGUMENT(sensor_msgs::msg::PointCloud2, std::bind(&Node::callback_pointcloud, this, _1, logger)));
    this->rgb_image_sub     = this->create_subscription<sensor_msgs::msg::Image>(custom_parameters.rgb_image_topic,                 rclcpp::SensorDataQoS(), MAKE_SINGLE_ARGUMENT(sensor_msgs::msg::Image, std::bind(&Node::callback_rgb_image, this, _1, logger)));
    this->depth_image_sub   = this->create_subscription<sensor_msgs::msg::Image>(custom_parameters.depth_image_topic,               rclcpp::SensorDataQoS(), MAKE_SINGLE_ARGUMENT(sensor_msgs::msg::Image, std::bind(&Node::callback_depth_image, this, _1, logger)));
    //this->publisher         = this->create_publisher<custom_msgs::msg::Distance>(custom_parameters.pub_topic,                       rclcpp::SystemDefaultsQoS());
}

void Node::callback_camera_info(const sensor_msgs::msg::CameraInfo::SharedPtr msg, rclcpp::Logger& logger) {
    RCLCPP_DEBUG_ONCE(logger, "[camera info] Got first callback.");

    this->K = cv::Mat(3, 3, CV_64FC1, &msg->k[0]);
}

void Node::callback_imu(const sensor_msgs::msg::Imu::SharedPtr msg, rclcpp::Logger& logger) {
    RCLCPP_DEBUG_ONCE(logger, "[imu] Got first callback.");

    cv::Quat quaternion(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    this->euler_angles = quaternion.toEulerAngles(cv::QuatEnum::INT_YXZ);
}

void Node::callback_depth_info(const zed_interfaces::msg::DepthInfoStamped::SharedPtr msg, rclcpp::Logger& logger) {
    RCLCPP_DEBUG_ONCE(logger, "[depth info] Got first callback.");

    this->range_limits.min = msg->min_depth;
    this->range_limits.max = msg->max_depth;
}

void Node::callback_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg, rclcpp::Logger& logger) {
    RCLCPP_DEBUG_ONCE(logger, "[pointcloud] Got first callback.");

    this->point_cloud = cv::Mat(msg->height, msg->width, CV_32FC4, &msg->data[0], msg->row_step);
}

void Node::callback_depth_image(const sensor_msgs::msg::Image::SharedPtr msg, rclcpp::Logger& logger) {
    RCLCPP_DEBUG_ONCE(logger, "[depth image] Got first callback.");

    filters::encoding_info_t image_encoding = filters::get_encoding_info(msg->encoding);
    RCLCPP_DEBUG_ONCE(logger, "[depth image] cv_type=%s conversion_code=%s", filters::encoding_output_mappings::CV_TYPE_MAPPING[image_encoding.type].c_str(), filters::encoding_output_mappings::CV_COLOR_CONVERSION_MAPPING.at(image_encoding.conversion).c_str());
    this->depth_image = cv::Mat(msg->height, msg->width, image_encoding.type, &(msg->data[0]), msg->step);
    if (image_encoding.conversion != cv::COLOR_COLORCVT_MAX) { //convert to BGR/BGRA if not already so
        cv::cvtColor(this->depth_image, this->depth_image, image_encoding.conversion);
    }
}

void Node::callback_rgb_image(const sensor_msgs::msg::Image::SharedPtr msg, rclcpp::Logger& logger) { //BGRA 8-bit
    if (this->point_cloud.empty()) {
        RCLCPP_DEBUG(logger, "Attempted to get rgb image before pointcloud was set.");
        return;
    } else
    if (this->depth_image.empty()) {
        RCLCPP_DEBUG(logger, "Attempted to get rgb image before depth image was set.");
        return;
    } else if (this->range_limits.min == -1) {
        RCLCPP_DEBUG(logger, "Attempted to get rgb image before depth limits were set.");
        return;
    } else if (this->K.empty()) {
        RCLCPP_DEBUG(logger, "Attempted to get rgb image before camera matrix was set.");
        return;
    } else if (!this->euler_angles.has_value()) {
        RCLCPP_DEBUG(logger, "Attempted to get rgb image before euler angles were set.");
        return;
    }
    RCLCPP_DEBUG_ONCE(logger, "[rgb image] Got first callback.");

    //retrieve image
    filters::encoding_info_t image_encoding = filters::get_encoding_info(msg->encoding);
    RCLCPP_DEBUG_ONCE(logger, "[rgb image] cv_type=%s conversion_code=%s", filters::encoding_output_mappings::CV_TYPE_MAPPING[image_encoding.type].c_str(), filters::encoding_output_mappings::CV_COLOR_CONVERSION_MAPPING.at(image_encoding.conversion).c_str());
    cv::Mat image(msg->height, msg->width, image_encoding.type, &(msg->data[0]), msg->step);
    if (image_encoding.conversion != cv::COLOR_COLORCVT_MAX) { //convert to BGR/BGRA if not already so
        cv::cvtColor(image, image, image_encoding.conversion);
    }

    //sanity check
    cv::Size image_size = image.size();
    RCLCPP_ERROR_EXPRESSION(logger, point_cloud.size() != image_size, "[rgb image] Image size (%dx%d) does not match point cloud dimensions (%dx%d).", image_size.width, image_size.height, this->point_cloud.cols, this->point_cloud.rows);

    //mask rgb image with depth image
    cv::Mat masked_image;
    cv::Mat mask = filters::create_range_mask(this->depth_image, this->range_limits.min, this->range_limits.max, custom_parameters.filter_distance);
    cv::copyTo(image, masked_image, mask);

    //convert camera view to top down
    cv::Mat warped_image, transformation_matrix;
    std::vector<cv::Point2i> trapeze;

    cv::Vec3d euler_angles = this->euler_angles.value();
    cv::Point2d vanishing_point = wayfinding::top_down::get_vanishing_point(this->K, euler_angles[0] - M_PI, euler_angles[1] + M_PI_2);
    if (!wayfinding::top_down::get_transformation(transformation_matrix, trapeze, image_size, vanishing_point)) {
        RCLCPP_WARN(logger, "Could not find transformation matrix.");
        return;
    }
    wayfinding::top_down::transform_to_top_down(masked_image, warped_image, transformation_matrix, image_size);

    //convert pointcloud the same way as the image to line up the values
    cv::Mat warped_point_cloud;
    wayfinding::top_down::transform_to_top_down(this->point_cloud, warped_point_cloud, transformation_matrix, image_size);


    //TODO: das scheint zu funktionieren, aber ob die Ergebnisse "korrekt" sind ist noch zu prüfen
    cv::Vec3d point{(double)image_size.width / 2, (double)image_size.height / 4, 1.0};
    cv::Mat warped_point = transformation_matrix * point;

    double denominator = warped_point.at<double>(2, 0);
    cv::Point2i normal_img_point(point[0], point[1]),
                warped_img_point(warped_point.at<double>(0, 0) / denominator, warped_point.at<double>(1, 0) / denominator);
    // std::cout << "normal point: " << point << '\n'
    //           << "warped point: " << warped_point << '\n'
    //           << "normal img point: " << normal_img_point << '\n'
    //           << "warped img point: " << warped_img_point << std::endl;
    cv::circle(image, normal_img_point, 3, cv::Scalar(255, 0, 0), 3);
    cv::circle(warped_image, warped_img_point, 3, cv::Scalar(255, 0, 0), 3);

    cv::Vec4f normal_result = this->point_cloud.at<cv::Vec4f>(normal_img_point);
    //cv::Vec4f warped_result = 
    float* values = warped_point_cloud.at<cv::Vec4f>(warped_img_point).val; //TODO: WHAT THE FUUUUUUUCK
    for (size_t i = 0; i < 4; i++) {
        //segfault
        std::cout << values[i] << ", ";
    }
    std::endl(std::cout);
    // std::cout << "normal coordinates: " << normal_result << '\n'
    //           << "warped coordinates: " << warped_result << std::endl;

    //show images if logging verbosity is set to debug and no debugger like gdb is running the program
    #ifdef DEBUG
    if (!this->is_run_in_debugger) { //debuggers sometimes can not foreward this call (e.g. gdb)
        wayfinding::top_down::draw_vanishing_lines(image, vanishing_point, trapeze);

        cv::imshow("image", image);
        //cv::imshow("masked", masked_image);
        cv::imshow("warped", warped_image);

        if (cv::waitKey(10) == 27) { //ESC
            RCLCPP_DEBUG(logger, "Exiting node upon user request.");
            exit(0);
        }
    }
    #endif //DEBUG

    // cv::Vec4f result = this->point_cloud->at<cv::Vec4f>(this->image_size.width / 2, this->image_size.height / 2);
    // std::cout << "IMG: " << result << std::endl;
}