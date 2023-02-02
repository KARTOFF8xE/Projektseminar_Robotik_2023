#include "node.hpp"

#include "filters/filters.hpp"
#include "wayfinding/wayfinding.hpp"

#include "rclcpp/qos.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/point_field.hpp"

#include "opencv2/core/quaternion.hpp"

#include <chrono>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <thread>
//#include <unistd.h>

using namespace std::chrono_literals;
using namespace std::placeholders;


#ifdef DEBUG
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
#endif //DEBUG


Node::Node(): rclcpp::Node("wayfinding") {
    this->declare_parameter<std::string>("topics.flux.camera_info",     "/zed2i_c1/zed_node_c1/rgb/camera_info");
    this->declare_parameter<std::string>("topics.flux.imu",             "/imu/data");
    this->declare_parameter<std::string>("topics.flux.pointcloud",      "/zed2i_c1/zed_node_c1/point_cloud/cloud_registered");
    this->declare_parameter<std::string>("topics.flux.rgb_image",       "/zed2i_c1/zed_node_c1/rgb/image_rect_color");
    this->declare_parameter<std::string>("topics.pub",                  "/camera_path_width");
    this->declare_parameter<double>("params.angle_filter",              75.0);  //minimal angle for a line to not be cast out [°]
    this->declare_parameter<double>("params.optimal_line_angle",        90.0);  //angle that gives the best (lowest) metric values [°]
    this->declare_parameter<double>("params.rel_scan_line_height",      .33);   //relative height (from bottom) on top down image at which to scan for distances
    this->declare_parameter<std::string>("debug.metric_output_file",   "metric_table");

    this->get_parameter("topics.flux.camera_info",      custom_parameters.camera_info_topic);
    this->get_parameter("topics.flux.imu",              custom_parameters.imu_topic);
    this->get_parameter("topics.flux.pointcloud",       custom_parameters.pointcloud_topic);
    this->get_parameter("topics.flux.rgb_image",        custom_parameters.rgb_image_topic);
    this->get_parameter("topics.pub",                   custom_parameters.pub_topic);
    this->get_parameter("params.angle_filter",          custom_parameters.angle_filter);
    this->get_parameter("params.optimal_line_angle",    custom_parameters.optimal_line_angle);
    this->get_parameter("params.rel_scan_line_height",  custom_parameters.relative_scan_line_height);
    std::string csv_file_name = this->get_parameter("debug.metric_output_file").as_string() + ".csv";


    //setup misc
    rclcpp::Logger logger = this->get_logger();

    #ifdef DEBUG
    //check if process is run in gdb
    this->is_run_in_debugger = isRunByDebugger();
    
    //check if metric tracking file is already initialized, meaning it exists and has a header
    bool is_init;
    {
        std::ifstream csv_file_check;
        csv_file_check.open(csv_file_name, std::ios::in);
        if (!csv_file_check.is_open()) {
            //if it cannot be opened to read it is likely not existend, ergo not initialized
            is_init = false;
        } else {
            //read from file to check if it is empty, ergo not initialized
            std::string content_check;
            csv_file_check >> content_check;
            is_init = (content_check.size() != 0);
        }
        csv_file_check.close();
    }

    this->csv_file.open(csv_file_name, std::ios::out | std::ios::app);
    if (!is_init) { //add the header if the file didn't have one
        csv_file << "Zeitstempel;Metrik;gesamte Linien;gez. Linien";
    }
    #endif //DEBUG


    //sanity check to analyze problems faster
    RCLCPP_ERROR_EXPRESSION(logger, custom_parameters.camera_info_topic == "", "Empty camera info topic.");
    RCLCPP_ERROR_EXPRESSION(logger, custom_parameters.imu_topic         == "", "Empty imu topic.");
    RCLCPP_ERROR_EXPRESSION(logger, custom_parameters.pointcloud_topic  == "", "Empty pointcloud topic.");
    RCLCPP_ERROR_EXPRESSION(logger, custom_parameters.rgb_image_topic   == "", "Empty rgb image topic.");
    RCLCPP_ERROR_EXPRESSION(logger, custom_parameters.pub_topic         == "", "Empty publishing topic.");

    //setup publishers/subscribers
    this->camera_info_sub   = this->create_subscription<sensor_msgs::msg::CameraInfo>(custom_parameters.camera_info_topic,          rclcpp::SensorDataQoS(), MAKE_SINGLE_ARGUMENT(sensor_msgs::msg::CameraInfo, std::bind(&Node::callback_camera_info, this, _1, logger)));
    this->imu_sub           = this->create_subscription<sensor_msgs::msg::Imu>(custom_parameters.imu_topic,                         rclcpp::SensorDataQoS(), MAKE_SINGLE_ARGUMENT(sensor_msgs::msg::Imu, std::bind(&Node::callback_imu, this, _1, logger)));
    this->pointcloud2_sub   = this->create_subscription<sensor_msgs::msg::PointCloud2>(custom_parameters.pointcloud_topic,          rclcpp::SensorDataQoS(), MAKE_SINGLE_ARGUMENT(sensor_msgs::msg::PointCloud2, std::bind(&Node::callback_pointcloud, this, _1, logger)));
    this->rgb_image_sub     = this->create_subscription<sensor_msgs::msg::Image>(custom_parameters.rgb_image_topic,                 rclcpp::SensorDataQoS(), MAKE_SINGLE_ARGUMENT(sensor_msgs::msg::Image, std::bind(&Node::callback_rgb_image, this, _1, logger)));
    this->publisher         = this->create_publisher<custom_msgs::msg::Distance>(custom_parameters.pub_topic,                       rclcpp::SystemDefaultsQoS());
}

#ifdef DEBUG
Node::~Node() {
    this->csv_file.close();
}
#endif //DEBUG

void Node::callback_camera_info(const sensor_msgs::msg::CameraInfo::SharedPtr msg, rclcpp::Logger& logger) {
    RCLCPP_DEBUG_ONCE(logger, "[camera info] Got first callback.");

    this->K = msg->k;
}

void Node::callback_imu(const sensor_msgs::msg::Imu::SharedPtr msg, rclcpp::Logger& logger) {
    RCLCPP_DEBUG_ONCE(logger, "[imu] Got first callback.");

    cv::Quat quaternion(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    this->euler_angles = quaternion.toEulerAngles(cv::QuatEnum::INT_YXZ);
}

void Node::callback_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg, rclcpp::Logger& logger) {
    RCLCPP_DEBUG_ONCE(logger, "[pointcloud] Got first callback.");

    cv::Mat point_cloud = cv::Mat(msg->height, msg->width, CV_32FC4, &msg->data[0], msg->row_step);

    //remove fourth (rgba) channel from this pointcloud
    std::vector<cv::Mat> channels(4);
    cv::split(point_cloud, channels);
    channels.erase(channels.end());
    cv::merge(channels, this->point_cloud);
}

void Node::callback_rgb_image(const sensor_msgs::msg::Image::SharedPtr msg, rclcpp::Logger& logger) { //BGRA 8-bit
    if (this->point_cloud.empty()) {
        RCLCPP_DEBUG(logger, "Attempted to get rgb image before pointcloud was set.");
        return;
    } else if (this->K.empty()) {
        RCLCPP_DEBUG(logger, "Attempted to get rgb image before camera matrix was set.");
        return;
    } else if (!this->euler_angles.has_value()) {
        RCLCPP_DEBUG(logger, "Attempted to get rgb image before euler angles were set.");
        return;
    }
    RCLCPP_INFO_ONCE(logger, "[rgb image] Got first callback.");


    //retrieve image
    filters::encoding_info_t image_encoding = filters::get_encoding_info(msg->encoding);
    RCLCPP_DEBUG_ONCE(logger, "[rgb image] cv_type=%s conversion_code=%s", filters::encoding_output_mappings::CV_TYPE_MAPPING[image_encoding.type].c_str(), filters::encoding_output_mappings::CV_COLOR_CONVERSION_MAPPING.at(image_encoding.conversion).c_str());
    cv::Mat image(msg->height, msg->width, image_encoding.type, &(msg->data[0]), msg->step);
    if (image_encoding.conversion != cv::COLOR_COLORCVT_MAX) { //convert to BGR/BGRA if not already so
        cv::cvtColor(image, image, image_encoding.conversion);
    }


    //convert camera view to top down
    cv::Mat warped_image, transformation_matrix;
    std::vector<cv::Point2i> trapeze;
    
    cv::Mat K(3, 3, CV_64FC1, &this->K[0]);
    cv::Size image_size = image.size();
    cv::Vec3d euler_angles = this->euler_angles.value();

    cv::Point2d vanishing_point = wayfinding::top_down::getVanishingPoint(K, euler_angles[0] - M_PI, euler_angles[1] + M_PI_2);
    if (!wayfinding::top_down::getTransformation(transformation_matrix, trapeze, image_size, vanishing_point)) {
        RCLCPP_WARN(logger, "Could not find transformation matrix.");
        return;
    }
    wayfinding::top_down::transformToTopDown(image, warped_image, transformation_matrix, image_size);


    //get hough lines on standard parameters
    wayfinding::line_detection::parameters_t default_algorithm_parameters;
    std::vector<cv::Vec4i> hough_lines;
    wayfinding::line_detection::getHoughLines(warped_image, hough_lines, default_algorithm_parameters);

    //calculate metric from standard parameter lines
    double metric = wayfinding::line_detection::imageMetric(hough_lines, image_size, custom_parameters.optimal_line_angle);
    RCLCPP_INFO_STREAM(logger, "[rgb image] Metrik: " << metric);

    //get hough lines on metric dependent parameter set
    wayfinding::line_detection::parameters_t algorithm_parameters = wayfinding::line_detection::getParametersFromMetric(metric);
    std::vector<cv::Vec4i> hough_lines_2; //TODO: get a better name
    cv::Mat canny = wayfinding::line_detection::getHoughLines(warped_image, hough_lines_2, algorithm_parameters);
    cv::cvtColor(canny, canny, cv::COLOR_GRAY2BGR);

    //filter lines with tube and angle filter to exclude unnecessary lines
    std::vector<cv::Vec4i> filtered_lines;
    wayfinding::line_detection::lineFilter(hough_lines_2, filtered_lines, image_size.width, default_algorithm_parameters.center_width, custom_parameters.angle_filter);


    //get left and right distance coordinates
    int left, right;
    int scan_line_height = (1 - custom_parameters.relative_scan_line_height) * image_size.height;
    std::tie(left, right) = wayfinding::line_detection::getLeftRightDistance(filtered_lines, scan_line_height, image_size.width);
    
    //create, populate and publish message
    custom_msgs::msg::Distance result_msg;
    result_msg.header.set__stamp(msg->header.stamp);
    if (left != -1 || right != -1) {
        cv::Point2i center_point_2d = wayfinding::top_down::unwarpPoint(transformation_matrix, cv::Point2i(image_size.width / 2, scan_line_height));
        cv::Vec3f center_point_3d = this->point_cloud.at<cv::Vec3f>(center_point_2d);

        if (left != -1) {
            cv::Point2i left_point_2d = wayfinding::top_down::unwarpPoint(transformation_matrix, cv::Point2i(left, scan_line_height));
            cv::Vec3f left_point_3d = this->point_cloud.at<cv::Vec3f>(left_point_2d);

            result_msg.left = cv::norm(left_point_3d - center_point_3d);
        } else {
            result_msg.left = -1;
        }
        if (right != -1) {
            cv::Point2i right_point_2d = wayfinding::top_down::unwarpPoint(transformation_matrix, cv::Point2i(right, scan_line_height));
            cv::Vec3f right_point_3d = this->point_cloud.at<cv::Vec3f>(right_point_2d);

            result_msg.right = cv::norm(right_point_3d - center_point_3d);
        } else {
            result_msg.right = -1;
        }
    } else {
        result_msg.left = -1;
        result_msg.right = -1;
    }

    this->publisher->publish(result_msg);


    #ifdef DEBUG
    //write csv line with metric and line counts for current image
    this->csv_file << '\n' << rclcpp::Time(msg->header.stamp.sec).nanoseconds() << ';' << metric << ';' << hough_lines.size() << ';' << filtered_lines.size();

    //show images if logging verbosity is set to debug and no debugger (e.g. gdb) is running this program
    if (!this->is_run_in_debugger) { //debuggers sometimes can not foreward this call (e.g. gdb)
        //draw indicator lines
        wayfinding::top_down::drawVanishingLines(image, vanishing_point, trapeze);
        wayfinding::line_detection::drawLines(canny, hough_lines_2, cv::Scalar(0, 0, 255));
        wayfinding::line_detection::drawLines(warped_image, filtered_lines, cv::Scalar(255, 0, 0));

        //draw tube filter lines
        const double rel_threshold = (100.0 - default_algorithm_parameters.center_width) / 2;
        const int left_threshold = image_size.width * (rel_threshold / 100.0),
                right_threshold = image_size.width * (1 - rel_threshold / 100.0);
        cv::line(warped_image, cv::Point2i(left_threshold, 0), cv::Point2i(left_threshold, image_size.height), cv::Scalar(0, 255, 0));
        cv::line(warped_image, cv::Point2i(right_threshold, 0), cv::Point2i(right_threshold, image_size.height), cv::Scalar(0, 255, 0));

        //draw scan line
        cv::line(warped_image, cv::Point2i(0, scan_line_height), cv::Point2i(image_size.width, scan_line_height), cv::Scalar(0, 255, 0));

        //rescale images for better screen usage and visibility
        cv::Mat scaled_image, scaled_canny, scaled_warped_image;
        cv::resize(image, scaled_image, cv::Size(), .6, .6);
        cv::resize(canny, scaled_canny, cv::Size(), .6, .6);
        cv::resize(warped_image, scaled_warped_image, cv::Size(), .6, .6);

        //actually display images
        cv::imshow("image", scaled_image);
        cv::imshow("warped", scaled_warped_image);
        cv::imshow("canny", scaled_canny);

        //wait 10ms for each frame to allow for keyboard inputs, etc.
        if (cv::waitKey(10) == 27) { //ESC
            RCLCPP_DEBUG(logger, "Exiting node upon user request.");
            rclcpp::shutdown();
        }
    }
    #endif //DEBUG
}