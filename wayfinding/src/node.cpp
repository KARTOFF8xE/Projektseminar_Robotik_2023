#include "node.hpp"

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
#include <cmath>
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


Node::Node(bool visualize): rclcpp::Node("wayfinding") {
    this->declare_parameter<std::string>("topics.flux.camera_info",     "/zed2i_c1/zed_node_c1/rgb/camera_info");
    this->declare_parameter<std::string>("topics.flux.imu",             "/imu/data");
    this->declare_parameter<std::string>("topics.flux.pointcloud",      "/zed2i_c1/zed_node_c1/point_cloud/cloud_registered");
    this->declare_parameter<std::string>("topics.flux.rgb_image",       "/zed2i_c1/zed_node_c1/rgb/image_rect_color");
    this->declare_parameter<std::string>("topics.pub",                  "/camera_path_width");
    this->declare_parameter<double>("params.angle_filter",              75.0);  // minimal angle for a line to not be cast out [°]
    this->declare_parameter<double>("params.optimal_line_angle",        90.0);  // angle that gives the best (lowest) metric values [°]
    this->declare_parameter<double>("params.rel_scan_line_height",      0.05);  // relative height (from bottom) on top down image at which to scan for distances
    this->declare_parameter<double>("params.filter.distance_thr",       0.2);   // filter: Threshold that tells the filter, how far another Point is allowed to be away to be valid
    this->declare_parameter<int>   ("params.filter.quantity_check",     14);    // filter: Amount of Limits that is checked in each direction (before and behind)
    this->declare_parameter<int>   ("params.filter.quantity_thr",       15);    // filter: Minimum amount of valid Points that needs to lay inside the distance_thr
    this->declare_parameter<int>   ("params.avg_dist.quantity_check",   15);    // avg_dist: Amount of Limits that is checked in each direction (before and behind)
    this->declare_parameter<int>   ("params.avg_dist.counter_thr",      12);    // avg_dist: Minimum amount of taken distance-differences that is needed for the average-calculation, otherwise the limit is not valid
    this->declare_parameter<double>("params.avg_dist.avg_dist_thr",     0.1);   // avg_dist: distance-averages below this Threshold are validating the Limit
    this->declare_parameter<int>   ("params.island.quantity_check",     45);    // island: Amount of Limits that is checked in each direction (before and behind)
    this->declare_parameter<int>   ("params.island.counter_thr",        40);    // island: Needed Quantity of Valid Limits to validate the looked up Limit
    this->declare_parameter<std::string>("debug.metric_output_file",    "metric_table");

    this->get_parameter("topics.flux.camera_info",      custom_parameters.camera_info_topic);
    this->get_parameter("topics.flux.imu",              custom_parameters.imu_topic);
    this->get_parameter("topics.flux.pointcloud",       custom_parameters.pointcloud_topic);
    this->get_parameter("topics.flux.rgb_image",        custom_parameters.rgb_image_topic);
    this->get_parameter("topics.pub",                   custom_parameters.pub_topic);
    this->get_parameter("params.angle_filter",          custom_parameters.angle_filter);
    this->get_parameter("params.optimal_line_angle",    custom_parameters.optimal_line_angle);
    this->get_parameter("params.rel_scan_line_height",  custom_parameters.relative_scan_line_height);
    this->get_parameter("filter.distance_thr",          custom_parameters.distance_thr);
    this->get_parameter("filter.quantity_check",        custom_parameters.quantity_check_for_runaways);
    this->get_parameter("filter.quantity_thr",          custom_parameters.quantity_thr);
    this->get_parameter("avg_dist.quantity_check",      custom_parameters.quantity_check_for_avg_dist);
    this->get_parameter("avg_dist.counter_thr",         custom_parameters.counter_thr_for_avg);
    this->get_parameter("avg_dist.avg_dist_thr",        custom_parameters.avg_dist_thr);
    this->get_parameter("island.quantity_check",        custom_parameters.quantity_check_for_island);
    this->get_parameter("island.counter_thr",           custom_parameters.counter_thr_for_island);
    std::string csv_file_name = this->get_parameter("debug.metric_output_file").as_string() + ".csv";


    //setup misc
    rclcpp::Logger logger = this->get_logger();

    this->is_run_in_debugger = isRunByDebugger();
    this->do_visualize = visualize;

    this->high_deviations_min_buffer_size = 2 * custom_parameters.quantity_check_for_runaways + 1;
    this->runaways_min_buffer_size = this->high_deviations_min_buffer_size + (2 * custom_parameters.quantity_check_for_avg_dist + 1);
    this->islands_min_buffer_size = this->runaways_min_buffer_size + (2 * custom_parameters.quantity_check_for_island + 1);

    
    #ifdef DEBUG
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
    this->camera_info_sub   = this->create_subscription<sensor_msgs::msg::CameraInfo>(custom_parameters.camera_info_topic,  rclcpp::SensorDataQoS(), MAKE_SINGLE_ARGUMENT(sensor_msgs::msg::CameraInfo, std::bind(&Node::callback_camera_info, this, _1, logger)));
    this->imu_sub           = this->create_subscription<sensor_msgs::msg::Imu>(custom_parameters.imu_topic,                 rclcpp::SensorDataQoS(), MAKE_SINGLE_ARGUMENT(sensor_msgs::msg::Imu, std::bind(&Node::callback_imu, this, _1, logger)));
    this->pointcloud2_sub   = this->create_subscription<sensor_msgs::msg::PointCloud2>(custom_parameters.pointcloud_topic,  rclcpp::SensorDataQoS(), MAKE_SINGLE_ARGUMENT(sensor_msgs::msg::PointCloud2, std::bind(&Node::callback_pointcloud, this, _1, logger)));
    this->rgb_image_sub     = this->create_subscription<sensor_msgs::msg::Image>(custom_parameters.rgb_image_topic,         rclcpp::SensorDataQoS(), MAKE_SINGLE_ARGUMENT(sensor_msgs::msg::Image, std::bind(&Node::callback_rgb_image, this, _1, logger)));
    this->publisher         = this->create_publisher<custom_msgs::msg::Distance>(custom_parameters.pub_topic,               rclcpp::SystemDefaultsQoS());
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

    this->pointcloud = cv::Mat(msg->height, msg->width, CV_32FC4, &msg->data[0], msg->row_step);

    //remove fourth (rgba) channel from this pointcloud
    // std::vector<cv::Mat> channels(4);
    // cv::split(pointcloud, channels);
    // channels.erase(channels.end());
    // cv::merge(channels, this->pointcloud);
}

void Node::callback_rgb_image(const sensor_msgs::msg::Image::SharedPtr msg, rclcpp::Logger& logger) { //BGRA 8-bit
    if (this->pointcloud.empty()) {
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
    
    cv::Mat K = cv::Mat(this->K).reshape(1, 3);
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
    std::vector<cv::Vec4i> general_hough_lines;
    wayfinding::line_detection::getHoughLines(warped_image, general_hough_lines, default_algorithm_parameters);

    //calculate metric from standard parameter lines
    double metric = wayfinding::line_detection::imageMetric(general_hough_lines, image_size, custom_parameters.optimal_line_angle);
    RCLCPP_DEBUG_STREAM(logger, "[rgb image] metric: " << metric);

    //get hough lines on metric dependent parameter set
    wayfinding::line_detection::parameters_t algorithm_parameters = wayfinding::line_detection::getParametersFromMetric(metric);
    std::vector<cv::Vec4i> specific_hough_lines;
    cv::Mat canny = wayfinding::line_detection::getHoughLines(warped_image, specific_hough_lines, algorithm_parameters);
    cv::cvtColor(canny, canny, cv::COLOR_GRAY2BGR);

    //filter lines with tube and angle filter to exclude unnecessary lines
    std::vector<cv::Vec4i> filtered_lines;
    wayfinding::line_detection::lineFilter(specific_hough_lines, filtered_lines, image_size.width, default_algorithm_parameters.center_width, custom_parameters.angle_filter);


    //get left and right distance coordinates
    int left, right;
    int scan_line_height = (1 - custom_parameters.relative_scan_line_height) * image_size.height;
    std::tie(left, right) = wayfinding::line_detection::getLeftRightDistance(filtered_lines, scan_line_height, image_size.width);
    
    //get distances and push to buffer
    filters::limit current_limit;
    current_limit.timestamp = msg->header.stamp;
    if (left != -1 || right != -1) {
        cv::Point2i center_point_2d = wayfinding::top_down::unwarpPoint(transformation_matrix, cv::Point2i(image_size.width / 2, scan_line_height));
        cv::Vec3f center_point_3d = wayfinding::top_down::fetchPointFromPointcloud(pointcloud, center_point_2d);

        cv::Vec3f left_point_3d, right_point_3d;
        if (left != -1) {
            cv::Point2i left_point_2d = wayfinding::top_down::unwarpPoint(transformation_matrix, cv::Point2i(left, scan_line_height));
            left_point_3d = wayfinding::top_down::fetchPointFromPointcloud(pointcloud, left_point_2d);

            // current_limit.left = cv::norm(left_point_2d - center_point_2d) / 1000.0 + 1;
            double limit = cv::norm(left_point_3d - center_point_3d);
            if (!std::isnan(limit) && !std::isinf(limit)) {
                current_limit.left = limit;
            }
        }
        if (right != -1) {
            cv::Point2i right_point_2d = wayfinding::top_down::unwarpPoint(transformation_matrix, cv::Point2i(right, scan_line_height));
            right_point_3d = wayfinding::top_down::fetchPointFromPointcloud(pointcloud, right_point_2d);

            // current_limit.right = cv::norm(right_point_2d - center_point_2d) / 1000.0 + 1;
            double limit = cv::norm(right_point_3d - center_point_3d);
            if (!std::isnan(limit) && !std::isinf(limit)) {
                current_limit.right = limit;
            }
        }

        /*
        if (left != -1 && right != -1) {
            cv::Point2i left_pt     = cv::Point2i(left, scan_line_height),
                        right_pt    = cv::Point2i(right, scan_line_height);
            double distance = cv::norm(left_point_3d - right_point_3d) / 1e38;
            RCLCPP_INFO_STREAM(logger, "[rgb image] distance (from: " << left_pt << " to: " << right_pt << "): " << distance);

            cv::Mat image_copy(image.size(), CV_8UC3);
            image.copyTo(image_copy);

            cv::line(image_copy, left_pt, right_pt, this->blue, 2);
            cv::circle(image_copy, left_pt, 3, this->red, 2);
            cv::circle(image_copy, right_pt, 3, this->green, 2);

            cv::imwrite("frame_" + std::to_string(rclcpp::Time(msg->header.stamp).nanoseconds()) + "_" + std::to_string(distance) + ".jpg", image_copy);
        }
        */
    }
    limits_buffer.push_back(current_limit);
    size_t buffer_size = limits_buffer.size();

    //filter for too high deviations
    if (buffer_size > this->high_deviations_min_buffer_size) {
        size_t i = buffer_size - (this->high_deviations_min_buffer_size - custom_parameters.quantity_check_for_runaways);
        limits_buffer[i] = filters::noise_filter(limits_buffer, custom_parameters.quantity_check_for_runaways, custom_parameters.counter_thr_for_avg, custom_parameters.avg_dist_thr, i);
    }

    //filter for runaways
    if (buffer_size > this->runaways_min_buffer_size) {
        size_t i = buffer_size - (this->runaways_min_buffer_size - custom_parameters.quantity_check_for_runaways);
        limits_buffer[i] = filters::filter_for_runaways(limits_buffer, custom_parameters.distance_thr, custom_parameters.quantity_check_for_runaways, custom_parameters.quantity_thr, i);
    } 

    //filter for islands
    // if (buffer_size > this->islands_min_buffer_size) {
    //     size_t i = buffer_size - (this->islands_min_buffer_size - custom_parameters.quantity_check_for_runaways);
    //     limits_buffer[i] = filters::check_for_valid_island(limits_buffer, custom_parameters.quantity_check_for_island, custom_parameters.counter_thr_for_island, i);
    // }

    if (limits_buffer.size() > this->islands_min_buffer_size) {
        size_t i = buffer_size - (this->islands_min_buffer_size - custom_parameters.quantity_check_for_runaways);

        //create, populate and publish message
        custom_msgs::msg::Distance result_msg;
        result_msg.header.set__stamp(limits_buffer[i].timestamp);
        result_msg.set__left(limits_buffer[i].left);
        result_msg.set__right(limits_buffer[i].right);
        this->publisher->publish(result_msg);
    } else {
        RCLCPP_DEBUG_STREAM(logger, "[rgb image] Not enough values yet. Still needing " << this->islands_min_buffer_size + 1 - buffer_size << " elements.");
    }


    #ifdef DEBUG
    //write csv line with metric and line counts for current image
    this->csv_file << '\n' << rclcpp::Time(msg->header.stamp.sec).nanoseconds() << ';' << metric << ';' << general_hough_lines.size() << ';' << filtered_lines.size();
    #endif //DEBUG

    //show images if logging verbosity is set to debug and no debugger (e.g. gdb) is running this program
    if (this->do_visualize && !this->is_run_in_debugger) { //debuggers sometimes can not foreward this call (e.g. gdb)
        //draw indicator lines
        wayfinding::top_down::drawVanishingLines(image, vanishing_point, trapeze);
        wayfinding::line_detection::drawLines(canny, specific_hough_lines, this->red);
        wayfinding::line_detection::drawLines(warped_image, filtered_lines, this->blue);

        //draw tube filter lines
        const double rel_threshold = (100.0 - default_algorithm_parameters.center_width) / 2;
        const int left_threshold = image_size.width * (rel_threshold / 100.0),
                right_threshold = image_size.width * (1 - rel_threshold / 100.0);
        cv::line(warped_image, cv::Point2i(left_threshold, 0), cv::Point2i(left_threshold, image_size.height), this->green);
        cv::line(warped_image, cv::Point2i(right_threshold, 0), cv::Point2i(right_threshold, image_size.height), this->green);

        //draw scan line
        cv::line(warped_image, cv::Point2i(0, scan_line_height), cv::Point2i(image_size.width, scan_line_height), this->green);

        //draw detected points
        if (left != -1 || right != -1) {
            cv::Point2i left_warped, left_normal,
                        right_warped, right_normal;

            if (left != -1) {
                left_warped = cv::Point2i(left, scan_line_height);
                left_normal = wayfinding::top_down::unwarpPoint(transformation_matrix, left_warped);
                
                cv::circle(image, left_normal, 3, this->red, 2);
                cv::circle(warped_image, left_warped, 3, this->red, 2);
            }
            if (right != -1) {
                right_warped = cv::Point2i(right, scan_line_height);
                right_normal = wayfinding::top_down::unwarpPoint(transformation_matrix, right_warped);
                
                cv::circle(image, right_normal, 3, this->red, 2);
                cv::circle(warped_image, right_warped, 3, this->red, 2);
            }
            if (left != -1 && right != -1) {
                cv::line(image, left_normal, right_normal, this->red, 2);
                cv::line(warped_image, left_warped, right_warped, this->red, 2);
            }
        }

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
}