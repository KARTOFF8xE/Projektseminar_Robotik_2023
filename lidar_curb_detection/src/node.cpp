#include "node.hpp"

#include "lidar_curb_detection.hpp"
#include "filters/post_filters.hpp"

#include "rclcpp/qos.hpp"

#include <cmath>
#include <fstream>

using std::placeholders::_1;

Node::Node(bool do_visualize): rclcpp::Node("lidar_curb_detection") {
    this->declare_parameter<std::string>("topics.dmc.laserscan",            "/sick/scan");
    this->declare_parameter<std::string>("topics.pub",                      "/lidar_path_width");
    this->declare_parameter<double> ("robot_specific.wheel_inside",         .2854);             // Distance of the vertical Plane in the center of the Robot to the vertical inside Plane of the Wheels
    this->declare_parameter<double> ("robot_specific.wheel_width",          .1143);             // Width of the Wheels
    this->declare_parameter<double> ("robot_specific.mounting_angle",       M_PI / 6.0);        // Default Tilt of the LiDAR
    this->declare_parameter<int>    ("detection_thr.angle_thr",             4);                 // Minimum angle between two Vectors that is needed to detect a Curbstone
    this->declare_parameter<double> ("detection_thr.height_diff",           .05);               // Height_Difference that must exist, to detect a Curbstone
    this->declare_parameter<double> ("detection_thr.advanced_ray_check_thr",.2);                // Distance a possible Curbstone should be proofed, that it isn't a Pothole
    this->declare_parameter<double> ("bubble.distance_thr",                 .4);                // Threshold that tells the filter, how far another Point is allowed to be away to be valid
    this->declare_parameter<int>    ("bubble.quantity_check",               14);                // Amount of Limits that is checked in each direction (before and behind)
    this->declare_parameter<int>    ("bubble.quantity_thr",                 15);                // Minimum amount of valid Points that needs to lay inside the distance_thr
    this->declare_parameter<double> ("avg_dist.avg_dist_thr",               .2);                // distance-averages below this Threshold are validating the Limit
    this->declare_parameter<int>    ("avg_dist.quantity_check",             15);                // Amount of Limits that is checked in each direction (before and behind)
    this->declare_parameter<int>    ("avg_dist.counter_thr",                12);                // Minimum amount of taken distance-differences that is needed for the average-calculation, otherwise the limit is not valid
    this->declare_parameter<int>    ("island.quantity_check",               45);                // Amount of Limits that is checked in each direction (before and behind)
    this->declare_parameter<int>    ("island.counter_thr",                  40);                // Needed Quantity of Valid Limits to validate the looked up Limit
    
    
    this->get_parameter("topics.dmc.laserscan",                 custom_parameters.sub_topic);
    this->get_parameter("topics.pub",                           custom_parameters.pub_topic);
    this->get_parameter("robot_specific.wheel_inside",          custom_parameters.wheel_inside);
    this->get_parameter("robot_specific.wheel_width",           custom_parameters.wheel_width);
    this->get_parameter("robot_specific.mounting_angle",        custom_parameters.mounting_angle);
    this->get_parameter("bubble.distance_thr",                  custom_parameters.distance_thr);
    this->get_parameter("bubble.quantity_check",                custom_parameters.quantity_check_for_runaways);
    this->get_parameter("bubble.quantity_thr",                  custom_parameters.quantity_thr);
    this->get_parameter("detection_thr.angle_thr",              custom_parameters.angle_thr);
    this->get_parameter("detection_thr.height_diff",            custom_parameters.height_diff);
    this->get_parameter("detection_thr.advanced_ray_check_thr", custom_parameters.advanced_ray_check_thr);
    this->get_parameter("avg_dist.quantity_check",              custom_parameters.quantity_check_for_avg_dist);
    this->get_parameter("avg_dist.counter_thr",                 custom_parameters.counter_thr_for_avg);
    this->get_parameter("avg_dist.avg_dist_thr",                custom_parameters.avg_dist_thr);
    this->get_parameter("island.quantity_check",                custom_parameters.quantity_check_for_island);
    this->get_parameter("island.counter_thr",                   custom_parameters.counter_thr_for_island);

    this->do_visualize = do_visualize;

    this->sub = this->create_subscription<sensor_msgs::msg::LaserScan>(custom_parameters.sub_topic, rclcpp::SensorDataQoS(), std::bind(&Node::callback, this, _1));
    this->pub = this->create_publisher<custom_msgs::msg::Distance>(custom_parameters.pub_topic, rclcpp::SystemDefaultsQoS());
}

void Node::callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    rclcpp::Logger logger = this->get_logger();

    /*** Create the Height_Line out of the LiDAR-Data ***/
    std::vector<lidar_curb_det::lidar_measures> height_line = lidar_curb_det::get_height_line(msg->ranges, msg->angle_increment, msg->range_min, msg->range_max, msg->angle_min, msg->angle_max, custom_parameters.mounting_angle); // TODO: M_PI / 6.0 input kam immer 0 raus @johannes
    
    /*** Sort the Height_Line ***/
    height_line = lidar_curb_det::sort_height_line(height_line);
    
    /*** Calculate Limits (if possible) with the help of the Vectors and add those Limits to a List of Limits ***/
    filters::limit limit = lidar_curb_det::curbstone_checker_vectors(height_line, custom_parameters.height_diff, custom_parameters.angle_thr, custom_parameters.advanced_ray_check_thr, custom_parameters.wheel_inside, msg->header.stamp);
    this->limits_vec.push_back(limit);

    /*** Filter for to high deviations ***/
    if (limits_vec.size() > 2 * custom_parameters.quantity_check_for_runaways + 1) {
        size_t i = limits_vec.size() - (custom_parameters.quantity_check_for_runaways + 1);
        limits_vec[i] = filters::noise_filter(limits_vec, custom_parameters.quantity_check_for_runaways, custom_parameters.counter_thr_for_avg, custom_parameters.avg_dist_thr, i);
        // limit.avg_dist_left = limits_vec[i].avg_dist_left;
        // limit.avg_dist_right = limits_vec[i].avg_dist_right;
    }

    /*** Filter for runaways ***/
    if (limits_vec.size() > ((2 * custom_parameters.quantity_check_for_runaways + 1) + (2 * custom_parameters.quantity_check_for_avg_dist + 1))) {
        size_t i = limits_vec.size() - ((custom_parameters.quantity_check_for_runaways + 1) + (2 * custom_parameters.quantity_check_for_avg_dist + 1));
        limits_vec[i] = filters::filter_for_runaways(limits_vec, custom_parameters.distance_thr, custom_parameters.quantity_check_for_runaways, custom_parameters.quantity_thr, i);

    } 

    /*** Filter for Islands ***/
    if (limits_vec.size() > ((2 * custom_parameters.quantity_check_for_runaways + 1) + (2 * custom_parameters.quantity_check_for_avg_dist + 1) + (2 * custom_parameters.quantity_check_for_island + 1))) {
        size_t i = limits_vec.size() - ((custom_parameters.quantity_check_for_island + 1) + (2 * custom_parameters.quantity_check_for_runaways + 1) + (2 * custom_parameters.quantity_check_for_avg_dist + 1));
        limits_vec[i] = filters::check_for_valid_island(limits_vec, custom_parameters.quantity_check_for_island, custom_parameters.counter_thr_for_island, i);
    }

    /*** Visualize anything, if wanted ***/
    if (this->do_visualize) {
         lidar_curb_det::visualize_cross_section(height_line, limit.left, limit.right, custom_parameters.wheel_inside, custom_parameters.wheel_width);
    //    lidar_curb_det::visualize_street_view(limits_vec, custom_parameters.wheel_inside, custom_parameters.wheel_width);
    }

    /*** Written Output (filtered) ***/
    u_int min_quantity_of_values = (2 * custom_parameters.quantity_check_for_runaways + 1) + (2 * custom_parameters.quantity_check_for_avg_dist + 1) + (2 * custom_parameters.quantity_check_for_island + 1);
    if (limits_vec.size() > (min_quantity_of_values)) {
        size_t i = limits_vec.size() - ((custom_parameters.quantity_check_for_island + 1) + (2 * custom_parameters.quantity_check_for_runaways + 1) + (2 * custom_parameters.quantity_check_for_avg_dist + 1));
        // RCLCPP_INFO(logger, "Left: %.5f\tRight: %.5f\tDistance: %.2f\tLeft_avg: %.5f\tRight_avg: %.5f", limits_vec[i].left, limits_vec[i].right, dist, limits_vec[i].avg_dist_left, limits_vec[i].avg_dist_right);
        RCLCPP_DEBUG(logger, "Quantity of pushed values: %d", limits_vec.size() - min_quantity_of_values);

        auto pub_msg = custom_msgs::msg::Distance();
        pub_msg.header.stamp = limits_vec[i].timestamp;
        pub_msg.left = limits_vec[i].left;
        pub_msg.right = limits_vec[i].right;
        double width = 0;
        if (limits_vec[i].left > 0 && limits_vec[i].right > 0) {
            width = limits_vec[i].left + limits_vec[i].right;
            pub_msg.width = width;

        }
        pub->publish(pub_msg);

        std::ofstream f;
        f.open("lidar_width_data.csv", std::ios::app);
        f << ((int)limits_vec[i].timestamp.seconds() % 10000) << ", " << limits_vec[i].left << ", " << limits_vec[i].right << ", " <<  width << "\n" ;
        f.close();

    } else {
        RCLCPP_DEBUG(logger, "Not enough values, i only have %d of %d", limits_vec.size(), min_quantity_of_values + 1);
    }
}