#include "node.hpp"
#include "lidar_curb_detection.hpp"
#include "filters/post_filters.hpp"

#include "rclcpp/qos.hpp"

#include <cmath>

using std::placeholders::_1;

Node::Node(bool do_visualize): rclcpp::Node("lidar_curb_detection") {
    this->declare_parameter<std::string>("subscribe_topic", "/sick/scan");
    this->declare_parameter<std::string>("publish_topic", "/lidar_path_width");
    this->declare_parameter<double> ("robot_specific.wheel_inside",         .2854);             // Distance of the vertical Plane in the center of the Robot to the verical inside Plane of the Wheels
    this->declare_parameter<double> ("robot_specific.wheel_width",          .1143);             // Width of the Wheels
    this->declare_parameter<double> ("robot_specific.mounting_angle",       M_PI / 6.0);        // 30° //TODO als mounting_angle kommt immer 0 raus :(, Georg traurig
    this->declare_parameter<double> ("filter.distance_thr",                 .2);                // Filter: Threshold that tells the filter, how far another Point is allowed to be away to be valid
    this->declare_parameter<int>    ("filter.quantity_check",               14);                // Filter: Amount of Limits that is checked in each direction (before and behind)
    this->declare_parameter<int>    ("filter.quantity_thr",                 15);                // Filter: Minimum amount of valid Points that needs to lay inside the distance_thr
    this->declare_parameter<int>    ("detection_thr.angle_thr",             4);                 // Minimum angle between two Vectors that is needed to detect a Curbstone
    this->declare_parameter<double> ("detection_thr.height_diff",           .05);               // Height_Difference that must exist, to detect a Curbstone
    this->declare_parameter<double> ("detection_thr.advanced_ray_check_thr",.2);                // Distance a possible Curbstone should be proofed, that it isn't a Pothole
    this->declare_parameter<int>    ("detection_thr.max_check_length",      50);                // Quantity of Rays that should be checked for a Curbstone
    this->declare_parameter<int>    ("smoothing.quantity_thr",              1);                 // Smoother: The amount of Points (in each direction) that is taken to calculate the average of a new Point
    this->declare_parameter<int>    ("smoothing.repetitions",               0);                 // Smoother: How often the Height_line should be smoothed
    this->declare_parameter<int>    ("avg_dist.quantity_check",             15);                // avg_dist: Amount of Limits that is checked in each direction (before and behind)
    this->declare_parameter<int>    ("avg_dist.counter_thr",                12);                // avg_dist: Minimum amount of taken distance-differences that is needed for the average-calculation, otherwise the limit is not valid
    this->declare_parameter<double> ("avg_dist.avg_dist_thr",               .1);                // avg_dist: distance-averages below this Threshold are validating the Limit
    this->declare_parameter<int>    ("island.quantity_check",               45);                // island: Amount of Limits that is checked in each direction (before and behind)
    this->declare_parameter<int>    ("island.counter_thr",                  40);                // island: Needed Quantity of Valid Limits to validate the looked up Limit
    
    
    this->get_parameter("subscribe_topic",                      custom_parameters.sub_topic);
    this->get_parameter("publish_topic",                        custom_parameters.pub_topic);
    this->get_parameter("robot_specific.wheel_inside",          custom_parameters.wheel_inside);
    this->get_parameter("robot_specific.wheel_width",           custom_parameters.wheel_width);
    this->get_parameter("robot_specific.mounting_point",        custom_parameters.mounting_angle);
    this->get_parameter("filter.distance_thr",                  custom_parameters.distance_thr);
    this->get_parameter("filter.quantity_check",                custom_parameters.quantity_check_for_runaways);
    this->get_parameter("filter.quantity_thr",                  custom_parameters.quantity_thr);
    this->get_parameter("detection_thr.angle_thr",              custom_parameters.angle_thr);
    this->get_parameter("detection_thr.height_diff",            custom_parameters.height_diff);
    this->get_parameter("detection_thr.advanced_ray_check_thr", custom_parameters.advanced_ray_check_thr);
    this->get_parameter("detection_thr.max_check_length",       custom_parameters.max_check_length);
    this->get_parameter("smoothing.quantity_thr",               custom_parameters.quantity_thr_for_smoother);
    this->get_parameter("smoothing.repetitions",                custom_parameters.repetitions);
    this->get_parameter("avg_dist.quantity_check",              custom_parameters.quantity_check_for_avg_dist);
    this->get_parameter("avg_dist.counter_thr",                 custom_parameters.counter_thr);
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
    std::vector<lidar_curb_det::lidar_measures> height_line = lidar_curb_det::get_height_line(msg->ranges, msg->angle_increment, msg->range_min, msg->range_max, msg->angle_min, msg->angle_max, M_PI / 6.0); // TODO: ggf Neigungssensor mit einbeziehen?
    
    /*** Sort the Height_Line ***/
    height_line = lidar_curb_det::sort_height_line(height_line);
    
    /*** Calculate Limits (if possible) with the help of the Vectors and add those Limits to a List of Limits ***/
    filters::limit limit = lidar_curb_det::curbstone_checker_vectors(height_line, custom_parameters.height_diff, custom_parameters.angle_thr, custom_parameters.max_check_length, custom_parameters.advanced_ray_check_thr, custom_parameters.wheel_inside);
    this->limits_vec.push_back(limit);

    /*** Filter for to high deviations ***/
    if (limits_vec.size() > 2 * custom_parameters.quantity_check_for_runaways + 1) {
        size_t i = limits_vec.size() - (custom_parameters.quantity_check_for_runaways + 1);
        limits_vec[i] = filters::get_avg_dist(limits_vec, custom_parameters.quantity_check_for_runaways, custom_parameters.counter_thr, custom_parameters.avg_dist_thr, i);
        // limit.avg_dist_left = limits_vec[i].avg_dist_left;
        // limit.avg_dist_right = limits_vec[i].avg_dist_right;
    }

    /*** Filter for runaways ***/
    if (limits_vec.size() > ((2 * custom_parameters.quantity_check_for_runaways + 1) + (2 * custom_parameters.quantity_check_for_avg_dist + 1))) {
        size_t i = limits_vec.size() - ((custom_parameters.quantity_check_for_runaways + 1) + (2 * custom_parameters.quantity_check_for_avg_dist + 1));
        limits_vec[i] = filters::filter_for_runaways(limits_vec, custom_parameters.distance_thr, custom_parameters.quantity_check_for_runaways, custom_parameters.quantity_thr, i);

    } 

    /*** Work in Progress ***/
    if (limits_vec.size() > ((2 * custom_parameters.quantity_check_for_runaways + 1) + (2 * custom_parameters.quantity_check_for_avg_dist + 1) + (2 * custom_parameters.quantity_check_for_island + 1))) {
        size_t i = limits_vec.size() - ((custom_parameters.quantity_check_for_island + 1) + (2 * custom_parameters.quantity_check_for_runaways + 1) + (2 * custom_parameters.quantity_check_for_avg_dist + 1));
        limits_vec[i] = filters::check_for_valid_island(limits_vec, custom_parameters.quantity_check_for_island, custom_parameters.counter_thr_for_island, i);
    }

    /*** Visualize anything, if wanted ***/
    if (this->do_visualize) {
        lidar_curb_det::visualize_cross_section(height_line, limit.left, limit.right, custom_parameters.wheel_inside, custom_parameters.wheel_width);
        lidar_curb_det::visualize_street_view(limits_vec, custom_parameters.wheel_inside, custom_parameters.wheel_width);
    }

    /*** written Output (filtered) ***/
    u_int min_quantity_of_values = (2 * custom_parameters.quantity_check_for_runaways + 1) + (2 * custom_parameters.quantity_check_for_avg_dist + 1) + (2 * custom_parameters.quantity_check_for_island + 1);
    if (limits_vec.size() > (min_quantity_of_values)) {
        size_t i = limits_vec.size() - ((custom_parameters.quantity_check_for_island + 1) + (2 * custom_parameters.quantity_check_for_runaways + 1) + (2 * custom_parameters.quantity_check_for_avg_dist + 1));
        double dist = (limits_vec[i].left == -1 || limits_vec[i].right == -1) ? -1 : limits_vec[i].left + limits_vec[i].right;
        RCLCPP_INFO(logger, "Left: %.5f\tRight: %.5f\tDistance: %.2f\tLeft_avg: %.5f\tRight_avg: %.5f", limits_vec[i].left, limits_vec[i].right, dist, limits_vec[i].avg_dist_left, limits_vec[i].avg_dist_right);

        auto pub_msg = custom_msgs::msg::Distance();
        // pub_msg.set__header =  // TODO: Header adden
        pub_msg.left = limits_vec[i].left;
        pub_msg.right = limits_vec[i].right;
        pub->publish(pub_msg);
    } else {
        RCLCPP_INFO(logger, "Not enough Values, i only have %d of %d", limits_vec.size(), min_quantity_of_values + 1);
    }
}