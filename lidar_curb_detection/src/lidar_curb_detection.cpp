#include "lidar_curb_detection.hpp"

#include <cmath>
#include <limits>
#include <iostream>
#include <algorithm>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

std::vector<lidar_curb_det::lidar_measures> lidar_curb_det::get_height_line(std::vector<float> ranges, double angle_increment, double min_range, double max_range, double min_angle, double max_angle, double mounting_angle) {    
    std::vector<lidar_curb_det::lidar_measures> normed_ranges;
    int i = 0;  // i is number of measurement
    int j = 1;

    /**
     * All Rays that point to the Sky will get sorted out (if existing), Otherwise the given min_angle will get assigned
     * Working with maths 0° (to the right)
     * For Example: A LiDAR-Sensor with -135° -> 135° will be set to 0° -> 180°
    */
    while (min_angle + i * angle_increment < -M_PI_2) { i++; }
    double start_angle = (min_angle < -M_PI_2) ? 0 : M_PI_2 + min_angle;

    /**
     * Invalid Ranges will be sorted out
     * Valid Ranges will be calculated to a distance and a Range (in relation to the height of the LiDAR-Sensor and the horizonal middle (vertical plane) of the Robot)
    */
    while ((start_angle + j * angle_increment < M_PI) && (start_angle + j * angle_increment < max_angle + M_PI)) {
        double range = ranges[i];
        if (range > min_range && range < max_range) {
            normed_ranges.push_back(lidar_curb_det::lidar_measures{
                range * std::sin(M_PI_2 - (j * angle_increment)),
                range * std::sin(mounting_angle) * std::cos(M_PI_2 - (j * angle_increment))
            });
        }
        j++;
        i++;
    }
    /**
     * Reversing the Data inside the Vector, to start with the tiniest // TODO: könnte man direkt so schreiben, dass es andersrum ist
    */
    std::reverse(normed_ranges.begin(), normed_ranges.end());

    return normed_ranges;
}

std::vector<lidar_curb_det::lidar_measures> lidar_curb_det::sort_height_line(std::vector<lidar_curb_det::lidar_measures> height_line) {
    bool sorted = false;
    
    /**
     * Checking if a Value is higher than the following Value, in this Case it switches the Values
     * Exiting when finished
    */
    while (!sorted) {
        sorted = true;
        for (size_t i = 0; i < height_line.size() - 1; i++) {
            if (height_line[i].distance > height_line[i + 1].distance) {
                lidar_curb_det::lidar_measures tmp = height_line[i];
                height_line[i] = height_line[i + 1];
                height_line[i + 1] = tmp;
                sorted = false;
            }
        }
    }
    

    return height_line;
}

bool lidar_curb_det::curb_still_valid(double height_diff, std::vector<lidar_curb_det::lidar_measures> height_line, double drive_line, size_t j, double advanced_ray_check_thr, int factor) {
    /**
     * Iterating above the next {advanced_ray_check_thr} Rays in Range and check, if the height_difference still exists
     * -> if not: detecting it as Pothole
    */
    double comp_dist = std::abs(height_line[j].distance);
    
    /**
     * If the detected unevenness is more like an obstacle/elevation, we say, that it is impossible to drive above it
    */
    if ((drive_line - height_line[j].height) > (height_diff / 2)) {
        return true;
    }

    /**
     * While we are still in Distance and not out of Values, we Check if the distance-threshold is fullfilled
    */
    while ((std::abs(std::abs(height_line[j].distance) - comp_dist) < advanced_ray_check_thr) && (j < height_line.size())) {
        if (std::abs(height_line[j].height - drive_line) < (height_diff / 2)) {
            return false;
        }
        j = j + factor;
    }
    return true;
}

double get_angle(lidar_curb_det::lidar_measures v1, lidar_curb_det::lidar_measures v2) {
    /**
     * Calculating the angle between 2 Vectors
    */
    double angle1 = atan2(std::abs(v1.distance), v1.height);
    double angle2 = atan2(std::abs(v2.distance), v2.height);
    double angle = angle1 - angle2;
    return angle;
}

filters::limit lidar_curb_det::curbstone_checker_vectors(std::vector<lidar_curb_det::lidar_measures> height_line, double height_diff, double angle_threshold, double advanced_ray_check_thr, double wheel_inside, rclcpp::Time tstamp) {    
    angle_threshold =  M_PI * std::abs(angle_threshold) / 180;
    double left_limit = 0, right_limit = 0;
    size_t i = 0;

    /**
     * Starting at the farest Ray at the left and iterating till the first Ray "hits" the left Wheel
     * Setting the maximum amount of Rays that are used to check for Curbstones to a given Value or the maximal possible Value
    */
    do {
        i++;
    } while (i < height_line.size() && height_line[i].distance < -(wheel_inside));

    /**
     * Create 2 Vectors out of 3 neighbored Points and calculate the angle between those
    */
    for (size_t j = i - 1; j > 2; j--) {
        lidar_curb_det::lidar_measures v1 = lidar_curb_det::lidar_measures{
            std::abs(height_line[j].distance - height_line[j - 1].distance),
            std::abs(height_line[j].height   - height_line[j - 1].height)
        };
        lidar_curb_det::lidar_measures v2 = lidar_curb_det::lidar_measures{
            std::abs(height_line[j - 1].distance - height_line[j - 2].distance),
            std::abs(height_line[j - 1].height   - height_line[j - 2].height)
        };
        double angle = ::get_angle(v1, v2);

        /**
         * If a given angle_threshold is reached:
         *      The possible Curbstone will be checked for further Curbstone-characteristics (to exclude Potholes)
         */
        double drive_line = height_line[j].height;
        if (std::abs(angle) > angle_threshold) {
            if (curb_still_valid(height_diff, height_line, drive_line, j - 2, advanced_ray_check_thr, - 1)) {
                left_limit = std::abs(height_line[j - 1].distance);
                break;
            }
        }
    }
    
    /**
     * Same Method for the right side
    */
    while (height_line[i].distance < wheel_inside && i < height_line.size()) {
        i++;
    }
    for (size_t j = i; j < height_line.size() - 3; j++) {
        lidar_curb_det::lidar_measures v1 = lidar_curb_det::lidar_measures{
            std::abs(height_line[j].distance - height_line[j + 1].distance),
            std::abs(height_line[j].height   - height_line[j + 1].height)
        };
        lidar_curb_det::lidar_measures v2 = lidar_curb_det::lidar_measures{
            std::abs(height_line[j + 1].distance - height_line[j + 2].distance),
            std::abs(height_line[j + 1].height   - height_line[j + 2].height)
        };
        double angle = ::get_angle(v1, v2);
        double drive_line = height_line[j].height;
        if (std::abs(angle) > angle_threshold) {
            if (curb_still_valid(height_diff, height_line, drive_line, j + 2, advanced_ray_check_thr, 1)) {
                right_limit = std::abs(height_line[j + 1].distance);
                break;
            }
        }
    }

    /**
     * Creating a Limit_Pair and returning it
    */
    return filters::limit {
        tstamp,
        left_limit,
        right_limit
    };
}

void lidar_curb_det::visualize_cross_section(std::vector<lidar_curb_det::lidar_measures> height_line, double left_limit, double right_limit, double wheel_inside, double wheel_width) {
    /**
     * All Points from the height_line are getting visualized in blue
     * The left Limit of the Curbstone is drawn in green
     * The right Limit of the Curbstone is drawn in red
     * The Edges of the wheels are drawn in yellow
     * 
     * For useful Visualization, the coordinate-System is getting Scaled, so that anything can be seen ("+ 5", that anything will be in the Window; "* 100", that anything is scaled useful)
    */

    /**
     * Creating a Window and adding all points of the height_line to a cv::Point2d-Vector
    */
    cv::Mat image(250, 2000, CV_8UC3, cv::Scalar(0, 0, 0));
    int thickness = 2;
    std::vector<cv::Point2d> points;
    for (size_t i = 0; i < height_line.size(); i++) {
        points.push_back(cv::Point2d{
            ((height_line[i].distance + 10) * 100),
            height_line[i].height * 200});
    }

    /**
     * Drawing all Lines between all neighbored Points
    */
    thickness = 2;
    for (size_t i = 0; i < height_line.size() - 1; i++) {
		line(image, points[i], points[i + 1], cv::Scalar(255, 0, 0), thickness);
    }

    /**
     * If a left Limit exists (means, that the distance-Value is > 0), it will be shown as an vertical, green Line
     * Analog procedure for the right Limit, just with a red Line
    */
    if (left_limit >  0) {
        left_limit = -left_limit;
        cv::Point2d left_bottom((left_limit + 10) * 100, 0), left_top((left_limit + 10) * 100, 250);
        line(image, left_bottom, left_top, cv::Scalar(0, 255, 0), thickness);   // green -> left
    }
    if (right_limit > 0) {
        cv::Point2d right_bottom((right_limit + 10) * 100, 0), right_top((right_limit + 10) * 100, 250);
        line(image, right_bottom, right_top, cv::Scalar(0, 0, 255), thickness); // red -> right
    }


    /**
     * For better (more advanced) visualization the Edges of the Wheels are drawn in Yellow
    */
    cv::Point2d left_wheel_top_left((-(wheel_width + wheel_inside) + 10) * 100, 0), left_wheel_top_right(((-wheel_inside) + 10) * 100, 0), left_wheel_bottom_left((-(wheel_width + wheel_inside) + 10) * 100, 255), left_wheel_bottom_right((-wheel_inside + 10) * 100, 255); 
    cv::Point2d right_wheel_top_left(((wheel_inside) + 10) * 100, 0), right_wheel_top_right(((wheel_width + wheel_inside) + 10) * 100, 0), right_wheel_bottom_left((wheel_inside + 10) * 100, 255), right_wheel_bottom_right(((wheel_width + wheel_inside) + 10) * 100, 255); 

    thickness = 1;
    line(image, left_wheel_top_left, left_wheel_bottom_left, cv::Scalar(0, 255, 255), thickness);
    line(image, left_wheel_top_right, left_wheel_bottom_right, cv::Scalar(0, 255, 255), thickness);
    line(image, right_wheel_top_left, right_wheel_bottom_left, cv::Scalar(0, 255, 255), thickness);
    line(image, right_wheel_top_right, right_wheel_bottom_right, cv::Scalar(0, 255, 255), thickness);

    /**
     * Show the Window and close the entire launch, if "Esc" is pressed
    */
    imshow("Relief", image);
	if (cv::waitKey(10) == 27) { //exit on ESC
        exit(0);
    }
    return;
}