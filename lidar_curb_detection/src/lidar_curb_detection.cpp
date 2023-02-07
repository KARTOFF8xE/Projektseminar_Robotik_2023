#include "lidar_curb_detection.hpp"

#include <cmath>
#include <limits>
#include <iostream>
#include <algorithm>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

std::vector<lidar_curb_det::lidar_measures> lidar_curb_det::get_height_line(std::vector<float> ranges, double angle_increment, double min_range, double max_range, double min_angle, double max_angle, double mounting_angle) {    
    std::vector<lidar_curb_det::lidar_measures> normed_ranges;
    int i = 0;  // i ist Index Messungsnummer
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
                range * std::sin(mounting_angle) * std::sin(M_PI_2 - (j * angle_increment)),
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

std::vector<lidar_curb_det::lidar_measures> lidar_curb_det::smooth_height_line(std::vector<lidar_curb_det::lidar_measures> height_line, u_int quantity_threshold, u_int repetitions) {
    std::vector<lidar_curb_det::lidar_measures> smoothed_height_line = height_line, tmp_height_line;

    /**
     * Calculating the average of {quantity_threshold * 2 + 1} Values (symmetrically)
     * Doing it for {repetitions} time
    */
    for (size_t i = 0; i < repetitions; i++) {
        tmp_height_line.clear();
        tmp_height_line = smoothed_height_line;
        smoothed_height_line.clear();
        for (size_t j = quantity_threshold; j < (tmp_height_line.size() - quantity_threshold); j++) {
            double height_of_j = 0;
            for (size_t k = (j - quantity_threshold); k <= (j + quantity_threshold); k++) {
                height_of_j += tmp_height_line[k].height;
            }
            smoothed_height_line.push_back(lidar_curb_det::lidar_measures{
                tmp_height_line[j].distance,
                height_of_j / (2 * quantity_threshold + 1)
            });
        }
    }

    return smoothed_height_line;
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

filters::limit lidar_curb_det::curbstone_checker_absolut(std::vector<lidar_curb_det::lidar_measures> height_line, double height_diff, size_t max_check_length, double advanced_ray_check_thr, double wheel_inside, double wheel_width) {
    /**
     * Searching for all Measures that are Hitting the Ground, where also the wheels will hit the Ground
     * The Average between those Measures will be taken as drive_line, the height with which the Values of the height_line will be compared
    */
    double drive_line = 0;
    int counter = 0;
    for (size_t i = 0; i < height_line.size(); i++) {
        if (std::abs(height_line[i].distance) > wheel_inside && std::abs(height_line[i].distance) < wheel_inside + wheel_width) {
            drive_line += height_line[i].height;
            counter++;
        }
    }
    drive_line /= counter;

    /**
     * Starting at the farest Ray at the left and iterating till the first Ray "hits" the left Wheel
     * Setting the maximum amound of Rays that are used to check for Curbstones to a given Value or the maximal possible Value
    */
    size_t i = 0;
    while (i < height_line.size() && height_line[i].distance < -(wheel_inside + wheel_width)) {    // Gehe von Links an die Seite
        i++;
    }
    
    size_t check_length = (i < max_check_length) ? i : max_check_length;

    /**
     * If unable to detect a limit on the left/right side, the limit will left set at 0
     * Checking from the found Ray (that "hits" the left wheel) to the left
     * If a given height_diff(erence) compared to the drive_line is found and the height_diff to the previous Ray is high enough:
     *      The possible Curbstone will be checked for further Curbstone-characteristics (to exclude Potholes)
    */
    double left_limit = 0, right_limit = 0;
    for (size_t j = i - 1; j > i - check_length; j--) {
        if ((std::abs(height_line[j].height - drive_line) > height_diff) && (std::abs(height_line[j].height - height_line[j + 1].height) > height_diff / 2)) {
            if (curb_still_valid(height_diff, height_line, drive_line, j, advanced_ray_check_thr, -1)) {
                left_limit = std::abs(height_line[j].distance);
                break;
            }
        }
    }

    /**
     * Same Method like for the left side, just for the right side
    */
    while (i < height_line.size() && right_limit == 0) {
        if (height_line[i].distance > wheel_inside+wheel_width) {
            for (size_t j = i; j < i + max_check_length; j++) {
                if ((std::abs(height_line[j].height - drive_line) > height_diff) && (std::abs(height_line[j].height - drive_line) > height_diff / 2)) {
                    if (curb_still_valid(height_diff, height_line, drive_line, j, advanced_ray_check_thr, 1)) {
                        right_limit = std::abs(height_line[j].distance);
                        break;
                    }
                }
            }
        }
        i++;
    }

    /**
     * Creating a Limit_Pair and returning it
    */
    return filters::limit {
        left_limit,
        right_limit
    };
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

filters::limit lidar_curb_det::curbstone_checker_vectors(std::vector<lidar_curb_det::lidar_measures> height_line, double height_diff, double angle_threshold, size_t max_check_length, double advanced_ray_check_thr, double wheel_inside) {    
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

    size_t check_length = ((i - 2) < max_check_length) ? i - 2 : max_check_length;

    /**
     * Create 2 Vectors out of 3 neighbored Points and calculate the angle between those
    */
    for (size_t j = i - 1; j > i - check_length; j--) {
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
        double drive_line = height_line[j].height;    // TODO: Lösung einfallen lassen: die drive_line ist hier nicht mehr wirklich die drive_line, allgemeine umbenennung? Auch in anderen Funktionen?
        if (std::abs(angle) > angle_threshold) {
            if (curb_still_valid(height_diff, height_line, drive_line, j - 2, advanced_ray_check_thr, -1)) {
                left_limit = std::abs(height_line[j - 1].distance);
                break;
            }
        }
    }
    
    /**
     * Same Method from the left side for the right side
    */
    while (height_line[i].distance < wheel_inside && i < height_line.size()) {
        i++;
    }
    for (size_t j = i; j < i + max_check_length; j++) {
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
    return filters::limit{
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
    cv::Mat image(250, 1000, CV_8UC3, cv::Scalar(0, 0, 0));
    int thickness = 2;
    std::vector<cv::Point2d> points;
    for (size_t i = 0; i < height_line.size(); i++) {
        points.push_back(cv::Point2d{
            ((height_line[i].distance + 5) * 100),
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
        cv::Point2d left_bottom((left_limit + 5) * 100, 0), left_top((left_limit+5) * 100, 250);
        line(image, left_bottom, left_top, cv::Scalar(0, 255, 0), thickness);   // grün ist Links
    }
    if (right_limit > 0) {
        cv::Point2d right_bottom((right_limit + 5) * 100, 0), right_top((right_limit + 5) * 100, 250);
        line(image, right_bottom, right_top, cv::Scalar(0, 0, 255), thickness); // rot ist Rechts
    }


    /**
     * For better (more advanced) visualization the Edges of the Wheels are drawn in Yellow
    */
    cv::Point2d left_wheel_top_left((-(wheel_width + wheel_inside) +5) * 100, 0), left_wheel_top_right(((-wheel_inside) + 5) * 100, 0), left_wheel_bottom_left((-(wheel_width + wheel_inside) + 5) * 100, 255), left_wheel_bottom_right((-wheel_inside + 5) * 100, 255); 
    cv::Point2d right_wheel_top_left(((wheel_inside) + 5) * 100, 0), right_wheel_top_right(((wheel_width + wheel_inside) + 5) * 100, 0), right_wheel_bottom_left((wheel_inside + 5) * 100, 255), right_wheel_bottom_right(((wheel_width + wheel_inside) + 5) * 100, 255); 

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

void lidar_curb_det::visualize_street_view(std::vector<filters::limit> limits_vec, double wheel_inside, double wheel_width) {
    /**
     * The left Limit of the Curbstone is drawn in green
     * The right Limit of the Curbstone is drawn in red
     * The Edges of the wheels are drawn in yellow
     * 
     * For useful Visualization, the coordinate-System is getting Scaled, so that anything can be seen ("+ 5", that anything will be in the Window; "* 100", that anything is scaled useful)
    */

    /**
     * Creating a Window
     * If a left Limit is Valid, it will be Drawn with a green Point
     * Analog procedure for the Right Limit, just with red Points
     * The latest up to 1000 Limits will be printed (if Valid)
     * This will give a view of the Street from the Bird-Perspective
    */
    size_t i = 0;
    cv::Mat img(1000, 1000, CV_8UC3, cv::Scalar(0, 0, 0));
    while (i < 1000 && i < limits_vec.size()) {
        if (limits_vec[limits_vec.size() - i].left > 0) {
            cv::Point2d left((-(limits_vec[limits_vec.size() - i].left) + 5) * 100, i);
            cv::circle(img, left, 1, cv::Scalar(0, 255, 0));
        } else if (limits_vec[limits_vec.size() - i].left < 0) {
            cv::Point2d left((-(std::abs(limits_vec[limits_vec.size() - i].left)) + 5) * 100, i);
            cv::circle(img, left, 1, cv::Scalar(150, 150, 0));
        }
        if (limits_vec[limits_vec.size() - i].right > 0) {
            cv::Point2d right(((limits_vec[limits_vec.size() - i].right) + 5) * 100, i);
            cv::circle(img, right, 1, cv::Scalar(0, 0, 255));
        } else if (limits_vec[limits_vec.size() - i].right < 0) {
            cv::Point2d right(((std::abs(limits_vec[limits_vec.size() - i].right)) + 5) * 100, i);
            cv::circle(img, right, 1, cv::Scalar(150, 150, 0));
        }
        i++;
    }

    /**
     * For better (more advanced) visualization the Edges of the Wheels are drawn in Yellow
    */
    cv::Point2d left_wheel_top_left((-(wheel_width + wheel_inside) + 5) * 100, 0), left_wheel_top_right(((-wheel_inside) + 5) * 100, 0), left_wheel_bottom_left((-(wheel_width + wheel_inside) + 5) * 100, 1000), left_wheel_bottom_right((-wheel_inside + 5) * 100, 1000);
    cv::Point2d right_wheel_top_left(((wheel_inside) + 5) * 100, 0), right_wheel_top_right(((wheel_width + wheel_inside) + 5) * 100, 0), right_wheel_bottom_left((wheel_inside + 5) * 100, 1000), right_wheel_bottom_right(((wheel_width + wheel_inside) + 5) * 100, 1000);

    int thickness = 1;
    line(img, left_wheel_top_left, left_wheel_bottom_left, cv::Scalar(0, 255, 255), thickness);
    line(img, left_wheel_top_right, left_wheel_bottom_right, cv::Scalar(0, 255, 255), thickness);
    line(img, right_wheel_top_left, right_wheel_bottom_left, cv::Scalar(0, 255, 255), thickness);
    line(img, right_wheel_top_right, right_wheel_bottom_right, cv::Scalar(0, 255, 255), thickness);

    /**
     * Show the Window and close the entire launch, if "Esc" is pressed
    */
    imshow("Streetview", img);
	if (cv::waitKey(10) == 27) { //exit on ESC
        exit(0);
    }

    return;
}