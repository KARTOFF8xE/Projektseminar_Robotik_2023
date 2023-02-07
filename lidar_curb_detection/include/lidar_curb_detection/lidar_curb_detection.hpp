#pragma once

#include <vector>
#include <tuple>
#include "filters/post_filters.hpp"

namespace lidar_curb_det {
    struct lidar_measures {
        // distance from center to vertical line
        double distance;
        // height of the vertical line
        double height;
    };

//    struct limit {
//        // left Limit
//        double left;
//        // right Limit
//        double right;
//        // average distance left limit
//        double avg_dist_left = 0;
//        // average distance right limit
//        double avg_dist_right = 0;
//    };

    /**
     * Transform scanner ranges to make them Top->Down use them in a cartesian coordinate system (right: X/distance; top: -Y/negative height;.
     * 
     * @param range: original scanner ranges
     * @param increment: laser scanner horizontal angle increment
     * @param min_range: minimal useable range value
     * @param max_range: maximal useable range value
     * @param min_angle: minimal angle of valid/usefull LiDAR-data
     * @param max_angle: maximal angle of valid/usefull LiDAR-data
     * @param mounting_angle: vertical angle at which the scanner is looking at the ground
     * 
     * @returns transformed scanner ranges
    */
    std::vector<lidar_measures> get_height_line(
        std::vector<float> range,
        double increment,
        double min_range,
        double max_range,
        double min_angle,
        double max_angle,
        double mounting_angle
    );

    /**
     * Smoothing the Height-Line (by average)
     * 
     * @param height_line: the in "get_height_line" transformed height line
     * @param smoothing_threshold: the quantity of values (per side [taken to the left; taken to the right]) that's used to calculate one point
     * @param repetitions: how often the given height_line should get smoothed
     * 
     * @returns smoothed height line
    */
    std::vector<lidar_curb_det::lidar_measures> smooth_height_line(
        std::vector<lidar_curb_det::lidar_measures> height_line,
        u_int smoothing_threshold,
        u_int repetitions
    );

    /**
     * Sort the height_line by distance
     *
     * @param height_line: the in "get_height_line" transformed height line
     * 
     * @returns sorted height_line 
    */
    std::vector<lidar_curb_det::lidar_measures> sort_height_line(
        std::vector<lidar_curb_det::lidar_measures> height_line
    );

    /**
     * Check for specified Threshold if the detected height_difference is an Curbstone or something else like a pott-hole
     * 
     * @param height_diff: defines the needed height difference [in meters] between the road and the curbstone
     * @param height_line: the in "get_height_line" transformed height line
     * @param drive_line: the height of the driving plane
     * @param j: tells the LiDAR-Ray that detected a possible curbstone
     * @param advanced_ray_check_thr: the threshold how many Meters should be checked to detect if a possible curbstone is more likely a pott hole
     * @param factor: tells if the pott-hole-search is going from "right to left" or from "left to right" 
     * 
     * @returns boolean if curbstone if really a curbstone
    */
    bool curb_still_valid(
        double height_diff,
        std::vector<lidar_curb_det::lidar_measures> height_line,
        double drive_line,
        size_t j,
        double advanced_ray_check_thr,
        int factor
    );

    /**
     * Absolut-Values check for a Curbstone    
     * 
     * @param height_line: the in "get_height_line" transformed height line
     * @param height_diff: defines the needed height difference [in meters] between the road and the curbstone
     * 
     * @returns left and/or right distance to curbstone as lidar_curb_det::limit-struct (from horizontal Robot-Center) if existing. Returning -1 per side if not existing fot the side
    */
    filters::limit curbstone_checker_absolut(
        std::vector<lidar_curb_det::lidar_measures> height_line,
        double height_diff,
        size_t max_check_length,
        double advanced_ray_check_thr,
        double wheel_inside,
        double wheel_width
    );

    /**
     * Get Angle between 2 Vectors
     * 
     * @param v1: first Vector
     * @param v2: second Vector
     * 
     * @returns Angle between v1 and v2
    */
    double get_angle(
        lidar_curb_det::lidar_measures v1,
        lidar_curb_det::lidar_measures v2
    );

    /**
     * Vector-Based check for a Curbstone
     * 
     * @param height_line: the in "get_height_line" transformed height line
     * @param angle_threshold: defines the needed angle difference [in degree] between the road and the curbstone
     * 
     * @returns left and/or right distance to curbstone as lidar_curb_det::limit-struct (from horizontal Robot-Center) if existing. Returning -1 per side if not existing fot the side
    */
    filters::limit curbstone_checker_vectors(
        std::vector<lidar_curb_det::lidar_measures> height_line,
        double height_diff,
        double angle_threshold,
        size_t max_check_length,
        double advanced_ray_check_thr,
        double wheel_inside
    );

    // /**
    //  * Filtering away RunAways
    //  * 
    //  * @param limits_vec: a Vector where the Limits of all records are inside
    //  * @param distance_thr: the maximum distance a Limit should be to be a valid neighbour
    //  * @param quantity_check: the quantity of Values in any direction that will be compared. So at the end we are comparing with (2*quantity_check+1) values
    //  * @param quantity_thr: the minimum quantity of neighbours that need to exist to be a valid Limit
    //  * @param i: Index to Limit_Pair that should be checked
    //  * 
    //  * returns a single limit_pair
    // */
    // lidar_curb_det::limit filter_for_runaways(
        // std::vector<lidar_curb_det::limit> limit_vec,
        // double distance_thr,
        // size_t quantity_check,
        // size_t quantity_thr,
        // size_t i
    // );

    // /**
    //  * Calculating the average Distance from the limits before and behind the looked-up Value
    //  * 
    //  * @param limits_vec: a Vector where the Limits of all records are inside
    //  * @param quantity_check: amount of Limits that is checked in each direction (before and behind)
    //  * @param counter_thr: minimum amount of taken distance-differences that is needed for the average-calculation, otherwise the limit is not valid
    //  * @param avg_dist_thr: distance-averages below this Threshold are validating the Limit
    //  * @param i: Index to Limit_Pair that should be checked
    //  * 
    //  * @returns a single limit_pair with belonging distance_average (if existing, otherwise: -1)
    // */
    // filters::limit get_avg_dist(
    //     std::vector<filters::limit> limits_vec,
    //     size_t quantity_check,
    //     size_t counter_thr,
    //     double avg_dist_thr,
    //     size_t i
    // );

    // /**
    //  * Scanning a set of Values for the quantity of valid Values, if there are to less valid Values, the checked limit-object will be made invalid
    //  * 
    //  * @param limits_vec: a Vector where the Limits of all records are inside
    //  * @param quantity_check: amount of Limits that is checked in each direction (before and behind)
    //  * @param counter_thr: minimum amount of taken distance-differences that is needed for the average-calculation, otherwise the limit is not valid
    //  * @param i: Index to Limit_Pair that should be checked
    //  * 
    //  * @returns a single limit_pair with belonging distance_average
    // */
    // filters::limit check_for_valid_island(
    //     std::vector<filters::limit> limits_vec,
    //     size_t quantity_check,
    //     size_t counter_thr,
    //     size_t i
    // );

    /**
     * Visualizing Data
     * 
     * @param height_line: the in "get_height_line" transformed height line
     * @param left_border: the left Limit of the way driven on
     * @param right_border: the right Limit of the way driven on
     * 
     * @returns nothing
    */
    void visualize_cross_section(
        std::vector<lidar_curb_det::lidar_measures> height_line,
        double left_border,
        double right_border,
        double wheel_inside,
        double wheel_width
    );

    /**
     * Get the Streetview from the recorded Data
     * 
     * @param limits_vec: a Vector where the Limits of all records are inside
     * 
     * @returns nothing
    */
    void visualize_street_view(
        std::vector<filters::limit> limits_vec,
        double wheel_inside,
        double wheel_width
    );
}