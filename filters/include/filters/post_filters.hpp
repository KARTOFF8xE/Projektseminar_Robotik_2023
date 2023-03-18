#pragma once

#include <vector>
#include <rclcpp/time.hpp>

namespace filters {
    struct limit {
        // time of the measurement
        rclcpp::Time timestamp;

        // left Limit
        double left = 0.0;
        // right Limit
        double right = 0.0;

        // average distance left limit
        double avg_dist_left = 0.0;
        // average distance right limit
        double avg_dist_right = 0.0;
    };

    /**
     * Calculates the validity of the given point at i by checking previous and following points.
     * 
     * @param limits_vec: vector of all recorded limits
     * @param distance_thr: the maximum distance a limit should have to be a valid neighbour
     * @param quantity_check: the number of values in any direction that will be compared to each other (comparing with (2*quantity_check+1) values)
     * @param quantity_thr: the minimum number of neighbors that need to exist to be a valid limit (quantity_thr !<= quantity_check)
     * @param i: index for limit pair that should be checked
     * 
     * @return the validated limit pair at i
    */
    filters::limit bubble_filter(
        std::vector<filters::limit> limit_vec,
        double distance_thr,
        size_t quantity_check,
        size_t quantity_thr,
        size_t i
    );

    /**
     * Calculates the average distance from the limits in front and behind the given value at i.
     * 
     * @param limits_vec: vector of all recorded limits
     * @param quantity_check: number of limits that are checked in each direction (in front and behind)
     * @param counter_thr: minimum amount of differences in distance that are needed for the calculation of averages (counter_thr !<= quantity_check)
     * @param avg_dist_thr: distance averages below this threshold are validating the limit
     * @param i: index for limit pair that should be checked
     * 
     * @return the limit pair at i with corresponding distance_average (if existing, otherwise -1)
    */
    filters::limit noise_filter(
        std::vector<filters::limit> limits_vec,
        size_t quantity_check,
        size_t counter_thr,
        double avg_dist_thr,
        const size_t i
    );

    /**
     * Scanning a set of values for the number of valid ones.
     * If there are not enough valid values, the checked limit-object will be made invalid.
     * 
     * @param limits_vec: vector of all recorded limits
     * @param quantity_check: number of limits that are checked in each direction (in front and behind)
     * @param counter_thr: minimum amount of differences in distance that are needed for the calculation of averages
     * @param i: index for limit pair that should be checked
     * 
     * @return the limit pair at i with corresponding distance_average (if existing, otherwise -1)
    */
    filters::limit island_filter(
        std::vector<filters::limit> limits_vec,
        size_t quantity_check,
        size_t counter_thr,
        const size_t i
    );
}