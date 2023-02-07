#pragma once
#include <vector>

namespace filters {
    struct limit {
        // left Limit
        double left;
        // right Limit
        double right;
        // average distance left limit
        double avg_dist_left = 0;
        // average distance right limit
        double avg_dist_right = 0;
    };

    /**
     * Filtering away RunAways
     * 
     * @param limits_vec: a Vector where the Limits of all records are inside
     * @param distance_thr: the maximum distance a Limit should be to be a valid neighbour
     * @param quantity_check: the quantity of Values in any direction that will be compared. So at the end we are comparing with (2*quantity_check+1) values
     * @param quantity_thr: the minimum quantity of neighbors that need to exist to be a valid Limit
     * @param i: Index to Limit_Pair that should be checked
     * 
     * returns a single limit_pair
    */
    filters::limit filter_for_runaways(
        std::vector<filters::limit> limit_vec,
        double distance_thr,
        size_t quantity_check,
        size_t quantity_thr,
        size_t i
    );

    /**
     * Calculating the average Distance from the limits before and behind the looked-up Value
     * 
     * @param limits_vec: a Vector where the Limits of all records are inside
     * @param quantity_check: amount of Limits that is checked in each direction (before and behind)
     * @param counter_thr: minimum amount of taken distance-differences that is needed for the average-calculation, otherwise the limit is not valid
     * @param avg_dist_thr: distance-averages below this Threshold are validating the Limit
     * @param i: Index to Limit_Pair that should be checked
     * 
     * @returns a single limit_pair with belonging distance_average (if existing, otherwise: -1)
    */
    filters::limit get_avg_dist(
        std::vector<filters::limit> limits_vec,
        size_t quantity_check,
        size_t counter_thr,
        double avg_dist_thr,
        size_t i
    );

    /**
     * Scanning a set of Values for the quantity of valid Values, if there are to less valid Values, the checked limit-object will be made invalid
     * 
     * @param limits_vec: a Vector where the Limits of all records are inside
     * @param quantity_check: amount of Limits that is checked in each direction (before and behind)
     * @param counter_thr: minimum amount of taken distance-differences that is needed for the average-calculation, otherwise the limit is not valid
     * @param i: Index to Limit_Pair that should be checked
     * 
     * @returns a single limit_pair with belonging distance_average
    */
    filters::limit check_for_valid_island(
        std::vector<filters::limit> limits_vec,
        size_t quantity_check,
        size_t counter_thr,
        size_t i
    );
}