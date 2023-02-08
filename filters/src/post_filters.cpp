#include "post_filters.hpp"
#include <math.h>

filters::limit filters::filter_for_runaways(std::vector<filters::limit> limits_vec, double distance_thr, size_t quantity_check, size_t quantity_thr, size_t i) {
    /**
     * Define Counter, to count valid neighbours (Later they will be decreased by one, cause any Limit also gets compared to itself).
     * Extract the checked Limit_Pair that will be returned later
    */
    size_t counter_left = 0, counter_right = 0;
    filters::limit limit_at_i = limits_vec[i];

    /**
     * Any relevant Value will be compared with the checked pair, if the distance is small enough, it will be counted as a neighbour
    */
    for (size_t j = i - quantity_check; j < i + quantity_check; j++) {
        if (std::abs(limit_at_i.left - std::abs(limits_vec[j].left)) < distance_thr && limit_at_i.left > 0) {
            counter_left++;
        }
        if (std::abs(limit_at_i.right - std::abs(limits_vec[j].right)) < distance_thr && limit_at_i.right >= 0) {
            counter_right++;
        }
    }
    counter_left--;
    counter_right--;

    /**
     * If the quantity of neighbors is too low, the checked Limit is invalid and will be made negative (unvalid Values)
    */
    if (counter_left < quantity_thr) {
        limit_at_i.left = -limit_at_i.left;
    }
    if (counter_right < quantity_thr) {
        limit_at_i.right = -limit_at_i.right;
    }

    return limit_at_i;
}

filters::limit filters::get_avg_dist(std::vector<filters::limit> limits_vec, size_t quantity_check, size_t counter_thr, double avg_dist_thr, size_t i) {
    /**
     * Define useful Variables
    */
    double avg_dist_diff_left = 0, avg_dist_diff_right = 0;
    size_t counter_for_valid_left = 0, counter_for_valid_right = 0;

    /**
     * Calculate all (2 * quantity_check + 1 - 1) distances and increase the Counter by 1 for each Distance
    */
    for (size_t j = i - quantity_check + 1; j < i + quantity_check; j++) {
        if (limits_vec[j - 1].left > 0 && limits_vec[j].left > 0) {
            avg_dist_diff_left += std::abs(limits_vec[j - 1].left - limits_vec[j].left);
            counter_for_valid_left++;
        }
        if (limits_vec[j - 1].left > 0 && limits_vec[j].left > 0) {
            avg_dist_diff_right += std::abs(limits_vec[j - 1].right - limits_vec[j].right);
            counter_for_valid_right++;
        }
    }

    /**
     * If there where enough distances taken, the Average of those will be calculated
     * Else it will be assigned to -1 (invalid)
    */
    if (counter_for_valid_left >= counter_thr) {
        avg_dist_diff_left = avg_dist_diff_left / counter_for_valid_left;
    } else {
        avg_dist_diff_left = -1;
    }
    if (counter_for_valid_right >= counter_thr) {
        avg_dist_diff_right = avg_dist_diff_right / counter_for_valid_right;
    } else {
        avg_dist_diff_right = -1;
    }

    /**
     * Negating the limits if there is no average Distance or the Threshold is reached
    */
    double new_left = (avg_dist_diff_left < 0 || avg_dist_diff_left > avg_dist_thr)? -std::abs(limits_vec[i].left) : limits_vec[i].left;
    double new_right = (avg_dist_diff_right < 0 || avg_dist_diff_right > avg_dist_thr)? -std::abs(limits_vec[i].right) : limits_vec[i].right;

    /**
     * Returning an Object of filters::limit
    */
    return filters::limit {
        limits_vec[i].timestamp,
        new_left,
        new_right,
        avg_dist_diff_left,
        avg_dist_diff_right
    };
}

filters::limit filters::check_for_valid_island(std::vector<filters::limit> limits_vec, size_t quantity_check, size_t counter_thr, size_t i) {
    /**
     * Needed Variables get instantiated and detected, if they are valid
    */
    uint left_counter = 0, right_counter = 0;
    bool left_valid_at_i = (limits_vec[i].left > 0) ? true : false;
    bool right_valid_at_i = (limits_vec[i].right > 0) ? true : false;

    /**
     * For the specified interval we check, how many limits inside of it are valid
     * If both checked limits are invalid, we don't need to check for anything
    */
    for (size_t j = i - quantity_check; j < i + quantity_check; j++) {
        if (!left_valid_at_i && !right_valid_at_i) {
            break;
        }
        if (limits_vec[j].left > 0 && left_valid_at_i){
            left_counter++;
        }
        if (limits_vec[j].right > 0 && right_valid_at_i) {
            right_counter++;
        }
    }

    /**
     * Comparing the quantity of valid Values with the given threshold and declaring if the Limit is valid or invalid
    */
    double new_left = (left_counter > counter_thr) ? limits_vec[i].left : -std::abs(limits_vec[i].left);
    double new_right = (right_counter > counter_thr) ? limits_vec[i].right : -std::abs(limits_vec[i].right);

    /**
     * Returning an Object of filters::limit
    */
    return filters::limit {
        limits_vec[i].timestamp,
        new_left,
        new_right,
        limits_vec[i].avg_dist_left,
        limits_vec[i].avg_dist_right
    };
}