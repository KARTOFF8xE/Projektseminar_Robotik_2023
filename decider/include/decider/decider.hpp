#pragma once

#include <vector>
#include <tuple>
#include <rclcpp/time.hpp>

namespace decider {
    struct limit {
        // left limit
        double limit = 0;
        // time of the measurement
        rclcpp::Time timestamp;
    };

    struct limits {
        // left limit
        double right_limit;
        // right limit
        double left_limit;
        // time of the measurement
        rclcpp::Time timestamp;
    };

    /**
     * Trys to combine several limit values from the given vectors.
     * 
     * @param limit_hf:  The limit for which we try to find a Partner
     * @param limits_lf: The Buffer with Limits of possible Partners
     * 
     * @return The Result-Limit of up to two Limits
    */
    decider::limit get_limits(
        decider::limit limit_hf,
        std::vector<decider::limit> limits_lf
    );

}