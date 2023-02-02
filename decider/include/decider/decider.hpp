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
     * Try's to Combine several Limit-Values from the given Vectors
     * 
     * @param limit_1:  The limit for which we try to find a Partner
     * @param limits_2: The Buffer with Limits of possible Partners
     * 
     * @return The Result-Limit of up to two Limits
    */
    decider::limit get_limits(
        decider::limit limit_1,
        std::vector<decider::limit> limits_2
    );

}