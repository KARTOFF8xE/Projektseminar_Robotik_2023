#pragma once

#include <vector>
#include <utility>
#include <optional>

#include "rclcpp/time.hpp"
#include "rclcpp/logger.hpp"

#include "filters/post_filters.hpp"


std::ostream &operator<<(std::ostream& stream, const rclcpp::Time& rvalue);

namespace filters {
    bool operator<(const limit& lvalue, const limit& rvalue);
    bool operator>(const limit& lvalue, const limit& rvalue);
}

namespace decider {
    rclcpp::Time getMostRecentLimitTimestamp(
        const std::vector<filters::limit>& camera_buffer,
        const std::vector<filters::limit>& lidar_buffer
    );

    std::vector<filters::limit> removeTooOldLimits(
        const std::vector<filters::limit>& buffer,
        rclcpp::Time most_recent_timestamp,
        double disgard_time_thr
    );

    std::optional<std::pair<filters::limit, filters::limit>> getTimedPair(
        std::vector<filters::limit>& camera_buffer,
        std::vector<filters::limit>& lidar_buffer,
        double time_difference_thr,
        rclcpp::Logger logger
    );

    filters::limit mergeTimedPair(
        const std::pair<filters::limit, filters::limit>& pair
    );
}