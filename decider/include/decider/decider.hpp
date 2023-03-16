#pragma once

#include <vector>
#include <utility>
#include <optional>

#include "rclcpp/time.hpp"
#include "rclcpp/logger.hpp"

#include "filters/post_filters.hpp"

std::ostream &operator<<(std::ostream& stream, const rclcpp::Time& rvalue);

namespace decider {
    /**
     * Retrieve pair of limits that are temporally close enough to each other.
     * 
     * @param camera_buffer: camera limits in ascending order (oldest to newest)
     * @param liar_buffer: lidar limits in ascending order (oldest to newest)
     * @param time_difference_thr: threshold in seconds that determines what "close enough" means
     * @param logger: ros2 logger for debug information
     * 
     * @return detected pair of values or nullopt if none was found
    */
    std::optional<std::pair<filters::limit, filters::limit>> getTimedPair(
        std::vector<filters::limit>& camera_buffer,
        std::vector<filters::limit>& lidar_buffer,
        double time_difference_thr,
        rclcpp::Logger logger
    );

    /**
     * Merge the pair into one limit by combining or choosing.
     * 
     * @param pair: pair to be merged
     * 
     * @return merged limit
    */
    filters::limit mergeTimedPair(
        const std::pair<filters::limit, filters::limit>& pair
    );
}