#include "decider.hpp"

#include "rclcpp/logging.hpp"

#include <numeric>

std::ostream &operator<<(std::ostream& stream, const rclcpp::Time& rvalue) {
    double time = rvalue.seconds();
    int32_t seconds = time;
    int32_t nanoseconds = (time - seconds) * 1000.0 * 1000.0;

    stream << seconds << "s " << nanoseconds << "ns";

    return stream;
}

std::optional<std::pair<filters::limit, filters::limit>> decider::getTimedPair(std::vector<filters::limit>& camera_buffer, std::vector<filters::limit>& lidar_buffer, double time_difference_thr, rclcpp::Logger logger) {
    filters::limit camera_limit, lidar_limit;
    std::vector<filters::limit>::iterator lidar_buffer_end  = lidar_buffer.end(),
                                          camera_buffer_end = camera_buffer.end();

    //iterate from beginning to end to go from oldest to newest
    //this assumes that all is in ascending order
    double difference, min_difference = std::numeric_limits<double>::max();
    std::vector<filters::limit>::iterator camera_limit_iter, lidar_limit_iter;
    for (camera_limit_iter = camera_buffer.begin(); camera_limit_iter != camera_buffer_end; camera_limit_iter++) { //camera is outer since it probably has more values
        for (lidar_limit_iter = lidar_buffer.begin(); lidar_limit_iter != lidar_buffer_end; lidar_limit_iter++) {
            //get difference and log the smallest for debugging
            difference = std::abs(camera_limit_iter->timestamp.seconds() - lidar_limit_iter->timestamp.seconds());
            min_difference = std::min(difference, min_difference);
            
            //check if any lidar values are in range
            if (difference < time_difference_thr) {
                camera_limit = *camera_limit_iter;
                lidar_limit = *lidar_limit_iter;

                break;
            }
        }
        if (lidar_limit_iter != lidar_buffer_end) {
            break;
        }
    }

    //check if match was found
    if (camera_limit_iter == camera_buffer_end) { //no match was found
        RCLCPP_INFO_STREAM(logger, "[get timed pair] No pair found - closest: " << min_difference);

        return std::nullopt;
    } else {
        RCLCPP_INFO_STREAM(logger, "[get timed pair] Found camera @ " << camera_limit.timestamp << " and lidar @ " << lidar_limit.timestamp << " with difference " << difference << 's');

        //erase current and all older limits
        lidar_buffer.erase(lidar_buffer.begin(), lidar_limit_iter + 1);
        camera_buffer.erase(camera_buffer.begin(), camera_limit_iter + 1);

        return std::make_pair(camera_limit, lidar_limit);
    }
}

filters::limit decider::mergeTimedPair(const std::pair<filters::limit, filters::limit>& pair) {
    filters::limit output;

    //TODO: is das sinnvoll?!?
    output.timestamp = rclcpp::Time((pair.first.timestamp.nanoseconds() + pair.second.timestamp.nanoseconds()) / 2.0f);

    //left
    bool first_has_left     = (pair.first.left  > 0.0f),
         second_has_left    = (pair.second.left > 0.0f);
    if (first_has_left && second_has_left) {
        output.left = (pair.first.left + pair.second.left) / 2.0f;
    } else if (first_has_left) {
        output.left = pair.first.left;
    } else if (second_has_left) { //if is unnecessary here, but it makes it more clear what happens
        output.left = pair.second.left;
    }

    //right
    bool first_has_right    = (pair.first.right  > 0.0f),
         second_has_right   = (pair.second.right > 0.0f);
    if (first_has_right && second_has_right) {
        output.right = (pair.first.right + pair.second.right) / 2.0f;
    } else if (first_has_right) {
        output.right = pair.first.right;
    } else if (second_has_right) { //if is unnecessary here, but it makes it more clear what happens
        output.right = pair.second.right;
    }

    return output;
}