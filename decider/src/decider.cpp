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

bool filters::operator<(const filters::limit& lvalue, const filters::limit& rvalue) {
    return lvalue.timestamp.nanoseconds() < rvalue.timestamp.nanoseconds();
}
bool filters::operator>(const filters::limit& lvalue, const filters::limit& rvalue) {
    return lvalue.timestamp.nanoseconds() > rvalue.timestamp.nanoseconds();
}

rclcpp::Time decider::getMostRecentLimitTimestamp(const std::vector<filters::limit>& camera_buffer, const std::vector<filters::limit>& lidar_buffer) {
    //get the newest limits in the buffer
    std::vector<filters::limit>::const_iterator max_camera_limit = std::max_element(camera_buffer.begin(), camera_buffer.end()),
                                                max_lidar_limit  = std::max_element(lidar_buffer.begin(),  lidar_buffer.end());
    // bool max_camera_limit_is_valid  = (max_camera_limit != camera_buffer.end()),
    //      max_lidar_limit_is_valid   = (max_lidar_limit  != lidar_buffer.end());

    // if (max_camera_limit_is_valid && max_lidar_limit_is_valid) {
        return max_camera_limit > max_lidar_limit ? max_camera_limit->timestamp : max_lidar_limit->timestamp;
    // } else if (max_camera_limit_is_valid) {
    //     return max_camera_limit->timestamp;
    // } else { //if (max_lidar_limit_is_valid) {
    //     return max_lidar_limit->timestamp;
    // }
}

std::vector<filters::limit> decider::removeTooOldLimits(const std::vector<filters::limit>& buffer, rclcpp::Time most_recent_timestamp, double disgard_time_thr) {
    std::vector<filters::limit> new_buffer;

    for (filters::limit element: buffer) {
        //only allow elements that are less then disgard_time_thr seconds in the past
        if ((most_recent_timestamp - element.timestamp).seconds() < disgard_time_thr) {
            element.avg_dist_left++;
            new_buffer.push_back(element);
        }
    }

    return new_buffer;
}

std::optional<std::pair<filters::limit, filters::limit>> decider::getTimedPair(std::vector<filters::limit>& camera_buffer, std::vector<filters::limit>& lidar_buffer, double time_difference_thr, rclcpp::Logger logger) {
    //TODO: make parameter
    // uint max_unused_iterations = 25u;
    
    //get the oldest limits in the buffer
    std::vector<filters::limit>::iterator min_camera_limit = std::min_element(camera_buffer.begin(), camera_buffer.end()),
                                          min_lidar_limit  = std::min_element(lidar_buffer.begin(),  lidar_buffer.end());

    // bool min_camera_limit_is_valid  = (min_camera_limit != camera_buffer.end()),
    //      min_lidar_limit_is_valid   = (min_lidar_limit  != lidar_buffer.end());
    filters::limit reference, other_value;
    // if (min_camera_limit_is_valid && min_lidar_limit_is_valid) {
        if (*min_camera_limit < *min_lidar_limit) {
            //camera is older so it is chosen as reference
            //therefor we search in the lidar buffer for a close enough value
            reference = *min_camera_limit;

            double min_time_difference = std::numeric_limits<double>::max(),
                time_difference;
            std::vector<filters::limit>::iterator lidar_iterator;
            for (lidar_iterator = lidar_buffer.begin(); lidar_iterator != lidar_buffer.end(); lidar_iterator++) {
                //get time difference
                time_difference = reference.timestamp.seconds() - lidar_iterator->timestamp.seconds();

                //records closest a value has ever come
                min_time_difference = std::min(std::abs(time_difference), min_time_difference);

                if (std::abs(time_difference) < time_difference_thr) {
                    other_value = *lidar_iterator;
                    break;
                }
            }

            if (lidar_iterator == lidar_buffer.end()) { //no value was found
                RCLCPP_INFO_STREAM(logger, "[merge] Reference (camera @ " << reference.timestamp << ") has closest diff: " << min_time_difference);
                return std::nullopt;
            }
            
            //being here means two (sufficiently) valid limits where found so they can be removed from the buffer
            camera_buffer.erase(min_camera_limit);
            lidar_buffer.erase(lidar_iterator);

            RCLCPP_INFO_STREAM(logger, "[merge] Reference (camera @ " << reference.timestamp << ") found lidar value (" << other_value.timestamp << ") | diff: " << time_difference);
        } else {
            //lidar is older so it is chosen as reference
            //therefor we search in the camera buffer for a close enough value
            reference = *min_lidar_limit;

            double min_time_difference = std::numeric_limits<double>::max(),
                time_difference;
            std::vector<filters::limit>::iterator camera_iterator;
            for (camera_iterator = camera_buffer.begin(); camera_iterator != camera_buffer.end(); camera_iterator++) {
                //get time difference
                time_difference = reference.timestamp.seconds() - camera_iterator->timestamp.seconds();

                //records closest a value has ever come
                min_time_difference = std::min(std::abs(time_difference), min_time_difference);

                if (std::abs(time_difference) < time_difference_thr) {
                    other_value = *camera_iterator;
                    break;
                }
            }

            if (camera_iterator == camera_buffer.end()) { //no value was found
                RCLCPP_INFO_STREAM(logger, "[merge] Reference (lidar @ " << reference.timestamp << ") has closest diff: " << min_time_difference);
                return std::nullopt;
            }
            
            //being here means two (sufficiently) valid limits where found so they can be removed from the buffer
            lidar_buffer.erase(min_lidar_limit);
            camera_buffer.erase(camera_iterator);

            RCLCPP_INFO_STREAM(logger, "[merge] Reference (lidar @ " << reference.timestamp << ") found camera value (" << other_value.timestamp << ") | diff: " << time_difference);
        }
    // } else if (min_camera_limit_is_valid && min_camera_limit->avg_dist_left >= max_unused_iterations) {
    //     reference = *min_camera_limit;
    //     camera_buffer.erase(min_camera_limit);
    //     other_value.timestamp = reference.timestamp; //since this value doesn't really exist the mean between its timestamp and the references should give the references timestamp

    //     RCLCPP_INFO_STREAM(logger, "[merge] Reference (camera @ " << reference.timestamp << ") forced after " << (int)reference.avg_dist_left << " iterations");
    // } else if (min_lidar_limit_is_valid  && min_lidar_limit->avg_dist_left  >= max_unused_iterations) {
    //     reference = *min_lidar_limit;
    //     lidar_buffer.erase(min_lidar_limit);
    //     other_value.timestamp = reference.timestamp; //since this value doesn't really exist the mean between its timestamp and the references should give the references timestamp

    //     RCLCPP_INFO_STREAM(logger, "[merge] Reference (lidar @ " << reference.timestamp << ") forced after " << (int)reference.avg_dist_left << " iterations");
    // } else {
    //     return std::nullopt;
    // }

    return std::make_pair(reference, other_value);
}

filters::limit decider::mergeTimedPair(const std::pair<filters::limit, filters::limit>& pair) {
    filters::limit output;

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