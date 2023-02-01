#pragma once

#include <vector>
#include <tuple>
#include <rclcpp/time.hpp>

namespace decider {
    struct received_msg {
        // left limit
        double left;
        // right limit
        double right;
        // time of the measurement
        rclcpp::Time timestamp;
    };

//TODO: nötige Structs und Funktionen adden (siehe Bsp)
//    /**
//     * Transform scanner ranges to make them Top->Down use them in a cartesian coordinate system (right: X/distance; top: -Y/negative height;.
//     * 
//     * @param range: original scanner ranges
//     * @param increment: laser scanner horizontal angle increment
//     * @param min_range: minimal useable range value
//     * @param max_range: maximal useable range value
//     * @param min_angle: minimal angle of valid/usefull LiDAR-data
//     * @param max_angle: maximal angle of valid/usefull LiDAR-data
//     * @param mounting_angle: vertical angle at which the scanner is looking at the ground
//     * 
//     * @returns transformed scanner ranges
//    */
//    std::vector<lidar_measures> get_height_line(
//        std::vector<float> range,
//        double increment,
//        double min_range,
//        double max_range,
//        double min_angle,
//        double max_angle,
//        double mounting_angle
//    );
}