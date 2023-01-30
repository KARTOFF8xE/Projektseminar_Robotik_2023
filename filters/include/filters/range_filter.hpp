#pragma once

#include <opencv2/opencv.hpp>

namespace filters {
    /**
     * Create a mask for the camera image to filter out objects above a certain range.
     * 
     * @param depth_image: single channel image filled with depth values for each pixel (type: CV_8UC1)
     * @param border_distance: critical value aka where to switch (precalculated to fit 0 <= border_distance <= 255)
     * 
     * @return the actual mask or dummy matrix if an error occured (check returned_state)
    */
    cv::Mat create_range_mask(
        const cv::Mat&  depth_image,
        uint8_t         border_distance
    );

    /**
     * Create a mask for the camera image to filter out objects above a certain range.
     * 
     * @param depth_image: single channel image filled with depth values for each pixel (type: CV_8UC1)
     * @param min_distance: minimal depth represented by 0
     * @param max_distance: maximum depth represented by 255
     * @param border_distance: critical value aka where to switch
     * 
     * @return the actual mask or dummy matrix if an error occured (check returned_state)
    */
    cv::Mat create_range_mask(
        const cv::Mat&  depth_image,
        float           min_distance,
        float           max_distance,
        float           border_distance
    );
}