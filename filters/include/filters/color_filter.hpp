#pragma once

#include <opencv2/opencv.hpp>

namespace filters {
    //some predefined color ranges for easy use
    //source: https://www.geeksforgeeks.org/multiple-color-detection-in-real-time-using-python-opencv/ (24.11.2022)
    namespace predefined_colors {
        const cv::Scalar red_lower(136, 87, 111);
        const cv::Scalar red_upper(180, 255, 255);

        const cv::Scalar green_lower(25, 52, 72);
        const cv::Scalar green_upper(102, 255, 255);

        const cv::Scalar blue_lower(94, 80, 2);
        const cv::Scalar blue_upper(120, 255, 255);
    }

    /**
     * Create a mask for a camera image to filter out certain color ranges.
     * 
     * @param image: camera image to create the mask for
     * @param lower_bounds: lower boundary as HSV color touple
     * @param upper_bounds: upper boundary as HSV color touble
     * 
     * @return the actual mask
    */
    cv::Mat create_color_mask(cv::Mat image, cv::Scalar lower_bounds, cv::Scalar upper_bounds);
}