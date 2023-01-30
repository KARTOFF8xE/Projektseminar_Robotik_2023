#include "color_filter.hpp"

//source: https://www.geeksforgeeks.org/multiple-color-detection-in-real-time-using-python-opencv/ (24.11.2022)
cv::Mat filters::create_color_mask(cv::Mat image, cv::Scalar lower_bounds, cv::Scalar upper_bounds) {
    cv::Mat mask, image_hsv,
            kernel = cv::Mat::ones(5, 5, CV_8UC1);

    //TODO: Evtl. wäre hier COLOR_BGR2HSV ausreichend? (wenn ja, dann predefined_colors anpassen!)
    // (Unterschied: BGR2HSV löst Hue in 0-180 (Grad) und BGR2HSV_FULL in 0-255 auf.)
    cv::cvtColor(image, image_hsv, cv::COLOR_BGR2HSV_FULL);

    cv::inRange(image_hsv, lower_bounds, upper_bounds, mask);

    //dilate mask with 5x5 kernel to get rid of noise
    cv::dilate(mask, mask, kernel);
    
    //invert mask so that green pixels are now black
    cv::bitwise_not(mask, mask);

    return mask;
}