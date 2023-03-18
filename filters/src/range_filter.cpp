#include "range_filter.hpp"

#include <memory>
#include <cmath>
#include <limits>

/**
 * Return the maximum possible value of the given OpenCV Type.
 * 
 * @param cv_type: integer flag as given by cv::Mat::type() or e.g. CV_8UC3, etc.
 * 
 * @throw invalid_argument: if the system does not have 16bit float
 * 
 * @return maximal possible value for the matrix type
 * 
 * Reference: http://ninghang.blogspot.com/2012/11/list-of-mat-type-in-opencv.html
*/
double get_max_mat_value(int cv_type) {
    switch (cv_type % 8) {
        case CV_8U:
            return std::numeric_limits<uint8_t>::max();
        case CV_8S:
            return std::numeric_limits<int8_t>::max();
        case CV_16U:
            return std::numeric_limits<uint16_t>::max();
        case CV_16S:
            return std::numeric_limits<int16_t>::max();
        case CV_32S:
            return std::numeric_limits<int32_t>::max();
        case CV_32F:
            return std::numeric_limits<float>::max();
        case CV_64F:
            return std::numeric_limits<double>::max();
        case CV_16F:
            #if (defined(__HAVE_FLOAT16) && __HAVE_FLOAT16)
            return std::numeric_limits<_Float16>::max();
            #else
            throw std::invalid_argument("16bit floating point type is not available.");
            #endif //__HAVE_FLOAT16
        default: //can never be reached -> just to satisfy the compiler
            throw std::out_of_range("Impossible value reached.");
    }
}

cv::Mat filters::create_range_mask(const cv::Mat& depth_image, uint8_t border_distance) {
    cv::Mat mask = (depth_image < border_distance);

    return mask * get_max_mat_value(mask.type());
}

cv::Mat filters::create_range_mask(const cv::Mat& depth_image, float min_distance, float max_distance, float border_distance) {
    return filters::create_range_mask(depth_image, static_cast<uint8_t>(255 * (border_distance - min_distance) / (max_distance - min_distance)));
}