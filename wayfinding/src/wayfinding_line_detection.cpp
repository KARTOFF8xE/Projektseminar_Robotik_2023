#include "wayfinding.hpp"

#include <cmath>

void wayfinding::line_detection::preFilter(cv::InputArray src, cv::OutputArray dst, wayfinding::line_detection::FilterType filter, int ksize) {
    switch (filter) {
        case FilterType::NO_FILTER:
            src.copyTo(dst);
            break;
        case FilterType::MEDIAN:    //Median Filter
            cv::medianBlur(src, dst, ksize);
            break;
        case FilterType::NORM_BOX:  //Normalized Box Filter
            cv::blur(src, dst, cv::Size(ksize, ksize));
            break;
        case FilterType::BILATERAL: //Bilateral Filter
            cv::bilateralFilter(src, dst, 5, 200.0, 200.0);
            break;
        case FilterType::BOX:       //Box Filter
            cv::boxFilter(src, dst, -1, cv::Size(ksize, ksize));
            break;
        case FilterType::GAUSS:     //Gaussian Filter
            cv::GaussianBlur(src, dst, cv::Size(ksize, ksize), 1.5, 2);
            break;
    }
}

cv::Mat wayfinding::line_detection::getHoughLines(cv::Mat& src, std::vector<cv::Vec4i>& dst, wayfinding::line_detection::parameters_t parameters) {
    //cv::Mat filtered;
    preFilter(src, src, parameters.filter_type, parameters.kernel_size);

    return getHoughLines(src, dst, parameters.sigma, parameters.apertureSize, parameters.rho, parameters.theta, parameters.threshold, parameters.min_line_length, parameters.max_line_gap);
}

/**
 * Calculate the median over all matrix elements
 * 
 * @param matrix: single channel source matrix
 * 
 * @return median value
*/
double getMatrixMedian(const cv::Mat& matrix) {
    //sanity check
    CV_Assert(matrix.channels() == 1);

    std::vector<double> matrix_as_vector;
    //reshape matrix into one dimensional array and write to vector
    matrix.reshape(0,1).copyTo(matrix_as_vector);

    std::vector<double>::iterator begin = matrix_as_vector.begin();
    size_t half_vector_size = matrix_as_vector.size() / 2;
    std::nth_element(begin, begin + half_vector_size, matrix_as_vector.end());

    return matrix_as_vector[half_vector_size];
}

cv::Mat wayfinding::line_detection::getHoughLines(cv::InputArray src, std::vector<cv::Vec4i>& dst, double sigma, int apertureSize, double rho, double theta, int threshold, double min_line_length, double max_line_gap) {
    cv::Mat canny_mat, temp;

    cv::cvtColor(src, temp, cv::COLOR_BGR2GRAY);
    double median = ::getMatrixMedian(temp);
	double lower_threshold = std::max(0.0, (1.0 - sigma) * median),
	       upper_threshold = std::min(255.0, (1.0 + sigma) * median);

    //create contour image
    cv::Canny(temp, canny_mat, lower_threshold, upper_threshold, apertureSize); //threshold1, threshold2, apertureSize);
    //heuristically identify lines in contour image
    cv::HoughLinesP(canny_mat, dst, rho, theta, threshold, min_line_length, max_line_gap);

    return canny_mat;
}

/**
 * Normalizes the angle of a line (given by dx, dy) to a value between 0 and 90.
 * 
 * @param dx: adjacent leg (x_1 - x_0) [px]
 * @param dy: opposite leg (y_1 - y_0) [px]
 * 
 * @return angle [°]
*/
double getNormalizedAngle(int dx, int dy) {
    //get angle and limit it to 0 - 180°
    double angle = std::abs(std::atan2(dy, dx));
    if (angle > M_PI_2) { //angles in the range (90, 180] are supposed to be treated like angles in the range (0, -90]
        angle = M_PI - angle; // pi/2 - (angle - pi/2)
    }

    return angle;
}

void wayfinding::line_detection::lineFilter(const std::vector<cv::Vec4i>& src, std::vector<cv::Vec4i>& dst, uint width, double rel_center_width, double angle) {
    const double rel_threshold = (100.0 - rel_center_width) / 2;
    const int left_threshold = width * (rel_threshold / 100.0),
              right_threshold = width * (1 - rel_threshold / 100.0);

    int x_0, y_0,
        x_1, y_1;
    int center_x;

    for(cv::Vec4i line: src) {
        x_0 = line[0]; y_0 = line[1];
        x_1 = line[2]; y_1 = line[3];

        center_x = (x_0 + x_1) / 2;
        
        if ((center_x < left_threshold || center_x > right_threshold) && (::getNormalizedAngle(x_1 - x_0, y_1 - y_0) * 180 / M_PI > angle)) {
            dst.push_back(line);
        }
    }
}

/**
 * Calculate a "quality" value for a line (given by angle, length, vertical and horizontal).
 * (higher means worse)
 * 
 * @param angle: line angle relative to the x-axis between 0 and 90 [°]
 * @param length: line length [px]
 * @param vertical: distance of the line center from the image top [px]
 * @param horizontal: distance of the line center from the horizontal image center [px]
 * @param image_size: size of the source image [px, px]
 * @param angle_threshold: the optimal angle to achieve; angles smaller or greater get a worse rating [°]
 * @param vertical_threshold: optimal vertical position to achieve; vertical positions smaller or greater get a worse rating [px]
 * @param horizontal_threshold: optimal horizontal position to achieve; horizontal positions smaller or greater get a worse rating [px]
 * 
 * @return rating value
*/
double lineMetric(double angle, double length, double vertical, double horizontal, double angle_threshold, int vertical_threshold, int horizontal_threshold) {
    double vertical_factor   = 1.0 - std::abs(1.0 - vertical / vertical_threshold),
           horizontal_factor = 1.0 - std::abs(1.0 - horizontal / horizontal_threshold),
           angle_factor      = std::abs(1.0 - angle / angle_threshold); //75.0

    return (angle_factor * length) + (horizontal_factor * horizontal) + (vertical_factor * vertical);
}

double wayfinding::line_detection::imageMetric(const std::vector<cv::Vec4i>& lines, const cv::Size& image_size, double angle_threshold) {
    const int vertical_threshold   = 2 * image_size.height / 3,
              horizontal_threshold = image_size.width / 2;

    int x_0, y_0,
        x_1, y_1;
    //dx == Ankathete; dy == Gegenkathete
    int dx, dy, //difference
        cx, cy; //center

    int vertical;
    int horizontal;
    double length;
    double angle;

    double metric_counter = 0;
    
    for (cv::Vec4i line: lines) {
        x_0 = line[0]; y_0 = line[1];
        x_1 = line[2]; y_1 = line[3];

        dx = x_1 - x_0; dy = y_1 - y_0;
        cx = (x_0 + x_1) / 2; cy = (y_0 + y_1) / 2;
        
        // vertical    = std::abs(vertical_threshold   - cy);
        vertical        = cy;
        // horizontal  = std::abs(horizontal_threshold - cx);
        if (cx < horizontal_threshold) {
            horizontal  = cx;
        } else {
            horizontal  = image_size.width - cx;
        }
        length          = std::sqrt(dx * dx + dy * dy);
        angle           = ::getNormalizedAngle(dx, dy) * 180 / M_PI; // unit: degrees [°]

        metric_counter += ::lineMetric(angle, length, vertical, horizontal, angle_threshold, vertical_threshold, horizontal_threshold);
    }

    return metric_counter;
}

wayfinding::line_detection::parameters_t wayfinding::line_detection::getParametersFromMetric(double metric) {
    wayfinding::line_detection::parameters_t return_parameters;

    if (metric < 5'000) {
        return_parameters.filter_type = FilterType::NO_FILTER;
    } else if (metric > 100'000) {
        return_parameters.filter_type = FilterType::GAUSS;
    }

    return return_parameters;
}

void wayfinding::line_detection::drawLines(cv::InputOutputArray img, const std::vector<cv::Vec4i>& lines, const cv::Scalar& color) {
    for (cv::Vec4i line: lines) {
        cv::line(img, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), color);
    }
}

/**
 * Get the x coordinate of the intersection point on a line
 * between A = (x_0, y_0) and  B = (x_1, y_1) with given y.
 * 
 * @param x_0: x coordinate of point A
 * @param y_0: y coordinate of point A
 * @param x_1: x coordinate of point B
 * @param y_1: y coordinate of point B
 * @param intersection_y: y coordinate of intersection point
 * 
 * @return x coordinate of intersection point
*/
int getLineIntersectionX(int x_0, int y_0, int x_1, int y_1, int intersection_y) {
    /**
     * given: line from A = (x_0, y_0) to B = (x_1, y_0) representable as C = A + t * (B - A)
     *        where C = (intersection_x, intersection_y)
     * wanted: intersection_y
     * solution:
     *  intersection_y = y_0 + t * (y_1 - y_0)
     *  => t = (intersection_y - y_0) / (y_1 - y_0)
     *  intersection_x = x_0 + t * (x_1 - x_0) with t from above
    */

    double t = static_cast<double>(intersection_y - y_0) / (y_1 - y_0);
    return x_0 + t * (x_1 - x_0);
}

std::pair<int, int> wayfinding::line_detection::getLeftRightDistance(const std::vector<cv::Vec4i>& lines, int scan_height, int width) {
    int left = -1, right = width + 1;
    const int middle = width / 2;
    
    int x_0, y_0,
        x_1, y_1;
    int min_y, max_y;
    int intersection_x;

    for (cv::Vec4i line: lines) {
        y_0 = line[1]; y_1 = line[3];
        //since we can't be sure that (y_0 < y_1) applies:
        min_y = std::min(y_0, y_1); max_y = std::max(y_0, y_1);

        if (min_y <= scan_height || scan_height <= max_y) { //check if the lines crosses the scan_line (horizontal @ y = scan_height)
            x_0 = line[0]; x_1 = line[2];
            intersection_x = ::getLineIntersectionX(x_0, y_0, x_1, y_1, scan_height);

            if (intersection_x < middle && intersection_x > left) { //valid left value if left of middle and greater (more to the right) then the last left
                left = intersection_x;
            } else if (intersection_x >= middle && intersection_x < right) { //valid right if right of middle and lower (more to the left) then the last right
                right = intersection_x;
            }
        }
    }

    //if left didn't ever get any valid values it will still be -1 (a non-reachable value)
    //if right didn't ever get any valid values it will still be [width + 1] (a non-reachable value)
    //to signal that a side didn't get a valid value it will be set to -1, thus a non-valid right value will also need to be set to -1
    if (right == width + 1) {
        right = -1;
    }

    return std::make_pair(left, right);
}