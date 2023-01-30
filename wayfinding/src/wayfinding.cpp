#include "wayfinding.hpp"

#include <cmath>

/* TODO:
**  - grobe/allg. Hough-Trafo zum generieren von Linien
**  - Auswertungsalgorithmus für Linien
**  - Planedetection
**  - switch case (?) welche Filter mit welchen Parametern genutzt werden
**  - Suggestion: dilate/erode auf Canny-Image anwenden? [EVTL] sinnvoll
**  -> allg. anwendbare Fkt.
*/

void wayfinding::line_detection::preFilter(cv::InputArray src, cv::OutputArray dst, wayfinding::line_detection::FilterType filter, int ksize, double sigma_color, double sigma_space) {
    //TODO: evtl sollte auch gleich nen struct mit den Parametern returned werden? struct dafür existiert bereits
    switch (filter) {
        case wayfinding::line_detection::FilterType::MEDIAN:    //Median Filter
            cv::medianBlur(src, dst, ksize);
            break;
        case wayfinding::line_detection::FilterType::NORM_BOX:  //Normalized Box Filter
            cv::blur(src, dst, cv::Size(ksize, ksize));
            break;
        case wayfinding::line_detection::FilterType::BILATERAL: //Bilateral Filter
            cv::bilateralFilter(src, dst, 5, sigma_color, sigma_space);
            break;
        case wayfinding::line_detection::FilterType::BOX:       //Box Filter
            cv::boxFilter(src, dst, -1, cv::Size(ksize, ksize));
            break;
        case wayfinding::line_detection::FilterType::GAUSS:     //Gaussian Filter
            cv::GaussianBlur(src, dst, cv::Size(ksize, ksize), 1.5, 2);
            break;
    }
}

std::vector<cv::Vec4i> wayfinding::line_detection::getHoughLines(cv::InputArray src, wayfinding::line_detection::parameters_t parameters) {
    return wayfinding::line_detection::getHoughLines(src, parameters.threshold1, parameters.threshold2, parameters.apertureSize, parameters.rho, parameters.theta, parameters.threshold, parameters.min_line_length, parameters.max_line_gap);
}

std::vector<cv::Vec4i> wayfinding::line_detection::getHoughLines(cv::InputArray src, double threshold1, double threshold2, int apertureSize, double rho, double theta, int threshold, double min_line_length, double max_line_gap) {
    cv::Mat canny_mat;
    std::vector<cv::Vec4i> lines;

    //create contour image
    cv::Canny(src, canny_mat, threshold1, threshold2, apertureSize);
    //heuristically identify lines in contour image
    cv::HoughLinesP(canny_mat, lines, 1, M_PI / 180, threshold, min_line_length, max_line_gap);

    return lines;
}


cv::Point2d wayfinding::top_down::get_vanishing_point(const cv::Mat& K, double pan, double tilt) {
    /**
     * camera rotation matrix:
     *  R = [r_1, r_2, r_3]
     *  r_3 = [sin(pan)sin(tilt); cos(tilt); cos(pan)sin(tilt)]
     * 
     * v = K * r_3
     * 
     *      z
     *      |
     *      |  /|
     * tilt.|_/ |
     *      |/__|____y
     * pan./_\  | /
     *    /   \ |/
     *   /_____\/
     * x/
    */

   cv::Vec3d r_3{std::sin(pan) * std::sin(tilt), std::cos(tilt), std::cos(pan) * std::sin(tilt)};

   cv::Mat vanishing_point = K * cv::Mat(r_3);

   return cv::Point2d(vanishing_point.at<double>(0, 0), vanishing_point.at<double>(1, 0));
}

bool wayfinding::top_down::get_transformation(cv::Mat& M, std::vector<cv::Point2i>& points, const cv::Size& image_size, const cv::Point2i& vanishing_point, double rel_upper_line_pos) {
    /** Concept:
     * unkown: x
     * known: A, B, y
     * 
     * A -> B ?= [x, y]
     * A + r * (B - A) ?= [x, y]
     * 
     * yA + r * (yB - yA) = y
     * r = (y - yA) / (yB - yA)
     * x = xA + r * (xB - xA)
     * 
     * here:
     *  A = bottom_left/bottom_right
     *  B = vanishing_point
     */

    //move bottom left and bottom right further out of the window to cover a wider window
    const int offset = 3 * vanishing_point.x;
    const cv::Point2i bottom_left(0 - offset, image_size.height),
                      bottom_right(image_size.width + offset, image_size.height);

    //assumes the vanishing point is close enough so this still lies within the image
    const double y = vanishing_point.y + (image_size.height - vanishing_point.y) * rel_upper_line_pos;
    const double y_rounded = std::round(y);

    //left line
    double r_left = (y - bottom_left.y) / (vanishing_point.y - bottom_left.y);
    cv::Point2i top_left(std::round(bottom_left.x + r_left * (vanishing_point.x - bottom_left.x)), y_rounded);

    //right line
    double r_right = (y - bottom_right.y) / (vanishing_point.y - bottom_right.y);
    cv::Point2i top_right(std::round(bottom_right.x + r_right * (vanishing_point.x - bottom_right.x)), y_rounded);

    points = std::vector<cv::Point2i>{top_left, top_right, bottom_left, bottom_right};


    //get transformation matrix
    M = cv::findHomography(
        points,
        std::vector<cv::Point2i>{
            cv::Point2i(0, 0), cv::Point2i(image_size.width, 0),
            cv::Point2i(0, image_size.height), cv::Point2i(image_size.width, image_size.height)
        }
    );
    
    return !M.empty();
}

bool wayfinding::top_down::get_transformation(cv::Mat& M, const cv::Size& image_size, const cv::Point2i& vanishing_point, double rel_upper_line_pos) {
    //move bottom left and bottom right further out of the window to cover a wider window
    const int offset = 2 * vanishing_point.x;
    const cv::Point2i bottom_left(0 - offset, image_size.height),
                      bottom_right(image_size.width + offset, image_size.height);

    //assumes the vanishing point is close enough so this still lies within the image
    const double y = vanishing_point.y + (image_size.height - vanishing_point.y) * rel_upper_line_pos;
    const double y_rounded = std::round(y);

    //left line
    const double r_left = (y - bottom_left.y) / (vanishing_point.y - bottom_left.y);
    cv::Point2i top_left(std::round(bottom_left.x + r_left * (vanishing_point.x - bottom_left.x)), y_rounded);

    //right line
    const double r_right = (y - bottom_right.y) / (vanishing_point.y - bottom_right.y);
    cv::Point2i top_right(std::round(bottom_right.x + r_right * (vanishing_point.x - bottom_right.x)), y_rounded);


    //get transformation matrix
    M = cv::findHomography(
        std::vector<cv::Point2i>{
            top_left,       top_right,
            bottom_left,    bottom_right
        },
        std::vector<cv::Point2i>{
            cv::Point2i(0, 0),                  cv::Point2i(image_size.width, 0),
            cv::Point2i(0, image_size.height),  cv::Point2i(image_size.width, image_size.height)
        }
    );
    
    return !M.empty();
}

void wayfinding::top_down::draw_vanishing_lines(cv::InputOutputArray img, const cv::Point2i& vanishing_point, const std::vector<cv::Point2i>& trapeze) {
    const cv::Scalar red(0, 0, 255),
                     green(0, 255, 0);
    cv::Point2i top_left = trapeze[0],    top_right = trapeze[1],
                bottom_left = trapeze[2], bottom_right = trapeze[3];

    cv::line(img, bottom_left,  vanishing_point, red, 3);
    cv::line(img, bottom_right, vanishing_point, red, 3);

    cv::line(img, bottom_left,  bottom_right,   green, 3);
    cv::line(img, bottom_left,  top_left,       green, 3);
    cv::line(img, bottom_right, top_right,      green, 3);
    cv::line(img, top_left,     top_right,      green, 3);
}

void wayfinding::top_down::transform_to_top_down(cv::InputArray src, cv::OutputArray dst, cv::InputArray M, const cv::Size& dst_size) {
    cv::warpPerspective(src, dst, M, dst_size, cv::INTER_NEAREST);
}