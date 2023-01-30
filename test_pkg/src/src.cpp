#include "src.hpp"

#include <cmath>
#include <optional>
#include <random>

cv::Point2d src::get_vanishing_point(const cv::Mat& K, double pan, double tilt) {
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

std::vector<cv::Point2i> src::get_transformation_points(const cv::Size& image_size, const cv::Point2i& vanishing_point, double rel_upper_line_pos) {
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
    const int offset = 2 * vanishing_point.x;
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

    return std::vector<cv::Point2i>{top_left, top_right, bottom_left, bottom_right};
}

void src::draw_vanishing_lines(cv::InputOutputArray img, const cv::Point2i& vanishing_point, const std::vector<cv::Point2i>& trapeze) {
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

bool src::transform_to_top_down(cv::InputArray src, cv::OutputArray dst, cv::InputArray src_points, const cv::Size& dst_size) {  
    //get transformation matrix
    cv::Mat transformation_matrix = cv::findHomography(
        src_points,
        std::vector<cv::Point2i>{
            cv::Point2i(0, 0), cv::Point2i(dst_size.width, 0),
            cv::Point2i(0, dst_size.height), cv::Point2i(dst_size.width, dst_size.height)
        }
    );
    
    if (transformation_matrix.empty()) {
        return false;
    }
    
    cv::warpPerspective(src, dst, transformation_matrix, dst_size, cv::INTER_NEAREST);

    return true;
}