#include "src.hpp"

#include <cmath>

cv::Point2d src::getVanishingPoint(const cv::Mat& K, double pan, double tilt) {
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

bool src::getTransformation(cv::Mat& M, std::vector<cv::Point2i>& points, const cv::Size& image_size, const cv::Point2i& vanishing_point, double rel_upper_line_pos) {
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

bool src::getTransformation(cv::Mat& M, const cv::Size& image_size, const cv::Point2i& vanishing_point, double rel_upper_line_pos) {
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

void src::drawVanishingLines(cv::InputOutputArray img, const cv::Point2i& vanishing_point, const std::vector<cv::Point2i>& trapeze) {
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

void src::transformToTopDown(cv::InputArray src, cv::OutputArray dst, cv::InputArray M, const cv::Size& dst_size) {
    cv::warpPerspective(src, dst, M, dst_size, cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(255, 255, 255, 0));
}

cv::Point2i src::warpPoint(const cv::Mat& transformation_matrix, const cv::Point2i& point) {
    cv::Vec3d normal_point{static_cast<double>(point.x), static_cast<double>(point.y), 1.0};
    cv::Mat warped_point = transformation_matrix * normal_point;

    double denominator = warped_point.at<double>(2, 0);
    return cv::Point2i(warped_point.at<double>(0, 0) / denominator, warped_point.at<double>(1, 0) / denominator);
}

cv::Point2i src::unwarpPoint(const cv::Mat& transformation_matrix, const cv::Point2i& point) {
    cv::Vec3d warped_point{static_cast<double>(point.x), static_cast<double>(point.y), 1.0};
    cv::Mat normal_point = transformation_matrix.inv() * warped_point;

    double denominator = normal_point.at<double>(2, 0);
    return cv::Point2i(normal_point.at<double>(0, 0) / denominator, normal_point.at<double>(1, 0) / denominator);
}

double src::getPpmFromImage(const cv::Mat& img, const cv::Mat& pointcloud, const cv::Point2d& vanishing_point) {
    const cv::Point2i pt1(vanishing_point.x, img.rows),
                      pt2(vanishing_point.x, (vanishing_point.y + 2.0 * img.rows) / 3.0); //y + 2.0 * (h - y) / 3.0);

    cv::Vec3f pt1_3d = pointcloud.at<cv::Vec3f>(pt1),
              pt2_3d = pointcloud.at<cv::Vec3f>(pt2);

    //TODO: Evtl. z bzw. Höhe ignorieren?
    double distance_meters = cv::norm(pt1_3d - pt2_3d);
    double distance_pixels = std::abs(pt1.y  - pt2.y);

    return distance_pixels / distance_meters;
}