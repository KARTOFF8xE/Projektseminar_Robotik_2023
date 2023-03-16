#include "visualize_path.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

void visualize_path::visualize_street_view(std::vector<filters::limit> limits_vec, double wheel_inside, double wheel_width, std::string window_name) {
    /**
     * The left Limit of the Curbstone is drawn in green
     * The right Limit of the Curbstone is drawn in red
     * The Edges of the wheels are drawn in yellow
     * 
     * For useful Visualization, the coordinate-System is getting Scaled, so that anything can be seen ("+ 5", that anything will be in the Window; "* 100", that anything is scaled useful)
    */

    /**
     * Creating a Window
     * If a left Limit is Valid, it will be Drawn with a green Point
     * Analog procedure for the Right Limit, just with red Points
     * The latest up to 1000 Limits will be printed (if Valid)
     * This will give a view of the Street from the Bird-Perspective
    */
    size_t i = 0;
    cv::Mat img(1000, 1000, CV_8UC3, cv::Scalar(0, 0, 0));
    while (i < 1000 && i < limits_vec.size()) {
        if (limits_vec[limits_vec.size() - i].left > 0) {
            cv::Point2d left((-(limits_vec[limits_vec.size() - i].left) + 5) * 100, i);
            cv::circle(img, left, 1, cv::Scalar(0, 255, 0));
        } else if (limits_vec[limits_vec.size() - i].left < 0) {
            cv::Point2d left((-(std::abs(limits_vec[limits_vec.size() - i].left)) + 5) * 100, i);
            cv::circle(img, left, 1, cv::Scalar(150, 150, 0));
        }
        if (limits_vec[limits_vec.size() - i].right > 0) {
            cv::Point2d right(((limits_vec[limits_vec.size() - i].right) + 5) * 100, i);
            cv::circle(img, right, 1, cv::Scalar(0, 0, 255));
        } else if (limits_vec[limits_vec.size() - i].right < 0) {
            cv::Point2d right(((std::abs(limits_vec[limits_vec.size() - i].right)) + 5) * 100, i);
            cv::circle(img, right, 1, cv::Scalar(150, 150, 0));
        }
        i++;
    }

    /**
     * For better (more advanced) visualization the Edges of the Wheels are drawn in Yellow
    */
    cv::Point2d left_wheel_top_left((-(wheel_width + wheel_inside) + 5) * 100, 0), left_wheel_top_right(((-wheel_inside) + 5) * 100, 0), left_wheel_bottom_left((-(wheel_width + wheel_inside) + 5) * 100, 1000), left_wheel_bottom_right((-wheel_inside + 5) * 100, 1000);
    cv::Point2d right_wheel_top_left(((wheel_inside) + 5) * 100, 0), right_wheel_top_right(((wheel_width + wheel_inside) + 5) * 100, 0), right_wheel_bottom_left((wheel_inside + 5) * 100, 1000), right_wheel_bottom_right(((wheel_width + wheel_inside) + 5) * 100, 1000);

    int thickness = 1;
    line(img, left_wheel_top_left, left_wheel_bottom_left, cv::Scalar(0, 255, 255), thickness);
    line(img, left_wheel_top_right, left_wheel_bottom_right, cv::Scalar(0, 255, 255), thickness);
    line(img, right_wheel_top_left, right_wheel_bottom_left, cv::Scalar(0, 255, 255), thickness);
    line(img, right_wheel_top_right, right_wheel_bottom_right, cv::Scalar(0, 255, 255), thickness);

    /**
     * Show the Window and close the entire launch, if "Esc" is pressed
    */
    imshow(window_name, img);
	if (cv::waitKey(10) == 27) { //exit on ESC
        exit(0);
    }
}