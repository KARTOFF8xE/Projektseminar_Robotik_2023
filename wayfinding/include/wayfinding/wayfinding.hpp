#pragma once

#include "opencv2/opencv.hpp"

#include <vector>

//DISCLAIMER: Empty lines in the parameter description lines indicate that the following parameters are from used methods.
//            Therefore their descriptions are taken from the respective method documentations and marked with [<function name>, ..].

namespace wayfinding {
    namespace line_detection {
        //filer type enumeration
        enum FilterType {
            MEDIAN,
            NORM_BOX,
            BILATERAL,
            BOX,
            GAUSS
        };

        //struct for predefiend canny/hough parameters
        struct parameters_t {
            double threshold1;
            double threshold2;
            int apertureSize;
            int threshold;
            double min_line_length;
            double max_line_gap;

            //both resolutions probably won't need to be changed to they have default values for ease of initialization
            double rho = 1;
            double theta = M_PI / 180;
        };

        //TODO: Da müssten noch Werte rein, aber die Frage ist: welche?
        // const parameters_t GENERAL_LINE_DET_PARAMS{
            
        // };

        /**
         * Filters the image to be used in hough line detection.
         *
         * @param src: source of original image
         * @param dst: destination for filtered image
         * @param filter: desired filter type
         * 
         * @param ksize: blurring kernel size [medianBlur, blur, boxFilter, GaussianBlur]
         * @param sigma_color: Filter sigma in the color space. A larger value of the parameter means that farther colors within the pixel neighborhood (see sigmaSpace) will be mixed together, resulting in larger areas of semi-equal color. [bilateralFilter]
         * @param sigma_space: Filter sigma in the coordinate space. A larger value of the parameter means that farther pixels will influence each other as long as their colors are close enough (see sigmaColor ). When d>0, it specifies the neighborhood size regardless of sigmaSpace. Otherwise, d is proportional to sigmaSpace. [bilateralFilter]
        */
        void preFilter(
            cv::InputArray src,
            cv::OutputArray dst,
            FilterType filter,
            int ksize,
            double sigma_color,
            double sigma_space
        );
        
        /**
         * Apply Canny edge detection algorithm and uses this
         * output to line segements via probabilistic Hough tranfsform.
         * @brief asd
         * 
         * @param src: source of original image
         * @param dst: destination for canny image
         * 
         * @param parameters: parameter struct holding predefined parameter values
         * 
         * @return vector of detected lines
        */
        std::vector<cv::Vec4i> getHoughLines(
            cv::InputArray src,
            parameters_t parameters
        );

        /**
         * Apply Canny edge detection algorithm and uses this
         * output to line segements via probabilistic Hough tranfsform.
         * 
         * @param src: source of original image
         * @param dst: destination for canny image
         * 
         * @param threshold1: first threshold for the hysteresis procedure. [Canny]
         * @param threshold2: second threshold for the hysteresis procedure. [Canny]
         * @param apertureSize: aperture size for the Sobel operator. [Canny]
         * @param rho: Distance resolution of the accumulator in pixels. [HoughLinesP]
         * @param theta: Angle resolution of the accumulator in radians. [HoughLinesP]
         * @param threshold: Accumulator threshold parameter. Only those lines are returned that get enough votes [HoughLinesP]
         * @param min_line_length: Minimum line length. Line segments shorter than that are rejected. [HoughLinesP]
         * @param max_line_gap: Maximum allowed gap between points on the same line to link them. [HougLinesP]
         * 
         * @return vector of detected lines
        */
        std::vector<cv::Vec4i> getHoughLines(
            cv::InputArray src,
            double threshold1,
            double threshold2,
            int apertureSize,
            double rho,
            double theta,
            int threshold,
            double min_line_length,
            double max_line_gap
        );
    }

    namespace top_down {
        /**
         * Calculate vanishing point from camera matrix and angle.
         * 
         * @param K: intrinsic camera matrix 
         * @param pan: pan angle of camera
         * @param tilt: tilt angle of camera
         * 
         * @return vanishing point
        */
        cv::Point2d get_vanishing_point(
            const cv::Mat& K,
            double pan,
            double tilt
        );

        /**
         * Calculate trapeze and transformation matrix M for perspective transformation into top town view.
         * 
         * @param M: transformation matrix
         * @param points: top_left, top_right, bottom_left, bottom_right
         * @param image_size: source image size
         * @param vanishing_point: predetermined vanishing point in which image lines should cross
         * @param rel_upper_line_pos: position of the upper horizontal trapeze line as a relative value between the vanishing point (0.0) and the image bottom (1.0)
         * 
         * @return if a valid transformation was found
        */
        bool get_transformation(
            cv::Mat& M,
            std::vector<cv::Point2i>& points,
            const cv::Size& image_size,
            const cv::Point2i& vanishing_point,
            double rel_upper_line_pos = .2
        );

        /**
         * Calculate transformation matrix M for perspective transformation into top town view.
         * 
         * @param M: transformation matrix
         * @param vanishing_point: predetermined vanishing point in which image lines should cross
         * @param rel_upper_line_pos: position of the upper horizontal trapeze line as a relative value between the vanishing point (0.0) and the image bottom (1.0)
         * 
         * @return if a valid transformation was found
        */
        bool get_transformation(
            cv::Mat& M,
            const cv::Size& image_size,
            const cv::Point2i& vanishing_point,
            double rel_upper_line_pos = .2
        );

        /**
         * Draw lines from the bottom left and right to the vanishing point.
         * 
         * @param img: image to be drawn on
         * @param vanishing_point: predetermined vanishing point in which image lines should cross
        */
        void draw_vanishing_lines(
            cv::InputOutputArray img,
            const cv::Point2i& vanishing_point,
            const std::vector<cv::Point2i>& trapeze
        );

        /**
         * Calculate transformation matrix from view window trapeze and apply transformation.
         * 
         * @param src: input image (unchanged)
         * @param dst: output image (content gets overwritten)
         * @param M: transformation matrix
         * @param dst_size: (new) size of output image
        */
        void transform_to_top_down(
            cv::InputArray src,
            cv::OutputArray dst,
            cv::InputArray M,
            const cv::Size& dst_size
        );
    }
}