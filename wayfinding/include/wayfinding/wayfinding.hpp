#pragma once

#include "opencv2/opencv.hpp"

#include <vector>
#include <utility>

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
            GAUSS,

            NO_FILTER
        };

        //TODO: CONTINUE
        // thresholds canny
        // line length und line gap
        // filter

        //struct for predefiend canny/hough parameters
        //default values represent universal hough parameters for metric generation
        //both resolutions (rho, theta) probably won't need to be changed to they have default values for ease of initialization
        struct parameters_t {
            //TODO: Warum eigentlich nicht ungefiltert?
            FilterType filter_type  = FilterType::MEDIAN;
            int kernel_size         = 3;
            double center_width     = 20.0;

            double sigma            = .33;
            int apertureSize        = 3;
            int threshold           = 100;
            double min_line_length  = 75;
            double max_line_gap     = 50;

            double rho = 1;
            double theta = M_PI / 180;
        };

        /**
         * Filters the image to be used in hough line detection.
         *
         * @param src: source of original image
         * @param dst: destination for filtered image
         * @param filter: desired filter type
         * 
         * @param ksize: blurring kernel size [medianBlur, blur, boxFilter, GaussianBlur]
        */
        void preFilter(
            cv::InputArray src,
            cv::OutputArray dst,
            FilterType filter,
            int ksize
        );
        
        /**
         * Apply Canny edge detection algorithm and uses this
         * output to line segements via probabilistic Hough tranfsform.
         * Also applies defined filter with specified kernel size beforehand.
         * 
         * @param src: source of original image
         * @param dst: destination for vector of detected lines
         * @param parameters: parameter struct holding predefined parameter values
        */
        cv::Mat getHoughLines(
            cv::Mat& src,
            std::vector<cv::Vec4i>& dst,
            parameters_t parameters
        );

        /**
         * Apply Canny edge detection algorithm and uses this
         * output to line segements via probabilistic Hough tranfsform.
         * 
         * @param src: source of original image
         * @param dst: destination for vector of detected lines
         * @param threshold1: first threshold for the hysteresis procedure. [Canny]
         * @param threshold2: second threshold for the hysteresis procedure. [Canny]
         * @param apertureSize: aperture size for the Sobel operator. [Canny]
         * @param rho: Distance resolution of the accumulator in pixels. [HoughLinesP]
         * @param theta: Angle resolution of the accumulator in radians. [HoughLinesP]
         * @param threshold: Accumulator threshold parameter. Only those lines are returned that get enough votes [HoughLinesP]
         * @param min_line_length: Minimum line length. Line segments shorter than that are rejected. [HoughLinesP]
         * @param max_line_gap: Maximum allowed gap between points on the same line to link them. [HougLinesP]
        */
        cv::Mat getHoughLines(
            cv::InputArray src,
            std::vector<cv::Vec4i>& dst,
            double sigma,
            int apertureSize,
            double rho,
            double theta,
            int threshold,
            double min_line_length,
            double max_line_gap
        );

        /**
         * Filter out lines that are within a certain threshold in the image center (tube).
         * 
         * @param src: vector of all detected lines
         * @param dst: (empty) destination for filtered lines
         * @param width: image width [px]
         * @param rel_center_width: relative width of the filter tube in percent [%]
         * @param angle: minimal allowed angle [°]
        */
        void lineFilter(
            const std::vector<cv::Vec4i>& src,
            std::vector<cv::Vec4i>& dst,
            uint width,
            double rel_center_width,
            double angle
        );

        /**
         * Calculate a "quality" value for image by the detected lines.
         * (higher means worse)
         * 
         * @param lines: detected lines
         * @param image_size: size of source image
         * 
         * @return rating value
        */
        double imageMetric(
            const std::vector<cv::Vec4i>& lines,
            const cv::Size& image_size,
            double angle_threshold
        );

        /**
         * Return a parameter set corresponding to the images metric.
         * 
         * @param metric
         * 
         * @return parameter set
        */
        parameters_t getParametersFromMetric(
            double metric
        );

        /**
         * Draw lines on image with given color.
         * 
         * @param img: image to be drawn on
         * @param lines: lines to be drawn on image
         * @param color: color of the lines to be drawn
        */
        void drawLines(
            cv::InputOutputArray img,
            const std::vector<cv::Vec4i>& lines,
            const cv::Scalar& color
        );

        /**
         * Extract distance from center to left/right side from detected lines.
         * If either side equals -1, then no value was found for this side.
         * 
         * @param lines: prefiltered, detected image lines
         * @param scan_height: level at which to scan for distances [px]
         * 
         * @return pair of left (first) and right (second) distances
        */
        std::pair<int, int> getLeftRightDistance(
            const std::vector<cv::Vec4i>& lines,
            int scan_height,
            int width
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
        cv::Point2d getVanishingPoint(
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
        bool getTransformation(
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
        bool getTransformation(
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
        void drawVanishingLines(
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
        void transformToTopDown(
            cv::InputArray src,
            cv::OutputArray dst,
            cv::InputArray M,
            const cv::Size& dst_size
        );

        /**
         * Transform point in warped perspective to point in original perspective.
         * 
         * @param M: transformation matrix
         * @param point: warped to be transformed
         * 
         * @return "unwarped" point
        */
        cv::Point2i unwarpPoint(
            const cv::Mat& M,
            const cv::Point2i& point
        );
    }
}