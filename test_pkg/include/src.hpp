#pragma once

#include "opencv2/opencv.hpp"

namespace src {
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