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
    cv::Point2d get_vanishing_point(
        const cv::Mat& K,
        double pan,
        double tilt
    );

    /**
     * Calculate trapeze for perspective transformation into top town view.
     * 
     * @param image_size: source image size
     * @param vanishing_point: predetermined vanishing point in which image lines should cross
     * @param rel_upper_line_pos: position of the upper horizontal trapeze line as a relative value between the vanishing point (0.0) and the image bottom (1.0)
     * 
     * @return top_left, top_right, bottom_left, bottom_right
    */
    std::vector<cv::Point2i> get_transformation_points(
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
     * @param src_points: 4 points specifying a view window (trapeze) to be transformed to top down
     * @param dst_size: (new) size of output image
     * 
     * return: if a valid  transformation could be 
    */
    bool transform_to_top_down(
        cv::InputArray src,
        cv::OutputArray dst,
        cv::InputArray src_points,
        const cv::Size& dst_size
    );
}