/**
 * @file Vis.cpp
 * @author Duncan Hamill (duncanrhamill@googlemail.com)
 * @brief Visualisation functions
 * @version 0.1
 * @date 2021-07-07
 * 
 * @copyright Copyright (c) 2021
 */

#include "Vis.hpp"
#include "../filtering/Filtering.hpp"

#include <opencv4/opencv2/imgproc.hpp>

cv::Mat get_vis_depth_img(cv::Mat &depth_img_in, uint16_t max_depth_mm_in) {
    cv::Mat vis_img;

    // Calculate max depth if not specified
    if (max_depth_mm_in == 0) {
        double max;
        cv::minMaxIdx(depth_img_in, NULL, &max);
        max_depth_mm_in = (uint16_t)max;
    }

    // Calculate scaling value
    double scaling = 255.0 / ((double)max_depth_mm_in);

    depth_img_in.convertTo(vis_img, CV_8UC1, scaling);
    cv::applyColorMap(vis_img, vis_img, cv::COLORMAP_VIRIDIS);
    return vis_img;
}

cv::Mat get_vis_shadows(cv::Mat &source_img_in, cv::Mat &depth_img_in) {
    cv::Mat zero_depth, vis_img;

    // First get the zero depth mask
    zero_depth = get_zero_depth_mask(depth_img_in);

    // Then convert both the mask and the left image to RGB so we can draw red
    // onto it where there is zero depth.
    cv::cvtColor(source_img_in, vis_img, cv::COLOR_GRAY2BGR);
    cv::cvtColor(zero_depth, zero_depth, cv::COLOR_GRAY2BGR);

    // Make the mask red where it has value
    zero_depth = (zero_depth - cv::Scalar(0, 0, 255)) / 2;
    
    // Highlight
    vis_img = vis_img - zero_depth;

    return vis_img;
}