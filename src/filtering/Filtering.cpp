/**
 * @file Filtering.cpp
 * @author Duncan Hamill (duncanrhamill@googlemail.com)
 * @brief Depth image filtering
 * @version 0.1
 * @date 2021-07-07
 * 
 * @copyright Copyright (c) 2021
 */

#include <iostream>

#include "Filtering.hpp"

#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/ximgproc.hpp>

cv::Mat get_zero_depth_mask(cv::Mat &depth_img_in) {
    cv::Mat mask;

    cv::threshold(depth_img_in, mask, 1.0, 255.0, cv::THRESH_BINARY_INV);
    mask.convertTo(mask, CV_8UC1);

    return mask;
}

void dilate_depth_img(cv::Mat &depth_img_inout) {
    cv::Mat mask, kernel;
    
    get_zero_depth_mask(depth_img_inout).convertTo(mask, CV_16UC1);
    kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

    cv::dilate(mask, mask, kernel);

    mask = UINT16_MAX - mask;

    cv::bitwise_and(depth_img_inout, mask, depth_img_inout);
}

const std::string DepthFilter::vis_window_name = "depth (filtered)";

DepthFilter::DepthFilter() {
    wls_filter = cv::ximgproc::createDisparityWLSFilterGeneric(false);
}

DepthFilter::DepthFilter(bool enable_vis) {
    wls_filter = cv::ximgproc::createDisparityWLSFilterGeneric(false);

    if (enable_vis) {
        cv::namedWindow(vis_window_name, cv::WINDOW_AUTOSIZE);
        cv::createTrackbar("lambda", vis_window_name, NULL, (int)lambda * 2, on_trackbar_lambda_change, this);
        cv::createTrackbar("sigma", vis_window_name, NULL, (int)(sigma * 10.0) * 2, on_trackbar_sigma_change, this);
    }
}

DepthFilter::~DepthFilter() {

}

void DepthFilter::filter(ImgFrame &img_frame_inout, double depth_scale_factor) {

    // For now we have to use the right image since the disparity is
    // right-aligned 
    wls_filter->filter(
        img_frame_inout.disp_img, 
        img_frame_inout.left_img, 
        img_frame_inout.disp_img
    );

    // Calculate depth
    img_frame_inout.disp_img.convertTo(img_frame_inout.depth_img, CV_16UC1);
    img_frame_inout.depth_img = (depth_scale_factor / img_frame_inout.depth_img);
}

void DepthFilter::set_wls_params(double lambda_in, double sigma_in) {
    cv::setTrackbarPos("lambda", vis_window_name, (int)lambda);
    cv::setTrackbarPos("sigma", vis_window_name, (int)(sigma * 10.0));

    wls_filter->setLambda(lambda_in);
    wls_filter->setSigmaColor(sigma_in);
}