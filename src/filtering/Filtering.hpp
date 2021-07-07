/**
 * @file Filtering.hpp
 * @author Duncan Hamill (duncanrhamill@googlemail.com)
 * @brief Depth image filtering
 * @version 0.1
 * @date 2021-07-07
 * 
 * @copyright Copyright (c) 2021
 */

#ifndef H_FILTERING_H
#define H_FILTERING_H

#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/ximgproc.hpp>

#include "../data_source/DataSource.hpp"

class DepthFilter {
    public:
        DepthFilter();
        ~DepthFilter();

        /**
         * @brief Filter the given ImgFrame and calculate the depth image.
         * 
         * @param img_frame_inout The frame to filter 
         * @param depth_scale_factor The mapping factor between disparity and
         * depth such that depth = depth_scale_factor / disparity.
         */
        void filter(ImgFrame &img_frame_inout, double depth_scale_factor);

        /**
         * @brief Set the parameters to be used for the WLS filter.
         * 
         * @param lambda_in WLS lambda
         * @param sigma_in WLS sigma
         */
        void set_wls_params(double lambda_in, double sigma_in);

    private:
        cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter;
};

/**
 * @brief Get a mask which selects all regions of zero depth from the given
 * depth image.
 * 
 * @param depth_img_in Depth image to sample
 * @return cv::Mat Mask which highlights zero depth regions
 */
cv::Mat get_zero_depth_mask(cv::Mat &depth_img_in);

/**
 * @brief Filter the depth image by dilating regions of zero depth.
 * 
 * @param depth_img_inout Depth image to modify
 */
void dilate_depth_img(cv::Mat &depth_img_inout);

#endif /* H_FILTERING_H */