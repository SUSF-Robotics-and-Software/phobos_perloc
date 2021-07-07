/**
 * @file OakdDataSource.hpp
 * @author Duncan Hamill (duncanrhamill@googlemail.com)
 * @brief Data source implementation for an OAK-D
 * @version 0.1
 * @date 2021-07-07
 * 
 * @copyright Copyright (c) 2021
 */

#ifndef H_OAKDDATASOURCE_H
#define H_OAKDDATASOURCE_H

#include <iostream>
#include <memory>

#include <eigen3/Eigen/Dense>
#include <depthai/depthai.hpp>

#include "DataSource.hpp"
#include "../filtering/Filtering.hpp"

/**
 * @brief Parameters to be used by the OakdDataSource class.
 * 
 */
class OakdDataSourceParams {
    // Methods
    public:
        OakdDataSourceParams() {

        }

        void read(const cv::FileNode &node) {
            stereo_depth_confidence_threshold = (uint8_t)((int)node["stereo_depth_confidence_threshold"]);
            stereo_depth_median_filter = (dai::StereoDepthProperties::MedianFilter)((int)node["stereo_depth_median_filter"]);
            stereo_depth_left_right_check = (bool)((int)node["stereo_depth_left_right_check"]);
            stereo_depth_extended_disparity = (bool)((int)node["stereo_depth_extended_disparity"]);
            stereo_depth_subpixel = (bool)((int)node["stereo_depth_subpixel"]);
            baseline_mm = (double)node["baseline_mm"];
            field_of_view_rad = (double)node["field_of_view_rad"];
            stereo_wls_lambda = (double)node["stereo_wls_lambda"];
            stereo_wls_sigma = (double)node["stereo_wls_sigma"];
        }
    
    // Data 
    public:
        uint8_t stereo_depth_confidence_threshold;
        dai::StereoDepthProperties::MedianFilter stereo_depth_median_filter;
        bool stereo_depth_left_right_check;
        bool stereo_depth_extended_disparity;
        bool stereo_depth_subpixel;
        double baseline_mm;
        double field_of_view_rad;
        double stereo_wls_lambda;
        double stereo_wls_sigma;
};

class OakdDataSource: DataSource {
    public:
        OakdDataSource();
        ~OakdDataSource();

        void get_img_frame(ImgFrame &img_frame_out);
        bool get_img_frame_nonblocking(ImgFrame &img_frame_out);

        void get_imu_frames(std::vector<ImuData> &imu_frames_out);
        bool get_imu_frames_nonblocking(std::vector<ImuData> &imu_frames_out);

    private:
        OakdDataSourceParams params;
        dai::Pipeline *p_pipeline;
        dai::Device *p_device;

        // Sources
        std::shared_ptr<dai::node::MonoCamera> left_node;
        std::shared_ptr<dai::node::MonoCamera> right_node;
        std::shared_ptr<dai::node::StereoDepth> stereo_node;

        // Outputs
        std::shared_ptr<dai::node::XLinkOut> left_out;
        std::shared_ptr<dai::node::XLinkOut> right_out;
        std::shared_ptr<dai::node::XLinkOut> disp_out;

        // Queues
        std::shared_ptr<dai::DataOutputQueue> left_queue;
        std::shared_ptr<dai::DataOutputQueue> right_queue;
        std::shared_ptr<dai::DataOutputQueue> disp_queue;

        // Depth image filterer
        DepthFilter depth_filter;
        double depth_scale_factor;
};

static void read(const cv::FileNode& node, OakdDataSourceParams& x, const OakdDataSourceParams& default_value = OakdDataSourceParams()){
    if(node.empty())
        x = default_value;
    else
        x.read(node);
}

#endif /* H_OAKDDATASOURCE_H */