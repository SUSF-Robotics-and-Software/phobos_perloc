/**
 * @file OakdDataSource.cpp
 * @author Duncan Hamill (duncanrhamill@googlemail.com)
 * @brief Data source implementation for an OAK-D
 * @version 0.1
 * @date 2021-07-07
 * 
 * @copyright Copyright (c) 2021
 */

#include <stdexcept>

#include "OakdDataSource.hpp"

#define OAKD_DATA_SOURCE_PARAMS_PATH "params/oakd_data_source.json"

OakdDataSource::OakdDataSource() {
    // Load parameters
    cv::FileStorage fs;
    fs.open(OAKD_DATA_SOURCE_PARAMS_PATH, cv::FileStorage::READ);

    if (!fs.isOpened()) {
        throw std::runtime_error(
            std::string("Couldn't open OAK-D data source params file at ") 
            + OAKD_DATA_SOURCE_PARAMS_PATH
        );
    }

    fs["OakdDataSourceParams"] >> params;

    // Create depth filter
    depth_filter = DepthFilter();

    // Create pipeline
    p_pipeline = new dai::Pipeline();

    // Create sources
    left_node = p_pipeline->create<dai::node::MonoCamera>();
    right_node = p_pipeline->create<dai::node::MonoCamera>();
    stereo_node = p_pipeline->create<dai::node::StereoDepth>();

    // Setup sources
    left_node->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    left_node->setBoardSocket(dai::CameraBoardSocket::LEFT);

    right_node->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    right_node->setBoardSocket(dai::CameraBoardSocket::RIGHT);

    stereo_node->setConfidenceThreshold(params.stereo_depth_confidence_threshold);
    stereo_node->setMedianFilter(params.stereo_depth_median_filter);
    stereo_node->setLeftRightCheck(params.stereo_depth_left_right_check);
    stereo_node->setExtendedDisparity(params.stereo_depth_extended_disparity);
    stereo_node->setSubpixel(params.stereo_depth_subpixel);
    stereo_node->setRectifyMirrorFrame(true);
    // TODO: when added to depthai-core :(
    // stereo_node->setDepthAlign(dai::CameraBoardSocket::LEFT);
    
    // Create outputs
    left_out = p_pipeline->create<dai::node::XLinkOut>();
    right_out = p_pipeline->create<dai::node::XLinkOut>();
    disp_out = p_pipeline->create<dai::node::XLinkOut>();

    // Set output data
    left_out->setStreamName("left");
    right_out->setStreamName("right");
    disp_out->setStreamName("disp");

    // Link sources to outputs
    left_node->out.link(stereo_node->left);
    stereo_node->rectifiedLeft.link(left_out->input);

    right_node->out.link(stereo_node->right);
    stereo_node->rectifiedRight.link(right_out->input);

    stereo_node->disparity.link(disp_out->input);

    // Connect to device and start pipeline
    p_device = new dai::Device(*p_pipeline);

    // Get output queues
    left_queue = p_device->getOutputQueue("left", 4, false);
    right_queue = p_device->getOutputQueue("right", 4, false);
    disp_queue = p_device->getOutputQueue("disp", 4, false);

    // Set the depth scale factor
    //   baseline_mm * width_pixels / (2 * tan(fov/2))
    depth_scale_factor 
        = params.baseline_mm * 640.0 
        / (2.0 * tan(params.field_of_view_rad / 2.0));
    depth_filter.set_wls_params(
        params.stereo_wls_lambda, 
        params.stereo_wls_sigma
    );
}

OakdDataSource::~OakdDataSource() {

}

void OakdDataSource::get_img_frame(ImgFrame &img_frame_out) {
    auto in_left = left_queue->get<dai::ImgFrame>();
    auto in_right = right_queue->get<dai::ImgFrame>();
    auto in_disp = disp_queue->get<dai::ImgFrame>();

    img_frame_out.left_img = in_left->getFrame(true);
    img_frame_out.right_img = in_right->getFrame(true);
    img_frame_out.disp_img = in_disp->getFrame(true);

    // Filter the frame to calculate the depth image
    depth_filter.filter(img_frame_out, depth_scale_factor);
}

bool OakdDataSource::get_img_frame_nonblocking(ImgFrame &img_frame_out) {
    auto in_left = left_queue->tryGet<dai::ImgFrame>();
    auto in_right = right_queue->tryGet<dai::ImgFrame>();
    auto in_disp = disp_queue->tryGet<dai::ImgFrame>();

    if (in_left == nullptr || in_right == nullptr || in_disp == nullptr) {
        return false;
    }

    img_frame_out.left_img = in_left->getFrame(true);
    img_frame_out.right_img = in_right->getFrame(true);
    img_frame_out.disp_img = in_disp->getFrame(true);

    // Filter the frame to calculate the depth image
    depth_filter.filter(img_frame_out, depth_scale_factor);

    return true;
}

void OakdDataSource::get_imu_frames(std::vector<ImuData> &imu_frames_out) {

}

bool OakdDataSource::get_imu_frames_nonblocking(std::vector<ImuData> &imu_frames_out) {
    
}