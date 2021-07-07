/**
 * @file DataSource.hpp
 * @author Duncan Hamill (duncanrhamill@googlemail.com)
 * @brief Generic data source class for perloc processing
 * @version 0.1
 * @date 2021-07-07
 * 
 * @copyright Copyright (c) 2021
 */

#ifndef H_DATASOURCE_H
#define H_DATASOURCE_H

#include <opencv4/opencv2/core.hpp>
#include <eigen3/Eigen/Dense>

using namespace Eigen;

/**
 * @brief Contains a single IMU data point
 */
struct ImuData {
    /**
     * @brief Acceleration in meters/second^2
     */
    Vector3d acc_mss;

    /**
     * @brief Rotation rate in radians/second
     */
    Vector3d rot_rads;
};

struct ImgFrame {
    /**
     * @brief The left image 
     */
    cv::Mat left_img;

    /**
     * @brief The right image
     */
    cv::Mat right_img;

    /**
     * @brief The disparity image
     */
    cv::Mat disp_img;

    /**
     * @brief The depth image wrt the left camera.
     */
    cv::Mat depth_img;
};

/**
 * @brief A generic data source that can produce images and imu data
 * 
 */
class DataSource {
    public:
        /**
         * @brief Get the latest image frame object.
         * 
         * This function will block until there is a frame available
         * 
         * @param img_frame_out The latest image frame
         */
        virtual void get_img_frame(ImgFrame &img_frame_out)=0;

        /**
         * @brief Attempt to get the latest image frame object
         * 
         * @param img_frame_out The latest image frame
         * @return true The frame was available and has been set
         * @return false No frame was available, img_frame_out is unchanged
         */
        virtual bool get_img_frame_nonblocking(ImgFrame &img_frame_out)=0;
        
        /**
         * @brief Get all IMU frames that have been recorded since the last
         * call to this function.
         * 
         * This function will block util there are frames available
         * 
         * @param imu_frames_out Vector of undelivered IMU frames
         */
        virtual void get_imu_frames(std::vector<ImuData> &imu_frames_out)=0;

        /**
         * @brief Get all IMU frames that have been recorded since the last
         * call to this function without blocking the current thread.
         * 
         * @param imu_frames_out Vector of undelivered IMU frames
         * @return true The frame was available and has been set
         * @return false No frame was available, imu_frames_out is unchanged
         */
        virtual bool get_imu_frames_nonblocking(std::vector<ImuData> &imu_frames_out)=0;
};

#endif /* H_DATASOURCE_H */