/**
 * @file Vis.hpp
 * @author Duncan Hamill (duncanrhamill@googlemail.com)
 * @brief Visualisation librarb
 * @version 0.1
 * @date 2021-07-07
 * 
 * @copyright Copyright (c) 2021
 */

#ifndef H_VIS_H
#define H_VIS_H

#include <opencv4/opencv2/core.hpp>

/**
 * @brief Convert the given depth image (U16 mm from camera) to an image fit
 * for visualisation.
 * 
 * @param depth_img_in Image to visualise
 * @param max_depth_m_in Maximum depth to visualise in mm. 0 means max in the
 * image. 
 * @return cv::Mat The converted RGB image.
 */
cv::Mat get_vis_depth_img(cv::Mat &depth_img_in, uint16_t max_depth_mm_in=0); 

/**
 * @brief Gets a new visualisation image showing shadows (zero depth) on the
 * provided source image
 * 
 * @param source_img_in The image the shadows will be drawn onto
 * @param depth_img_in The depth image, which will be used to get the shadows
 * @return cv::Mat Output visualisation image
 */
cv::Mat get_vis_shadows(cv::Mat &source_img_in, cv::Mat &depth_img_in);

#endif /* H_VIS_H */