/**
 * @file main.cpp
 * @author Duncan Hamill (duncanrhamill@googlemail.com)
 * @brief Main perloc application
 * @version 0.1
 * @date 2021-07-07
 * 
 * @copyright Copyright (c) 2021
 */


#include <iostream>
#include <opencv4/opencv2/core.hpp>

#include "data_source/OakdDataSource.hpp"
#include "vis/Vis.hpp"
#include "streaming/Streaming.hpp"

int main(int argc, char *p_argv[]) {
    std::cout << "---- Phobos Perloc ----" << std::endl;

    std::cout << "Connecting to OAK-D" << std::endl;

    OakdDataSource oakd;

    std::cout << "Createing ImgStreamer" << std::endl;

    ImgStreamer img_streamer;
    img_streamer.set_stream_enabled(ImgStreamer::StreamId::LEFT, true);
    img_streamer.start();

    ImgFrame frame;
    while (true) {
        oakd.get_img_frame(frame);

        // Display image previews
        cv::imshow(
            "left (zero depth higlighted)", 
            get_vis_shadows(frame.left_img, frame.depth_img)
        );
        cv::imshow(DepthFilter::vis_window_name, get_vis_depth_img(frame.depth_img, 4000));
        
        img_streamer.feed_next(frame);

        if (cv::waitKey(1) == 'q') {
            break;
        }
    }

    img_streamer.stop();
}