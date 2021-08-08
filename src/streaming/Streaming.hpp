/**
 * @file Streaming.hpp
 * @author Duncan Hamill (duncanrhamill@googlemail.com)
 * @brief Handles streaming data to external apps
 * @version 0.1
 * @date 2021-07-08
 * 
 * @copyright Copyright (c) 2021
 */

#include <map>
#include <thread>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <atomic>
#include <gstreamer-1.0/gst/gst.h>
#include "../data_source/DataSource.hpp"

class ImgStreamer {
    public:
        ImgStreamer();
        ~ImgStreamer();

        enum StreamId {
            LEFT,
            RIGHT,
            DISPARITY,
            DEPTH
        };

        /**
         * @brief Sets whether or not the given stream will be enabled
         * 
         * @param stream_in The stream to set
         * @param enabled_in True if the stream should be produced.
         */
        void set_stream_enabled(StreamId stream_in, bool enabled_in);

        /**
         * @brief Start the enabled streams.
         * 
         * This function will create a new thread responsible for managing the
         * stream. 
         */
        void start();

        /**
         * @brief Stops the stream.
         * 
         * This will destroy the background thread, and will block until that
         * thread is returned.
         */
        void stop();

        /**
         * @brief Feeds the next image frame into the streamer
         * 
         * @param img_frame_in Frame to add to the stream
         */
        void feed_next(ImgFrame img_frame_in);

    private:
        struct BgThreadData {
            std::map<StreamId, bool> stream_enabled;
            std::shared_ptr<std::mutex> mutex;
            std::shared_ptr<std::queue<ImgFrame>> img_queue;
            std::shared_ptr<std::condition_variable> cond_var;
            std::shared_ptr<std::atomic_bool> stop;
        };

        ImgStreamer::BgThreadData thread_data;
        std::thread *p_bg_handle;

        static void bg_thread(ImgStreamer::BgThreadData data);
};