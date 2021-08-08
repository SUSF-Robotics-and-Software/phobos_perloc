/**
 * @file Streaming.cpp
 * @author Duncan Hamill (duncanrhamill@googlemail.com)
 * @brief Handles streaming data to external apps
 * @version 0.1
 * @date 2021-07-08
 * 
 * @copyright Copyright (c) 2021
 */

#include <iostream>
#include <gst/webrtc/webrtc.h>
#include <nlohmann/json.hpp>
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

#include "Streaming.hpp"

// Forward declarations
static void on_webrtc_neg_needed(GstElement *p_webrtc, gpointer user_data);
static void on_offer_created(GstPromise *p_promise, gpointer p_webrtc);
static void send_sdp_to_peer(GstWebRTCSessionDescription *p_desc);

typedef websocketpp::server<websocketpp::config::asio> server;


ImgStreamer::ImgStreamer() {
    // Set all streams to disabled
    for (int stream_int = StreamId::LEFT; stream_int != StreamId::DEPTH + 1; stream_int++) {
        StreamId stream = static_cast<StreamId>(stream_int);
        thread_data.stream_enabled[stream] = false;
    }

    // Create all threading data
    thread_data.cond_var = std::make_shared<std::condition_variable>();
    thread_data.mutex = std::make_shared<std::mutex>();
    thread_data.stop = std::make_shared<std::atomic_bool>(false);
    thread_data.img_queue = std::make_shared<std::queue<ImgFrame>>();
}

ImgStreamer::~ImgStreamer() {

}

void ImgStreamer::set_stream_enabled(StreamId stream_in, bool enabled_in) {
    thread_data.stream_enabled[stream_in] = enabled_in;
}

void ImgStreamer::start() {
    std::cout << "Starting ImgStreamer background thread" << std::endl;

    // Copy the thread data
    ImgStreamer::BgThreadData data = thread_data;

    // Start the thread
    p_bg_handle = new std::thread(ImgStreamer::bg_thread, data);
}

void ImgStreamer::stop() {
    
    // Set the stop value in the thread data
    thread_data.stop->store(true);

    // Wait for the background thread to stop
    p_bg_handle->join();

    std::cout << "ImgStreamer background thread stopped" << std::endl;
}

void ImgStreamer::feed_next(ImgFrame img_frame_in) {
    // Scope so unique lock is dropped before the condvar is notified
    {
        // Get a lock on the mutex
        std::unique_lock<std::mutex> lock(*thread_data.mutex);

        // Push the new image into the queue
        thread_data.img_queue->push(img_frame_in);
    }

    // Notify the condvar that we've pushed something else in
    thread_data.cond_var->notify_one();
}

void ImgStreamer::bg_thread(ImgStreamer::BgThreadData data) {
    GstFlowReturn flow_ret;
    GstElement *p_pipeline, *p_appsrc, *p_webrtc;

    // Init GST
    gst_init(NULL, NULL);

    std::cout << "Streams enabled: " << std::endl;
    for (auto &kv: data.stream_enabled) {
        std::cout << "    " << kv.first << ": " << kv.second << std::endl;
    }

    // Create pipeline
    p_pipeline = gst_parse_launch(
        "appsrc name=source ! queue ! vp8enc ! rtpvp8pay ! "
        "application/x-raw,media=video,encoding-name=VP8,payload=98 ! "
        "webrtcbin name=sendrecv", 
        // "appsrc name=source ! videoconvert ! autovideosink",
        NULL
    );
    
    // Get the appsrc and webrtc from the pipeline
    p_appsrc = gst_bin_get_by_name(GST_BIN(p_pipeline), "source");
    p_webrtc = gst_bin_get_by_name(GST_BIN(p_pipeline), "sendrecv");

    // Set appsrc caps
    g_object_set(G_OBJECT(p_appsrc), "caps", gst_caps_new_simple(
        "video/x-raw",
        "format", G_TYPE_STRING, "GRAY8",
        "width", G_TYPE_INT, 640,
        "height", G_TYPE_INT, 400,
        "framerate", GST_TYPE_FRACTION, 1, 30,
        NULL
    ), NULL);

    // Set appsrc data
    g_object_set(G_OBJECT(p_appsrc), 
        "stream-type", 0,
        "format", GST_FORMAT_TIME,
        NULL
    );

    // Set webrtc signals
    g_signal_connect(
        p_webrtc, 
        "on-negotiation-needed", 
        G_CALLBACK(on_webrtc_neg_needed), 
        NULL
    );
    g_signal_connect(
        p_webrtc, 
        "on-ice-candidate", 
        G_CALLBACK(on_webrtc_send_ice), 
        NULL
    );

    // Play the pipeline
    gst_element_set_state(p_pipeline, GST_STATE_PLAYING);

    // Main loop
    do {
        // Get a lock on the mutex
        std::unique_lock<std::mutex> lock(*data.mutex);

        // Wait until there's data in the queue
        while (data.img_queue->empty()) {
            data.cond_var->wait(lock, [&](){
                return !data.img_queue->empty();
            });
        }

        // Consume all images from the queue
        while (!data.img_queue->empty()) {
            ImgFrame frame = data.img_queue->front();

            // TODO: support all streams, for now we just do left.

            // Build the buffer to send to the appsrc
            GstBuffer *p_buffer;
            GstMapInfo map_info;
            
            // First need to get the size of the image, which we can grab from
            // the width * height * channels, can then make a buffer of that
            // size 
            uint img_bytes = frame.left_img.total() * frame.left_img.elemSize();
            p_buffer = gst_buffer_new_and_alloc(img_bytes);

            // TODO: set timestamps?

            // Put the image data into the buffer, by first getting a writable
            // map to the buffer
            gst_buffer_map(p_buffer, &map_info, GST_MAP_WRITE);
            
            // Then copy the data in using memcpy
            memcpy(
                (guchar *)map_info.data, 
                (guchar *)frame.left_img.data, 
                gst_buffer_get_size(p_buffer)
            );

            // Emmit the data for the appsrc
            g_signal_emit_by_name(p_appsrc, "push-buffer", p_buffer, &flow_ret);

            // Check the flow worked
            if (flow_ret != GST_FLOW_OK) {
                std::cerr << "ERROR! Couldn't push image into GST buffer" << std::endl;
                // TODO: handle this
            }

            data.img_queue->pop();
        }

        // If the stop flag was set exit
    } while (!data.stop->load());
}

static void on_webrtc_neg_needed(GstElement *p_webrtc, gpointer user_data) {
    // This is based on http://blog.nirbheek.in/2018/02/gstreamer-webrtc.html
    GstPromise *p_promise;

    p_promise = gst_promise_new_with_change_func(
        on_offer_created,
        user_data, 
        NULL
    );
    g_signal_emit_by_name (
        p_webrtc, 
        "create-offer", 
        NULL,
        p_promise
    );
}

static void on_offer_created(GstPromise *p_promise, gpointer p_webrtc) {
    GstWebRTCSessionDescription *p_offer = NULL;
    const GstStructure *p_reply;
    gchar *p_desc;
    GstElement *p_webrtc = (GstElement *)p_webrtc;

    // Get the promise reply
    p_reply = gst_promise_get_reply(p_promise);
    gst_structure_get(
        p_reply, 
        "offer", 
        GST_TYPE_WEBRTC_SESSION_DESCRIPTION,
        &p_offer,
        NULL  
    );

    g_signal_emit_by_name(p_webrtc, "set-local-description", p_offer, NULL);

    // Send the session description to the peer
    send_sdp_to_peer(p_offer);
    gst_webrtc_session_description_free(p_offer);
}

static void send_sdp_to_peer(GstWebRTCSessionDescription *p_desc) {
    gchar *text;
    nlohmann::json msg, sdp;

    text = gst_sdp_message_as_text(p_desc->sdp);

    if (p_desc->type == GST_WEBRTC_SDP_TYPE_OFFER) {
        gst_print ("Sending offer:\n%s\n", text);
        sdp["type"] = "offer";
    } else if (p_desc->type == GST_WEBRTC_SDP_TYPE_ANSWER) {
        gst_print ("Sending answer:\n%s\n", text);
        sdp["type"] = "answer";
    } else {
        throw std::runtime_error("Unexpected GstWebRTCSessionDescription::type");
    }

    sdp["sdp"] = text;
    g_free (text);

    msg["sdp"] = sdp;

    std::string msg_str = msg.dump();

    soup_websocket_connection_send_text (ws_conn, text);
    g_free (text);

}

static void on_webrtc_send_ice() {
    
}
