#pragma once

#include <libcamera/libcamera.h>
#include <opencv2/opencv.hpp>
#include <memory>
#include <deque>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <map>

class DualCameraCapture;

// RAII Wrapper for synchronized frame pairs.
// Exposes the Y-channel as cv::Mat without copying.
// Destructor automatically re-queues the DMA-BUFs back to the camera ISP.
struct StereoFramePair {
    cv::Mat left_y;
    cv::Mat right_y;
    
    libcamera::Request* left_request;
    libcamera::Request* right_request;
    DualCameraCapture* capture_manager;

    StereoFramePair(cv::Mat l, cv::Mat r, libcamera::Request* l_req, libcamera::Request* r_req, DualCameraCapture* mgr)
        : left_y(std::move(l)), right_y(std::move(r)), left_request(l_req), right_request(r_req), capture_manager(mgr) {}
        
    ~StereoFramePair();
    
    // Disable copy
    StereoFramePair(const StereoFramePair&) = delete;
    StereoFramePair& operator=(const StereoFramePair&) = delete;
    
    // Allow move
    StereoFramePair(StereoFramePair&& other) noexcept
        : left_y(std::move(other.left_y)), right_y(std::move(other.right_y)),
          left_request(other.left_request), right_request(other.right_request),
          capture_manager(other.capture_manager) {
        other.left_request = nullptr;
        other.right_request = nullptr;
        other.capture_manager = nullptr;
    }
};

class DualCameraCapture {
public:
    DualCameraCapture();
    ~DualCameraCapture();

    // Initializes CameraManager, finds 2 cameras, configures 640x480 @ 30FPS, maps DMA-BUFs
    bool initialize();
    
    // Starts the camera streams
    void start();
    
    // Stops the camera streams
    void stop();

    // Blocks until a synchronized pair (< 3ms delta) is available. 
    std::unique_ptr<StereoFramePair> get_stereo_pair();

    // Internal method called by StereoFramePair destructor
    void requeue_request(int camera_index, libcamera::Request* request);

private:
    void requestComplete(libcamera::Request* request, int camera_index);
    void mapBuffers(libcamera::FrameBufferAllocator* allocator, int camera_index, libcamera::StreamConfiguration const& cfg);
    void processSyncQueue();

    std::unique_ptr<libcamera::CameraManager> cm_;
    std::shared_ptr<libcamera::Camera> cameras_[2];
    std::unique_ptr<libcamera::FrameBufferAllocator> allocators_[2];
    std::unique_ptr<libcamera::CameraConfiguration> configs_[2];
    libcamera::Stream* streams_[2];
    
    // Memory mapping tracking
    struct MappedBuffer {
        void* memory;
        size_t length;
    };
    std::map<libcamera::FrameBuffer*, MappedBuffer> mapped_buffers_[2];
    
    int stride_[2];
    int height_[2];

    struct PendingFrame {
        libcamera::Request* request;
        uint64_t timestamp;
    };

    std::deque<PendingFrame> queue_[2];
    std::mutex queue_mutex_;
    
    std::deque<std::pair<libcamera::Request*, libcamera::Request*>> ready_queue_;
    std::condition_variable ready_cond_;

    std::atomic<bool> running_{false};
};
