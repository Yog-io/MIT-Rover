#include "DualCameraCapture.hpp"
#include <iostream>
#include <sys/mman.h>
#include <unistd.h>
#include <libcamera/control_ids.h>

using namespace libcamera;

StereoFramePair::~StereoFramePair() {
    if (capture_manager) {
        if (left_request) capture_manager->requeue_request(0, left_request);
        if (right_request) capture_manager->requeue_request(1, right_request);
    }
}

DualCameraCapture::DualCameraCapture() {
    cm_ = std::make_unique<CameraManager>();
}

DualCameraCapture::~DualCameraCapture() {
    stop();
    for (int i = 0; i < 2; i++) {
        if (allocators_[i]) {
            allocators_[i].reset();
        }
        if (cameras_[i]) {
            cameras_[i]->release();
            cameras_[i].reset();
        }
        for (auto const& [fb, mapped] : mapped_buffers_[i]) {
            munmap(mapped.memory, mapped.length);
        }
        mapped_buffers_[i].clear();
    }
    cm_->stop();
}

bool DualCameraCapture::initialize() {
    cm_->start();
    
    if (cm_->cameras().size() < 2) {
        std::cerr << "Less than 2 cameras found!" << std::endl;
        return false;
    }

    for (int i = 0; i < 2; i++) {
        cameras_[i] = cm_->get(cm_->cameras()[i]->id());
        if (!cameras_[i]) return false;
        
        if (cameras_[i]->acquire()) {
            std::cerr << "Failed to acquire camera " << i << std::endl;
            return false;
        }

        configs_[i] = cameras_[i]->generateConfiguration({StreamRole::VideoRecording});
        StreamConfiguration& cfg = configs_[i]->at(0);
        cfg.size.width = 640;
        cfg.size.height = 480;
        cfg.pixelFormat = formats::YUV420; // YUV420 allows easy zero-copy extraction of Y plane
        cfg.bufferCount = 4; // Pool of 4 buffers per camera

        if (configs_[i]->validate() == CameraConfiguration::Invalid) {
            std::cerr << "Camera configuration invalid!" << std::endl;
            return false;
        }

        if (cameras_[i]->configure(configs_[i].get())) {
            std::cerr << "Failed to configure camera " << i << std::endl;
            return false;
        }
        
        streams_[i] = configs_[i]->at(0).stream();
        stride_[i] = configs_[i]->at(0).stride;
        height_[i] = configs_[i]->at(0).size.height;

        allocators_[i] = std::make_unique<FrameBufferAllocator>(cameras_[i]);
        allocators_[i]->allocate(streams_[i]);

        mapBuffers(allocators_[i].get(), i, configs_[i]->at(0));
        
        // Connect the request completed signal
        cameras_[i]->requestCompleted.connect(this, [this, i](Request* req) {
            this->requestComplete(req, i);
        });
    }

    return true;
}

void DualCameraCapture::mapBuffers(FrameBufferAllocator* allocator, int camera_index, StreamConfiguration const& cfg) {
    for (const auto& buffer : allocator->buffers(streams_[camera_index])) {
        // Map the DMA buffer for zero-copy access
        // For Raspberry Pi ISP, the YUV420 planes are typically contiguous in a single dma-buf fd
        int fd = buffer->planes()[0].fd.get();
        size_t length = 0;
        for (const auto& plane : buffer->planes()) {
            length += plane.length;
        }
        
        void* memory = mmap(NULL, length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        if (memory == MAP_FAILED) {
            std::cerr << "Failed to mmap buffer for camera " << camera_index << std::endl;
            continue;
        }
        mapped_buffers_[camera_index][buffer.get()] = {memory, length};
    }
}

void DualCameraCapture::start() {
    running_ = true;
    for (int i = 0; i < 2; i++) {
        cameras_[i]->start();
        // Queue all buffers to start the capture pipeline
        for (const auto& buffer : allocators_[i]->buffers(streams_[i])) {
            std::unique_ptr<Request> request = cameras_[i]->createRequest();
            if (!request) {
                std::cerr << "Failed to create request" << std::endl;
                continue;
            }
            request->addBuffer(streams_[i], buffer.get());
            cameras_[i]->queueRequest(request.release());
        }
    }
}

void DualCameraCapture::stop() {
    running_ = false;
    ready_cond_.notify_all();
    for (int i = 0; i < 2; i++) {
        if (cameras_[i]) cameras_[i]->stop();
    }
}

void DualCameraCapture::requeue_request(int camera_index, Request* request) {
    if (running_ && cameras_[camera_index]) {
        request->reuse(Request::ReuseBuffers);
        cameras_[camera_index]->queueRequest(request);
    } else {
        delete request;
    }
}

void DualCameraCapture::requestComplete(Request* request, int camera_index) {
    if (request->status() == Request::RequestCancelled) {
        delete request;
        return;
    }

    // Extract sensor timestamp
    auto controls = request->metadata();
    uint64_t timestamp = controls.contains(controls::SensorTimestamp.id()) 
                         ? controls.get(controls::SensorTimestamp).value()
                         : request->buffers().begin()->second->metadata().timestamp;

    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        queue_[camera_index].push_back({request, timestamp});
    }

    processSyncQueue();
}

void DualCameraCapture::processSyncQueue() {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    
    while (!queue_[0].empty() && !queue_[1].empty()) {
        auto& req0 = queue_[0].front();
        auto& req1 = queue_[1].front();
        
        int64_t diff = static_cast<int64_t>(req0.timestamp) - static_cast<int64_t>(req1.timestamp);
        
        // Match threshold: 3.0 ms = 3,000,000 ns
        if (std::abs(diff) <= 3000000LL) {
            // Matched!
            ready_queue_.push_back({req0.request, req1.request});
            queue_[0].pop_front();
            queue_[1].pop_front();
            ready_cond_.notify_one();
        } else if (diff > 0) {
            // req1 is older
            requeue_request(1, req1.request);
            queue_[1].pop_front();
        } else {
            // req0 is older
            requeue_request(0, req0.request);
            queue_[0].pop_front();
        }
    }
}

std::unique_ptr<StereoFramePair> DualCameraCapture::get_stereo_pair() {
    std::unique_lock<std::mutex> lock(queue_mutex_);
    ready_cond_.wait(lock, [this]() { return !ready_queue_.empty() || !running_; });
    
    if (!running_ && ready_queue_.empty()) {
        return nullptr;
    }
    
    auto [req0, req1] = ready_queue_.front();
    ready_queue_.pop_front();
    
    lock.unlock(); // Unlock before OpenCV Mat creation

    // Extract mapped memory
    FrameBuffer* fb0 = req0->buffers().begin()->second;
    FrameBuffer* fb1 = req1->buffers().begin()->second;
    
    void* mem0 = mapped_buffers_[0][fb0].memory;
    void* mem1 = mapped_buffers_[1][fb1].memory;
    
    // Create zero-copy cv::Mat wrapper for the Y channel (luminance)
    // The Y plane is exactly at the start of the buffer with size height * stride.
    cv::Mat left_y(height_[0], stride_[0], CV_8UC1, mem0);
    cv::Mat right_y(height_[1], stride_[1], CV_8UC1, mem1);
    
    return std::make_unique<StereoFramePair>(left_y, right_y, req0, req1, this);
}
