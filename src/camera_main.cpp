#include "camera/DualCameraCapture.hpp"
#include <iostream>
#include <iomanip>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <libcamera/control_ids.h>

int main() {
    std::cout << "Starting Headless Dual Camera Synchronization Test..." << std::endl;

    DualCameraCapture dual_cam;
    
    if (!dual_cam.initialize()) {
        std::cerr << "Failed to initialize dual camera capture system." << std::endl;
        return -1;
    }

    std::cout << "Starting camera streams..." << std::endl;
    dual_cam.start();

    const int target_frames = 100;
    int frame_count = 0;
    
    auto overall_start_time = std::chrono::steady_clock::now();

    while (frame_count < target_frames) {
        auto wait_start = std::chrono::steady_clock::now();
        
        // Blocks until a paired frame (< 3ms delta) is available
        auto stereo_pair = dual_cam.get_stereo_pair();
        
        auto wait_end = std::chrono::steady_clock::now();
        
        if (!stereo_pair) {
            std::cerr << "Capture stopped or failed at frame " << frame_count << std::endl;
            break;
        }

        frame_count++;
        
        // Print diagnostics every 10th frame
        if (frame_count % 10 == 0) {
            double acquire_time_ms = std::chrono::duration<double, std::milli>(wait_end - wait_start).count();
            
            // Reconstruct timestamps from metadata to calculate the delta
            auto meta_l = stereo_pair->left_request->metadata();
            auto meta_r = stereo_pair->right_request->metadata();
            
            // Fallback to buffer timestamp if SensorTimestamp metadata isn't explicitly exposed on this ISP
            uint64_t ts_l = meta_l.contains(libcamera::controls::SensorTimestamp) ? 
                            meta_l.get(libcamera::controls::SensorTimestamp) : 
                            stereo_pair->left_request->buffers().begin()->second->metadata().timestamp;
                            
            uint64_t ts_r = meta_r.contains(libcamera::controls::SensorTimestamp) ? 
                            meta_r.get(libcamera::controls::SensorTimestamp) : 
                            stereo_pair->right_request->buffers().begin()->second->metadata().timestamp;
                            
            double delta_ms = std::abs(static_cast<double>(ts_l) - static_cast<double>(ts_r)) / 1000000.0;
            
            std::cout << "--- Frame " << frame_count << " Diagnostics ---" << std::endl;
            std::cout << "  Acquisition Time: " << std::fixed << std::setprecision(2) << acquire_time_ms << " ms" << std::endl;
            std::cout << "  Timestamp Delta:  " << delta_ms << " ms" << std::endl;
            
            const cv::Mat& left = stereo_pair->left_y;
            const cv::Mat& right = stereo_pair->right_y;
            std::cout << "  Left Mat:         " << left.cols << "x" << left.rows 
                      << ", Continuous: " << (left.isContinuous() ? "Yes" : "No") << std::endl;
            std::cout << "  Right Mat:        " << right.cols << "x" << right.rows 
                      << ", Continuous: " << (right.isContinuous() ? "Yes" : "No") << std::endl;
            std::cout << "------------------------------" << std::endl;
        }
        
        // As the loop iterates, stereo_pair goes out of scope and the buffers are 
        // automatically re-queued to the cameras via the StereoFramePair destructor.
    }

    auto overall_end_time = std::chrono::steady_clock::now();
    double total_time_s = std::chrono::duration<double>(overall_end_time - overall_start_time).count();
    
    if (frame_count > 0) {
        std::cout << "\nTest Complete. Captured " << frame_count << " synchronized pairs." << std::endl;
        std::cout << "Average FPS: " << std::fixed << std::setprecision(2) << (frame_count / total_time_s) << std::endl;
    }

    std::cout << "Stopping cameras and releasing resources..." << std::endl;
    dual_cam.stop();

    return 0;
}
