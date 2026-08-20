#include "camera/DualCameraCapture.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip>
#include <sstream>

int main() {
    std::cout << "[Calibration] Starting Headless Stereo Calibration Capture..." << std::endl;
    DualCameraCapture dual_cam;
    
    if (!dual_cam.initialize()) {
        std::cerr << "Failed to initialize cameras." << std::endl;
        return -1;
    }
    
    dual_cam.start();
    
    // Allow cameras to warm up and auto-expose
    std::cout << "[Calibration] Warming up cameras for 2 seconds..." << std::endl;
    for(int i=0; i<60; i++) {
        auto pair = dual_cam.get_stereo_pair();
    }
    
    int pairs_captured = 0;
    int target_pairs = 20;
    
    while (pairs_captured < target_pairs) {
        // Countdown loop (3 seconds)
        for (int i = 3; i > 0; --i) {
            std::cout << "\r[Calibration] Capturing pair " << (pairs_captured + 1) << "/" << target_pairs 
                      << " in " << i << "...   " << std::flush;
            
            // Drain queue for 1 second to keep zero-copy buffers fresh
            auto start_drain = std::chrono::steady_clock::now();
            while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_drain).count() < 1000) {
                auto pair = dual_cam.get_stereo_pair();
            }
        }
        
        std::cout << "\r[Calibration] Capturing pair " << (pairs_captured + 1) << "/" << target_pairs 
                  << " ... SNAP!    " << std::endl;
                  
        // Fetch the targeted pair
        auto pair = dual_cam.get_stereo_pair();
        if (!pair || pair->left_y.empty() || pair->right_y.empty()) {
            std::cerr << "[Calibration] Failed to capture synchronized pair. Retrying..." << std::endl;
            continue;
        }
        
        std::stringstream ss_left, ss_right;
        ss_left << "calib_left_" << std::setfill('0') << std::setw(2) << pairs_captured << ".png";
        ss_right << "calib_right_" << std::setfill('0') << std::setw(2) << pairs_captured << ".png";
        
        cv::imwrite(ss_left.str(), pair->left_y);
        cv::imwrite(ss_right.str(), pair->right_y);
        
        pairs_captured++;
    }
    
    dual_cam.stop();
    std::cout << "[Calibration] Successfully captured 20 pairs. Terminating." << std::endl;
    return 0;
}
