#include "camera/DualCameraCapture.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <thread>

int main() {
    std::cout << "[Calibration] Starting Headless Stereo Calibration Capture..." << std::endl;
    DualCameraCapture dual_cam;
    
    if (!dual_cam.initialize()) {
        std::cerr << "Failed to initialize cameras." << std::endl;
        return -1;
    }
    
    dual_cam.start();
    
    int pairs_captured = 0;
    int target_pairs = 20;
    
    auto start_time = std::chrono::steady_clock::now();
    int last_countdown = 3;
    
    std::cout << "[Calibration] Continuously polling frames to prevent buffer exhaustion..." << std::endl;

    while (pairs_captured < target_pairs) {
        // Continuous polling: NEVER block the thread. Keep fetching frames to free DMA-BUFs.
        auto pair = dual_cam.get_stereo_pair();
        
        if (!pair || pair->left_y.empty() || pair->right_y.empty()) {
            continue;
        }
        
        auto now = std::chrono::steady_clock::now();
        float elapsed_seconds = std::chrono::duration<float>(now - start_time).count();
        
        int current_countdown = 3 - static_cast<int>(elapsed_seconds);
        
        // Print the countdown elegantly without flooding the terminal
        if (current_countdown != last_countdown && current_countdown > 0) {
            std::cout << "\r[Calibration] Capturing pair " << (pairs_captured + 1) << "/" << target_pairs 
                      << " in " << current_countdown << "...   " << std::flush;
            last_countdown = current_countdown;
        }
        
        // 3.0 seconds elapsed: Perform the SNAP!
        if (elapsed_seconds >= 3.0f) {
            std::cout << "\r[Calibration] Capturing pair " << (pairs_captured + 1) << "/" << target_pairs 
                      << " ... SNAP!    " << std::endl;
                      
            std::stringstream ss_left, ss_right;
            ss_left << "calib_left_" << std::setfill('0') << std::setw(2) << pairs_captured << ".png";
            ss_right << "calib_right_" << std::setfill('0') << std::setw(2) << pairs_captured << ".png";
            
            cv::imwrite(ss_left.str(), pair->left_y);
            cv::imwrite(ss_right.str(), pair->right_y);
            
            pairs_captured++;
            
            // Reset timer for the next snapshot
            start_time = std::chrono::steady_clock::now();
            last_countdown = 3;
        }
    }
    
    dual_cam.stop();
    std::cout << "[Calibration] Successfully captured " << target_pairs << " pairs. Terminating." << std::endl;
    return 0;
}
