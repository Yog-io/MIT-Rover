#include "camera/DualCameraCapture.hpp"
#include <iostream>
#include <chrono>
#include <iomanip>
#include <thread>

int main() {
    std::cout << "[Diagnostic] Starting Phase 2 Camera Sync Test..." << std::endl;
    DualCameraCapture dual_cam;
    
    if (!dual_cam.initialize()) {
        std::cerr << "Failed to initialize cameras." << std::endl;
        return -1;
    }
    
    dual_cam.start();
    
    int frames_captured = 0;
    int target_frames = 500;
    int dropped_frames = 0;
    
    auto last_time = std::chrono::steady_clock::now();
    
    while (frames_captured < target_frames) {
        auto pair = dual_cam.get_stereo_pair();
        
        if (!pair || pair->left_y.empty() || pair->right_y.empty()) {
            dropped_frames++;
            continue;
        }
        
        auto now = std::chrono::steady_clock::now();
        auto delta_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time).count();
        last_time = now;
        
        std::cout << "\r[Diagnostic] Frame " << frames_captured + 1 << "/" << target_frames 
                  << " | Loop Delta: " << delta_ms << " ms | Dropped: " << dropped_frames << "   " << std::flush;
        
        frames_captured++;
    }
    
    dual_cam.stop();
    
    float drop_percent = (static_cast<float>(dropped_frames) / target_frames) * 100.0f;
    std::cout << "\n[Diagnostic] Test complete. Drop Rate: " << std::fixed << std::setprecision(2) << drop_percent << "%" << std::endl;
    
    return 0;
}
