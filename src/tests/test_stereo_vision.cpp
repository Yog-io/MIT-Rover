#include "camera/DualCameraCapture.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <thread>
#include <omp.h>

int main() {
    std::cout << "[Diagnostic] Starting Phase 3 Stereo Vision Test..." << std::endl;
    DualCameraCapture dual_cam;
    
    if (!dual_cam.initialize()) {
        std::cerr << "Failed to initialize cameras." << std::endl;
        return -1;
    }
    
    dual_cam.start();
    
    // Allow cameras to warm up and auto-expose
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    auto stereo = cv::StereoBM::create(64, 9);
    
    for (int frame = 0; frame < 10; ++frame) {
        auto pair = dual_cam.get_stereo_pair();
        if (!pair || pair->left_y.empty() || pair->right_y.empty()) { 
            std::cerr << "Failed to get synchronized frame pair." << std::endl; 
            continue; 
        }
        
        cv::Mat disparity_16s;
        auto start_time = std::chrono::steady_clock::now();
        
        // 1. StereoBM
        stereo->compute(pair->left_y, pair->right_y, disparity_16s);
        
        // 2. OpenMP Binning (Mocked HazardMapper workload)
        int width = disparity_16s.cols;
        int height = disparity_16s.rows;
        int binned_points = 0;
        
        #pragma omp parallel for reduction(+:binned_points)
        for (int v = 0; v < height; ++v) {
            const int16_t* row_ptr = disparity_16s.ptr<int16_t>(v);
            for (int u = 0; u < width; ++u) {
                int16_t d_val = row_ptr[u];
                if (d_val > 0) {
                    binned_points++;
                }
            }
        }
        
        auto end_time = std::chrono::steady_clock::now();
        auto exec_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
        
        std::cout << "[Diagnostic] Frame " << frame + 1 << "/10 | Execution time: " << exec_time << " ms | Binned points: " << binned_points << std::endl;
        
        // Save images only on the very first successful frame
        if (frame == 0) {
            cv::imwrite("left_test.png", pair->left_y);
            cv::imwrite("right_test.png", pair->right_y);
            
            cv::Mat disp_8u;
            disparity_16s.convertTo(disp_8u, CV_8U, 255.0 / (64 * 16.0));
            cv::imwrite("disparity_test.png", disp_8u);
            std::cout << "[Diagnostic] Saved 'left_test.png', 'right_test.png', and 'disparity_test.png' on Frame 1." << std::endl;
        }
    }
    
    dual_cam.stop();
    return 0;
}
