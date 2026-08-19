#include "lidar/TFLunaSensor.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip>

int main() {
    std::cout << "[Diagnostic] Starting Phase 6 TF-Luna I2C Test..." << std::endl;
    TFLunaSensor lidar;
    
    if (!lidar.initialize("/dev/i2c-3", 0x10)) {
        std::cerr << "Failed to initialize TF-Luna I2C driver." << std::endl;
        return -1;
    }
    
    for (int i = 0; i < 100; ++i) { // 10 seconds at 10Hz
        float dist = lidar.get_distance_meters();
        std::cout << "\r[Diagnostic] Distance: ";
        if (dist < 0.0f) {
            std::cout << "NO LOCK (Timeout/Low Strength)        ";
        } else {
            std::cout << std::fixed << std::setprecision(2) << dist << " m                  ";
        }
        std::cout << std::flush;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    std::cout << "\n[Diagnostic] Test complete." << std::endl;
    lidar.stop();
    return 0;
}
