#include "imu/MPU6050Sensor.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip>

int main() {
    std::cout << "[Diagnostic] Starting Phase 5 MPU6050 Test (50Hz / 10s)..." << std::endl;
    MPU6050Sensor imu;
    
    // MPU6050Sensor typically doesn't take args in our setup, defaults to /dev/i2c-1 or 3 and 0x68
    if (!imu.initialize()) {
        std::cerr << "Failed to initialize IMU." << std::endl;
        return -1;
    }
    
    // MPU6050Sensor::start() is typically called inside initialize(), but just in case:
    // imu.start(); 
    
    auto start_time = std::chrono::steady_clock::now();
    int iterations = 0;
    
    while (true) {
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - start_time).count() >= 10) {
            break;
        }
        
        auto orientation = imu.get_orientation();
        std::cout << "\r[Diagnostic] Roll: " << std::fixed << std::setprecision(2) << std::setw(6) << orientation.roll
                  << " | Pitch: " << std::setw(6) << orientation.pitch
                  << " | Yaw: " << std::setw(6) << orientation.yaw << "    " << std::flush;
                  
        iterations++;
        std::this_thread::sleep_for(std::chrono::milliseconds(20)); // 50Hz polling
    }
    
    std::cout << "\n[Diagnostic] Completed " << iterations << " iterations in 10s." << std::endl;
    imu.stop();
    return 0;
}
