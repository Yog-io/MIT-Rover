#include "imu/MPU6050Sensor.hpp"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>

int main() {
    std::cout << "Starting MPU6050 Orientation Test..." << std::endl;

    MPU6050Sensor imu;
    
    // Address 0x68 is default
    if (!imu.initialize("/dev/i2c-1", 0x68)) {
        std::cerr << "Failed to initialize MPU6050 sensor." << std::endl;
        return -1;
    }

    std::cout << "Starting 50 Hz background polling thread..." << std::endl;
    imu.start();

    // Print telemetry to terminal at 10 Hz for 10 seconds
    const int target_frames = 100;
    
    for (int i = 0; i < target_frames; ++i) {
        IMUData data = imu.get_orientation();
        
        std::cout << "\r[IMU] Roll: " << std::fixed << std::setprecision(2) << std::setw(7) << data.roll
                  << " | Pitch: " << std::setw(7) << data.pitch
                  << " | Yaw: " << std::setw(7) << data.yaw << std::flush;
                  
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "\nTest Complete. Stopping sensor..." << std::endl;
    imu.stop();

    return 0;
}
