#include "lidar/TFLunaSensor.hpp"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>

int main() {
    std::cout << "Starting TF-Luna 1D LiDAR Test..." << std::endl;

    TFLunaSensor lidar;
    
    if (!lidar.initialize("/dev/serial0")) {
        std::cerr << "Failed to initialize TF-Luna sensor." << std::endl;
        return -1;
    }

    std::cout << "Starting background UART reading thread..." << std::endl;
    lidar.start();

    // Print telemetry to terminal at 10 Hz for 10 seconds
    const int target_frames = 100;
    
    for (int i = 0; i < target_frames; ++i) {
        float distance = lidar.get_distance_meters();
        
        std::cout << "\r[LiDAR] Ground-Truth Distance: ";
        if (distance < 0.0f) {
            std::cout << "NO LOCK (Strength < 100 or Blocked)   ";
        } else {
            std::cout << std::fixed << std::setprecision(2) << std::setw(5) << distance << " meters                  ";
        }
        std::cout << std::flush;
                  
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "\nTest Complete. Stopping sensor..." << std::endl;
    lidar.stop();

    return 0;
}
