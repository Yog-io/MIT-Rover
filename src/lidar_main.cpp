#include "lidar/TFLunaSensor.hpp"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>

std::atomic<bool> g_running{true};

void sigint_handler(int) {
    g_running = false;
}

int main() {
    std::cout << "Starting TF-Luna 1D LiDAR I2C Test..." << std::endl;

    // Register signal handler for clean exit
    std::signal(SIGINT, sigint_handler);

    TFLunaSensor lidar;
    
    // Initialize I2C driver on /dev/i2c-3 at address 0x10
    if (!lidar.initialize("/dev/i2c-3", 0x10)) {
        std::cerr << "Failed to initialize TF-Luna I2C driver." << std::endl;
        return -1;
    }

    // Polling loop at 10 Hz
    while (g_running) {
        float distance = lidar.get_distance_meters();
        
        std::cout << "\r[LiDAR] Ground-Truth Distance: ";
        if (distance < 0.0f) {
            std::cout << "NO LOCK (Strength < 100 or Blocked)   ";
        } else {
            std::cout << std::fixed << std::setprecision(2) << std::setw(5) << distance << " meters                      ";
        }
        std::cout << std::flush;
                  
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "\nTest Complete. Stopping I2C sensor..." << std::endl;
    lidar.stop();

    return 0;
}
