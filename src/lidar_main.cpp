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
    std::cout << "Starting TF-Luna 1D LiDAR UDP Bridge Test..." << std::endl;

    // Register signal handler for clean exit
    std::signal(SIGINT, sigint_handler);

    TFLunaSensor lidar;
    
    // Initialize UDP receiver on port 9090
    if (!lidar.initialize(9090)) {
        std::cerr << "Failed to initialize TF-Luna UDP receiver." << std::endl;
        return -1;
    }

    // Polling loop at 10 Hz
    while (g_running) {
        float distance = lidar.get_distance_meters();
        
        std::cout << "\r[LiDAR] Ground-Truth Distance: ";
        if (distance < 0.0f) {
            std::cout << "NO LOCK (Python Bridge offline/blocked)   ";
        } else {
            std::cout << std::fixed << std::setprecision(2) << std::setw(5) << distance << " meters                      ";
        }
        std::cout << std::flush;
                  
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "\nTest Complete. Stopping UDP receiver..." << std::endl;
    lidar.stop();

    return 0;
}
