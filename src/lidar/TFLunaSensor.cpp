#include "lidar/TFLunaSensor.hpp"
#include <iostream>
#include <string>
#include <cstring>
#include <chrono>

#ifdef __linux__
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#endif

TFLunaSensor::TFLunaSensor() {}

TFLunaSensor::~TFLunaSensor() {
    stop();
#ifdef __linux__
    if (i2c_fd_ >= 0) {
        close(i2c_fd_);
        i2c_fd_ = -1;
    }
#endif
}

bool TFLunaSensor::initialize(const std::string& i2c_device, int address) {
#ifdef __linux__
    i2c_fd_ = open(i2c_device.c_str(), O_RDWR);
    if (i2c_fd_ < 0) {
        std::cerr << "[TFLuna I2C] Failed to open I2C bus: " << i2c_device << std::endl;
        return false;
    }

    if (ioctl(i2c_fd_, I2C_SLAVE, address) < 0) {
        std::cerr << "[TFLuna I2C] Failed to acquire bus access/talk to slave at address 0x" 
                  << std::hex << address << std::dec << std::endl;
        return false;
    }

    initialized_ = true;
    std::cout << "[TFLuna I2C] Initialized I2C on " << i2c_device << " at address 0x" 
              << std::hex << address << std::dec << " successfully." << std::endl;

    start();
    return true;
#else
    std::cout << "[TFLuna I2C] Mock initialized (macOS fallback)." << std::endl;
    initialized_ = true;
    start();
    return true;
#endif
}

void TFLunaSensor::start() {
    if (!initialized_ || running_) return;

    running_ = true;
    poll_thread_ = std::thread(&TFLunaSensor::polling_loop, this);
}

void TFLunaSensor::stop() {
    running_ = false;
    if (poll_thread_.joinable()) {
        poll_thread_.join();
    }
}

float TFLunaSensor::get_distance_meters() const {
    return current_distance_m_.load(std::memory_order_relaxed);
}

void TFLunaSensor::polling_loop() {
    int consecutive_failures = 0;
    
    // Command buffer to trigger a reading
    uint8_t trigger_cmd[5] = {0x5A, 0x05, 0x00, 0x01, 0x60};

    while (running_) {
#ifdef __linux__
        // 1. Write the trigger command
        if (write(i2c_fd_, trigger_cmd, 5) != 5) {
            consecutive_failures++;
        } else {
            // 2. Wait 20ms for the sensor to compute the distance
            std::this_thread::sleep_for(std::chrono::milliseconds(20));

            // 3. Read 9 bytes
            uint8_t buffer[9];
            if (read(i2c_fd_, buffer, 9) == 9) {
                // Check headers
                if (buffer[0] == 0x59 && buffer[1] == 0x59) {
                    // Checksum
                    uint16_t sum = 0;
                    for (int i = 0; i < 8; ++i) {
                        sum += buffer[i];
                    }
                    uint8_t computed_checksum = static_cast<uint8_t>(sum & 0xFF);

                    if (computed_checksum == buffer[8]) {
                        uint16_t dist_cm = static_cast<uint16_t>(buffer[2]) | (static_cast<uint16_t>(buffer[3]) << 8);
                        uint16_t strength = static_cast<uint16_t>(buffer[4]) | (static_cast<uint16_t>(buffer[5]) << 8);
                        float distance_m = dist_cm / 100.0f;

                        if (strength >= min_strength_) {
                            current_distance_m_.store(distance_m, std::memory_order_relaxed);
                            consecutive_failures = 0;
                        } else {
                            consecutive_failures++;
                        }
                    } else {
                        consecutive_failures++;
                    }
                } else {
                    consecutive_failures++;
                }
            } else {
                consecutive_failures++;
            }
        }
#else
        // macOS mock: simulate 1.5m at 50 Hz
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        current_distance_m_.store(1.5f, std::memory_order_relaxed);
        consecutive_failures = 0;
#endif

        if (consecutive_failures > max_consecutive_failures_) {
            current_distance_m_.store(-1.0f, std::memory_order_relaxed);
            consecutive_failures = max_consecutive_failures_ + 1; // Cap to prevent overflow
        }
        
        // If we failed, add a small sleep to prevent 100% CPU lock in an error loop
#ifdef __linux__
        if (consecutive_failures > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
#endif
    }
}
