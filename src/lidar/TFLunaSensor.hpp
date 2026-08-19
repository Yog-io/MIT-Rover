#pragma once

#include <atomic>
#include <thread>
#include <string>

class TFLunaSensor {
public:
    TFLunaSensor();
    ~TFLunaSensor();

    // Opens I2C connection to the sensor
    bool initialize(const std::string& i2c_device = "/dev/i2c-3", int address = 0x10);

    // Starts the continuous background reading thread
    void start();

    // Stops the reading thread cleanly
    void stop();

    // Lock-free atomic read of the ground-truth distance in meters.
    // Returns -1.0f if the sensor has completely lost lock.
    float get_distance_meters() const;

private:
    void polling_loop();

    int i2c_fd_ = -1;
    bool initialized_ = false;

    std::atomic<bool> running_{false};
    std::thread poll_thread_;

    // Lock-free storage of the latest valid distance
    std::atomic<float> current_distance_m_{-1.0f};

    // Configurable thresholds
    const int min_strength_ = 100;
    const int max_consecutive_failures_ = 50; // ~1 second at 50Hz
};
