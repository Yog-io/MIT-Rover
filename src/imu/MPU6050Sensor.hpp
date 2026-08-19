#pragma once

#include <atomic>
#include <thread>
#include <mutex>
#include <string>

// Structure to hold current orientation data in degrees
struct IMUData {
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
};

class MPU6050Sensor {
public:
    MPU6050Sensor();
    ~MPU6050Sensor();

    // Initializes the I2C connection to the sensor.
    bool initialize(const std::string& i2c_device = "/dev/i2c-1", int address = 0x68);

    // Starts the 50 Hz background polling thread.
    void start();

    // Stops the polling thread cleanly.
    void stop();

    // Retrieves the most recent computed orientation.
    IMUData get_orientation() const;

private:
    void polling_loop();
    bool read_raw_data(int& ax, int& ay, int& az, int& gx, int& gy, int& gz);

    int i2c_fd_ = -1;
    bool initialized_ = false;

    std::atomic<bool> running_{false};
    std::thread poll_thread_;

    mutable std::mutex data_mutex_;
    IMUData current_data_;
    
    // Time delta for integration
    const float dt_ = 0.02f; // 50 Hz

    // Complementary filter coefficients
    const float alpha_ = 0.98f;
};
