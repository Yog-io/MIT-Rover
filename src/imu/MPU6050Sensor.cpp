#include "imu/MPU6050Sensor.hpp"
#include <iostream>
#include <cmath>
#include <chrono>

#ifdef __linux__
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <pthread.h>
#endif

// MPU6050 Registers
#define SMPLRT_DIV   0x19
#define CONFIG       0x1A
#define GYRO_CONFIG  0x1B
#define ACCEL_CONFIG 0x1C
#define PWR_MGMT_1   0x6B
#define ACCEL_XOUT_H 0x3B

MPU6050Sensor::MPU6050Sensor() {}

MPU6050Sensor::~MPU6050Sensor() {
    stop();
#ifdef __linux__
    if (i2c_fd_ >= 0) {
        close(i2c_fd_);
    }
#endif
}

bool MPU6050Sensor::initialize(const std::string& i2c_device, int address) {
#ifdef __linux__
    i2c_fd_ = open(i2c_device.c_str(), O_RDWR);
    if (i2c_fd_ < 0) {
        std::cerr << "[MPU6050] Failed to open I2C bus: " << i2c_device << std::endl;
        return false;
    }

    if (ioctl(i2c_fd_, I2C_SLAVE, address) < 0) {
        std::cerr << "[MPU6050] Failed to acquire bus access/talk to slave." << std::endl;
        return false;
    }

    // Wake up the MPU6050 (write 0 to Power Management 1)
    uint8_t buf[2] = {PWR_MGMT_1, 0x00};
    if (write(i2c_fd_, buf, 2) != 2) {
        std::cerr << "[MPU6050] Failed to wake up sensor." << std::endl;
        return false;
    }
    
    // Set sample rate divider for 50Hz (assume 1kHz internal sample rate)
    buf[0] = SMPLRT_DIV; buf[1] = 0x13; 
    write(i2c_fd_, buf, 2);

    // Set config for low pass filter
    buf[0] = CONFIG; buf[1] = 0x03; 
    write(i2c_fd_, buf, 2);

    initialized_ = true;
    std::cout << "[MPU6050] Initialized successfully." << std::endl;
    return true;
#else
    std::cout << "[MPU6050] Mock initialized (macOS fallback)." << std::endl;
    initialized_ = true;
    return true;
#endif
}

void MPU6050Sensor::start() {
    if (!initialized_ || running_) return;
    
    running_ = true;
    poll_thread_ = std::thread(&MPU6050Sensor::polling_loop, this);

#ifdef __linux__
    // Pin thread strictly to Core 3 to decouple from Vision processing (Cores 0-2)
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(3, &cpuset);

    int rc = pthread_setaffinity_np(poll_thread_.native_handle(), sizeof(cpu_set_t), &cpuset);
    if (rc != 0) {
        std::cerr << "[MPU6050] Warning: Failed to set thread affinity to Core 3." << std::endl;
    } else {
        std::cout << "[MPU6050] Thread pinned strictly to Core 3." << std::endl;
    }
#endif
}

void MPU6050Sensor::stop() {
    running_ = false;
    if (poll_thread_.joinable()) {
        poll_thread_.join();
    }
}

IMUData MPU6050Sensor::get_orientation() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return current_data_;
}

bool MPU6050Sensor::read_raw_data(int& ax, int& ay, int& az, int& gx, int& gy, int& gz) {
#ifdef __linux__
    if (i2c_fd_ < 0) return false;

    // Start reading at ACCEL_XOUT_H (0x3B)
    uint8_t reg = ACCEL_XOUT_H;
    if (write(i2c_fd_, &reg, 1) != 1) return false;

    // Read 14 bytes (Accel X, Y, Z, Temp, Gyro X, Y, Z)
    uint8_t data[14];
    if (read(i2c_fd_, data, 14) != 14) return false;

    ax = (data[0] << 8) | data[1];
    ay = (data[2] << 8) | data[3];
    az = (data[4] << 8) | data[5];
    
    gx = (data[8] << 8) | data[9];
    gy = (data[10] << 8) | data[11];
    gz = (data[12] << 8) | data[13];

    // Convert to signed 16-bit
    if (ax > 32767) ax -= 65536;
    if (ay > 32767) ay -= 65536;
    if (az > 32767) az -= 65536;
    if (gx > 32767) gx -= 65536;
    if (gy > 32767) gy -= 65536;
    if (gz > 32767) gz -= 65536;

    return true;
#else
    // Mock data on macOS to prevent breaking compilation
    ax = 0; ay = 0; az = 16384; // 1g
    gx = 0; gy = 0; gz = 0;
    return true;
#endif
}

void MPU6050Sensor::polling_loop() {
    auto next_tick = std::chrono::steady_clock::now();
    const auto interval = std::chrono::milliseconds(20); // Exactly 50 Hz

    float roll_est = 0.0f;
    float pitch_est = 0.0f;
    float yaw_est = 0.0f;

    while (running_) {
        next_tick += interval;

        int ax, ay, az, gx, gy, gz;
        if (read_raw_data(ax, ay, az, gx, gy, gz)) {
            // Sensitivity scale factors based on default config (FS_SEL=0, AFS_SEL=0)
            float accel_x = ax / 16384.0f;
            float accel_y = ay / 16384.0f;
            float accel_z = az / 16384.0f;

            float gyro_x_rate = gx / 131.0f;
            float gyro_y_rate = gy / 131.0f;
            float gyro_z_rate = gz / 131.0f;

            // Calculate Roll and Pitch from Accelerometer using Atan2
            float accel_roll  = std::atan2(accel_y, std::sqrt(accel_x * accel_x + accel_z * accel_z)) * 180.0f / M_PI;
            float accel_pitch = std::atan2(-accel_x, std::sqrt(accel_y * accel_y + accel_z * accel_z)) * 180.0f / M_PI;

            // Apply Complementary Filter
            roll_est = alpha_ * (roll_est + gyro_x_rate * dt_) + (1.0f - alpha_) * accel_roll;
            pitch_est = alpha_ * (pitch_est + gyro_y_rate * dt_) + (1.0f - alpha_) * accel_pitch;

            // Integrate Yaw (Relative only due to lack of magnetometer)
            yaw_est += gyro_z_rate * dt_;

            // Normalize Yaw to [-180, 180]
            if (yaw_est > 180.0f) yaw_est -= 360.0f;
            if (yaw_est < -180.0f) yaw_est += 360.0f;

            // Update thread-safe data
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                current_data_.roll = roll_est;
                current_data_.pitch = pitch_est;
                current_data_.yaw = yaw_est;
            }
        }

        std::this_thread::sleep_until(next_tick);
    }
}
