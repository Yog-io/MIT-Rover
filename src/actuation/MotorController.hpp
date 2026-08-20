#pragma once

#include <mutex>
#include <thread>
#include <atomic>
#include <cstdint>

class MotorController {
public:
    MotorController();
    ~MotorController();

    // Initialize GPIO pins
    bool initialize();

    // Set linear and angular velocity
    void set_velocity(float linear_speed, float angular_speed);

    // Emergency stop
    void stop();

private:
    void watchdog_loop();
    void set_motor_pwms(float left_pwm, float right_pwm);
    uint64_t get_current_time_ms();

    std::mutex hardware_mutex_;
    std::atomic<bool> running_{false};
    std::thread watchdog_thread_;
    std::atomic<uint64_t> last_cmd_time_ms_{0};

    int gpio_handle_ = -1;

    // Pin definitions (BCM numbering)
    const int LEFT_DIR_PIN = 23;
    const int LEFT_PWM_PIN = 18;
    const int RIGHT_DIR_PIN = 24;
    const int RIGHT_PWM_PIN = 19;
    
    // Physical params
    const float TRACK_WIDTH = 0.5f; // meters
};
