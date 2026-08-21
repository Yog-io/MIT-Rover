#include "actuation/MotorController.hpp"
#include "utils/Logger.hpp"
#include <cmath>
#include <algorithm>
#include <chrono>

#ifndef __APPLE__
#include <lgpio.h>
#endif

MotorController::MotorController() {
    last_cmd_time_ms_ = get_current_time_ms();
}

MotorController::~MotorController() {
    running_ = false;
    if (watchdog_thread_.joinable()) {
        watchdog_thread_.join();
    }
    stop();
    
#ifndef __APPLE__
    if (gpio_handle_ >= 0) {
        lgGpiochipClose(gpio_handle_);
    }
#endif
}

uint64_t MotorController::get_current_time_ms() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

bool MotorController::initialize() {
    std::lock_guard<std::mutex> lock(hardware_mutex_);
#ifdef __APPLE__
    LOG_INFO("MotorController", "Mock hardware initialized (macOS fallback).");
    running_ = true;
    last_cmd_time_ms_ = get_current_time_ms();
    watchdog_thread_ = std::thread(&MotorController::watchdog_loop, this);
    return true;
#else
    gpio_handle_ = lgGpiochipOpen(0);
    if (gpio_handle_ < 0) {
        LOG_ERROR("MotorController", "Failed to open GPIO chip.");
        return false;
    }

    lgGpioClaimOutput(gpio_handle_, 0, LEFT_DIR_PIN, 0);
    lgGpioClaimOutput(gpio_handle_, 0, LEFT_PWM_PIN, 0);
    lgGpioClaimOutput(gpio_handle_, 0, RIGHT_DIR_PIN, 0);
    lgGpioClaimOutput(gpio_handle_, 0, RIGHT_PWM_PIN, 0);
    
    LOG_INFO("MotorController", "GPIO initialized successfully.");
    running_ = true;
    last_cmd_time_ms_ = get_current_time_ms();
    watchdog_thread_ = std::thread(&MotorController::watchdog_loop, this);
    return true;
#endif
}

void MotorController::set_velocity(float linear_speed, float angular_speed) {
    last_cmd_time_ms_ = get_current_time_ms();
    
    // Differential drive kinematics
    float left_pwm = linear_speed - (angular_speed * TRACK_WIDTH / 2.0f);
    float right_pwm = linear_speed + (angular_speed * TRACK_WIDTH / 2.0f);

    // Normalize to -1.0 to 1.0 range if we exceed bounds
    float max_pwm = std::max(1.0f, std::max(std::abs(left_pwm), std::abs(right_pwm)));
    left_pwm /= max_pwm;
    right_pwm /= max_pwm;

    set_motor_pwms(left_pwm, right_pwm);
}

void MotorController::stop() {
    set_motor_pwms(0.0f, 0.0f);
}

void MotorController::set_motor_pwms(float left_pwm, float right_pwm) {
    std::lock_guard<std::mutex> lock(hardware_mutex_);
#ifdef __APPLE__
    // Optional: Log mock PWM updates for debugging
    // std::cout << "[MotorController Mock] L: " << left_pwm << " R: " << right_pwm << std::endl;
#else
    if (gpio_handle_ < 0) return;

    int left_dir = (left_pwm >= 0) ? 1 : 0;
    int right_dir = (right_pwm >= 0) ? 1 : 0;

    lgGpioWrite(gpio_handle_, LEFT_DIR_PIN, left_dir);
    // lgTxPwm arguments: handle, pin, frequency, duty_percent, offset, cycles
    lgTxPwm(gpio_handle_, LEFT_PWM_PIN, 1000, std::abs(left_pwm) * 100.0f, 0, 0); 
    
    lgGpioWrite(gpio_handle_, RIGHT_DIR_PIN, right_dir);
    lgTxPwm(gpio_handle_, RIGHT_PWM_PIN, 1000, std::abs(right_pwm) * 100.0f, 0, 0);
#endif
}

void MotorController::watchdog_loop() {
    uint64_t last_heartbeat = get_current_time_ms();
    while (running_) {
        uint64_t now = get_current_time_ms();
        uint64_t last = last_cmd_time_ms_.load();
        
        if (now - last > 500) {
            // Safety watchdog: stop motors if no velocity command received in 500ms
            stop();
        }
        
        if (now - last_heartbeat > 5000) {
            LOG_INFO("SYS", "Motor_Watchdog: ACTIVE");
            last_heartbeat = now;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}
