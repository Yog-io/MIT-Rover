#include "navigation/NavigationManager.hpp"
#include <iostream>

NavigationManager::NavigationManager(MotorController* motor_ctrl, RoverState* rover_state, HazardState* hazard_state, std::mutex* state_mutex, std::mutex* hazard_mutex)
    : motor_ctrl_(motor_ctrl), rover_state_(rover_state), hazard_state_(hazard_state), state_mutex_(state_mutex), hazard_mutex_(hazard_mutex) {}

void NavigationManager::tick(const HazardReport& report) {
    std::string mode_str;
    float cmd_linear_v = 0.0f;
    float cmd_angular_w = 0.0f;

    // Read current requested state
    {
        std::lock_guard<std::mutex> lock(*state_mutex_);
        mode_str = rover_state_->mode;
        cmd_linear_v = rover_state_->linear_v;
        cmd_angular_w = rover_state_->angular_w;
    }

    if (mode_str == "STANDBY") {
        motor_ctrl_->stop();
        return;
    }

    if (mode_str == "MANUAL") {
        // Hazard Arbitration (Manual Mode)
        if (report.closest_lethal_distance_m < 0.5f && cmd_linear_v > 0.0f) {
            std::cout << "[NavManager] HAZARD OVERRIDE: Suppressing forward drive!" << std::endl;
            cmd_linear_v = 0.0f; // Override forward drive, but allow turning/reversing
            
            std::lock_guard<std::mutex> lock(*hazard_mutex_);
            hazard_state_->level = "CRITICAL";
        }
        motor_ctrl_->set_velocity(cmd_linear_v, cmd_angular_w);

    } else if (mode_str == "AUTONOMOUS") {
        // Simple Autonomous Obstacle Avoidance MVP
        if (report.closest_lethal_distance_m < 0.5f) {
            // Blocked closely: Spin in place to find a clear path
            cmd_linear_v = 0.0f;
            cmd_angular_w = 1.0f; // Priority spin
        } else if (report.closest_lethal_distance_m < 1.0f) {
            // Obstacle approaching: Slow down and curve away
            cmd_linear_v = 0.2f;
            cmd_angular_w = 0.5f; 
        } else {
            // Clear path: Move forward
            cmd_linear_v = 0.5f;
            cmd_angular_w = 0.0f;
        }

        // Update the state so the UI dashboard reflects the autonomous commands
        {
            std::lock_guard<std::mutex> lock(*state_mutex_);
            rover_state_->linear_v = cmd_linear_v;
            rover_state_->angular_w = cmd_angular_w;
        }

        motor_ctrl_->set_velocity(cmd_linear_v, cmd_angular_w);
    }
}
