#pragma once

#include "actuation/MotorController.hpp"
#include "vision/HazardMapper.hpp"
#include "SharedState.hpp"
#include <mutex>
#include <string>

class NavigationManager {
public:
    NavigationManager(MotorController* motor_ctrl, RoverState* rover_state, HazardState* hazard_state, std::mutex* state_mutex, std::mutex* hazard_mutex);
    ~NavigationManager() = default;

    // Called continuously in the vision loop
    void tick(const HazardReport& report);

private:
    MotorController* motor_ctrl_;
    RoverState* rover_state_;
    HazardState* hazard_state_;
    std::mutex* state_mutex_;
    std::mutex* hazard_mutex_;
};
