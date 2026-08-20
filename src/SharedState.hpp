#pragma once

#include <string>
#include <vector>
#include <cstdint>

struct RoverState {
    std::string mode = "STANDBY";
    float linear_v = 0.0f;
    float angular_w = 0.0f;
    float pos_x = 0.0f;
    float pos_y = 0.0f;
    float heading_deg = 0.0f;
    bool arm_deployed = false;
};

struct HazardState {
    std::string level = "CLEAR";
    float distance_m = 99.0f;
    std::string sector = "CENTER";
    std::string type = "ROCK"; // Kept static for mock
    std::vector<uint8_t> mini_map; // Downsampled 10x10 map for lightweight UI streaming
};
