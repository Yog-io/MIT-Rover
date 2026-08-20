#pragma once

#include "vision/HazardMapper.hpp"
#include <array>
#include <mutex>
#include <string>
#include <vector>
#include <cstdint>

struct ScientificPOI {
    std::string type;
    float x;
    float y;
    float temp;
    float pressure;
    int moisture_val;
    uint64_t timestamp_ms;
};

class GlobalMapper {
public:
    GlobalMapper();
    ~GlobalMapper() = default;

    // Updates the global map with a new local hazard report.
    // current_yaw_deg is the rover's heading.
    void update_map(const HazardReport& local_report, float current_yaw_deg);

    // Adds a scientific point of interest to the map.
    void add_poi(const ScientificPOI& poi);

    // Returns a serialized JSON representation of the active global grid.
    std::string get_global_map_json();

private:
    // 400x400 grid representing 20m x 20m at 5cm resolution
    std::array<int8_t, 160000> global_grid_;
    
    // Mutex for thread-safe access to the global grid
    std::mutex grid_mutex_;

    // Scientific points of interest
    std::vector<ScientificPOI> scientific_pois_;
    std::mutex pois_mutex_;
    
    // Constants
    static constexpr int GLOBAL_SIZE = 400;
    static constexpr int LOCAL_SIZE = 100;
    static constexpr float RESOLUTION_M = 0.05f;
};
