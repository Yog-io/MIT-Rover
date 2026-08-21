#pragma once

#include "vision/HazardMapper.hpp"
#include <array>
#include <mutex>
#include <string>
#include <vector>
#include <cstdint>
#include <utility> // for std::pair

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
    // current_yaw_deg : rover heading from IMU (degrees).
    // cmd_linear_v    : the velocity command that was active during this frame (m/s).
    // delta_time_sec  : elapsed time since the last call to this function (seconds).
    //                   Used to integrate dead-reckoning translation.
    void update_map(const HazardReport& local_report,
                    float current_yaw_deg,
                    float cmd_linear_v,
                    float delta_time_sec);

    // Adds a scientific point of interest to the map.
    void add_poi(const ScientificPOI& poi);

    // Returns a serialized JSON representation of only the grid cells that changed
    // since the last call. Clears the internal delta queue after serializing.
    // The client should apply these incremental updates to its local copy of the map.
    std::string get_global_map_json();

private:
    // 400x400 grid representing 20m x 20m at 5cm resolution
    std::array<int8_t, 160000> global_grid_;
    
    // Mutex for thread-safe access to the global grid and delta queue
    std::mutex grid_mutex_;

    // Scientific points of interest
    std::vector<ScientificPOI> scientific_pois_;
    std::mutex pois_mutex_;

    // Dead-reckoning rover position in the global map coordinate frame (meters).
    // Origin (0, 0) corresponds to the center of the global grid.
    float rover_global_x_m_ = 0.0f;
    float rover_global_y_m_ = 0.0f;

    // Delta queue: accumulates (flat_index, value) pairs for cells that changed
    // since the last call to get_global_map_json(). Guarded by grid_mutex_.
    std::vector<std::pair<int, int8_t>> map_deltas_;
    
    // Constants
    static constexpr int GLOBAL_SIZE = 400;
    static constexpr int LOCAL_SIZE = 100;
    static constexpr float RESOLUTION_M = 0.05f;
};
