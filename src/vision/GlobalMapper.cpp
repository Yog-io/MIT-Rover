#include "vision/GlobalMapper.hpp"
#include <nlohmann/json.hpp>
#include <cmath>

using json = nlohmann::json;

GlobalMapper::GlobalMapper() {
    global_grid_.fill(0);
}

void GlobalMapper::update_map(const HazardReport& local_report, float current_yaw_deg) {
    std::lock_guard<std::mutex> lock(grid_mutex_);

    // Convert yaw to radians
    float yaw_rad = current_yaw_deg * M_PI / 180.0f;
    float cos_yaw = std::cos(yaw_rad);
    float sin_yaw = std::sin(yaw_rad);

    // Center indices of the local map and global map
    int local_cx = LOCAL_SIZE / 2;
    int local_cy = LOCAL_SIZE / 2;
    int global_cx = GLOBAL_SIZE / 2;
    int global_cy = GLOBAL_SIZE / 2;

    for (int y = 0; y < LOCAL_SIZE; ++y) {
        for (int x = 0; x < LOCAL_SIZE; ++x) {
            uint8_t cell_value = local_report.costmap[y][x];
            
            // Only map active hazard points
            if (cell_value > 0) {
                // Local coordinate relative to rover center
                float lx = static_cast<float>(x - local_cx);
                float ly = static_cast<float>(y - local_cy);

                // Rotate by current yaw
                float gx = lx * cos_yaw - ly * sin_yaw;
                float gy = lx * sin_yaw + ly * cos_yaw;

                // Map to global grid index
                int global_x = global_cx + static_cast<int>(std::round(gx));
                int global_y = global_cy + static_cast<int>(std::round(gy));

                // Check bounds
                if (global_x >= 0 && global_x < GLOBAL_SIZE && global_y >= 0 && global_y < GLOBAL_SIZE) {
                    global_grid_[global_y * GLOBAL_SIZE + global_x] = static_cast<int8_t>(cell_value);
                }
            }
        }
    }
}

std::string GlobalMapper::get_global_map_json() {
    std::lock_guard<std::mutex> lock(grid_mutex_);

    // Serialize active points to keep JSON lightweight instead of a full 160k array
    json active_points = json::array();
    
    for (int i = 0; i < GLOBAL_SIZE * GLOBAL_SIZE; ++i) {
        if (global_grid_[i] != 0) {
            int y = i / GLOBAL_SIZE;
            int x = i % GLOBAL_SIZE;
            active_points.push_back({x, y, global_grid_[i]});
        }
    }
    
    json output;
    output["type"] = "global_map";
    output["size"] = GLOBAL_SIZE;
    output["resolution_m"] = RESOLUTION_M;
    output["points"] = active_points;

    return output.dump();
}
