#include "vision/GlobalMapper.hpp"
#include <nlohmann/json.hpp>
#include <cmath>

using json = nlohmann::json;

GlobalMapper::GlobalMapper() {
    global_grid_.fill(0);
}

void GlobalMapper::add_poi(const ScientificPOI& poi) {
    std::lock_guard<std::mutex> lock(pois_mutex_);
    scientific_pois_.push_back(poi);
}

void GlobalMapper::update_map(const HazardReport& local_report,
                               float current_yaw_deg,
                               float cmd_linear_v,
                               float delta_time_sec) {
    std::lock_guard<std::mutex> lock(grid_mutex_);

    // --- Dead-Reckoning Integration ---
    // Integrate the rover's velocity over the elapsed time to update its
    // estimated position in the global map. This is the missing translation
    // that prevents the local map from spinning in place at the global origin.
    float yaw_rad = current_yaw_deg * M_PI / 180.0f;
    rover_global_x_m_ += cmd_linear_v * delta_time_sec * std::cos(yaw_rad);
    rover_global_y_m_ += cmd_linear_v * delta_time_sec * std::sin(yaw_rad);

    float cos_yaw = std::cos(yaw_rad);
    float sin_yaw = std::sin(yaw_rad);

    // Center indices of the local map and global map
    int local_cx = LOCAL_SIZE / 2;
    int local_cy = LOCAL_SIZE / 2;
    int global_cx = GLOBAL_SIZE / 2;
    int global_cy = GLOBAL_SIZE / 2;

    // Translation offset in grid cells from the global origin to rover position
    int tx_cells = static_cast<int>(std::round(rover_global_x_m_ / RESOLUTION_M));
    int ty_cells = static_cast<int>(std::round(rover_global_y_m_ / RESOLUTION_M));

    for (int y = 0; y < LOCAL_SIZE; ++y) {
        for (int x = 0; x < LOCAL_SIZE; ++x) {
            uint8_t cell_value = local_report.costmap[y][x];
            
            // Only map active hazard points
            if (cell_value > 0) {
                // Local coordinate relative to rover center (in grid cells)
                float lx = static_cast<float>(x - local_cx);
                float ly = static_cast<float>(y - local_cy);

                // Rotate local cell by rover yaw to align with global frame
                float gx = lx * cos_yaw - ly * sin_yaw;
                float gy = lx * sin_yaw + ly * cos_yaw;

                // Translate from rover's current global position into the global grid
                int global_x = global_cx + tx_cells + static_cast<int>(std::round(gx));
                int global_y = global_cy + ty_cells + static_cast<int>(std::round(gy));

                // Bounds check
                if (global_x >= 0 && global_x < GLOBAL_SIZE &&
                    global_y >= 0 && global_y < GLOBAL_SIZE) {
                    int flat_idx = global_y * GLOBAL_SIZE + global_x;
                    int8_t new_val = static_cast<int8_t>(cell_value);

                    // Only update & log a delta if the cell value actually changes
                    if (global_grid_[flat_idx] != new_val) {
                        global_grid_[flat_idx] = new_val;
                        map_deltas_.emplace_back(flat_idx, new_val);
                    }
                }
            }
        }
    }
}

std::string GlobalMapper::get_global_map_json() {
    // Snapshot and clear the delta queue under the grid lock
    std::vector<std::pair<int, int8_t>> deltas_snapshot;
    {
        std::lock_guard<std::mutex> lock(grid_mutex_);
        deltas_snapshot = std::move(map_deltas_);
        map_deltas_.clear();
    }

    // Serialize only the changed points — O(deltas) not O(160k)
    json active_points = json::array();
    for (const auto& [flat_idx, val] : deltas_snapshot) {
        int y = flat_idx / GLOBAL_SIZE;
        int x = flat_idx % GLOBAL_SIZE;
        active_points.push_back({x, y, val});
    }

    json pois_array = json::array();
    {
        std::lock_guard<std::mutex> poi_lock(pois_mutex_);
        for (const auto& poi : scientific_pois_) {
            pois_array.push_back({
                {"type", poi.type},
                {"x", poi.x},
                {"y", poi.y},
                {"temp", poi.temp},
                {"pressure", poi.pressure},
                {"moisture_val", poi.moisture_val},
                {"timestamp_ms", poi.timestamp_ms}
            });
        }
    }
    
    json output;
    output["type"] = "global_map";
    output["size"] = GLOBAL_SIZE;
    output["resolution_m"] = RESOLUTION_M;
    output["points"] = active_points; // Delta updates only — not the full 160k grid
    output["pois"] = pois_array;

    return output.dump();
}
