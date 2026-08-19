#include "vision/HazardMapper.hpp"
#include <omp.h>
#include <cmath>

HazardMapper::HazardMapper() {
    stereo_ = cv::StereoBM::create(64, 9);
}

HazardMapper::~HazardMapper() {}

HazardReport HazardMapper::process(const cv::Mat& left_y, const cv::Mat& right_y) {
    cv::Mat disparity_16s;
    
    // OpenCV natively optimizes this block matching. 
    // We let it run on the full image rather than slicing, avoiding border artifacts.
    stereo_->compute(left_y, right_y, disparity_16s);

    int width = disparity_16s.cols;
    int height = disparity_16s.rows;
    float cx = width / 2.0f;
    float cy = height / 2.0f;

    // Master grid to hold the merged results from all threads
    std::vector<std::vector<Cell>> master_grid(grid_size_, std::vector<Cell>(grid_size_));

    // OpenMP parallel region for 3D reprojection and grid binning
    #pragma omp parallel
    {
        // Thread-local grid to avoid mutex locks during the intense binning phase
        std::vector<std::vector<Cell>> local_grid(grid_size_, std::vector<Cell>(grid_size_));

        // Split the work by image rows across available CPU cores
        #pragma omp for nowait
        for (int v = 0; v < height; ++v) {
            const int16_t* row_ptr = disparity_16s.ptr<int16_t>(v);
            
            for (int u = 0; u < width; ++u) {
                int16_t d_val = row_ptr[u];
                
                // Discard invalid disparities (StereoBM returns <= 0 for invalid/occluded pixels)
                if (d_val <= 0) continue;
                
                float d = d_val / 16.0f;
                
                // Standard pinhole geometry reprojection
                float Z = (focal_length_px_ * baseline_m_) / d;
                float X = (u - cx) * Z / focal_length_px_;
                float Y = (v - cy) * Z / focal_length_px_;
                
                // Map Camera Frame to Rover/World Frame:
                // Camera Z (depth) -> World Y (forward into the grid)
                // Camera X (right) -> World X (right in the grid)
                // Camera Y (down)  -> World Z (elevation/height, flip sign so up is positive)
                float world_y = Z; 
                float world_x = X;
                float world_z = -Y; 

                // Discard points behind the rover or outside the 5-meter range
                if (world_y < 0.0f || world_y >= (grid_size_ * cell_resolution_m_)) continue;
                
                // Calculate grid cell indices
                // Rover is positioned at bottom-center (X=50, Y=0)
                int grid_y = static_cast<int>(world_y / cell_resolution_m_);
                int grid_x = static_cast<int>(world_x / cell_resolution_m_) + (grid_size_ / 2);
                
                // Bounds check
                if (grid_x >= 0 && grid_x < grid_size_ && grid_y >= 0 && grid_y < grid_size_) {
                    auto& cell = local_grid[grid_y][grid_x];
                    if (world_z < cell.z_min) cell.z_min = world_z;
                    if (world_z > cell.z_max) cell.z_max = world_z;
                }
            }
        }

        // Merge thread-local grid into the master grid safely
        #pragma omp critical
        {
            for (int r = 0; r < grid_size_; ++r) {
                for (int c = 0; c < grid_size_; ++c) {
                    if (local_grid[r][c].z_min < master_grid[r][c].z_min) 
                        master_grid[r][c].z_min = local_grid[r][c].z_min;
                    if (local_grid[r][c].z_max > master_grid[r][c].z_max) 
                        master_grid[r][c].z_max = local_grid[r][c].z_max;
                }
            }
        }
    }

    // Flag LETHAL obstacles and construct HazardReport
    HazardReport report;
    report.closest_lethal_distance_m = std::numeric_limits<float>::infinity();

    // Iterate through the master grid
    for (int y = 0; y < grid_size_; ++y) {
        for (int x = 0; x < grid_size_; ++x) {
            report.costmap[y][x] = 0; // Initialize as clear
            
            const auto& cell = master_grid[y][x];
            
            // Only evaluate cells that actually received 3D points
            if (cell.z_min <= cell.z_max) {
                float height_diff = cell.z_max - cell.z_min;
                
                // 5 cm lethal threshold
                if (height_diff > 0.05f) { 
                    report.costmap[y][x] = 255;
                    
                    // Check if obstacle is directly in the forward driving sector (central 1 meter)
                    // X indices between 40 and 60 represent -0.5m to +0.5m from the rover's center
                    if (x >= 40 && x <= 60) {
                        float dist_m = y * cell_resolution_m_;
                        if (dist_m < report.closest_lethal_distance_m) {
                            report.closest_lethal_distance_m = dist_m;
                        }
                    }
                }
            }
        }
    }

    return report;
}
