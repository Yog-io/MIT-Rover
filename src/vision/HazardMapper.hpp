#pragma once

#include <opencv2/opencv.hpp>
#include <array>
#include <vector>
#include <limits>

struct HazardReport {
    // 100x100 grid representing 5m x 5m area at 5cm resolution
    // Cell value: 0 (clear), 255 (lethal obstacle)
    std::array<std::array<uint8_t, 100>, 100> costmap;
    
    // Distance to the closest lethal cell in the forward sector (+Y axis)
    float closest_lethal_distance_m;
};

class HazardMapper {
public:
    HazardMapper();
    ~HazardMapper();

    // Computes disparity, projects to 3D, and generates the 2.5D hazard grid.
    HazardReport process(const cv::Mat& left_y, const cv::Mat& right_y);

private:
    cv::Ptr<cv::StereoBM> stereo_;
    
    // Camera intrinsic/extrinsic assumptions
    const float baseline_m_ = 0.08f;
    const float focal_length_px_ = 600.0f;
    
    // Grid configuration
    const int grid_size_ = 100;
    const float cell_resolution_m_ = 0.05f;

    // A single cell in the 2.5D elevation grid
    struct Cell {
        float z_min = std::numeric_limits<float>::max();
        float z_max = std::numeric_limits<float>::lowest();
    };
};
