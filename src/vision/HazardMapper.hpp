#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include <array>
#include <vector>
#include <limits>
#include <string>

// A single cell in the 2.5D elevation grid
struct LocalCell {
    float z_min = std::numeric_limits<float>::max();
    float z_max = std::numeric_limits<float>::lowest();
};

struct HazardReport {
    // 100x100 grid representing 5m x 5m area at 5cm resolution
    // Cell value: 0 (clear), 255 (lethal obstacle)
    std::array<std::array<uint8_t, 100>, 100> costmap;
    
    // Raw heights array so GlobalMapper can project the actual 3D structure
    std::array<std::array<LocalCell, 100>, 100> raw_elevation_grid;

    // Distance to the closest lethal cell in the forward sector (+Y axis)
    float closest_lethal_distance_m;
    
    // The metric depth (meters) at the exact center pixel (cx, cy) of the stereo image
    float center_stereo_depth_m;
};

class HazardMapper {
public:
    // Pass the path to a calibration XML/YML. If empty/invalid, uses synthetic identity matrices.
    HazardMapper(const std::string& calib_file = "");
    ~HazardMapper();

    // Computes SGBM disparity, applies WLS filter, dynamically scales using lidar_distance_m,
    // and generates the 2.5D hazard grid.
    HazardReport process(const cv::Mat& left_y, const cv::Mat& right_y, float lidar_distance_m);

private:
    void load_calibration(const std::string& calib_file);
    void setup_stereo_matchers();

    // StereoSGBM matchers and WLS filter
    cv::Ptr<cv::StereoSGBM> left_matcher_;
    cv::Ptr<cv::StereoMatcher> right_matcher_;
    cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter_;
    
    // Epipolar Rectification Maps
    cv::Mat map1x_, map1y_;
    cv::Mat map2x_, map2y_;

    // Calibration matrices (either loaded or synthetic)
    cv::Mat K1_, D1_, R1_, P1_;
    cv::Mat K2_, D2_, R2_, P2_;
    cv::Mat Q_;

    // Camera intrinsic/extrinsic assumptions
    float baseline_m_ = 0.08f;
    float focal_length_px_ = 600.0f;
    
    // Grid configuration
    const int grid_size_ = 100;
    const float cell_resolution_m_ = 0.05f;
};
