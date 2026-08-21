#include "vision/HazardMapper.hpp"
#include "utils/Logger.hpp"
#include <omp.h>
#include <cmath>

HazardMapper::HazardMapper(const std::string& calib_file) {
    load_calibration(calib_file);
    setup_stereo_matchers();
}

HazardMapper::~HazardMapper() {}

void HazardMapper::load_calibration(const std::string& calib_file) {
    bool loaded = false;
    
    if (!calib_file.empty()) {
        cv::FileStorage fs(calib_file, cv::FileStorage::READ);
        if (fs.isOpened()) {
            fs["K1"] >> K1_; fs["D1"] >> D1_;
            fs["K2"] >> K2_; fs["D2"] >> D2_;
            fs["R1"] >> R1_; fs["P1"] >> P1_;
            fs["R2"] >> R2_; fs["P2"] >> P2_;
            fs["Q"] >> Q_;
            fs.release();
            loaded = true;
            LOG_INFO("HazardMapper", "Loaded stereo calibration from " + calib_file);
            
            // Extract baseline and focal length from P2
            if (!P2_.empty()) {
                focal_length_px_ = P2_.at<double>(0, 0);
                baseline_m_ = std::abs(P2_.at<double>(0, 3) / focal_length_px_);
            }
        } else {
            LOG_ERROR("HazardMapper", "Failed to open calibration file: " + calib_file);
        }
    }
    
    if (!loaded) {
        LOG_WARN("HazardMapper", "Using synthetic identity calibration matrices!");
        // Provide synthetic/dummy identity matrices to prevent crashes
        K1_ = cv::Mat::eye(3, 3, CV_64F); K1_.at<double>(0,0) = focal_length_px_; K1_.at<double>(1,1) = focal_length_px_;
        K1_.at<double>(0,2) = 320.0; K1_.at<double>(1,2) = 240.0;
        K2_ = K1_.clone();
        D1_ = cv::Mat::zeros(1, 5, CV_64F);
        D2_ = cv::Mat::zeros(1, 5, CV_64F);
        R1_ = cv::Mat::eye(3, 3, CV_64F);
        R2_ = cv::Mat::eye(3, 3, CV_64F);
        P1_ = cv::Mat::eye(3, 4, CV_64F); P1_.at<double>(0,0) = focal_length_px_; P1_.at<double>(1,1) = focal_length_px_;
        P2_ = P1_.clone();
        P2_.at<double>(0,3) = -focal_length_px_ * baseline_m_; // Tx
        Q_ = cv::Mat::zeros(4, 4, CV_64F);
    }

    // Precompute the rectification maps for a 640x480 resolution
    cv::Size img_size(640, 480);
    cv::initUndistortRectifyMap(K1_, D1_, R1_, P1_, img_size, CV_32FC1, map1x_, map1y_);
    cv::initUndistortRectifyMap(K2_, D2_, R2_, P2_, img_size, CV_32FC1, map2x_, map2y_);
}

void HazardMapper::setup_stereo_matchers() {
    int minDisparity = 0;
    int numDisparities = 64; // Decreased from 128 to 64 to reduce CPU load
    int blockSize = 7;       // Smaller block size

    left_matcher_ = cv::StereoSGBM::create(
        minDisparity, numDisparities, blockSize,
        8 * 3 * blockSize * blockSize,  // P1: 3 channels
        32 * 3 * blockSize * blockSize, // P2: 3 channels
        1,   // disp12MaxDiff
        63,  // preFilterCap
        10,  // uniquenessRatio
        100, // speckleWindowSize
        32,  // speckleRange
        cv::StereoSGBM::MODE_SGBM_3WAY
    );

    right_matcher_ = cv::ximgproc::createRightMatcher(left_matcher_);

    wls_filter_ = cv::ximgproc::createDisparityWLSFilter(left_matcher_);
    wls_filter_->setLambda(8000.0);
    wls_filter_->setSigmaColor(1.5);
}

HazardReport HazardMapper::process(const cv::Mat& left_y, const cv::Mat& right_y, float lidar_distance_m) {
    cv::Mat left_rect, right_rect;
    
    // 1. Mandatory Epipolar Rectification
    cv::remap(left_y, left_rect, map1x_, map1y_, cv::INTER_LINEAR);
    cv::remap(right_y, right_rect, map2x_, map2y_, cv::INTER_LINEAR);

    // 2. StereoSGBM + WLS Filtering
    cv::Mat left_disp, right_disp, filtered_disp;
    
    left_matcher_->compute(left_rect, right_rect, left_disp);
    right_matcher_->compute(right_rect, left_rect, right_disp);
    
    wls_filter_->filter(left_disp, left_rect, filtered_disp, right_disp);

    // Filtered disparity is 16-bit signed, with a scale factor of 16
    int width = filtered_disp.cols;
    int height = filtered_disp.rows;
    float cx = width / 2.0f;
    float cy = height / 2.0f;

    HazardReport report;
    report.closest_lethal_distance_m = std::numeric_limits<float>::infinity();

    // 3. LiDAR Failsafe — stored as a parallel reading, NOT used to scale stereo depth.
    // This preserves absolute stereo geometry correctness while still giving the
    // downstream consumer (NavigationManager, UI) a single-point ground-truth distance.
    report.lidar_depth_m = (lidar_distance_m > 0.0f) ? lidar_distance_m : -1.0f;

    int16_t center_d_val = filtered_disp.at<int16_t>(static_cast<int>(cy), static_cast<int>(cx));
    float center_d = center_d_val / 16.0f;
    
    if (center_d > 0.0f) {
        // Absolute stereo depth from calibrated geometry: Z = (f * B) / d
        report.center_stereo_depth_m = (focal_length_px_ * baseline_m_) / center_d;
    }

    // Initialize raw elevation grid bounds
    for (int y = 0; y < grid_size_; ++y) {
        for (int x = 0; x < grid_size_; ++x) {
            report.raw_elevation_grid[y][x].z_min = std::numeric_limits<float>::max();
            report.raw_elevation_grid[y][x].z_max = std::numeric_limits<float>::lowest();
            report.costmap[y][x] = 0;
        }
    }

    // Thread-local master grid for openmp
    std::vector<std::vector<LocalCell>> master_grid(grid_size_, std::vector<LocalCell>(grid_size_));

    #pragma omp parallel
    {
        std::vector<std::vector<LocalCell>> local_grid(grid_size_, std::vector<LocalCell>(grid_size_));

        #pragma omp for nowait
        for (int v = 0; v < height; ++v) {
            const int16_t* row_ptr = filtered_disp.ptr<int16_t>(v);
            
            for (int u = 0; u < width; ++u) {
                int16_t d_val = row_ptr[u];
                if (d_val <= 0) continue;
                
                float d = d_val / 16.0f;
                
                // Absolute stereo depth: Z = (f * B) / d
                float Z = (focal_length_px_ * baseline_m_) / d;
                
                float X = (u - cx) * Z / focal_length_px_;
                float Y = (v - cy) * Z / focal_length_px_;
                
                float world_y = Z; 
                float world_x = X;
                float world_z = -Y; 

                if (world_y < 0.0f || world_y >= (grid_size_ * cell_resolution_m_)) continue;
                
                int grid_y = static_cast<int>(world_y / cell_resolution_m_);
                int grid_x = static_cast<int>(world_x / cell_resolution_m_) + (grid_size_ / 2);
                
                if (grid_x >= 0 && grid_x < grid_size_ && grid_y >= 0 && grid_y < grid_size_) {
                    auto& cell = local_grid[grid_y][grid_x];
                    if (world_z < cell.z_min) cell.z_min = world_z;
                    if (world_z > cell.z_max) cell.z_max = world_z;
                }
            }
        }

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

    // Flag LETHAL obstacles and populate raw elevation
    for (int y = 0; y < grid_size_; ++y) {
        for (int x = 0; x < grid_size_; ++x) {
            const auto& cell = master_grid[y][x];
            
            // Populate the raw grid for GlobalMapper
            report.raw_elevation_grid[y][x] = cell;

            if (cell.z_min <= cell.z_max) {
                float height_diff = cell.z_max - cell.z_min;
                
                // 5 cm lethal threshold
                if (height_diff > 0.05f) { 
                    report.costmap[y][x] = 255;
                    
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
