#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <iomanip>
#include <sstream>

int main() {
    std::cout << "[Calibration] Starting Stereo Calibration Processor..." << std::endl;

    // --- Configuration Parameters ---
    // Change these if your checkerboard dimensions differ
    const float square_size_mm = 25.0f;
    const cv::Size board_size(9, 6); // Number of inner corners (width, height)
    const int num_pairs = 20;
    
    std::vector<std::vector<cv::Point3f>> object_points;
    std::vector<std::vector<cv::Point2f>> image_points_left;
    std::vector<std::vector<cv::Point2f>> image_points_right;
    
    // Precompute 3D coordinates of the checkerboard corners
    std::vector<cv::Point3f> obj;
    for (int i = 0; i < board_size.height; i++) {
        for (int j = 0; j < board_size.width; j++) {
            obj.push_back(cv::Point3f(j * square_size_mm, i * square_size_mm, 0.0f));
        }
    }
    
    cv::Size image_size;
    int valid_pairs = 0;

    for (int i = 0; i < num_pairs; i++) {
        std::stringstream ss_left, ss_right;
        ss_left << "calib_left_" << std::setfill('0') << std::setw(2) << i << ".png";
        ss_right << "calib_right_" << std::setfill('0') << std::setw(2) << i << ".png";
        
        cv::Mat img_left = cv::imread(ss_left.str(), cv::IMREAD_GRAYSCALE);
        cv::Mat img_right = cv::imread(ss_right.str(), cv::IMREAD_GRAYSCALE);
        
        if (img_left.empty() || img_right.empty()) {
            std::cerr << "[Calibration] Failed to load pair " << i << std::endl;
            continue;
        }
        
        if (image_size.width == 0) {
            image_size = img_left.size();
        }
        
        std::vector<cv::Point2f> corners_left, corners_right;
        bool found_left = cv::findChessboardCorners(img_left, board_size, corners_left,
                                                    cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
        bool found_right = cv::findChessboardCorners(img_right, board_size, corners_right,
                                                     cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
        
        if (found_left && found_right) {
            cv::cornerSubPix(img_left, corners_left, cv::Size(11, 11), cv::Size(-1, -1),
                             cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.1));
            cv::cornerSubPix(img_right, corners_right, cv::Size(11, 11), cv::Size(-1, -1),
                             cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.1));
                             
            image_points_left.push_back(corners_left);
            image_points_right.push_back(corners_right);
            object_points.push_back(obj);
            
            std::cout << "[Calibration] Pair " << i << ": SUCCESS" << std::endl;
            valid_pairs++;
        } else {
            std::cout << "[Calibration] Pair " << i << ": FAILED (Corners not fully detected)" << std::endl;
        }
    }
    
    if (valid_pairs < 5) {
        std::cerr << "[Calibration] ERROR: Not enough valid pairs for robust calibration. Found: " << valid_pairs << std::endl;
        return -1;
    }
    
    std::cout << "\n[Calibration] Extracted corners from " << valid_pairs << " pairs. Commencing stereo calibration..." << std::endl;

    cv::Mat K1, D1, K2, D2, R, T, E, F;
    
    // Initial intrinsic guess can help, but we'll let the solver calculate it
    double rms = cv::stereoCalibrate(object_points, image_points_left, image_points_right,
                                     K1, D1, K2, D2, image_size, R, T, E, F,
                                     cv::CALIB_FIX_ASPECT_RATIO | cv::CALIB_ZERO_TANGENT_DIST | cv::CALIB_SAME_FOCAL_LENGTH,
                                     cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 100, 1e-5));
                                     
    std::cout << "[Calibration] Stereo Calibration finished. RMS Error: " << rms << std::endl;
    
    std::cout << "[Calibration] Commencing stereo rectification..." << std::endl;
    cv::Mat R1, R2, P1, P2, Q;
    cv::stereoRectify(K1, D1, K2, D2, image_size, R, T, R1, R2, P1, P2, Q,
                      cv::CALIB_ZERO_DISPARITY, 0, image_size);
                      
    std::cout << "[Calibration] Saving matrices to stereo_calib.xml..." << std::endl;
    cv::FileStorage fs("stereo_calib.xml", cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        std::cerr << "[Calibration] ERROR: Failed to open stereo_calib.xml for writing!" << std::endl;
        return -1;
    }
    
    fs << "K1" << K1; fs << "D1" << D1;
    fs << "K2" << K2; fs << "D2" << D2;
    fs << "R1" << R1; fs << "P1" << P1;
    fs << "R2" << R2; fs << "P2" << P2;
    fs << "Q" << Q;
    fs.release();
    
    std::cout << "[Calibration] Success! Calibration saved." << std::endl;
    return 0;
}
