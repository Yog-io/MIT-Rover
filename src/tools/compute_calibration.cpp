#include <iomanip>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>
#include <unistd.h>
#include <vector>

int main() {
  std::cout << "=================================================="
            << std::endl;
  std::cout << "[Calibration] Starting Stereo Calibration Processor"
            << std::endl;
  std::cout << "=================================================="
            << std::endl;

  // --- Configuration Parameters ---
  // User must change these to match their screen's INNER corners
  const int BOARD_W = 8;
  const int BOARD_H = 8;
  const float SQUARE_SIZE = 25.0f;
  const int NUM_PAIRS = 20;

  std::cout << "[Calibration] Expected Board: " << BOARD_W << "x" << BOARD_H
            << " (Inner Corners)" << std::endl;
  std::cout << "[Calibration] Square Size: " << SQUARE_SIZE << " mm"
            << std::endl;

  const cv::Size board_size(BOARD_W, BOARD_H);

  std::vector<std::vector<cv::Point3f>> object_points;
  std::vector<std::vector<cv::Point2f>> image_points_left;
  std::vector<std::vector<cv::Point2f>> image_points_right;

  // Precompute 3D coordinates of the checkerboard corners
  std::vector<cv::Point3f> obj;
  for (int i = 0; i < BOARD_H; i++) {
    for (int j = 0; j < BOARD_W; j++) {
      obj.push_back(cv::Point3f(j * SQUARE_SIZE, i * SQUARE_SIZE, 0.0f));
    }
  }

  cv::Size image_size;
  int valid_pairs = 0;

  std::cout << "\n[Calibration] --- Commencing Corner Detection ---"
            << std::endl;
  for (int i = 0; i < NUM_PAIRS; i++) {
    std::stringstream ss_left, ss_right;
    ss_left << "calib_left_" << std::setfill('0') << std::setw(2) << i
            << ".png";
    ss_right << "calib_right_" << std::setfill('0') << std::setw(2) << i
             << ".png";

    cv::Mat img_left = cv::imread(ss_left.str(), cv::IMREAD_GRAYSCALE);
    cv::Mat img_right = cv::imread(ss_right.str(), cv::IMREAD_GRAYSCALE);

    if (img_left.empty() || img_right.empty()) {
      std::cout << "[Pair " << i << "] FAILED (Images not found/readable)"
                << std::endl;
      continue;
    }

    if (image_size.width == 0) {
      image_size = img_left.size();
    }
    
    // Apply CLAHE to aggressively fix screen glare, low contrast, and lighting issues
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(8, 8));
    cv::Mat img_left_eq, img_right_eq;
    clahe->apply(img_left, img_left_eq);
    clahe->apply(img_right, img_right_eq);

    std::vector<cv::Point2f> corners_left, corners_right;
    int flags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK;
    
    bool found_left = cv::findChessboardCorners(img_left_eq, board_size, corners_left, flags);
    bool found_right = cv::findChessboardCorners(img_right_eq, board_size, corners_right, flags);
    
    // Auto-detect common sizes on failure (Debug assistance)
    if ((!found_left || !found_right) && i == 0) {
        std::vector<cv::Size> common_sizes = {cv::Size(9, 6), cv::Size(8, 6), cv::Size(7, 7), cv::Size(7, 5)};
        for (auto& s : common_sizes) {
            if (cv::findChessboardCorners(img_left_eq, s, corners_left, flags)) {
                std::cerr << "\n[Calibration] DEBUG HINT: We failed to find an " << BOARD_W << "x" << BOARD_H 
                          << " board, but successfully found a " << s.width << "x" << s.height 
                          << " board in the first image! Please update BOARD_W and BOARD_H.\n" << std::endl;
                break;
            }
        }
    }

    if (found_left && found_right) {
      cv::cornerSubPix(
          img_left, corners_left, cv::Size(11, 11), cv::Size(-1, -1),
          cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER,
                           30, 0.1));
      cv::cornerSubPix(
          img_right, corners_right, cv::Size(11, 11), cv::Size(-1, -1),
          cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER,
                           30, 0.1));

      image_points_left.push_back(corners_left);
      image_points_right.push_back(corners_right);
      object_points.push_back(obj);

      std::cout << "[Pair " << i << "] LEFT: OK | RIGHT: OK -> ACCEPTED"
                << std::endl;
      valid_pairs++;
    } else {
      std::cout << "[Pair " << i << "] LEFT: " << (found_left ? "OK" : "FAILED")
                << " | RIGHT: " << (found_right ? "OK" : "FAILED")
                << " -> REJECTED" << std::endl;
    }
  }

  // Lowered threshold: proceed if at least 5 valid pairs are found
  if (valid_pairs < 5) {
    std::cerr << "\n[Calibration] FATAL ERROR: Only " << valid_pairs
              << " valid pairs found. At least 5 are required." << std::endl;
    return -1;
  }

  std::cout << "\n[Calibration] Extracted corners from " << valid_pairs
            << " pairs. Commencing stereo calibration..." << std::endl;

  cv::Mat K1, D1, K2, D2, R, T, E, F;

  double rms = cv::stereoCalibrate(
      object_points, image_points_left, image_points_right, K1, D1, K2, D2,
      image_size, R, T, E, F,
      cv::CALIB_FIX_ASPECT_RATIO | cv::CALIB_ZERO_TANGENT_DIST |
          cv::CALIB_SAME_FOCAL_LENGTH,
      cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 100,
                       1e-5));

  std::cout << "[Calibration] Stereo Calibration finished. RMS Error: " << rms
            << std::endl;

  std::cout << "[Calibration] Commencing stereo rectification..." << std::endl;
  cv::Mat R1, R2, P1, P2, Q;
  cv::stereoRectify(K1, D1, K2, D2, image_size, R, T, R1, R2, P1, P2, Q,
                    cv::CALIB_ZERO_DISPARITY, 0, image_size);

  std::string filename = "stereo_calib.xml";
  std::cout << "[Calibration] Saving matrices to " << filename << "..."
            << std::endl;
  cv::FileStorage fs(filename, cv::FileStorage::WRITE);

  if (!fs.isOpened()) {
    std::cerr << "[Calibration] FATAL ERROR: Failed to open " << filename
              << " for writing!" << std::endl;
    return -1;
  }

  fs << "K1" << K1;
  fs << "D1" << D1;
  fs << "K2" << K2;
  fs << "D2" << D2;
  fs << "R1" << R1;
  fs << "P1" << P1;
  fs << "R2" << R2;
  fs << "P2" << P2;
  fs << "Q" << Q;
  fs.release();

  char cwd[1024];
  if (getcwd(cwd, sizeof(cwd)) != NULL) {
    std::cout << "[Calibration] Success! Calibration saved to: " << cwd << "/"
              << filename << std::endl;
  } else {
    std::cout << "[Calibration] Success! Calibration saved to: " << filename
              << std::endl;
  }

  return 0;
}
