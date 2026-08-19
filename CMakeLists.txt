cmake_minimum_required(VERSION 3.10)
project(RoverMockBackend)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find necessary dependencies
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(Threads REQUIRED)
find_package(nlohmann_json REQUIRED)

# OpenCV for camera processing and display
find_package(OpenCV REQUIRED)

# libcamera using pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBCAMERA REQUIRED libcamera)

# OpenMP for parallel processing
find_package(OpenMP REQUIRED)

# ---------------------------------------------------------
# Phase 4: Unified Backend Executable
# ---------------------------------------------------------
add_executable(rover_backend 
    src/main.cpp
    src/camera/DualCameraCapture.cpp
    src/vision/HazardMapper.cpp
)

target_include_directories(rover_backend PRIVATE src ${LIBCAMERA_INCLUDE_DIRS})
target_link_directories(rover_backend PRIVATE ${LIBCAMERA_LIBRARY_DIRS})

target_link_libraries(rover_backend
    PRIVATE
        Boost::system
        Boost::thread
        nlohmann_json::nlohmann_json
        ${OpenCV_LIBS}
        ${LIBCAMERA_LIBRARIES}
        OpenMP::OpenMP_CXX
        Threads::Threads
)

# ---------------------------------------------------------
# Phase 2: Dual Camera Synchronization Test
# ---------------------------------------------------------
add_executable(camera_test src/camera_main.cpp src/camera/DualCameraCapture.cpp)
target_include_directories(camera_test PRIVATE src ${LIBCAMERA_INCLUDE_DIRS})
target_link_directories(camera_test PRIVATE ${LIBCAMERA_LIBRARY_DIRS})
target_link_libraries(camera_test
    PRIVATE
        ${OpenCV_LIBS}
        ${LIBCAMERA_LIBRARIES}
        OpenMP::OpenMP_CXX
        Threads::Threads
)

# ---------------------------------------------------------
# Phase 5: IMU Synchronization Test
# ---------------------------------------------------------
add_executable(imu_test src/imu_main.cpp src/imu/MPU6050Sensor.cpp)
target_include_directories(imu_test PRIVATE src)
target_link_libraries(imu_test
    PRIVATE
        Threads::Threads
)

# ---------------------------------------------------------
# Phase 6: LiDAR Ground-Truth Test
# ---------------------------------------------------------
add_executable(lidar_test src/lidar_main.cpp src/lidar/TFLunaSensor.cpp)
target_include_directories(lidar_test PRIVATE src)
target_link_libraries(lidar_test
    PRIVATE
        Threads::Threads
)

# ---------------------------------------------------------
# Diagnostic Suite
# ---------------------------------------------------------

add_executable(test_websocket src/tests/test_websocket.cpp)
target_include_directories(test_websocket PRIVATE src)
target_link_libraries(test_websocket PRIVATE Boost::system Boost::thread nlohmann_json::nlohmann_json Threads::Threads)

add_executable(test_camera_sync src/tests/test_camera_sync.cpp src/camera/DualCameraCapture.cpp)
target_include_directories(test_camera_sync PRIVATE src ${LIBCAMERA_INCLUDE_DIRS})
target_link_directories(test_camera_sync PRIVATE ${LIBCAMERA_LIBRARY_DIRS})
target_link_libraries(test_camera_sync PRIVATE ${OpenCV_LIBS} ${LIBCAMERA_LIBRARIES} OpenMP::OpenMP_CXX Threads::Threads)

add_executable(test_stereo_vision src/tests/test_stereo_vision.cpp src/camera/DualCameraCapture.cpp)
target_include_directories(test_stereo_vision PRIVATE src ${LIBCAMERA_INCLUDE_DIRS})
target_link_directories(test_stereo_vision PRIVATE ${LIBCAMERA_LIBRARY_DIRS})
target_link_libraries(test_stereo_vision PRIVATE ${OpenCV_LIBS} ${LIBCAMERA_LIBRARIES} OpenMP::OpenMP_CXX Threads::Threads)

add_executable(test_mpu6050 src/tests/test_mpu6050.cpp src/imu/MPU6050Sensor.cpp)
target_include_directories(test_mpu6050 PRIVATE src)
target_link_libraries(test_mpu6050 PRIVATE Threads::Threads)

add_executable(test_tfluna_i2c src/tests/test_tfluna_i2c.cpp src/lidar/TFLunaSensor.cpp)
target_include_directories(test_tfluna_i2c PRIVATE src)
target_link_libraries(test_tfluna_i2c PRIVATE Threads::Threads)
