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
