cmake_minimum_required(VERSION 3.10)
project(RoverMockBackend)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find necessary dependencies
# Boost Beast is header only and requires system/thread for basic networking if not fully header-only configured.
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
# Phase 1: Backend
# ---------------------------------------------------------
add_executable(rover_backend src/main.cpp)
target_link_libraries(rover_backend
    PRIVATE
        Boost::system
        Boost::thread
        nlohmann_json::nlohmann_json
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
