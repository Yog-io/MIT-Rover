#include "lidar/TFLunaSensor.hpp"
#include <iostream>
#include <string>
#include <cstring>
#include <chrono>

#ifdef __linux__
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#endif

TFLunaSensor::TFLunaSensor() {}

TFLunaSensor::~TFLunaSensor() {
    stop();
}

bool TFLunaSensor::initialize(int udp_port) {
#ifdef __linux__
    udp_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_socket_ < 0) {
        std::cerr << "[TFLuna C++ Bridge] Failed to create UDP socket." << std::endl;
        return false;
    }

    // Set timeout to 1 second so recvfrom doesn't block thread shutdown
    struct timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    setsockopt(udp_socket_, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);

    struct sockaddr_in server_addr;
    std::memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(udp_port);

    if (bind(udp_socket_, (const struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        std::cerr << "[TFLuna C++ Bridge] Failed to bind UDP socket to port " << udp_port << std::endl;
        close(udp_socket_);
        return false;
    }

    initialized_ = true;
    std::cout << "[TFLuna C++ Bridge] Listening for Python telemetry on UDP port " << udp_port << std::endl;

    start();
    return true;
#else
    std::cout << "[TFLuna C++ Bridge] Mock initialized (macOS fallback)." << std::endl;
    initialized_ = true;
    start();
    return true;
#endif
}

void TFLunaSensor::start() {
    if (!initialized_ || running_) return;

    running_ = true;
    poll_thread_ = std::thread(&TFLunaSensor::polling_loop, this);
}

void TFLunaSensor::stop() {
    running_ = false;
    if (poll_thread_.joinable()) {
        poll_thread_.join();
    }
#ifdef __linux__
    if (udp_socket_ >= 0) {
        close(udp_socket_);
        udp_socket_ = -1;
    }
#endif
}

float TFLunaSensor::get_distance_meters() const {
    return current_distance_m_.load(std::memory_order_relaxed);
}

void TFLunaSensor::polling_loop() {
    while (running_) {
#ifdef __linux__
        char buffer[64];
        struct sockaddr_in client_addr;
        socklen_t len = sizeof(client_addr);
        
        int n = recvfrom(udp_socket_, (char *)buffer, sizeof(buffer) - 1, 
                         0, (struct sockaddr *) &client_addr, &len);
        
        if (n > 0) {
            buffer[n] = '\0';
            try {
                float dist = std::stof(std::string(buffer));
                current_distance_m_.store(dist, std::memory_order_relaxed);
            } catch (const std::exception& e) {
                // Ignore malformed float conversions
            }
        }
#else
        // macOS mock: simulate 1.5m at 100 Hz
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        current_distance_m_.store(1.5f, std::memory_order_relaxed);
#endif
    }
}
