#include "network/UDPServer.hpp"
#include <iostream>
#include <nlohmann/json.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

using json = nlohmann::json;

// Define global atomics
std::atomic<float> g_env_temp{0.0f};
std::atomic<float> g_env_humidity{0.0f};
std::atomic<float> g_env_moisture{0.0f};

UDPServer::UDPServer(int port) : port_(port), socket_fd_(-1) {}

UDPServer::~UDPServer() {
    stop();
}

void UDPServer::start() {
    if (running_) return;

    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
        std::cerr << "[UDPServer] Failed to create socket." << std::endl;
        return;
    }

    int optval = 1;
    setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));

    sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(port_);

    if (bind(socket_fd_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        std::cerr << "[UDPServer] Failed to bind to port " << port_ << std::endl;
        close(socket_fd_);
        socket_fd_ = -1;
        return;
    }

    struct timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    running_ = true;
    listener_thread_ = std::thread(&UDPServer::listener_loop, this);
    std::cout << "[UDPServer] Started listening on UDP port " << port_ << std::endl;
}

void UDPServer::stop() {
    if (!running_) return;
    running_ = false;
    
    if (socket_fd_ >= 0) {
        close(socket_fd_);
        socket_fd_ = -1;
    }

    if (listener_thread_.joinable()) {
        listener_thread_.join();
    }
    std::cout << "[UDPServer] Stopped." << std::endl;
}

void UDPServer::listener_loop() {
    char buffer[1024];

    while (running_) {
        sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
        
        int bytes_received = recvfrom(socket_fd_, buffer, sizeof(buffer) - 1, 0,
                                      (struct sockaddr*)&client_addr, &client_len);
                                      
        if (bytes_received > 0) {
            buffer[bytes_received] = '\0';
            try {
                json payload = json::parse(buffer);
                if (payload.contains("temp")) {
                    g_env_temp.store(payload["temp"].get<float>());
                }
                if (payload.contains("humidity")) {
                    g_env_humidity.store(payload["humidity"].get<float>());
                }
                if (payload.contains("moisture")) {
                    g_env_moisture.store(payload["moisture"].get<float>());
                }
            } catch (const std::exception& e) {
                // Ignore parsing errors for robust background listening
            }
        }
    }
}
