#include "network/UDPServer.hpp"
#include "utils/Logger.hpp"
#include <nlohmann/json.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <chrono>

using json = nlohmann::json;

// Define global atomics
std::atomic<float> g_env_temp{0.0f};
std::atomic<float> g_env_humidity{0.0f};
std::atomic<float> g_env_moisture{0.0f};
std::atomic<float> g_env_press{0.0f};
std::atomic<int> g_env_servo_pos{0};

UDPServer::UDPServer(GlobalMapper* mapper, RoverState* rover_state, std::mutex* state_mutex, int port)
    : global_mapper_(mapper), rover_state_(rover_state), state_mutex_(state_mutex), port_(port), socket_fd_(-1) {}

UDPServer::~UDPServer() {
    stop();
}

uint64_t UDPServer::get_current_time_ms() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

void UDPServer::start() {
    if (running_) return;

    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
        LOG_ERROR("UDPServer", "Failed to create socket.");
        return;
    }

    int optval = 1;
    setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));

    sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(port_);

    if (bind(socket_fd_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        LOG_ERROR("UDPServer", "Failed to bind to port " + std::to_string(port_));
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
    LOG_INFO("UDPServer", "Started listening on UDP port " + std::to_string(port_));
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
    LOG_INFO("UDPServer", "Stopped.");
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
                if (payload.contains("bmp_press")) {
                    g_env_press.store(payload["bmp_press"].get<float>());
                }
                if (payload.contains("servo_pos")) {
                    g_env_servo_pos.store(payload["servo_pos"].get<int>());
                }
                if (payload.contains("soil_raw")) {
                    float soil_raw = payload["soil_raw"].get<float>();
                    g_env_moisture.store(soil_raw); // Or map to another variable if needed
                    
                    if (soil_raw > 300.0f) {
                        uint64_t now = get_current_time_ms();
                        if (now - last_poi_time_ > 5000) {
                            last_poi_time_ = now;
                            
                            ScientificPOI poi;
                            poi.type = "soil_anomaly";
                            poi.temp = g_env_temp.load();
                            poi.pressure = g_env_press.load();
                            poi.moisture_val = static_cast<int>(soil_raw);
                            poi.timestamp_ms = now;

                            // Safely fetch current location
                            if (state_mutex_ && rover_state_) {
                                std::lock_guard<std::mutex> lock(*state_mutex_);
                                poi.x = rover_state_->pos_x;
                                poi.y = rover_state_->pos_y;
                            } else {
                                poi.x = 0.0f;
                                poi.y = 0.0f;
                            }

                            if (global_mapper_) {
                                global_mapper_->add_poi(poi);
                                LOG_INFO("UDPServer", "Scientific POI Pinned at (" + std::to_string(poi.x) + ", " + std::to_string(poi.y) + ")!");
                            }
                        }
                    }
                }
            } catch (const std::exception& e) {
                LOG_ERROR("UDPServer", std::string("JSON parse error: ") + e.what());
            }
        }
    }
}
