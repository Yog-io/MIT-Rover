#pragma once

#include <atomic>
#include <thread>
#include <mutex>
#include "vision/GlobalMapper.hpp"
#include "SharedState.hpp"

// Global atomics to hold environmental state
extern std::atomic<float> g_env_temp;
extern std::atomic<float> g_env_humidity;
extern std::atomic<float> g_env_moisture;
extern std::atomic<float> g_env_press;
extern std::atomic<int> g_env_servo_pos;

class UDPServer {
public:
    UDPServer(GlobalMapper* mapper, RoverState* rover_state, std::mutex* state_mutex, int port = 9098);
    ~UDPServer();

    void start();
    void stop();

private:
    void listener_loop();
    uint64_t get_current_time_ms();

    int port_;
    int socket_fd_;
    std::atomic<bool> running_{false};
    std::thread listener_thread_;
    
    GlobalMapper* global_mapper_;
    RoverState* rover_state_;
    std::mutex* state_mutex_;
    
    uint64_t last_poi_time_{0};
};
