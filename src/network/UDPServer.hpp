#pragma once

#include <atomic>
#include <thread>

// Global atomics to hold environmental state
extern std::atomic<float> g_env_temp;
extern std::atomic<float> g_env_humidity;
extern std::atomic<float> g_env_moisture;

class UDPServer {
public:
    UDPServer(int port = 9098);
    ~UDPServer();

    void start();
    void stop();

private:
    void listener_loop();

    int port_;
    int socket_fd_;
    std::atomic<bool> running_{false};
    std::thread listener_thread_;
};
