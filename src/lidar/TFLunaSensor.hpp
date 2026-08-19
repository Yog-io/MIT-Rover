#pragma once

#include <atomic>
#include <thread>
#include <string>

class TFLunaSensor {
public:
    TFLunaSensor();
    ~TFLunaSensor();

    // Opens a UDP socket bound to the specified port
    bool initialize(int udp_port = 9090);

    // Starts the continuous background reading thread
    void start();

    // Stops the reading thread cleanly and closes the socket
    void stop();

    // Lock-free atomic read of the ground-truth distance in meters.
    // Returns -1.0f if the sensor has completely lost lock.
    float get_distance_meters() const;

private:
    void polling_loop();

    int udp_socket_ = -1;
    bool initialized_ = false;

    std::atomic<bool> running_{false};
    std::thread poll_thread_;

    // Lock-free storage of the latest valid distance
    std::atomic<float> current_distance_m_{-1.0f};
};
