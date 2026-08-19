#include "lidar/TFLunaSensor.hpp"
#include <iostream>
#include <chrono>

#ifdef __linux__
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#endif

TFLunaSensor::TFLunaSensor() {}

TFLunaSensor::~TFLunaSensor() {
    stop();
#ifdef __linux__
    if (serial_fd_ >= 0) {
        close(serial_fd_);
    }
#endif
}

bool TFLunaSensor::initialize(const std::string& port) {
#ifdef __linux__
    serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd_ < 0) {
        std::cerr << "[TFLuna] Failed to open UART port: " << port << std::endl;
        return false;
    }

    struct termios tty;
    if (tcgetattr(serial_fd_, &tty) != 0) {
        std::cerr << "[TFLuna] Failed to get termios attributes." << std::endl;
        return false;
    }

    // Set Baud Rate 115200
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    // 8N1 Mode
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    
    // Disable hardware flow control
    tty.c_cflag &= ~CRTSCTS;
    
    // Turn on READ & ignore ctrl lines
    tty.c_cflag |= CREAD | CLOCAL;

    // Non-canonical mode
    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ECHONL;
    tty.c_lflag &= ~ISIG;
    
    // Disable software flow control
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
    
    // Raw output
    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;

    // Blocking read for at least 1 byte
    tty.c_cc[VTIME] = 10; // 1 second timeout (deciseconds)
    tty.c_cc[VMIN] = 1;

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
        std::cerr << "[TFLuna] Failed to set termios attributes." << std::endl;
        return false;
    }

    initialized_ = true;
    std::cout << "[TFLuna] Initialized UART successfully." << std::endl;
    return true;
#else
    std::cout << "[TFLuna] Mock initialized (macOS fallback)." << std::endl;
    initialized_ = true;
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
}

float TFLunaSensor::get_distance_meters() const {
    return current_distance_m_.load(std::memory_order_relaxed);
}

bool TFLunaSensor::parse_frame(uint8_t* buffer, float& distance_m, int& strength) {
    // Checksum: lower 8 bits of the sum of first 8 bytes
    int checksum = 0;
    for (int i = 0; i < 8; ++i) {
        checksum += buffer[i];
    }
    
    if ((checksum & 0xFF) != buffer[8]) {
        return false;
    }

    // Distance in cm
    int dist_cm = buffer[2] + (buffer[3] << 8);
    distance_m = dist_cm / 100.0f;
    
    // Signal strength
    strength = buffer[4] + (buffer[5] << 8);
    
    return true;
}

void TFLunaSensor::polling_loop() {
    int consecutive_failures = 0;
    uint8_t buffer[9];
    
    while (running_) {
#ifdef __linux__
        // Search for the frame header 0x59 0x59
        uint8_t byte;
        int n = read(serial_fd_, &byte, 1);
        if (n == 1 && byte == 0x59) {
            n = read(serial_fd_, &byte, 1);
            if (n == 1 && byte == 0x59) {
                buffer[0] = 0x59;
                buffer[1] = 0x59;
                
                // Read the rest of the 7 bytes
                int bytes_read = 0;
                while (bytes_read < 7 && running_) {
                    n = read(serial_fd_, buffer + 2 + bytes_read, 7 - bytes_read);
                    if (n > 0) bytes_read += n;
                }
                
                if (!running_) break;

                float dist_m = 0.0f;
                int strength = 0;
                
                if (parse_frame(buffer, dist_m, strength) && strength >= min_strength_) {
                    current_distance_m_.store(dist_m, std::memory_order_relaxed);
                    consecutive_failures = 0;
                } else {
                    consecutive_failures++;
                }
            }
        } else if (n < 0) {
            // Read error or timeout
            consecutive_failures++;
        }
#else
        // Mock data stream: 1.5 meters constant
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        current_distance_m_.store(1.5f, std::memory_order_relaxed);
        consecutive_failures = 0;
#endif

        // If sensor fails repeatedly, mark as -1.0f (loss of lock)
        if (consecutive_failures > max_consecutive_failures_) {
            current_distance_m_.store(-1.0f, std::memory_order_relaxed);
            // Cap to prevent overflow
            consecutive_failures = max_consecutive_failures_ + 1;
        }
    }
}
