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
    // Removed O_NDELAY to ensure read() can block up to VTIME
    serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY);
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

    // Blocking read for at least 1 byte, 0.1s timeout
    tty.c_cc[VTIME] = 1;
    tty.c_cc[VMIN] = 1;

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
        std::cerr << "[TFLuna] Failed to set termios attributes." << std::endl;
        return false;
    }

    // Flush stale bytes from the serial buffer
    tcflush(serial_fd_, TCIFLUSH);

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

void TFLunaSensor::polling_loop() {
    int consecutive_failures = 0;
    uint8_t buffer[9];
    
    while (running_) {
#ifdef __linux__
        uint8_t byte;
        int n = read(serial_fd_, &byte, 1);
        
        if (n > 0) {
            static auto last_debug = std::chrono::steady_clock::now();
            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::seconds>(now - last_debug).count() >= 1) {
                std::cout << "\r[Debug] UART reading active...                                    \n";
                last_debug = now;
            }
        }
        
        if (n == 1 && byte == 0x59) {
            n = read(serial_fd_, &byte, 1);
            if (n == 1 && byte == 0x59) {
                buffer[0] = 0x59;
                buffer[1] = 0x59;
                
                // Read remaining 7 bytes
                int bytes_read = 0;
                while (bytes_read < 7 && running_) {
                    n = read(serial_fd_, buffer + 2 + bytes_read, 7 - bytes_read);
                    if (n > 0) bytes_read += n;
                }
                
                if (!running_) break;

                // Validate Checksum: sum(bytes 0..7) & 0xFF
                uint8_t checksum = 0;
                for (int i = 0; i < 8; ++i) {
                    checksum += buffer[i];
                }
                
                if (checksum == buffer[8]) {
                    float distance = (buffer[2] | (buffer[3] << 8)) / 100.0f;
                    int strength = buffer[4] | (buffer[5] << 8);
                    
                    if (strength >= min_strength_) {
                        current_distance_m_.store(distance, std::memory_order_relaxed);
                        consecutive_failures = 0;
                    } else {
                        consecutive_failures++;
                    }
                } else {
                    consecutive_failures++;
                }
            } else {
                consecutive_failures++;
            }
        } else if (n <= 0) {
            consecutive_failures++;
        }
#else
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        current_distance_m_.store(1.5f, std::memory_order_relaxed);
        consecutive_failures = 0;
#endif

        if (consecutive_failures > max_consecutive_failures_) {
            current_distance_m_.store(-1.0f, std::memory_order_relaxed);
            // Cap to prevent overflow
            consecutive_failures = max_consecutive_failures_ + 1;
        }
    }
}
