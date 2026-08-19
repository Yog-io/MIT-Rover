#include "lidar/TFLunaSensor.hpp"
#include <iostream>
#include <cstdio>
#include <cstring>
#include <cerrno>
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
    // Open with strict blocking mode — NO O_NDELAY, NO O_NONBLOCK
    serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd_ < 0) {
        std::cerr << "[TFLuna] Failed to open UART port: " << port << std::endl;
        return false;
    }

    struct termios tty;
    // 1. Zero the Memory to prevent leftover garbage
    std::memset(&tty, 0, sizeof(tty));
    if (tcgetattr(serial_fd_, &tty) != 0) {
        std::cerr << "[TFLuna] Failed to get termios attributes." << std::endl;
        return false;
    }

    // Nuclear option: cfmakeraw strips ALL processing
    cfmakeraw(&tty);

    // Explicitly set 8N1, no flow control, enable receiver
    tty.c_cflag |= (CLOCAL | CREAD | CS8);
    tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);

    // True blocking read: wait forever for at least 1 byte
    tty.c_cc[VMIN]  = 1;
    tty.c_cc[VTIME] = 0;

    // Baud rate 115200
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
        std::cerr << "[TFLuna] Failed to set termios attributes." << std::endl;
        return false;
    }

    // Flush ALL stale bytes from both input and output queues
    tcflush(serial_fd_, TCIOFLUSH);

    initialized_ = true;
    std::cout << "[TFLuna] Initialized UART on " << port << " successfully." << std::endl;

    // Auto-start background thread
    start();

    return true;
#else
    std::cout << "[TFLuna] Mock initialized (macOS fallback)." << std::endl;
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
}

float TFLunaSensor::get_distance_meters() const {
    return current_distance_m_.load(std::memory_order_relaxed);
}

// Helper: read exactly 1 byte into a uint8_t. Returns true on success.
static bool read_one_byte(int fd, uint8_t& out) {
    uint8_t buf;
    ssize_t n = read(fd, &buf, 1);
    
    // 2. Print EVERY Byte immediately
    if (n > 0) {
        std::printf("%02X ", buf);
        std::fflush(stdout);
        out = buf;
        return true;
    } 
    // 3. Error Logging
    else if (n < 0) {
        std::printf("\n[Read Error] %s\n", strerror(errno));
        std::fflush(stdout);
    }
    return false;
}

void TFLunaSensor::polling_loop() {
    int consecutive_failures = 0;

    while (running_) {
#ifdef __linux__
        // ---- State 0: Wait for first 0x59 ----
        uint8_t byte0 = 0;
        if (!read_one_byte(serial_fd_, byte0)) {
            consecutive_failures++;
            continue;
        }
        if (byte0 != 0x59) {
            continue; // Not a header byte, keep scanning
        }

        // ---- State 1: Wait for second 0x59 ----
        uint8_t byte1 = 0;
        if (!read_one_byte(serial_fd_, byte1)) {
            consecutive_failures++;
            continue;
        }
        if (byte1 != 0x59) {
            continue; // False start, back to State 0
        }

        // ---- State 2: Read exactly 7 more bytes (payload + checksum) ----
        uint8_t frame[9];
        frame[0] = 0x59;
        frame[1] = 0x59;

        bool frame_ok = true;
        for (int i = 0; i < 7; ++i) {
            if (!running_) return;
            if (!read_one_byte(serial_fd_, frame[2 + i])) {
                frame_ok = false;
                break;
            }
        }
        
        // Print a newline after a full frame or attempt so the hex dump is readable
        std::printf("\n");
        std::fflush(stdout);

        if (!frame_ok) {
            consecutive_failures++;
            continue;
        }

        // ---- Checksum: sum bytes 0..7 into uint16_t, mask to 8 bits ----
        uint16_t sum = 0;
        for (int i = 0; i < 8; ++i) {
            sum += frame[i];
        }
        uint8_t computed_checksum = static_cast<uint8_t>(sum & 0xFF);

        if (computed_checksum != frame[8]) {
            consecutive_failures++;
            continue;
        }

        // ---- Parse distance and strength using strict uint8_t math ----
        uint16_t dist_cm  = static_cast<uint16_t>(frame[2]) | (static_cast<uint16_t>(frame[3]) << 8);
        uint16_t strength = static_cast<uint16_t>(frame[4]) | (static_cast<uint16_t>(frame[5]) << 8);
        float distance_m  = dist_cm / 100.0f;

        if (strength >= static_cast<uint16_t>(min_strength_)) {
            current_distance_m_.store(distance_m, std::memory_order_relaxed);
            consecutive_failures = 0;
        } else {
            consecutive_failures++;
        }

#else
        // macOS mock: simulate 1.5m at 100 Hz
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        current_distance_m_.store(1.5f, std::memory_order_relaxed);
        consecutive_failures = 0;
#endif

        // Loss-of-lock detection: >100 consecutive failures (~1 second)
        if (consecutive_failures > max_consecutive_failures_) {
            current_distance_m_.store(-1.0f, std::memory_order_relaxed);
            consecutive_failures = max_consecutive_failures_ + 1; // Cap to prevent overflow
        }
    }
}
