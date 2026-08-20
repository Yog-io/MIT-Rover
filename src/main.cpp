#include <iostream>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <cmath>
#include <random>
#include <set>
#include <vector>
#include <memory>

#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/strand.hpp>
#include <nlohmann/json.hpp>

// Phase 2 & 3 Hardware/Vision Includes
#include "camera/DualCameraCapture.hpp"
#include "vision/HazardMapper.hpp"

namespace beast = boost::beast;
namespace http = beast::http;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = boost::asio::ip::tcp;
using json = nlohmann::json;

// Global Mock Flag (for kinematics only now)
std::atomic<bool> MOCK_HARDWARE_MODE{true};

// Shared State
struct RoverState {
    std::string mode = "STANDBY";
    float linear_v = 0.0f;
    float angular_w = 0.0f;
    float pos_x = 0.0f;
    float pos_y = 0.0f;
    float heading_deg = 0.0f;
    bool arm_deployed = false;
};

struct HazardState {
    std::string level = "CLEAR";
    float distance_m = 99.0f;
    std::string sector = "CENTER";
    std::string type = "ROCK"; // Kept static for mock, would map to crater/rock realistically
    std::vector<uint8_t> mini_map; // Downsampled 10x10 map for lightweight UI streaming
};

RoverState g_rover_state;
std::mutex g_state_mutex;

HazardState g_hazard_state;
std::mutex g_hazard_mutex;

// WebSocket Active Connections
class session;
std::set<std::shared_ptr<session>> g_connections;
std::mutex g_connections_mutex;

class session : public std::enable_shared_from_this<session> {
    websocket::stream<beast::tcp_stream> ws_;
    beast::flat_buffer buffer_;
    std::vector<std::string> write_queue_;

public:
    explicit session(tcp::socket&& socket)
        : ws_(std::move(socket)) {}

    void run() {
        ws_.set_option(websocket::stream_base::timeout::suggested(beast::role_type::server));
        ws_.async_accept(
            beast::bind_front_handler(&session::on_accept, shared_from_this()));
    }

    void send_message(std::string msg) {
        net::post(
            ws_.get_executor(),
            beast::bind_front_handler(
                &session::on_send, shared_from_this(), std::move(msg)));
    }

private:
    void on_accept(beast::error_code ec) {
        if(ec) return;
        
        {
            std::lock_guard<std::mutex> lock(g_connections_mutex);
            g_connections.insert(shared_from_this());
        }
        std::cout << "[WebSocket] Client connected." << std::endl;
        do_read();
    }

    void do_read() {
        ws_.async_read(
            buffer_,
            beast::bind_front_handler(&session::on_read, shared_from_this()));
    }

    void on_read(beast::error_code ec, std::size_t bytes_transferred) {
        boost::ignore_unused(bytes_transferred);
        if(ec) {
            handle_close();
            return;
        }

        std::string payload = beast::buffers_to_string(buffer_.data());
        buffer_.consume(buffer_.size());
        
        try {
            json parsed_payload = json::parse(payload);
            std::string command = parsed_payload.value("command", "");
            
            std::lock_guard<std::mutex> lock(g_state_mutex);
            if (command == "drive") {
                g_rover_state.linear_v = parsed_payload.value("linear_v", 0.0f);
                g_rover_state.angular_w = parsed_payload.value("angular_w", 0.0f);
            } else if (command == "set_mode") {
                g_rover_state.mode = parsed_payload.value("mode", "STANDBY");
            } else if (command == "deploy_arm") {
                g_rover_state.arm_deployed = true;
            } else {
                std::cerr << "Unknown command received: " << command << std::endl;
            }
        } catch (const std::exception& e) {
            std::cerr << "Error parsing incoming JSON: " << e.what() << std::endl;
        }
        do_read();
    }

    void on_send(std::string msg) {
        bool write_in_progress = !write_queue_.empty();
        write_queue_.push_back(std::move(msg));
        if (!write_in_progress) {
            do_write();
        }
    }

    void do_write() {
        ws_.text(true);
        ws_.async_write(
            net::buffer(write_queue_.front()),
            beast::bind_front_handler(&session::on_write, shared_from_this()));
    }

    void on_write(beast::error_code ec, std::size_t bytes_transferred) {
        boost::ignore_unused(bytes_transferred);
        if(ec) {
            handle_close();
            return;
        }
        write_queue_.erase(write_queue_.begin());
        if(!write_queue_.empty()) {
            do_write();
        }
    }

    void handle_close() {
        std::lock_guard<std::mutex> lock(g_connections_mutex);
        if (g_connections.find(shared_from_this()) != g_connections.end()) {
            g_connections.erase(shared_from_this());
            std::cout << "[WebSocket] Client disconnected." << std::endl;
        }
    }
};

void do_accept(tcp::acceptor& acceptor, net::io_context& ioc) {
    acceptor.async_accept(
        net::make_strand(ioc),
        [&acceptor, &ioc](beast::error_code ec, tcp::socket socket) {
            if (!ec) {
                std::make_shared<session>(std::move(socket))->run();
            }
            do_accept(acceptor, ioc);
        });
}

// ----------------------------------------------------------------------------
// Phase 4: Vision & Hardware Processing Thread (Replaces old mock thread)
// ----------------------------------------------------------------------------
void vision_thread_loop(DualCameraCapture* dual_cam, HazardMapper* mapper) {
    std::cout << "[Vision Thread] Started real-time stereo processing loop." << std::endl;
    
    while (true) {
        // Blocks until a valid timestamp-paired frame arrives from the hardware
        auto stereo_pair = dual_cam->get_stereo_pair();
        
        if (!stereo_pair) {
            std::cerr << "[Vision Thread] Camera capture returned null. Exiting loop." << std::endl;
            break;
        }

        // Process stereo frame through OpenMP HazardMapper Pipeline
        // TODO(Phase 7): Wire in actual LiDAR distance. Passing 1.0f dummy to compile.
        HazardReport report = mapper->process(stereo_pair->left_y, stereo_pair->right_y, 1.0f);

        // State Fusion & Downsampling
        std::lock_guard<std::mutex> lock(g_hazard_mutex);
        
        g_hazard_state.distance_m = report.closest_lethal_distance_m;
        
        if (report.closest_lethal_distance_m < 0.75f) {
            g_hazard_state.level = "CRITICAL";
        } else if (report.closest_lethal_distance_m < 2.0f) {
            g_hazard_state.level = "CAUTION";
        } else {
            g_hazard_state.level = "CLEAR";
        }

        // Downsample the 100x100 5cm grid into a lightweight 10x10 50cm grid for UI streaming
        g_hazard_state.mini_map.clear();
        g_hazard_state.mini_map.reserve(100);
        
        for (int r = 0; r < 100; r += 10) {
            for (int c = 0; c < 100; c += 10) {
                int lethal_count = 0;
                for (int ir = 0; ir < 10; ++ir) {
                    for (int ic = 0; ic < 10; ++ic) {
                        if (report.costmap[r + ir][c + ic] > 0) {
                            lethal_count++;
                        }
                    }
                }
                // Mark macro-cell as lethal if any inner cells were lethal
                g_hazard_state.mini_map.push_back(lethal_count > 0 ? 255 : 0);
            }
        }
        
        // As loop iterates, stereo_pair goes out of scope and memory is re-queued to ISP!
    }
}

// ----------------------------------------------------------------------------
// Phase 4: Lightweight Broadcasting Loop (15 Hz)
// ----------------------------------------------------------------------------
void broadcast_thread_loop() {
    std::cout << "[Broadcast Thread] Started 15 Hz telemetry loop." << std::endl;
    
    auto last_time = std::chrono::steady_clock::now();
    
    // Environment Mock
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dist_env_temp(10.0f, 35.0f);
    std::uniform_real_distribution<float> dist_env_hum(20.0f, 60.0f);

    while (true) {
        auto now = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(now - last_time).count();
        last_time = now;

        RoverState current_state;
        {
            std::lock_guard<std::mutex> lock(g_state_mutex);
            // Simulate kinematics based on WebSocket drive commands
            float heading_rad = g_rover_state.heading_deg * (M_PI / 180.0f);
            g_rover_state.pos_x += g_rover_state.linear_v * std::cos(heading_rad) * dt;
            g_rover_state.pos_y += g_rover_state.linear_v * std::sin(heading_rad) * dt;
            g_rover_state.heading_deg += g_rover_state.angular_w * dt; 
            
            while (g_rover_state.heading_deg >= 360.0f) g_rover_state.heading_deg -= 360.0f;
            while (g_rover_state.heading_deg < 0.0f) g_rover_state.heading_deg += 360.0f;

            current_state = g_rover_state;
        }

        HazardState current_hazard;
        {
            std::lock_guard<std::mutex> lock(g_hazard_mutex);
            current_hazard = g_hazard_state;
        }

        // Package real hazard data & simulated kinematics into Phase 1 JSON schema
        json telemetry;
        telemetry["type"] = "telemetry";
        telemetry["timestamp_ms"] = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();

        telemetry["state"] = {
            {"mode", current_state.mode},
            {"speed_m_s", current_state.linear_v},
            {"heading_deg", current_state.heading_deg},
            {"pos_x", current_state.pos_x},
            {"pos_y", current_state.pos_y}
        };

        float distance = current_hazard.distance_m;
        if (std::isinf(distance)) {
            distance = 99.0f; // Default far distance for JSON parsing ease
        }

        telemetry["hazard"] = {
            {"level", current_hazard.level},
            {"distance_m", distance},
            {"sector", current_hazard.sector},
            {"type", current_hazard.type},
            {"mini_map", current_hazard.mini_map} // Added downsampled map array
        };

        telemetry["environment"] = {
            {"temp_c", dist_env_temp(gen)},
            {"humidity", dist_env_hum(gen)},
            {"soil_moisture_detected", false} 
        };

        std::string payload = telemetry.dump();

        {
            std::lock_guard<std::mutex> lock(g_connections_mutex);
            std::vector<std::shared_ptr<session>> active_connections(g_connections.begin(), g_connections.end());
            
            std::cout << "[Broadcast] Pushing telemetry to " << active_connections.size() << " clients...\n";
            
            for (auto& session_ptr : active_connections) {
                session_ptr->send_message(payload);
            }
        }

        // Maintain ~15 Hz
        std::this_thread::sleep_for(std::chrono::milliseconds(66));
    }
}

int main() {
    std::cout << "=========================================================" << std::endl;
    std::cout << "[Backend] Phase 4: Unified Rover Backend Integration" << std::endl;
    std::cout << "=========================================================" << std::endl;
    
    std::unique_ptr<DualCameraCapture> dual_cam = std::make_unique<DualCameraCapture>();
    std::unique_ptr<HazardMapper> mapper = std::make_unique<HazardMapper>();

    // 1. Hardware Initialization
    if (!dual_cam->initialize()) {
        std::cerr << "[Fatal Error] Failed to initialize hardware cameras via libcamera." << std::endl;
        return -1;
    }
    std::cout << "[Backend] Cameras initialized successfully. Starting hardware streams..." << std::endl;
    dual_cam->start();

    // 2. Spawn core threads
    std::thread vision_thread(vision_thread_loop, dual_cam.get(), mapper.get());
    std::thread broadcast_thread(broadcast_thread_loop);

    // 3. Setup Networking
    net::io_context ioc{1};
    tcp::acceptor acceptor(ioc, {tcp::v4(), 8080});

    std::cout << "[Backend] Starting WebSocket Server on port 8080..." << std::endl;
    do_accept(acceptor, ioc);
    
    // Block the main thread on networking io_context
    ioc.run();

    // Cleanup
    std::cout << "[Backend] Shutting down..." << std::endl;
    dual_cam->stop();
    vision_thread.join();
    broadcast_thread.join();
    
    return 0;
}
