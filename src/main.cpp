#include "utils/Logger.hpp"
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
#include "imu/MPU6050Sensor.hpp"
#include "lidar/TFLunaSensor.hpp"
#include "vision/GlobalMapper.hpp"
#include "network/UDPServer.hpp"
#include "actuation/MotorController.hpp"
#include "navigation/NavigationManager.hpp"
#include "SharedState.hpp"

namespace beast = boost::beast;
namespace http = beast::http;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = boost::asio::ip::tcp;
using json = nlohmann::json;

// Global Mock Flag (for kinematics only now)
std::atomic<bool> MOCK_HARDWARE_MODE{true};

// Shared State instances
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
        LOG_INFO("WebSocket", "Client connected.");
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
                LOG_WARN("WebSocket", "Unknown command received: " + command);
            }
        } catch (const std::exception& e) {
            LOG_ERROR("WebSocket", std::string("Error parsing incoming JSON: ") + e.what());
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
            LOG_INFO("WebSocket", "Client disconnected.");
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
void vision_thread_loop(DualCameraCapture* dual_cam, HazardMapper* mapper, TFLunaSensor* lidar, GlobalMapper* global_mapper, NavigationManager* nav_manager) {
    LOG_INFO("Vision", "Started real-time stereo processing loop.");
    
    auto last_heartbeat = std::chrono::steady_clock::now();
    auto last_frame_time = std::chrono::steady_clock::now(); // For dead-reckoning dt
    int frames_since_heartbeat = 0;
    
    while (true) {
        // Blocks until a valid timestamp-paired frame arrives from the hardware
        auto stereo_pair = dual_cam->get_stereo_pair();
        
        if (!stereo_pair) {
            LOG_ERROR("Vision", "Camera capture returned null. Exiting loop.");
            break;
        }

        // Process stereo frame through OpenMP HazardMapper Pipeline
        float lidar_dist = lidar->get_distance_meters();
        if (lidar_dist < 0.0f) {
            lidar_dist = 1.0f; // Default if invalid
        }
        HazardReport report = mapper->process(stereo_pair->left_y, stereo_pair->right_y, lidar_dist);

        // --- Delta time for dead-reckoning integration ---
        auto now_frame = std::chrono::steady_clock::now();
        float delta_time_sec = std::chrono::duration<float>(now_frame - last_frame_time).count();
        last_frame_time = now_frame;
        // Clamp to avoid large jumps on the first frame or after stalls
        delta_time_sec = std::min(delta_time_sec, 0.5f);

        // Update global map: pass heading, active velocity command, and elapsed time
        // so GlobalMapper can integrate dead-reckoning translation.
        float current_heading = 0.0f;
        float current_linear_v = 0.0f;
        {
            std::lock_guard<std::mutex> state_lock(g_state_mutex);
            current_heading = g_rover_state.heading_deg;
            current_linear_v = g_rover_state.linear_v;
        }
        global_mapper->update_map(report, current_heading, current_linear_v, delta_time_sec);
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
        
        // Navigation arbitration (tick runs every frame to process avoidance/overrides)
        nav_manager->tick(report);
        
        frames_since_heartbeat++;
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_heartbeat).count() > 5000) {
            float fps = frames_since_heartbeat / 5.0f;
            LOG_INFO("SYS", "Vision_FPS: " + std::to_string(fps));
            frames_since_heartbeat = 0;
            last_heartbeat = now;
        }
        
        // As loop iterates, stereo_pair goes out of scope and memory is re-queued to ISP!
    }
}

// ----------------------------------------------------------------------------
// Phase 4: IMU Polling Thread (50 Hz)
// ----------------------------------------------------------------------------
void imu_thread_loop(MPU6050Sensor* imu) {
    LOG_INFO("IMU", "Started 50 Hz state update loop.");
    while (true) {
        IMUData data = imu->get_orientation();
        {
            std::lock_guard<std::mutex> lock(g_state_mutex);
            g_rover_state.heading_deg = data.yaw;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

// ----------------------------------------------------------------------------
// Phase 4: Lightweight Broadcasting Loop (15 Hz)
// ----------------------------------------------------------------------------
void broadcast_thread_loop(GlobalMapper* global_mapper) {
    LOG_INFO("Broadcast", "Started 15 Hz telemetry loop.");
    
    auto last_time = std::chrono::steady_clock::now();

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
            
            // Note: heading_deg is now updated by IMU thread, so we skip simulating angular velocity here.
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
            {"temp_c", g_env_temp.load()},
            {"humidity", g_env_humidity.load()},
            {"soil_moisture_detected", g_env_moisture.load() > 0.0f} // Boolean mapping from raw analog value
        };
        
        telemetry["global_map"] = json::parse(global_mapper->get_global_map_json());

        std::string payload = telemetry.dump();

        {
            std::lock_guard<std::mutex> lock(g_connections_mutex);
            std::vector<std::shared_ptr<session>> active_connections(g_connections.begin(), g_connections.end());
            
            for (auto& session_ptr : active_connections) {
                session_ptr->send_message(payload);
            }
        }

        // Maintain ~15 Hz
        std::this_thread::sleep_for(std::chrono::milliseconds(66));
    }
}

int main() {
    LOG_INFO("SYS", "=========================================================");
    LOG_INFO("SYS", "[Backend] Phase 4: Unified Rover Backend Integration");
    LOG_INFO("SYS", "=========================================================");
    
    std::unique_ptr<DualCameraCapture> dual_cam = std::make_unique<DualCameraCapture>();
    std::unique_ptr<HazardMapper> mapper = std::make_unique<HazardMapper>();
    std::unique_ptr<GlobalMapper> global_mapper = std::make_unique<GlobalMapper>();
    std::unique_ptr<TFLunaSensor> lidar = std::make_unique<TFLunaSensor>();
    std::unique_ptr<MPU6050Sensor> imu = std::make_unique<MPU6050Sensor>();
    std::unique_ptr<MotorController> motor_ctrl = std::make_unique<MotorController>();
    std::unique_ptr<UDPServer> udp_server = std::make_unique<UDPServer>(
        global_mapper.get(), &g_rover_state, &g_state_mutex, 9098);
    std::unique_ptr<NavigationManager> nav_manager = std::make_unique<NavigationManager>(
        motor_ctrl.get(), &g_rover_state, &g_hazard_state, &g_state_mutex, &g_hazard_mutex);

    // 1. Hardware Initialization
    if (!lidar->initialize("/dev/i2c-3", 0x10)) {
        LOG_ERROR("Hardware", "Failed to initialize TF-Luna LiDAR.");
    } else {
        LOG_INFO("Hardware", "TF-Luna LiDAR initialized successfully.");
    }
    lidar->start();

    if (!imu->initialize("/dev/i2c-1", 0x68)) {
        LOG_ERROR("Hardware", "Failed to initialize MPU6050 IMU.");
    } else {
        LOG_INFO("Hardware", "MPU6050 IMU initialized successfully.");
    }
    imu->start();

    if (!dual_cam->initialize()) {
        LOG_ERROR("Hardware", "Failed to initialize hardware cameras via libcamera.");
        return -1;
    }
    
    if (!motor_ctrl->initialize()) {
        LOG_ERROR("Hardware", "Failed to initialize motor controller.");
    } else {
        LOG_INFO("Hardware", "Motor controller initialized successfully.");
    }

    LOG_INFO("Hardware", "Cameras initialized successfully. Starting hardware streams...");
    dual_cam->start();
    udp_server->start();
    LOG_INFO("Hardware", "UDPServer initialized and started successfully.");

    // 2. Spawn core threads
    std::thread vision_thread(vision_thread_loop, dual_cam.get(), mapper.get(), lidar.get(), global_mapper.get(), nav_manager.get());
    std::thread imu_thread(imu_thread_loop, imu.get());
    std::thread broadcast_thread(broadcast_thread_loop, global_mapper.get());

    // 3. Setup Networking
    net::io_context ioc{1};
    tcp::acceptor acceptor(ioc, {tcp::v4(), 8080});

    LOG_INFO("Network", "Starting WebSocket Server on port 8080...");
    do_accept(acceptor, ioc);
    
    // Block the main thread on networking io_context
    ioc.run();

    // Cleanup
    LOG_INFO("SYS", "Shutting down...");
    udp_server->stop();
    motor_ctrl->stop();
    dual_cam->stop();
    lidar->stop();
    imu->stop();
    vision_thread.join();
    imu_thread.join();
    broadcast_thread.join();
    
    return 0;
}

