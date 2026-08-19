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

namespace beast = boost::beast;         // from <boost/beast.hpp>
namespace http = beast::http;           // from <boost/beast/http.hpp>
namespace websocket = beast::websocket; // from <boost/beast/websocket.hpp>
namespace net = boost::asio;            // from <boost/asio.hpp>
using tcp = boost::asio::ip::tcp;       // from <boost/asio/ip/tcp.hpp>
using json = nlohmann::json;

// Global Mock Flag
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

RoverState g_rover_state;
std::mutex g_state_mutex;

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
        // Set suggested timeout settings for the websocket
        ws_.set_option(websocket::stream_base::timeout::suggested(beast::role_type::server));

        // Accept the websocket handshake
        ws_.async_accept(
            beast::bind_front_handler(&session::on_accept, shared_from_this()));
    }

    void send_message(std::string msg) {
        // Post the write to the strand to avoid concurrent access
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

void mock_generator_thread() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dist_env_temp(10.0f, 35.0f);
    std::uniform_real_distribution<float> dist_env_hum(20.0f, 60.0f);
    
    // Hazard generation distributions
    std::uniform_int_distribution<int> hazard_level_dist(0, 2); 
    std::uniform_real_distribution<float> hazard_dist(0.5f, 10.0f);
    std::uniform_int_distribution<int> hazard_sector_dist(0, 2); 
    std::uniform_int_distribution<int> hazard_type_dist(0, 2); 

    auto next_hazard_update = std::chrono::steady_clock::now();
    std::string current_hazard_level = "CLEAR";
    float current_hazard_dist = 0.0f;
    std::string current_hazard_sector = "CENTER";
    std::string current_hazard_type = "ROCK";

    auto last_time = std::chrono::steady_clock::now();

    while (true) {
        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<float> dt_duration = now - last_time;
        float dt = dt_duration.count();
        last_time = now;

        RoverState current_state;
        {
            std::lock_guard<std::mutex> lock(g_state_mutex);
            // Simulate kinematics
            float heading_rad = g_rover_state.heading_deg * (M_PI / 180.0f);
            g_rover_state.pos_x += g_rover_state.linear_v * std::cos(heading_rad) * dt;
            g_rover_state.pos_y += g_rover_state.linear_v * std::sin(heading_rad) * dt;
            g_rover_state.heading_deg += g_rover_state.angular_w * dt; 
            
            // Normalize heading [0, 360)
            while (g_rover_state.heading_deg >= 360.0f) g_rover_state.heading_deg -= 360.0f;
            while (g_rover_state.heading_deg < 0.0f) g_rover_state.heading_deg += 360.0f;

            current_state = g_rover_state;
        }

        if (MOCK_HARDWARE_MODE) {
            // Update hazards randomly every 2.5 seconds
            if (now > next_hazard_update) {
                const char* levels[] = {"CLEAR", "CAUTION", "CRITICAL"};
                const char* sectors[] = {"LEFT", "CENTER", "RIGHT"};
                const char* types[] = {"ROCK", "DROP", "SLOPE"};

                current_hazard_level = levels[hazard_level_dist(gen)];
                current_hazard_dist = hazard_dist(gen);
                current_hazard_sector = sectors[hazard_sector_dist(gen)];
                current_hazard_type = types[hazard_type_dist(gen)];

                next_hazard_update = now + std::chrono::milliseconds(2500);
            }

            // Construct telemetry JSON payload
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

            telemetry["hazard"] = {
                {"level", current_hazard_level},
                {"distance_m", current_hazard_dist},
                {"sector", current_hazard_sector},
                {"type", current_hazard_type}
            };

            telemetry["environment"] = {
                {"temp_c", dist_env_temp(gen)},
                {"humidity", dist_env_hum(gen)},
                {"soil_moisture_detected", (dist_env_temp(gen) > 28.0f)} 
            };

            std::string payload = telemetry.dump();

            // Broadcast telemetry to all connected clients
            std::lock_guard<std::mutex> lock(g_connections_mutex);
            // Copy active connections to vector
            std::vector<std::shared_ptr<session>> active_connections(g_connections.begin(), g_connections.end());
            for (auto& session_ptr : active_connections) {
                session_ptr->send_message(payload);
            }
        }

        // Sleep to maintain ~15 Hz (66.6 ms)
        std::this_thread::sleep_for(std::chrono::milliseconds(66));
    }
}

int main() {
    try {
        std::cout << "[Backend] Starting Mock Generator Thread (15 Hz)..." << std::endl;
        std::thread mock_thread(mock_generator_thread);

        net::io_context ioc{1};
        tcp::acceptor acceptor(ioc, {tcp::v4(), 8080});

        std::cout << "[Backend] Starting WebSocket Server on port 8080..." << std::endl;
        do_accept(acceptor, ioc);
        
        ioc.run();

        mock_thread.join();
    } catch (const std::exception& e) {
        std::cerr << "[Fatal Error] " << e.what() << std::endl;
    }
    return 0;
}
