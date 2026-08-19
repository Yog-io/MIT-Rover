#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <nlohmann/json.hpp>
#include <iostream>
#include <thread>
#include <chrono>

namespace beast = boost::beast;
namespace http = beast::http;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = boost::asio::ip::tcp;

int main() {
    try {
        net::io_context ioc{1};
        tcp::acceptor acceptor{ioc, {net::ip::make_address("0.0.0.0"), 8080}};
        
        std::cout << "[Diagnostic] Starting Phase 1 WebSocket Test on port 8080..." << std::endl;
        
        tcp::socket socket{ioc};
        acceptor.accept(socket);
        
        websocket::stream<tcp::socket> ws{std::move(socket)};
        ws.accept();
        std::cout << "[Diagnostic] Client connected. Broadcasting dummy payload at 15Hz." << std::endl;

        while (true) {
            auto start_time = std::chrono::steady_clock::now();
            
            nlohmann::json payload = {
                {"telemetry", {
                    {"status", "diagnostic"},
                    {"timestamp", std::chrono::duration_cast<std::chrono::milliseconds>(start_time.time_since_epoch()).count()}
                }}
            };
            
            ws.write(net::buffer(payload.dump()));
            
            auto end_time = std::chrono::steady_clock::now();
            auto loop_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
            
            std::cout << "\r[Diagnostic] Broadcast loop time: " << loop_time_ms << " ms    " << std::flush;
            
            // 15 Hz = 66.6 ms per frame
            std::this_thread::sleep_for(std::chrono::milliseconds(66));
        }
    } catch(std::exception const& e) {
        std::cerr << "\nError: " << e.what() << std::endl;
    }
    return 0;
}
