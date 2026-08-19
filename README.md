# Lunar Rover Mock WebSocket Backend (Phase 1)

This project provides the C++ Mock WebSocket Backend for the Lunar Rover Communications Infrastructure. It hosts a WebSocket server on port 8080 and broadcasts 15 Hz mock telemetry and hazard data to connected clients, simulating a swappable hardware backend.

## Prerequisites (Debian/Ubuntu)

Install the required dependencies using `apt`:

```bash
sudo apt update
sudo apt install -y build-essential cmake libboost-all-dev libwebsocketpp-dev nlohmann-json3-dev
```

## Build Instructions

1. Navigate to the project root directory.
2. Create a build directory and run CMake:
   ```bash
   mkdir build
   cd build
   cmake ..
   make
   ```

## Running the Server

Start the backend server:

```bash
./rover_backend
```

The server will start listening on `ws://localhost:8080`.

## JSON Protocol Specification

The protocol adheres strictly to the defined schema. 
*Note: The server silently applies valid commands to the internal thread-safe state without sending an explicit ACK. The client will observe state changes reflected in the 15 Hz telemetry stream.*

### Client to Server (Commands)
- **Drive**: `{"command": "drive", "linear_v": 1.5, "angular_w": 10.0}`
- **Set Mode**: `{"command": "set_mode", "mode": "AUTO_EXPLORE"}`
- **Deploy Arm**: `{"command": "deploy_arm"}`

### Server to Client (Telemetry Broadcast at 15 Hz)
```json
{
  "type": "telemetry",
  "timestamp_ms": 1690000000000,
  "state": {
    "mode": "STANDBY",
    "speed_m_s": 0.0,
    "heading_deg": 45.0,
    "pos_x": 10.5,
    "pos_y": 5.2
  },
  "hazard": {
    "level": "CAUTION",
    "distance_m": 2.5,
    "sector": "RIGHT",
    "type": "DROP"
  },
  "environment": {
    "temp_c": 22.5,
    "humidity": 45.0,
    "soil_moisture_detected": false
  }
}
```
# Rover
# Rover
