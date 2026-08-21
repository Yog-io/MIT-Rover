// DOM Elements
const wsUrlInput = document.getElementById('ws-url');
const connectBtn = document.getElementById('connect-btn');
const connectionStatus = document.getElementById('connection-status');
const canvas = document.getElementById('map-canvas');
const ctx = canvas.getContext('2d');

const valYaw = document.getElementById('val-yaw');
const valLidar = document.getElementById('val-lidar');
const valTemp = document.getElementById('val-temp');
const valHumidity = document.getElementById('val-humidity');
const valMoisture = document.getElementById('val-moisture');
const valPressure = document.getElementById('val-pressure');
const valFps = document.getElementById('val-fps');
const valMode = document.getElementById('val-mode');

const hazardIndicator = document.getElementById('hazard-indicator');
const toggleModeBtn = document.getElementById('toggle-mode-btn');
const estopBtn = document.getElementById('estop-btn');

// Constants
const GRID_SIZE = 400; // 400x400 grid cells
const CELL_SIZE = canvas.width / GRID_SIZE; // 800 / 400 = 2px per cell
const MAP_METER_SIZE = 20; // 20x20m map

// State
let ws = null;
let currentMode = "MANUAL";
let mapGrid = new Uint8Array(GRID_SIZE * GRID_SIZE); // 0: Unknown, 1: Traversable, 2: Hazard
let roverPose = { x: 0, y: 0, yaw: 0 };
let activeKeys = { w: false, a: false, s: false, d: false };

// Initialize default WS URL
wsUrlInput.value = `ws://${window.location.hostname || 'localhost'}:8080`;

// Setup Canvas Initial State
ctx.fillStyle = '#111111'; // Unknown black
ctx.fillRect(0, 0, canvas.width, canvas.height);

// WebSocket Connection
function connectWebSocket() {
    if (ws) ws.close();
    
    const url = wsUrlInput.value;
    connectionStatus.textContent = 'Connecting...';
    connectionStatus.className = 'status-indicator yellow';
    
    try {
        ws = new WebSocket(url);
        
        ws.onopen = () => {
            connectionStatus.textContent = 'Connected';
            connectionStatus.className = 'status-indicator connected';
        };
        
        ws.onclose = () => {
            connectionStatus.textContent = 'Disconnected';
            connectionStatus.className = 'status-indicator disconnected';
        };
        
        ws.onerror = (error) => {
            console.error('WebSocket Error:', error);
            connectionStatus.textContent = 'Error';
            connectionStatus.className = 'status-indicator disconnected';
        };
        
        ws.onmessage = (event) => {
            try {
                const data = JSON.parse(event.data);
                handleTelemetry(data);
            } catch (e) {
                console.error("Failed to parse telemetry", e);
            }
        };
    } catch(e) {
        alert("Invalid WebSocket URL");
        connectionStatus.textContent = 'Disconnected';
        connectionStatus.className = 'status-indicator disconnected';
    }
}

connectBtn.addEventListener('click', connectWebSocket);

// Handle Incoming Telemetry
function handleTelemetry(data) {
    // 1. Update HUD
    if (data.telemetry) {
        if (data.telemetry.yaw !== undefined) valYaw.textContent = `${data.telemetry.yaw.toFixed(1)}°`;
        if (data.telemetry.lidar_depth !== undefined) valLidar.textContent = `${data.telemetry.lidar_depth.toFixed(2)} m`;
        if (data.telemetry.env) {
            if (data.telemetry.env.temp !== undefined) valTemp.textContent = `${data.telemetry.env.temp.toFixed(1)} °C`;
            if (data.telemetry.env.humidity !== undefined) valHumidity.textContent = `${data.telemetry.env.humidity.toFixed(1)} %`;
            if (data.telemetry.env.moisture !== undefined) valMoisture.textContent = `${data.telemetry.env.moisture.toFixed(1)} %`;
            if (data.telemetry.env.pressure !== undefined) valPressure.textContent = `${data.telemetry.env.pressure.toFixed(1)} hPa`;
        }
        if (data.telemetry.vision_fps !== undefined) valFps.textContent = data.telemetry.vision_fps;
        
        if (data.telemetry.mode !== undefined) {
            if (data.telemetry.mode !== currentMode) {
                currentMode = data.telemetry.mode;
                updateModeUI();
            }
        }
        
        if (data.telemetry.hazard_status) {
            updateHazardStatus(data.telemetry.hazard_status);
        }
    }
    
    // 2. Update Map Grid
    if (data.map_deltas && Array.isArray(data.map_deltas)) {
        data.map_deltas.forEach(delta => {
            const idx = delta.y * GRID_SIZE + delta.x;
            if (idx >= 0 && idx < mapGrid.length) {
                if (delta.state === "TRAVERSABLE") mapGrid[idx] = 1;
                else if (delta.state === "HAZARD") mapGrid[idx] = 2;
                else mapGrid[idx] = 0;
            }
        });
    }
    
    // 3. Update Rover Pose
    if (data.pose) {
        roverPose.x = data.pose.x;
        roverPose.y = data.pose.y;
        roverPose.yaw = data.pose.yaw;
    }
    
    requestAnimationFrame(() => renderFullMap(data.pois));
}

function updateModeUI() {
    valMode.textContent = currentMode;
    if (currentMode === "MANUAL") {
        toggleModeBtn.textContent = "Switch to AUTONOMOUS";
        valMode.style.color = "var(--accent-blue)";
    } else {
        toggleModeBtn.textContent = "Switch to MANUAL";
        valMode.style.color = "var(--success)";
    }
}

function updateHazardStatus(status) {
    hazardIndicator.className = 'hazard-indicator';
    status = status.toUpperCase();
    if (status === 'GREEN' || status === 'CLEAR') {
        hazardIndicator.classList.add('green');
        hazardIndicator.textContent = 'ALL CLEAR';
    } else if (status === 'YELLOW' || status === 'WARNING') {
        hazardIndicator.classList.add('yellow');
        hazardIndicator.textContent = 'WARNING';
    } else if (status === 'RED' || status === 'DANGER') {
        hazardIndicator.classList.add('red');
        hazardIndicator.textContent = 'DANGER';
    }
}

function paintCell(cx, cy, state) {
    if (state === 1) ctx.fillStyle = '#333333'; // Traversable (dark grey)
    else if (state === 2) ctx.fillStyle = '#ff3333'; // Hazard (red)
    else ctx.fillStyle = '#111111'; // Unknown (black)
    
    ctx.fillRect(cx * CELL_SIZE, cy * CELL_SIZE, CELL_SIZE, CELL_SIZE);
}

// Render loop for Map
function renderFullMap(pois) {
    // Redraw grid
    for (let y = 0; y < GRID_SIZE; y++) {
        for (let x = 0; x < GRID_SIZE; x++) {
            const state = mapGrid[y * GRID_SIZE + x];
            paintCell(x, y, state);
        }
    }
    
    // Render POIs
    if (pois && Array.isArray(pois)) {
        pois.forEach(poi => {
            const px = (poi.x / MAP_METER_SIZE) * canvas.width;
            const py = (poi.y / MAP_METER_SIZE) * canvas.height;
            
            ctx.fillStyle = '#00ffff';
            ctx.beginPath();
            ctx.arc(px, py, 4, 0, Math.PI * 2);
            ctx.fill();
        });
    }
    
    // Render Rover
    const rx = (roverPose.x / MAP_METER_SIZE) * canvas.width;
    const ry = (roverPose.y / MAP_METER_SIZE) * canvas.height;
    
    ctx.save();
    ctx.translate(rx, ry);
    ctx.rotate(roverPose.yaw * Math.PI / 180); 
    
    ctx.fillStyle = '#3b82f6';
    ctx.beginPath();
    ctx.moveTo(0, -10); // tip
    ctx.lineTo(8, 8); // bottom right
    ctx.lineTo(0, 4); // inner bottom
    ctx.lineTo(-8, 8); // bottom left
    ctx.closePath();
    ctx.fill();
    
    ctx.restore();
}

// Teleop Controls
function sendTeleopCmd() {
    if (!ws || ws.readyState !== WebSocket.OPEN) return;
    
    let linear = 0;
    let angular = 0;
    
    if (activeKeys.w) linear += 0.5;
    if (activeKeys.s) linear -= 0.5;
    if (activeKeys.a) angular += 0.5;
    if (activeKeys.d) angular -= 0.5;
    
    const cmd = {
        cmd_linear_v: linear,
        cmd_angular_v: angular,
        mode: currentMode
    };
    
    ws.send(JSON.stringify(cmd));
}

function handleKey(key, isDown) {
    if (currentMode !== "MANUAL") return;
    
    let changed = false;
    key = key.toLowerCase();
    
    if (key === 'w' || key === 'a' || key === 's' || key === 'd') {
        if (activeKeys[key] !== isDown) {
            activeKeys[key] = isDown;
            changed = true;
            
            const btn = document.getElementById(`btn-${key}`);
            if (btn) {
                if (isDown) btn.classList.add('active');
                else btn.classList.remove('active');
            }
        }
    }
    
    if (changed) {
        sendTeleopCmd();
    }
}

window.addEventListener('keydown', (e) => handleKey(e.key, true));
window.addEventListener('keyup', (e) => handleKey(e.key, false));

['w','a','s','d'].forEach(k => {
    const btn = document.getElementById(`btn-${k}`);
    if (btn) {
        btn.addEventListener('mousedown', () => handleKey(k, true));
        btn.addEventListener('mouseup', () => handleKey(k, false));
        btn.addEventListener('mouseleave', () => handleKey(k, false));
    }
});

toggleModeBtn.addEventListener('click', () => {
    currentMode = currentMode === "MANUAL" ? "AUTONOMOUS" : "MANUAL";
    updateModeUI();
    
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({
            cmd_linear_v: 0,
            cmd_angular_v: 0,
            mode: currentMode
        }));
    }
});

estopBtn.addEventListener('click', () => {
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({
            cmd_linear_v: 0,
            cmd_angular_v: 0,
            estop: true,
            mode: "MANUAL"
        }));
    }
    
    activeKeys = { w: false, a: false, s: false, d: false };
    ['w','a','s','d'].forEach(k => {
        document.getElementById(`btn-${k}`)?.classList.remove('active');
    });
    
    currentMode = "MANUAL";
    updateModeUI();
});
