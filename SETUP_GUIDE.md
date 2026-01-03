# AI Pet Robot - Complete Setup Guide

## Multi-Port Architecture

This robot system uses **three separate servers** running on different ports for different purposes:

### Port 8000 - Primary Control Server
**Purpose:** Main control hub for the robot
**Features:**
- Movement control (manual and autonomous modes)
- Camera streaming from Raspberry Pi
- WebSocket connections for control clients
- Raspberry Pi hardware interface
- ROS bridge for autonomous navigation
- Mental health monitoring and AI conversation
- Command routing and state management

**Access:** `http://localhost:8000`

### Port 10000 - Emotion Display Server
**Purpose:** Dedicated animated emotion/face display
**Features:**
- Full-screen animated robot face
- Real-time emotion updates
- Synchronized with primary server
- Smooth animations and transitions
- Can be displayed on separate monitor/screen

**Access:** `http://localhost:1000`

### Port 3000 - Mobile Web Interface
**Purpose:** Mobile-friendly control interface
**Features:**
- Touch-optimized controls
- Basic robot movement (d-pad)
- Camera view display
- Emotion controls
- Battery and status display
- Lightweight and responsive

**Access:** `http://localhost:3000`

## Architecture Diagram

```
┌─────────────────────────────────────────────────┐
│           Mobile/Web Clients                     │
│  - Desktop Browser → Port 8000                   │
│  - Mobile Browser  → Port 3000                   │
│  - Display Screen  → Port 10000                   │
└──────────────────┬──────────────────────────────┘
                   │
                   ▼
┌──────────────────────────────────────────────────┐
│         Server Cluster (Your Computer)            │
│  ┌──────────────────────────────────────────┐   │
│  │ Port 8000: Primary Control Server        │   │
│  │ - WebSocket: /ws/control                 │   │
│  │ - WebSocket: /ws/raspberry_pi            │   │
│  │ - WebSocket: /ws/ros                     │   │
│  │ - REST API: /api/*                       │   │
│  └──────────┬──────────────┬────────────────┘   │
│             │              │                     │
│  ┌──────────▼─────────┐  ┌▼─────────────────┐   │
│  │ Port 10000:         │  │ Port 3000:       │   │
│  │ Emotion Display    │  │ Mobile Web       │   │
│  │ - Polls 8000       │  │ - Forwards to    │   │
│  │ - POST /api/emotion│  │   port 8000      │   │
│  └────────────────────┘  └──────────────────┘   │
└──────────┬───────────────────────────────────────┘
           │
           ▼
┌──────────────────────────────────────────────────┐
│         Raspberry Pi (Robot Hardware)             │
│  - Motor Control (4 wheels via 2x L298N)         │
│  - Camera (USB or Pi Camera)                     │
│  - Speaker/TTS (espeak)                          │
│  - Face Display (HDMI monitor - optional)        │
│  - Connects to: ws://SERVER_IP:8000/ws/raspberry_pi │
└──────────────────────────────────────────────────┘
           │
           ▼
┌──────────────────────────────────────────────────┐
│         ROS System (Optional - Autonomous)        │
│  - Navigation stack                              │
│  - SLAM and mapping                              │
│  - Obstacle avoidance                            │
│  - Connects to: ws://SERVER_IP:8000/ws/ros       │
└──────────────────────────────────────────────────┘
```

## Quick Start

### 1. Prerequisites

**Server Machine Requirements:**
- Python 3.9+
- 2GB+ RAM
- Linux/Windows/macOS

**Raspberry Pi Requirements:**
- Raspberry Pi 4 (4GB+ recommended)
- Raspberry Pi OS (Bullseye or newer)
- 2x L298N Motor Drivers
- 4x DC Motors
- USB Camera or Pi Camera (optional)
- HDMI Display (optional, for face)
- Speaker/Audio output

**Optional (for ROS):**
- Ubuntu 20.04 with ROS Noetic
- Or Ubuntu 18.04 with ROS Melodic

### 2. Server Setup

#### Step 1: Clone Repository
```bash
git clone https://github.com/Eshwarpawanpeddi/AI-Pet-robot-for-mental-health-and-personal-assistance.git
cd AI-Pet-robot-for-mental-health-and-personal-assistance
```

#### Step 2: Install Dependencies
```bash
cd server
pip install -r requirements.txt
```

#### Step 3: Configure Environment
```bash
# Copy environment template
cp .env.example .env

# Edit .env and add your Gemini API key
nano .env
```

Add this line to `.env`:
```
GEMINI_API_KEY=your-actual-api-key-here
```

Get your API key from: https://makersuite.google.com/app/apikey

#### Step 4: Start All Servers

**Option A: Start All Three Servers (Recommended)**
```bash
python launch_all.py
```

This starts:
- Port 8000: Primary control
- Port 10000: Emotion display
- Port 3000: Mobile web interface

**Option B: Start Servers Individually**
```bash
# Terminal 1 - Primary server
python server.py

# Terminal 2 - Emotion display
python emotion_display_server.py

# Terminal 3 - Mobile web interface
python mobile_web_server.py
```

**Option C: Other Modes**
```bash
# All servers + hardware simulation
python launch_all.py --full

# All servers + Pi simulation
python launch_all.py --with-pi

# Only primary server (legacy mode)
python launch_all.py --server-only
```

#### Step 5: Verify Servers are Running
```bash
# Check primary server
curl http://localhost:8000/health

# Check emotion display
curl http://localhost:1000/health

# Check mobile web
curl http://localhost:3000/health
```

### 3. Raspberry Pi Setup

#### Step 1: Prepare Raspberry Pi
```bash
# Update system
sudo apt-get update
sudo apt-get upgrade -y

# Install dependencies
sudo apt-get install -y python3-pip python3-websockets python3-rpi.gpio espeak

# Install Python packages
pip3 install websockets asyncio picamera opencv-python-headless
```

#### Step 2: Configure Hardware
Connect motors to Raspberry Pi GPIO pins as follows:

**Motor Driver 1 (Front Wheels):**
- GPIO17 → Motor A IN1
- GPIO27 → Motor A IN2
- GPIO22 → Motor A ENA (PWM)
- GPIO23 → Motor B IN3
- GPIO24 → Motor B IN4
- GPIO25 → Motor B ENB (PWM)

**Motor Driver 2 (Rear Wheels):**
- GPIO5 → Motor C IN1
- GPIO6 → Motor C IN2
- GPIO13 → Motor C ENA (PWM)
- GPIO19 → Motor D IN3
- GPIO26 → Motor D IN4
- GPIO12 → Motor D ENB (PWM)

#### Step 3: Copy Controller Script
```bash
cd /home/pi
git clone https://github.com/Eshwarpawanpeddi/AI-Pet-robot-for-mental-health-and-personal-assistance.git
cd AI-Pet-robot-for-mental-health-and-personal-assistance/hardware/raspberry_pi
```

#### Step 4: Configure Server Connection
Edit `raspberry_pi_controller.py` and update the server URL:
```bash
nano raspberry_pi_controller.py
```

Find this line and change to your server's IP:
```python
SERVER_URL = "ws://192.168.1.100:8000/ws/raspberry_pi"  # Change to your server IP
```

#### Step 5: Run Controller
```bash
python3 raspberry_pi_controller.py
```

The Raspberry Pi will:
- Connect to primary server (port 8000)
- Control motors based on commands
- Stream camera feed (if enabled)
- Display face animations (if HDMI display connected)
- Speak responses via TTS

### 4. ROS Integration (Optional - For Autonomous Mode)

#### Step 1: Install ROS
```bash
# Ubuntu 20.04 - ROS Noetic
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-noetic-desktop-full
source /opt/ros/noetic/setup.bash
```

#### Step 2: Build ROS Workspace
```bash
cd ros_workspace
catkin_make
source devel/setup.bash
```

#### Step 3: Launch ROS Bridge
```bash
roslaunch pet_robot_ros ros_bridge.launch
```

The ROS bridge connects to port 8000 at `/ws/ros` and enables:
- Autonomous navigation
- Obstacle avoidance
- SLAM and mapping
- High-level movement commands

## Usage

### Accessing the Interfaces

#### Primary Control Interface (Port 8000)
1. Open browser: `http://localhost:8000`
2. Features:
   - Full robot control
   - Camera streaming
   - AI conversation
   - Mental health monitoring
   - Mode switching (manual/autonomous)
   - Complete status display

#### Emotion Display (Port 10000)
1. Open browser: `http://localhost:1000`
2. Best on separate monitor/screen
3. Shows animated robot face
4. Automatically syncs emotions from primary server
5. Full-screen for best experience

#### Mobile Web Interface (Port 3000)
1. Open on mobile: `http://SERVER_IP:3000`
2. Touch-optimized controls
3. Lightweight and fast
4. Perfect for smartphone control

### Using Multiple Displays

**Recommended Setup:**
- **Main Monitor:** Primary control (port 8000)
- **Second Monitor:** Emotion display (port 10000) - Full screen
- **Smartphone:** Mobile interface (port 3000)

### Emotion Synchronization

Emotions automatically sync across all servers:

1. **Change emotion on port 8000** (primary control)
   ```javascript
   // Via WebSocket
   ws.send(JSON.stringify({type: 'emotion', emotion: 'happy'}));
   ```

2. **Emotion display (port 10000) automatically updates**
   - Polls port 8000 every second
   - Also accepts direct POST requests
   
3. **Mobile interface (port 3000) shows updated emotion**
   - Receives via WebSocket from primary server

### Control Modes

#### Manual Mode (Default)
- Direct control via web interface or mobile
- Commands sent to Raspberry Pi immediately
- Use arrow keys or d-pad buttons

#### Autonomous Mode (ROS)
- Requires ROS bridge running
- High-level navigation commands
- Obstacle avoidance
- Mapping capabilities

Switch modes:
```javascript
// Via WebSocket
ws.send(JSON.stringify({type: 'set_mode', mode: 'autonomous'}));
```

Or use the mode toggle buttons in the web interface.

## API Reference

### Primary Server (Port 8000)

#### WebSocket Endpoints

**`/ws/control`** - Control clients (web/mobile)
```json
// Move robot
{"type": "move", "direction": "forward", "speed": 200}

// Change emotion
{"type": "emotion", "emotion": "happy"}

// Toggle camera
{"type": "start_camera"}
{"type": "stop_camera"}

// Set control mode
{"type": "set_mode", "mode": "autonomous"}
```

**`/ws/raspberry_pi`** - Raspberry Pi connection
```json
// Camera frame (Pi → Server)
{"type": "camera_frame", "frame": "base64_jpeg_data"}

// Motor command (Server → Pi)
{"type": "move", "direction": "forward", "speed": 200}
```

**`/ws/ros`** - ROS bridge connection
```json
// ROS state update
{"type": "ros_state", "data": {...}}

// Mode change
{"type": "set_mode", "mode": "autonomous"}
```

#### REST Endpoints

- `GET /health` - Health check
- `GET /api/status` - Full system status
- `GET /api/state` - Current robot state
- `POST /api/control_mode` - Set control mode
- `POST /api/speak` - Make robot speak
- `GET /api/mental_health/insights` - Mental health data

### Emotion Display Server (Port 10000)

#### WebSocket Endpoints

**`/ws/emotion_display`** - Display clients
```json
// Emotion update (Server → Client)
{"type": "emotion_update", "emotion": "happy"}
```

#### REST Endpoints

- `POST /api/emotion` - Update emotion
  ```json
  {"emotion": "happy"}
  ```
- `GET /api/emotion` - Get current emotion
- `GET /health` - Health check

### Mobile Web Server (Port 3000)

#### WebSocket Endpoints

**`/ws/mobile`** - Mobile clients

Same commands as `/ws/control` on port 8000 - automatically forwarded.

#### REST Endpoints

- `GET /health` - Health check
- `GET /api/status` - Connection status

## Troubleshooting

### Servers Won't Start

**Port already in use:**
```bash
# Find process using port
lsof -i :8000
lsof -i :1000
lsof -i :3000

# Kill process
kill -9 <PID>
```

**Missing dependencies:**
```bash
pip install -r server/requirements.txt
```

**API key not set:**
- Verify `.env` file exists
- Check `GEMINI_API_KEY` is set correctly
- Get key from https://makersuite.google.com/app/apikey

### Emotion Display Not Syncing

**Check primary server is running:**
```bash
curl http://localhost:8000/health
```

**Check network connectivity:**
```bash
curl http://localhost:1000/health
```

**Manual emotion update:**
```bash
curl -X POST http://localhost:1000/api/emotion \
  -H "Content-Type: application/json" \
  -d '{"emotion": "happy"}'
```

### Mobile Interface Not Connecting

**Check server connection:**
- Verify port 3000 server is running
- Verify port 8000 server is running
- Check websocket connection in browser console

**Firewall issues:**
```bash
# Allow ports
sudo ufw allow 8000
sudo ufw allow 1000
sudo ufw allow 3000
```

### Raspberry Pi Not Connecting

**Check server URL:**
- Verify `SERVER_URL` in `raspberry_pi_controller.py`
- Use server IP address, not localhost
- Check port 8000 is accessible from Pi

**Test connection:**
```bash
# From Raspberry Pi
ping SERVER_IP
curl http://SERVER_IP:8000/health
```

**Check GPIO permissions:**
```bash
sudo usermod -a -G gpio pi
```

## Deployment

### Development
- Use `python launch_all.py` for local development
- All three servers on localhost
- Suitable for testing and development

### Production

#### Using Docker (Recommended)
```bash
# Build images
docker-compose build

# Start all services
docker-compose up -d

# View logs
docker-compose logs -f
```

#### Using systemd (Linux)

Create service files for each server:

**`/etc/systemd/system/robot-primary.service`**
```ini
[Unit]
Description=Robot Primary Control Server
After=network.target

[Service]
Type=simple
User=robot
WorkingDirectory=/home/robot/robot/server
ExecStart=/usr/bin/python3 /home/robot/robot/server/server.py
Restart=always

[Install]
WantedBy=multi-user.target
```

Repeat for `robot-emotion.service` and `robot-mobile.service`.

Enable and start:
```bash
sudo systemctl enable robot-primary robot-emotion robot-mobile
sudo systemctl start robot-primary robot-emotion robot-mobile
```

### Security Considerations

**Production Checklist:**
- [ ] Use HTTPS/WSS (not HTTP/WS)
- [ ] Add authentication
- [ ] Use environment variables for secrets
- [ ] Enable firewall
- [ ] Regular security updates
- [ ] Monitor logs
- [ ] Backup configurations

## Advanced Topics

### Custom Emotion Display

Create your own emotion display by connecting to port 10000's WebSocket:
```javascript
const ws = new WebSocket('ws://localhost:1000/ws/emotion_display');
ws.onmessage = (event) => {
    const data = JSON.parse(event.data);
    if (data.type === 'emotion_update') {
        // Update your custom display
        console.log('New emotion:', data.emotion);
    }
};
```

### Integrating Additional Sensors

Add sensors to Raspberry Pi and send data via WebSocket:
```python
# In raspberry_pi_controller.py
async def send_sensor_data(websocket):
    while True:
        sensor_data = read_sensors()
        await websocket.send(json.dumps({
            'type': 'sensor_data',
            'data': sensor_data
        }))
        await asyncio.sleep(1)
```

### Multi-Robot Setup

Run multiple instances on different ports:
```bash
# Robot 1
PORT=8001 python server.py

# Robot 2  
PORT=8002 python server.py
```

## Support

**Documentation:**
- [README.md](../README.md) - Overview
- [MENTAL_HEALTH_FEATURES.md](../MENTAL_HEALTH_FEATURES.md) - Mental health features
- [INTEGRATION_UPDATE.md](../INTEGRATION_UPDATE.md) - Latest updates

**Issues:**
- GitHub Issues: https://github.com/Eshwarpawanpeddi/AI-Pet-robot-for-mental-health-and-personal-assistance/issues

**Community:**
- Discussions: Use GitHub Discussions for questions

---

*Last Updated: January 2026*
*Version: 2.0 - Multi-Port Architecture*
