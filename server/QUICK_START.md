# AI Pet Robot - Quick Start Guide

## Running the Complete System

### Option 1: Run Server Only (Recommended for Production)

This starts just the FastAPI server with all features available. Connect external components as needed.

```bash
cd server
python server.py
```

Then open http://localhost:8000 in your browser.

**What works:**
- ✓ Web interface with all controls
- ✓ Gemini AI conversations
- ✓ Mental health monitoring
- ✓ WebSocket endpoints ready

**To enable additional features:**
- Camera & Motors: Run `raspberry_pi_controller.py` on your Raspberry Pi
- Autonomous Mode: Run `roslaunch pet_robot_ros ros_bridge.launch`

---

### Option 2: Integrated Launcher (Development/Testing)

Start all components together with automatic monitoring.

```bash
cd server
python launch_all.py
```

**Available modes:**

```bash
# Server only (same as Option 1)
python launch_all.py

# Server + Raspberry Pi simulation
python launch_all.py --with-pi

# Everything (server + Pi sim + ROS if available)
python launch_all.py --full
```

---

## Web Interface Features

Once the server is running, access http://localhost:8000 to see:

### Left Panel - Robot Face
- Animated emotions (happy, sad, angry, neutral)
- Real-time emotion changes based on conversation
- Smooth animations and effects

### Right Panel - Controls
- **📹 Camera View**: Live feed from Raspberry Pi camera
- **🎤 Audio Controls**: Start/stop listening and speaking
- **🎮 Control Mode**: Switch between Manual and ROS/Autonomous
- **📊 Connection Status**: See what's connected in real-time

---

## API Endpoints

### Health & Status
```bash
# Quick health check
curl http://localhost:8000/health

# Comprehensive status
curl http://localhost:8000/api/status

# Robot state
curl http://localhost:8000/api/state
```

### Mental Health
```bash
# Get mental health insights
curl http://localhost:8000/api/mental_health/insights
```

### Control
```bash
# Set control mode
curl -X POST http://localhost:8000/api/control_mode \
  -H "Content-Type: application/json" \
  -d '{"mode": "manual"}'

# Trigger speech
curl -X POST http://localhost:8000/api/speak \
  -H "Content-Type: application/json" \
  -d '{"text": "Hello, how can I help you?"}'
```

---

## Connection Status Monitoring

The server automatically logs connection status every 30 seconds:

```
📊 Connection Status:
   - Web Clients: 1
   - Raspberry Pi: ✓ Connected
   - ROS Bridge: ✗ Not connected
   - Camera: ✓ Active
   - Control Mode: MANUAL
   - Mental Health: Concern Level 2/10
```

---

## Setup for Different Environments

### Local Development
```bash
# Terminal 1: Server
cd server
python server.py

# Terminal 2: Optional - Pi simulation
cd hardware/raspberry_pi
python raspberry_pi_controller.py

# Terminal 3: Optional - ROS
roslaunch pet_robot_ros ros_bridge.launch
```

### Production (Distributed)
```bash
# On Main Computer:
cd server
python server.py

# On Raspberry Pi:
cd hardware/raspberry_pi
python raspberry_pi_controller.py

# On ROS Computer (optional):
roslaunch pet_robot_ros ros_bridge.launch
```

### Docker (Coming Soon)
```bash
docker-compose up
```

---

## Troubleshooting

### Server won't start
- Check if port 8000 is available: `lsof -i :8000`
- Verify Gemini API key: Check `.env` file
- Check logs for specific errors

### No camera feed
- Ensure Raspberry Pi is connected (check /api/status)
- Click "Start Camera" in web interface
- Verify picamera2 is installed on Pi

### ROS mode not available
- Check if ROS bridge is running
- Verify connection at /api/status
- Switch mode using the toggle in web interface

### Mental health insights not updating
- Interact with the robot via text
- Check concern level at /api/mental_health/insights
- Verify Gemini AI is responding

---

## Environment Variables

Create a `.env` file in the `server` directory:

```env
GEMINI_API_KEY=your_api_key_here
LOG_LEVEL=INFO
```

---

## Mobile App

Configure the server IP in the mobile app settings:
1. Open app settings
2. Enter your computer's IP address (e.g., 192.168.1.100)
3. Port: 8000
4. Connect

---

## For More Information

- Full documentation: [INTEGRATION_UPDATE.md](../INTEGRATION_UPDATE.md)
- Mental health features: [MENTAL_HEALTH_FEATURES.md](../MENTAL_HEALTH_FEATURES.md)
- Architecture: [README.md](../README.md)
