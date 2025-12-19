# AI Pet Robot - Updated Wi-Fi Architecture

## System Overview

```
Mobile App (Android/iOS)
├── Voice Input (Gemini API)
├── Manual Control Commands
├── Mood Tracking & Mental Health Features
└── UI Control Interface
    ↓ (WebSocket / HTTP)
    ↓
Central Server (Laptop/Cloud - Processing Core)
├── Gemini AI Integration
├── LLM Processing & Voice Synthesis
├── Command Orchestration & Routing
├── Mode Management (Manual/Autonomous)
└── WebSocket Hub for All Components
    ↓
    ├─→ (WebSocket /ws/esp)
    │   ESP12E (Motor Controller over Wi-Fi)
    │   ├── Wi-Fi Communication
    │   ├── Motor Driver PWM Control (L298N)
    │   ├── Movement Commands (Forward, Back, Left, Right, Stop)
    │   ├── Fallback Mode on Connection Loss
    │   └── Sensor Data Reporting
    │
    └─→ (WebSocket /ws/raspberry_pi)
        Raspberry Pi 4 (Face Display & Audio)
        ├── HDMI Face Display Rendering
        ├── Audio Output via Audio Jack
        ├── Real-time Emotion Synchronization
        └── WebSocket Communication with Server
```

## Architecture Changes

### Key Updates from Original Design

1. **No Direct Pi-ESP Communication**: Removed I2C connection between Raspberry Pi and ESP12E
2. **Server as Central Hub**: All communication flows through the central server
3. **Wi-Fi Based**: ESP12E connects to server via Wi-Fi WebSocket
4. **Raspberry Pi for Display**: Handles face animations on HDMI and audio output
5. **Mobile App Control**: Primary interface for manual control and mode switching

## Component Responsibilities

### Central Server (Laptop/Cloud)
- **Port**: 8000
- **Technology**: Python FastAPI, WebSocket, Gemini AI
- **Responsibilities**:
  - Process voice commands via Gemini API
  - Route movement commands to ESP12E
  - Route emotion/face updates to Raspberry Pi
  - Manage control modes (manual vs autonomous)
  - Coordinate mobile app, ESP12E, and Raspberry Pi
  - Handle mental health features (mood logging, affirmations, etc.)

### ESP12E (Motor Controller)
- **Connection**: Wi-Fi WebSocket to Server (`ws://server:8000/ws/esp`)
- **Hardware**: L298N Motor Driver, DC Motors
- **Responsibilities**:
  - Receive movement commands from server
  - Control motors with PWM (speed 0-255)
  - Execute movements: forward, backward, left, right, stop
  - Enter fallback mode if connection drops (auto-stop)
  - Send heartbeat and sensor data to server
  - Report status and connection state

### Raspberry Pi 4 (Display & Audio Hub)
- **Connection**: Wi-Fi WebSocket to Server (`ws://server:8000/ws/raspberry_pi`)
- **Hardware**: HDMI Display, Audio Jack, Optional GPIO sensors
- **Responsibilities**:
  - Render animated face on HDMI output
  - Sync face emotions with server commands
  - Play audio responses via audio jack
  - Handle real-time emotion updates from server
  - Display animations synchronized with robot state
  - Report status to server

### Mobile App (Flutter)
- **Connection**: WebSocket to Server (`ws://server:8000/ws/control`)
- **Platforms**: Android, iOS
- **Responsibilities**:
  - Manual robot control (joystick/buttons)
  - Toggle between manual and autonomous modes
  - Voice interaction (Gemini integration)
  - Mood tracking and mental health features
  - Display robot status and connection state
  - Show face preview and emotion state

## Communication Protocols

### WebSocket Endpoints

#### `/ws/control` - Mobile App & Web Interface
**Client → Server:**
```json
{
  "type": "move",
  "direction": "forward",
  "speed": 200
}

{
  "type": "emotion",
  "emotion": "happy"
}

{
  "type": "set_mode",
  "mode": "autonomous"
}
```

**Server → Client:**
```json
{
  "type": "state",
  "data": {
    "emotion": "happy",
    "is_speaking": false,
    "battery_level": 85,
    "control_mode": "manual",
    "esp_connected": true,
    "raspberry_pi_connected": true
  }
}
```

#### `/ws/esp` - ESP12E Motor Controller
**Server → ESP12E:**
```json
{
  "type": "move",
  "direction": "forward",
  "speed": 200
}
```

**ESP12E → Server:**
```json
{
  "type": "heartbeat",
  "device": "ESP12E",
  "status": "normal",
  "uptime": 12345
}

{
  "type": "sensor",
  "sensor": "touch",
  "value": true
}
```

#### `/ws/raspberry_pi` - Raspberry Pi Display
**Server → Raspberry Pi:**
```json
{
  "type": "emotion",
  "emotion": "happy"
}

{
  "type": "face_animation",
  "animation": {
    "emotion": "excited",
    "duration": 2000
  }
}

{
  "type": "play_audio",
  "audio": {
    "text": "Hello! How are you?",
    "emotion": "friendly"
  }
}
```

**Raspberry Pi → Server:**
```json
{
  "type": "status",
  "device": "RaspberryPi",
  "emotion": "happy"
}
```

## Control Modes

### Manual Mode (Default)
- User controls robot directly via mobile app
- Movement commands sent immediately to ESP12E
- No autonomous navigation
- Face responds to manual emotion changes

### Autonomous Mode (ROS Integration)
- ROS system controls robot movements
- Autonomous navigation and obstacle avoidance
- Server coordinates between ROS and hardware
- Face emotions reflect autonomous decisions

## Fallback & Error Handling

### ESP12E Fallback Mode
- Triggered when no commands received for 10+ seconds
- Automatically stops all motors
- Continues sending heartbeat to indicate alive status
- Resumes normal operation when commands resume

### Connection Loss Handling
- Server tracks connection status for all components
- Mobile app shows connection indicators
- Graceful degradation when components disconnect
- Auto-reconnect with exponential backoff

## Network Configuration

### Required Network Setup
1. **Same Wi-Fi Network**: Server, ESP12E, and Raspberry Pi must be on same network
2. **Static IP Recommended**: Server should have static IP or DHCP reservation
3. **Firewall**: Allow port 8000 for WebSocket connections
4. **Router Configuration**: Enable mDNS/Bonjour for easy discovery (optional)

### Configuration Files
- **ESP12E**: `hardware/esp12e/config.h` - Set Wi-Fi SSID, password, and server IP
- **Raspberry Pi**: Update `SERVER_URL` in controller script
- **Mobile App**: Set server URL in app settings

## Future Provisions

### Ultrasonic Sensor (Placeholder)
- Can be added to ESP12E in future
- Would report distance data via sensor messages
- Could trigger autonomous obstacle avoidance

### IMU/Gyroscope (Placeholder)
- Can be added for motion sensing
- Would improve autonomous navigation
- Could detect falls or orientation changes

### Camera Integration (Placeholder)
- Could be added to Raspberry Pi
- Would enable vision-based navigation
- Could support gesture recognition

## Technology Stack

| Component | Technology |
|-----------|-----------|
| Server | Python 3.9+, FastAPI, WebSocket, Uvicorn |
| ESP12E | Arduino C++, WebSocketsClient, ArduinoJson |
| Raspberry Pi | Python 3.9+, asyncio, websockets |
| Mobile App | Flutter/Dart, WebSocket, HTTP |
| AI Processing | Google Gemini API |
| Communication | WebSocket (real-time), HTTP REST (API) |

## Security Considerations

1. **WebSocket Security**: Use WSS (WebSocket Secure) in production
2. **Authentication**: Implement token-based auth for mobile app
3. **Network Isolation**: Consider separate IoT network for robot components
4. **API Keys**: Secure Gemini API key (environment variable, not in code)
5. **Firewall**: Restrict server port 8000 to local network only
