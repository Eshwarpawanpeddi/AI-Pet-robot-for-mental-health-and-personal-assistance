# AI Pet Robot - Updated Direct Pi Motor Control Architecture

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
    └─→ (WebSocket /ws/raspberry_pi)
        Raspberry Pi 4 (Motor Control, Face Display & Audio)
        ├── GPIO-based 4-Wheel Motor Control
        ├── 2x L298N Motor Drivers
        ├── HDMI Face Display Rendering
        ├── Audio Output via Audio Jack
        ├── Real-time Emotion Synchronization
        └── WebSocket Communication with Server
```

## Architecture Changes

### Key Updates from Previous Design

1. **Eliminated ESP12E Module**: Raspberry Pi now handles motor control directly via GPIO
2. **Direct Motor Control**: 2 L298N drivers controlled by Raspberry Pi GPIO pins
3. **4-Wheel Setup**: Four DC motors for better stability and maneuverability
4. **Server as Central Hub**: All communication flows through the central server
5. **Unified Pi Controller**: Single device handles motors, display, and audio

## Component Responsibilities

### Central Server (Laptop/Cloud)
- **Port**: 8000
- **Technology**: Python FastAPI, WebSocket, Gemini AI
- **Responsibilities**:
  - Process voice commands via Gemini API
  - Route movement commands to Raspberry Pi
  - Route emotion/face updates to Raspberry Pi
  - Manage control modes (manual vs autonomous)
  - Coordinate mobile app and Raspberry Pi
  - Handle mental health features (mood logging, affirmations, etc.)

### Raspberry Pi 4 (Motor Control, Display & Audio Hub)
- **Connection**: Wi-Fi WebSocket to Server (`ws://server:8000/ws/raspberry_pi`)
- **Hardware**: 2x L298N Motor Drivers, 4 DC Motors, HDMI Display, Audio Jack, GPIO
- **Responsibilities**:
  - **Motor Control**: Direct control of 4 DC motors via GPIO
  - Control 2 L298N motor drivers for 4-wheel movement
  - Execute movements: forward, backward, left, right, stop
  - Manage PWM speed control for all motors (0-100%)
  - **Display**: Render animated face on HDMI output
  - Sync face emotions with server commands
  - **Audio**: Play audio responses via audio jack
  - Handle real-time emotion updates from server
  - Display animations synchronized with robot state
  - Report status to server

### GPIO Pin Mapping (Raspberry Pi)

#### Motor Driver 1 (Front Wheels - Motors A & B):
- GPIO17 → IN1 (Motor A Direction 1)
- GPIO27 → IN2 (Motor A Direction 2)
- GPIO22 → ENA (Motor A Speed/PWM Enable)
- GPIO23 → IN3 (Motor B Direction 1)
- GPIO24 → IN4 (Motor B Direction 2)
- GPIO25 → ENB (Motor B Speed/PWM Enable)

#### Motor Driver 2 (Rear Wheels - Motors C & D):
- GPIO5 → IN1 (Motor C Direction 1)
- GPIO6 → IN2 (Motor C Direction 2)
- GPIO13 → ENA (Motor C Speed/PWM Enable)
- GPIO19 → IN3 (Motor D Direction 1)
- GPIO26 → IN4 (Motor D Direction 2)
- GPIO12 → ENB (Motor D Speed/PWM Enable)

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
  "speed": 75
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
    "raspberry_pi_connected": true
  }
}
```

#### `/ws/raspberry_pi` - Raspberry Pi Motor Control & Display
**Server → Raspberry Pi:**
```json
{
  "type": "move",
  "direction": "forward",
  "speed": 75
}

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
- Movement commands sent immediately to Raspberry Pi
- No autonomous navigation
- Face responds to manual emotion changes

### Autonomous Mode (ROS Integration)
- ROS system controls robot movements
- Autonomous navigation and obstacle avoidance
- Server coordinates between ROS and hardware
- Face emotions reflect autonomous decisions

## Motor Control Implementation

### Movement Logic

#### Forward Movement
- All 4 motors rotate forward
- Synchronized speed control via PWM
- Duty cycle: 0-100% based on speed command

#### Backward Movement
- All 4 motors rotate backward
- Synchronized speed control via PWM

#### Left Turn
- Left motors (A & C) rotate backward
- Right motors (B & D) rotate forward
- Tank-style turning for tight maneuvers

#### Right Turn
- Left motors (A & C) rotate forward
- Right motors (B & D) rotate backward
- Tank-style turning for tight maneuvers

#### Stop
- All motors stopped
- PWM duty cycle set to 0%
- Direction pins set LOW

### PWM Configuration
- **Frequency**: 1000Hz (1kHz) for smooth operation
- **Duty Cycle Range**: 0-100%
- **Speed Mapping**: Command speed (0-100) directly maps to duty cycle

## Fallback & Error Handling

### Connection Loss Handling
- Server tracks connection status for Raspberry Pi
- Mobile app shows connection indicators
- Graceful degradation when Pi disconnects
- Auto-reconnect with exponential backoff
- Motors automatically stop on connection loss (safety feature)

## Network Configuration

### Required Network Setup
1. **Same Wi-Fi Network**: Server and Raspberry Pi must be on same network
2. **Static IP Recommended**: Server should have static IP or DHCP reservation
3. **Firewall**: Allow port 8000 for WebSocket connections
4. **Router Configuration**: Enable mDNS/Bonjour for easy discovery (optional)

### Configuration Files
- **Raspberry Pi**: Update `SERVER_URL` in `raspberry_pi_controller.py`
- **Mobile App**: Set server URL in app settings

## Hardware Wiring Guide

### L298N Motor Driver Connections

#### Motor Driver 1 (Front Wheels):
```
Raspberry Pi GPIO  →  L298N Driver 1
GPIO17            →  IN1
GPIO27            →  IN2
GPIO22            →  ENA
GPIO23            →  IN3
GPIO24            →  IN4
GPIO25            →  ENB

L298N Driver 1    →  Motors
OUT1, OUT2        →  Motor A (Front Left)
OUT3, OUT4        →  Motor B (Front Right)
```

#### Motor Driver 2 (Rear Wheels):
```
Raspberry Pi GPIO  →  L298N Driver 2
GPIO5             →  IN1
GPIO6             →  IN2
GPIO13            →  ENA
GPIO19            →  IN3
GPIO26            →  IN4
GPIO12            →  ENB

L298N Driver 2    →  Motors
OUT1, OUT2        →  Motor C (Rear Left)
OUT3, OUT4        →  Motor D (Rear Right)
```

#### Power Connections:
```
7-12V Power Supply  →  L298N 12V Input (both drivers)
Power Supply GND    →  L298N GND (both drivers)
Raspberry Pi GND    →  L298N GND (common ground - IMPORTANT!)
```

**IMPORTANT**: Always connect all grounds together (Raspberry Pi GND, both L298N GND pins, and power supply GND) to ensure proper signal reference.

## Future Provisions

### Ultrasonic Sensor (Placeholder)
- Can be added to Raspberry Pi GPIO in future
- Would report distance data via WebSocket
- Could trigger autonomous obstacle avoidance

### IMU/Gyroscope (Placeholder)
- Can be added via I2C for motion sensing
- Would improve autonomous navigation
- Could detect falls or orientation changes

### Camera Integration (Placeholder)
- Could be added to Raspberry Pi via CSI or USB
- Would enable vision-based navigation
- Could support gesture recognition

## Technology Stack

| Component | Technology |
|-----------|-----------|
| Server | Python 3.9+, FastAPI, WebSocket, Uvicorn |
| Raspberry Pi | Python 3.9+, asyncio, websockets, RPi.GPIO |
| Motor Drivers | 2x L298N H-Bridge drivers |
| Mobile App | Flutter/Dart, WebSocket, HTTP |
| AI Processing | Google Gemini API |
| Communication | WebSocket (real-time), HTTP REST (API) |

## Security Considerations

1. **WebSocket Security**: Use WSS (WebSocket Secure) in production
2. **Authentication**: Implement token-based auth for mobile app
3. **Network Isolation**: Consider separate IoT network for robot components
4. **API Keys**: Secure Gemini API key (environment variable, not in code)
5. **Firewall**: Restrict server port 8000 to local network only
