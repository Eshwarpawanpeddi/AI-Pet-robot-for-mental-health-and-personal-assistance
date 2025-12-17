# Pet Robot OS - Complete Architecture Redesign

## System Overview

```
Phone (Android/iOS)
├── Voice Input (Gemini Live API)
├── Manual Commands
└── UI Control Interface
    ↓ (WebSocket / HTTP)
    ↓
Laptop Server (Processing Core)
├── Gemini Live API Integration
├── LLM Processing
├── Voice Processing & Synthesis
├── Face Animation Engine
└── Command Orchestration
    ↓ (MQTT / WebSocket)
    ↓
Raspberry Pi 4 (Central Hub)
├── Hardware Management
├── GPIO/I2C Interface
├── Sensor Data Collection
└── Actuator Control
    ↓ (I2C / UART / WiFi)
    ├─→ ESP12E (Motion Control)
    │   ├── Motor Driver PWM
    │   ├── Servo Control
    │   └── Movement Commands
    │
    ├─→ Display/Speaker (Pi GPIO)
    │   ├── LCD/OLED Display
    │   └── Speaker Output
    │
    └─→ Sensors
        ├── Touch Sensors
        ├── Distance Sensors
        └── IMU/Gyroscope
```

## Hardware Connections

### Raspberry Pi 4 GPIO Pinout
- **GPIO 17, 27**: PWM Motor Control
- **GPIO 22, 23, 24**: Direction Control (Motor Driver)
- **GPIO 25, 26**: SPI Communication (Display)
- **GPIO 2, 3**: I2C (SDA/SCL) - Communication with ESP12E
- **GPIO 4, 14, 15**: Serial UART (If needed)
- **GPIO 18, 12, 13**: Audio Output (PWM or I2S)
- **GPIO 16-20**: Touch/Sensor Inputs

### ESP12E Connections
- **D0-D3**: Motor Control Outputs (to L298N Motor Driver)
- **D4-D5**: PWM Pins (Motor Speed)
- **D6-D7**: Sensor Inputs
- **I2C (D1, D2)**: Communication with Raspberry Pi

## Technology Stack

| Component | Technology |
|-----------|-----------|
| Pi Hardware Hub | Python 3.9+ |
| Motor/Motion Controller | Arduino (ESP12E) |
| Server (Processing) | Python FastAPI / Flask |
| Frontend (Phone) | React Native / Flutter |
| Face Animation | Three.js / Canvas / WebGL |
| Voice Processing | Gemini Live API |
| Communication | MQTT, WebSocket, gRPC |
| Database | MongoDB / Redis |
| Containerization | Docker |

## Phase-wise Implementation

### Phase 1: Hardware Communication Layer
- Raspberry Pi ↔ ESP12E Communication (I2C)
- GPIO Control & PWM Configuration
- Sensor Data Reading

### Phase 2: Server Core
- Gemini Live API Integration
- Real-time Audio Streaming
- Command Processing & Routing

### Phase 3: Front-end & Display
- Eilik-style Animated Face
- Phone App for Control
- Voice Command Processing

### Phase 4: Integration & Optimization
- End-to-End Testing
- Performance Optimization
- Deployment & Monitoring
