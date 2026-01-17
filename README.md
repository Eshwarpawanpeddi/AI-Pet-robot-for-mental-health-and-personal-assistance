# AI Pet Robot for Mental Health and Personal Assistance

An empathetic AI-powered companion robot designed specifically for **mental health support** and **personal assistance**. This robot uses advanced AI, emotional intelligence, and compassionate interaction to provide daily support, companionship, and wellness assistance for individuals managing their mental health.

## 🎉 Latest Updates (January 2026)

### 🆕 Industrial-Level Features (v2.2) - NEW!
- 🎛️ **Port-Specific Gemini Control**: Enable/disable AI on ports 8000 and 3000 independently
- 😊 **Integrated Emotion Detection**: Automatic emotion detection from port 9999 for context-aware AI
- 📅 **Task Scheduling & Reminders**: Schedule tasks and set reminders through AI assistant
- 🔍 **Information Retrieval**: Query information through Gemini for quick answers
- 🏠 **Smart Home Integration**: Control IoT devices via MQTT, HTTP (lights, thermostats, locks, etc.)
- 🔊 **Enhanced Audio Support**: Multiple speaker output modes (3.5mm, GPIO PWM, I2S DAC)
- 📚 **Comprehensive API**: RESTful endpoints for all features with detailed documentation

**See [INDUSTRIAL_FEATURES.md](INDUSTRIAL_FEATURES.md) for complete industrial-level features documentation.**

### 🆕 Multi-Port Architecture (v2.1)
- **Port 8000**: Primary control server (movement, camera, AI, ROS, autonomous navigation)
- **Port 10000**: Dedicated emotion display server (animated face)
- **Port 3000**: ✨ Enhanced mobile web interface with joystick, WASD controls, and speed adjustment
- **Port 9999**: 😊 Real-time facial emotion detection with AI analysis
- **Emotion Synchronization**: Automatic sync across all ports
- **Flexible Deployment**: Run servers independently or together

### ✨ Core Features:
- 🤖 **Autonomous Navigation**: YOLO-based obstacle detection and avoidance with shared camera stream
- 🎮 **Enhanced Port 3000 Controls**: Joystick mode, WASD keyboard controls, speed slider, larger camera view
- 😊 **Emotion Detection (Port 9999)**: Real-time facial expression analysis using emotion_model.h5
- 🔊 **Speaker Support**: Complete guide for 40mm 8Ω speaker setup with multiple connection options
- 📖 **ROS Setup Guide**: Comprehensive documentation for autonomous navigation
- 🧠 **Gemini-1.5-Flash**: Using cost-effective model for faster, cheaper AI responses
- 🎥 **Live Camera Streaming**: Real-time video feed from Raspberry Pi to web and mobile interfaces
- 🤖 **ROS Integration**: Full autonomous navigation mode with ROS bridge
- 🧠 **Advanced Mental Health Monitoring**: Real-time emotion tracking with crisis detection
- 🎨 **Enhanced UI**: Split-panel interface with animated emotions and camera view
- 📱 **Mobile App Enhancements**: Camera view, control mode toggle, improved connectivity

**See [AUTONOMOUS_NAVIGATION.md](AUTONOMOUS_NAVIGATION.md) for autonomous navigation setup, [FEATURES_GUIDE.md](FEATURES_GUIDE.md) for other features, or [SETUP_GUIDE.md](SETUP_GUIDE.md) for setup instructions.**

## 🧠 Mental Health Focus

This robot is designed to:
- **Provide Emotional Support**: Active listening and empathetic responses
- **Combat Loneliness**: Consistent, non-judgmental companionship
- **Support Daily Routines**: Medication reminders, self-care prompts
- **Teach Coping Skills**: Breathing exercises, grounding techniques
- **Track Mood Patterns**: Help identify triggers and trends
- **Encourage Wellness**: Positive affirmations, gratitude practices

**⚠️ Important**: This robot is a *supportive companion*, NOT a replacement for professional mental health care. Always consult qualified healthcare providers for medical advice.

## 🌟 Key Features

### Mental Health Support
- **Empathetic Conversations**: Powered by Google's Gemini AI with mental health-focused prompting
- **Real-time Emotion Tracking**: Monitors emotional patterns over time (last 50 interactions)
- **Crisis Detection**: Automatically detects concerning keywords and escalates concern level
- **Mental State Analysis**: Analyzes emotional trends and provides insights
- **Concern Level Monitoring**: 0-10 scale tracking with automated recommendations
- **Mood Tracking**: Log and monitor emotional patterns
- **Crisis Resources**: Immediate access to professional help resources
- **Breathing Exercises**: Guided relaxation techniques
- **Positive Affirmations**: Daily encouragement and support
- **Enhanced Emotion Recognition**: Detects anxiety, stress, sadness, anger, and more
- **Personalized Recommendations**: Based on current mental state and concern level

### Personal Assistance
- **Task Scheduling**: Schedule tasks and appointments through AI
- **Smart Reminders**: Medication, appointments, and daily activity reminders
- **Information Retrieval**: Quick answers to questions via Gemini
- **Routine Building**: Establish healthy daily habits
- **Self-Care Prompts**: Hydration, movement, rest reminders
- **Goal Tracking**: Set and achieve wellness goals
- **Social Connection**: Encouragement for healthy relationships
- **Smart Home Control**: Control lights, thermostats, locks, and other IoT devices
- **Image Analysis**: Visual context understanding for better assistance

### Technical Features
- **Port-Specific Gemini Control**: Enable/disable AI independently on different ports
- **Integrated Emotion Detection**: Real-time facial emotion detection via port 9999
- **Smart Home IoT Integration**: MQTT, HTTP, and other protocol support
- **Autonomous Navigation**: YOLO-based obstacle detection and avoidance with intelligent pathfinding
- **Shared Camera Stream**: Multi-consumer architecture for parallel processing (navigation + emotion detection)
- **Live Camera Streaming**: Real-time video feed at 10 FPS from Raspberry Pi camera
- **Multi-Mode Audio**: Support for 3.5mm, GPIO PWM, and I2S DAC speakers
- **Text-to-Speech**: Offline speech synthesis on Raspberry Pi (espeak)
- **Voice Interaction**: Natural conversation via Gemini AI
- **Multimodal API**: Process text, images, and audio simultaneously
- **Animated Face Display**: Expressive emotions with advanced animations
- **Emotion Synchronization**: Smooth face/audio alignment
- **Autonomous Movement**: Physical robot capabilities with ROS navigation
- **ROS Integration**: Full Robot Operating System support with autonomous mode
- **WebSocket Communication**: Real-time responsiveness with multiple connection types
- **Mental Health Monitoring**: Real-time emotion tracking and crisis detection
- **Mobile App**: Android/iOS app with camera view and full control (Flutter)
- **Control Modes**: Manual control, autonomous navigation, or ROS autonomous
- **Security**: Token-based authentication and encrypted connections
- **Privacy-First**: All data stays on your device

## 🏗️ System Architecture

### Multi-Port Server Setup

The system uses **three separate servers** for different purposes:

```
┌─────────────────────────────────────────────────────────────────┐
│                     CLIENT INTERFACES                            │
│  ┌──────────────┐ ┌──────────────┐ ┌─────────────────────────┐ │
│  │ Desktop Web  │ │ Mobile Web   │ │ Emotion Display         │ │
│  │ Port 8000    │ │ Port 3000    │ │ Port 10000              │ │
│  └──────┬───────┘ └──────┬───────┘ └────────────┬────────────┘ │
└─────────┼────────────────┼──────────────────────┼──────────────┘
          │                │                      │
          ▼                ▼                      ▼
┌──────────────────────────────────────────────────────────────────┐
│                    SERVER CLUSTER (Your Computer)                 │
│  ┌────────────────────────────────────────────────────────────┐  │
│  │ Port 8000: Primary Control Server                          │  │
│  │ - Movement control (manual/autonomous)                     │  │
│  │ - Camera streaming                                         │  │
│  │ - Gemini AI integration                                    │  │
│  │ - Mental health monitoring                                 │  │
│  │ - WebSocket: /ws/control, /ws/raspberry_pi, /ws/ros       │  │
│  └──────────┬──────────────────────────────┬──────────────────┘  │
│             │                              │                      │
│  ┌──────────▼─────────────┐  ┌────────────▼──────────────────┐  │
│  │ Port 10000:            │  │ Port 3000:                    │  │
│  │ Emotion Display        │  │ Mobile Web Interface          │  │
│  │ - Animated face        │  │ - Touch controls              │  │
│  │ - Auto-sync emotions   │  │ - Camera view                 │  │
│  │ - Full-screen display  │  │ - Lightweight UI              │  │
│  └────────────────────────┘  └───────────────────────────────┘  │
└──────────┬───────────────────────────────────────────────────────┘
           │
           ▼
┌──────────────────────────────────────────────────────────────────┐
│                  RASPBERRY PI (Robot Hardware)                    │
│  - Motor Control (4 wheels via 2x L298N)                         │
│  - Camera (USB or Pi Camera)                                     │
│  - Speaker/TTS (espeak)                                          │
│  - Connects to: ws://SERVER_IP:8000/ws/raspberry_pi             │
└──────────┬───────────────────────────────────────────────────────┘
           │
           ▼
┌──────────────────────────────────────────────────────────────────┐
│                    ROS SYSTEM (Optional)                          │
│  - Autonomous navigation                                         │
│  - SLAM and mapping                                              │
│  - Obstacle avoidance                                            │
│  - Connects to: ws://SERVER_IP:8000/ws/ros                       │
└──────────────────────────────────────────────────────────────────┘
```

### Port Breakdown

| Port | Purpose | Features |
|------|---------|----------|
| **8000** | Primary Control | Movement, camera, AI, ROS, mental health, autonomous navigation |
| **10000** | Emotion Display | Dedicated animated face, auto-sync |
| **3000** | Mobile Web | ✨ Touch controls, joystick, WASD, speed adjustment |
| **9999** | Emotion Detection | 😊 Facial expression analysis, AI responses |

## 📋 Prerequisites

### Hardware Requirements
- **Central Server**: Laptop or Desktop (Linux/Windows/Mac) or Cloud Server
- **Raspberry Pi 4** (4GB+ recommended) - For motor control, face display, and audio
- **2x L298N Motor Driver** - For controlling 4 DC motors
- **DC Motors** (4x) for 4-wheel movement
- **HDMI Display** connected to Raspberry Pi for face animations
- **Speaker/Audio Output** via Raspberry Pi audio jack
- **Wi-Fi Router** - Server and Raspberry Pi must be on same network
- **Power supplies**: 5V for Pi, appropriate voltage (7-12V) for motors
- **Optional**: Touch sensors, ultrasonic distance sensor (for future expansion)

#### GPIO Pin Mappings (Raspberry Pi BCM Mode)

**Motor Control Pins:**
```
Left Motors (Front & Rear wired in parallel):
  - IN1: GPIO 17  (Direction control)
  - IN2: GPIO 27  (Direction control)
  - ENA: GPIO 22  (PWM speed control)

Right Motors (Front & Rear wired in parallel):
  - IN3: GPIO 23  (Direction control)
  - IN4: GPIO 24  (Direction control)
  - ENB: GPIO 25  (PWM speed control)
```

**Additional GPIO Pins:**
```
Audio (I2S):
  - BCK:  GPIO 18
  - LRCK: GPIO 20
  - DATA: GPIO 21

Touch Sensor:
  - INPUT: GPIO 16
```

**Wiring Notes:**
- Ensure all grounds are connected (Pi, motor drivers, power supplies)
- Use appropriate power supply for motors (7-12V recommended)
- Connect motor driver inputs to GPIO pins as specified above
- Motor driver outputs connect to DC motors (OUT1/OUT2 → Left, OUT3/OUT4 → Right)
- PWM frequency: 1kHz for smooth motor operation

### Software Requirements
- Python 3.9+
- Docker and Docker Compose (for server - optional)
- Flutter SDK (for mobile app - optional)
- Node.js 18+ (for mobile app development - optional)
- **ROS Noetic** (Ubuntu 20.04) or **ROS Melodic** (Ubuntu 18.04) - **Optional for autonomous mode**

## 🚀 Quick Start

### Multi-Port Setup

This system runs three servers on different ports. You can run them all together or separately.

### 1. Clone the Repository

```bash
git clone https://github.com/Eshwarpawanpeddi/AI-Pet-robot-for-mental-health-and-personal-assistance.git
cd AI-Pet-robot-for-mental-health-and-personal-assistance
```

### 2. Server Setup (Your Computer)

#### Install Dependencies
```bash
cd server
pip install -r requirements.txt
```

#### Configure Environment
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

#### Start All Servers

**Option A: Start All Three Servers (Recommended)**
```bash
python launch_all.py
```

This starts:
- **Port 8000**: Primary control server
- **Port 10000**: Emotion display server
- **Port 3000**: Mobile web interface

**Option B: Start Servers Individually**
```bash
# Terminal 1 - Primary server (port 8000)
python server.py

# Terminal 2 - Emotion display (port 10000)
python emotion_display_server.py

# Terminal 3 - Mobile web interface (port 3000)
python mobile_web_server.py
```

**Option C: Other Launch Modes**
```bash
# All servers + hardware simulation
python launch_all.py --full

# All servers + Pi simulation
python launch_all.py --with-pi

# Only primary server (legacy mode)
python launch_all.py --server-only
```

#### Verify Servers are Running
```bash
# Check primary server
curl http://localhost:8000/health

# Check emotion display
curl http://localhost:10000/health

# Check mobile web
curl http://localhost:3000/health
```

### 3. Access the Interfaces

**Primary Control (Port 8000)**
- URL: `http://localhost:8000`
- Features: Full robot control, camera, AI conversation

**Emotion Display (Port 10000)**
- URL: `http://localhost:10000`
- Features: Animated face, emotion display
- Best viewed full-screen on a dedicated monitor

**Mobile Web Interface (Port 3000)**
- URL: `http://localhost:3000` or `http://YOUR_IP:3000` on mobile
- Features: Touch controls, camera view, emotion buttons

### 4. Raspberry Pi Setup

### 4. Raspberry Pi Setup

```bash
# On Raspberry Pi
cd hardware/raspberry_pi

# Install dependencies
sudo apt-get update
sudo apt-get install -y python3-pip python3-websockets python3-rpi.gpio espeak
pip3 install websockets asyncio picamera opencv-python-headless

# Update the SERVER_URL in raspberry_pi_controller.py
# Edit the file and set SERVER_URL to your server's IP
nano raspberry_pi_controller.py
# Change: SERVER_URL = "ws://YOUR_SERVER_IP:8000/ws/raspberry_pi"

# Run controller
python3 raspberry_pi_controller.py
```

**Note**: The Raspberry Pi connects to **port 8000** (primary server) and handles:
- Motor control via GPIO
- Camera streaming
- Face display (if HDMI connected)
- Audio output via TTS

### 5. ROS Integration (Optional - For Autonomous Mode)

For advanced robotics features and ROS ecosystem integration:

#### 1. Install ROS

```bash
# Ubuntu 20.04 (ROS Noetic)
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-noetic-desktop-full
source /opt/ros/noetic/setup.bash
```

#### 2. Build ROS Workspace

```bash
cd ros_workspace
catkin_make
source devel/setup.bash
```

#### 3. Launch ROS Nodes

```bash
# Launch all robot nodes
roslaunch pet_robot_ros pet_robot.launch

# Or launch only mental health features
roslaunch pet_robot_ros mental_health.launch
```

#### 4. Test ROS Integration

```bash
# Get an affirmation
rosservice call /mental_health/get_affirmation "{}"

# Log mood
rosservice call /mental_health/log_mood "mood: 'happy'
intensity: 8
notes: 'Feeling great today!'"

# Control robot movement
rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.5}"
```

**📚 Full ROS Documentation**: See [ros_workspace/src/pet_robot_ros/README.md](ros_workspace/src/pet_robot_ros/README.md)

### Option 3: Mobile App Setup (Android/iOS)

Control the robot from your smartphone with the Flutter mobile app:

#### 1. Install Flutter

```bash
git clone https://github.com/flutter/flutter.git -b stable
export PATH="$PATH:`pwd`/flutter/bin"
flutter doctor
```

#### 2. Setup Mobile App

```bash
cd mobile_app
flutter pub get
```

#### 3. Configure Server Connection

Edit `lib/config/app_config.dart` or use the app's Settings screen to set your server IP.

#### 4. Run the App

```bash
# Run on connected Android device
flutter run

# Build APK for Android
flutter build apk

# For iOS (requires macOS)
flutter build ios
```

**Features:**
- Real-time robot control via joystick
- Mood tracking and logging
- Positive affirmations
- Breathing exercises
- Crisis resources
- Animated robot face display
- WebSocket connection status

**📱 Full Mobile App Documentation**: See [mobile_app/README.md](mobile_app/README.md)

## 🎮 Usage

### Web Interface Controls

- **🎤 Listen**: Toggle voice input for conversations
- **⬆️ Forward**: Move robot forward  
- **⏹️ Stop**: Stop all motors
- **😊 Happy**: Change emotion to happy
- **💙 Affirmation**: Get a positive affirmation
- **🫁 Breathe**: Start breathing exercise
- **Keyboard Controls**:
  - Arrow keys: Move robot
  - Spacebar: Toggle listening
  - A: Get affirmation
  - B: Breathing exercise

### Mental Health API Endpoints

```bash
# Log your mood
curl -X POST http://localhost:8000/api/mood \
  -H "Content-Type: application/json" \
  -d '{
    "mood": "anxious",
    "intensity": 7,
    "notes": "Feeling worried about tomorrow"
  }'

# Get a positive affirmation
curl -X POST http://localhost:8000/api/affirmation

# Get breathing exercise guidance
curl -X POST http://localhost:8000/api/breathing

# Access crisis resources
curl http://localhost:8000/api/crisis_resources
```

### WebSocket API

Connect to `ws://localhost:8000/ws/control` and send JSON commands:

```json
// Move robot
{
  "type": "move",
  "direction": "forward",
  "speed": 200
}

// Change emotion
{
  "type": "emotion",
  "emotion": "happy"
}

// Get robot state
{
  "type": "get_state"
}
```

### REST API

```bash
# Health check
curl http://localhost:8000/health

# Get robot state
curl http://localhost:8000/api/state

# Send command
curl -X POST http://localhost:8000/api/command \
  -H "Content-Type: application/json" \
  -d '{"type": "move", "direction": "forward", "speed": 200}'
```

## 📁 Project Structure

```
AI-Pet-robot-for-mental-health-and-personal-assistance/
├── server/                      # Backend server (FastAPI)
│   ├── server.py               # Main server application
│   ├── gemini_integration.py   # Gemini AI integration
│   ├── requirements.txt        # Python dependencies
│   └── Dockerfile             # Docker configuration
├── frontend/                   # Web interface
│   ├── face_display.html      # Animated face display
│   └── styles.css             # Styling
├── hardware/
│   └── raspberry_pi/          # Raspberry Pi controller
│       ├── raspberry_pi_controller.py  # Main controller with motor control
│       ├── test_motors.py     # Motor testing script
│       ├── config.yaml        # Hardware configuration
│       └── requirements.txt
├── mobile_app/                 # Mobile App (Flutter) (NEW)
│   ├── lib/
│   │   ├── main.dart          # App entry point
│   │   ├── config/            # Configuration
│   │   ├── models/            # Data models
│   │   ├── services/          # WebSocket & API services
│   │   ├── providers/         # State management
│   │   ├── screens/           # UI screens
│   │   └── widgets/           # Reusable widgets
│   ├── android/               # Android configuration
│   ├── ios/                   # iOS configuration
│   ├── pubspec.yaml          # Flutter dependencies
│   └── README.md             # Mobile app docs
├── ros_workspace/              # ROS Integration
│   └── src/
│       └── pet_robot_ros/     # ROS package
│           ├── msg/           # Custom messages
│           ├── srv/           # Service definitions
│           ├── action/        # Action definitions
│           ├── nodes/         # ROS nodes
│           ├── launch/        # Launch files
│           ├── config/        # ROS configuration
│           └── README.md      # ROS documentation
├── downloads/                  # Documentation
│   ├── setup-guide.md
│   ├── robot-architecture.md
│   ├── api-integration.md
│   ├── hardware-code.md
│   ├── 4-wheel-setup-guide.md  # NEW: 4-wheel setup guide
│   └── face-display.md
├── docker-compose.yml         # Docker Compose configuration
├── .env.example              # Environment template
├── .gitignore               # Git ignore rules
└── README.md                # This file
```

## 🔧 Configuration

### Environment Variables

Create a `.env` file in the root directory:

```bash
# Gemini API Configuration
GEMINI_API_KEY=your-gemini-api-key-here

# Server Configuration
LOG_LEVEL=INFO

# Server Host (for inter-server communication)
# Use "localhost" for local development (default)
# Use "0.0.0.0" or your machine's IP for remote access
# Example: SERVER_HOST=192.168.1.100
SERVER_HOST=localhost

# Network Configuration
RASPBERRY_PI_IP=192.168.1.100
ROBOT_NAME=MyPetRobot
```

### Multi-Device Access Configuration

**All servers automatically support concurrent access from multiple devices** as they bind to `0.0.0.0` (all network interfaces).

**To access from different devices on your network:**

1. **Find your server machine's IP address:**
   ```bash
   # Linux/Mac
   hostname -I
   # or
   ifconfig | grep "inet "
   
   # Windows
   ipconfig
   ```

2. **Set SERVER_HOST in `.env` file:**
   ```bash
   SERVER_HOST=192.168.1.100  # Use your actual IP address
   ```

3. **Restart all servers:**
   ```bash
   cd server
   python launch_all.py
   ```

4. **Access from any device on the same network:**
   - **Primary Control:** `http://192.168.1.100:8000` 
   - **Mobile Interface:** `http://192.168.1.100:3000`
   - **Emotion Display:** `http://192.168.1.100:10000`
   - **Emotion Detection:** `http://192.168.1.100:9999`
   
   Replace `192.168.1.100` with your actual server IP.

**Features:**
- ✅ Multiple devices can connect simultaneously
- ✅ Each port accepts unlimited WebSocket connections
- ✅ Real-time synchronization across all connected devices
- ✅ Camera stream shared across all ports
- ✅ Gemini AI responses broadcast to all connected clients

**Firewall Configuration:**
- Ensure ports 8000, 3000, 9999, and 10000 are allowed through your firewall
- For Linux: `sudo ufw allow 8000,3000,9999,10000/tcp`
- For Windows: Add inbound rules for these ports in Windows Firewall

### Raspberry Pi Configuration

Edit `hardware/raspberry_pi/config.yaml` to configure GPIO pins, I2C settings, and server connection.

**Note:** GPIO pin mappings are documented above in the Hardware Requirements section.

### ESP12E Configuration

Edit `hardware/esp12e/config.h` to configure pin mappings and motor settings.

## 🧪 Testing

### Test Server

```bash
# Test health endpoint
curl http://localhost:8000/health

# Test WebSocket (using websocat)
websocat ws://localhost:8000/ws/control
```

### Test I2C Communication

```bash
# On Raspberry Pi
i2cdetect -y 1
# Should show ESP12E at address 0x08
```

### Test Motor Control

```python
from hardware.raspberry_pi.raspberry_pi_controller import RaspberryPiController

controller = RaspberryPiController()
controller.move_forward(200)
time.sleep(2)
controller.stop()
```

## 🐛 Troubleshooting

### Server Won't Start

- Check if Gemini API key is set correctly
- Ensure port 8000 is not in use
- Check logs: `docker-compose logs robot-server`

### Can't Access Server from Other Devices (IP Address Not Working)

If the server shows "running" but you can't access it via IP address from other devices:

1. **Set SERVER_HOST in .env file**:
   ```bash
   # Find your machine's IP address
   # Linux/Mac: ifconfig or ip addr
   # Windows: ipconfig
   
   # Edit .env file and set:
   SERVER_HOST=192.168.1.100  # Use your actual IP
   ```

2. **Restart all servers** after changing SERVER_HOST:
   ```bash
   cd server
   python launch_all.py
   ```

3. **Access using the same IP**:
   - `http://192.168.1.100:8000` (Primary Control)
   - `http://192.168.1.100:3000` (Mobile Interface)
   - `http://192.168.1.100:10000` (Emotion Display)
   - `http://192.168.1.100:9999` (Emotion Detection)

4. **Check firewall settings**: Ensure ports 8000, 3000, 9999, and 10000 are allowed

### Raspberry Pi Not Connecting

- Verify server URL in `raspberry_pi_controller.py`
- Check network connectivity: `ping <server_ip>`
- Ensure WebSocket port 8000 is accessible
- Check server logs for Raspberry Pi connection attempts
- Verify RPi.GPIO library is installed: `python3 -c "import RPi.GPIO"`

### Motors Not Moving

- Check Raspberry Pi WebSocket connection status
- Verify motor driver (L298N) connections to GPIO pins (see GPIO Pin Mappings section)
- Check power supply voltage (7-12V) and current capacity
- Test motors directly with power supply
- Verify GPIO pin numbers match configuration:
  - Left: IN1=17, IN2=27, ENA=22
  - Right: IN3=23, IN4=24, ENB=25
- Check server logs for motor command transmission
- Monitor Raspberry Pi logs for command receipt
- Ensure all grounds are connected properly (Pi, motor drivers, power supplies)
- Verify GPIO pins are in valid range (2-27 for BCM mode)

### Camera Not Working or Working on One Port Only

**Camera initialization issues:**
- Verify picamera2 is installed: `pip3 install picamera2`
- Check camera hardware connection
- Enable camera interface: `sudo raspi-config` → Interface Options → Camera
- Test camera: `libcamera-hello --list-cameras`
- Check Raspberry Pi logs for camera initialization errors
- Verify camera is not being used by another process

**Camera streaming issues:**
- Ensure Raspberry Pi is connected to primary server (port 8000)
- Check WebSocket connection status in logs
- Camera streams from Raspberry Pi to port 8000, then broadcasts to other ports
- Verify `start_camera` command is being sent
- Check for threading conflicts in logs (fixed in latest version)

### Gemini AI Not Responding from Some Ports

**Issue:** Gemini works on port 8000 but not on ports 3000 or 9999

**Solution (Fixed):**
- Ports 3000 and 9999 now have `/api/speak` endpoints that proxy to port 8000
- Ensure all servers are running with latest code
- Verify SERVER_HOST is set correctly in `.env` file
- Check that port 8000 (primary server) has Gemini API key configured
- Test each port:
  ```bash
  # Test port 8000 (primary)
  curl -X POST http://localhost:8000/api/speak -H "Content-Type: application/json" -d '{"text":"Hello"}'
  
  # Test port 3000 (mobile)
  curl -X POST http://localhost:3000/api/speak -H "Content-Type: application/json" -d '{"text":"Hello"}'
  
  # Test port 9999 (emotion detection)
  curl -X POST http://localhost:9999/api/speak -H "Content-Type: application/json" -d '{"text":"Hello"}'
  ```
- Check server logs for proxying errors
- TTS responses are automatically broadcast to all connected WebSocket clients on all ports

### WebSocket Connection Failed

- Check firewall settings on server
- Verify server is running: `curl http://<server_ip>:8000/health`
- Ensure all devices are on same network
- Use correct server IP address (not localhost from remote devices)
- Check router allows WebSocket connections

## 📚 Additional Documentation

- **[INDUSTRIAL_FEATURES.md](INDUSTRIAL_FEATURES.md)** - 🆕 **Industrial-level features: Gemini control, emotion integration, tasks, smart home**
- **[AUTONOMOUS_NAVIGATION.md](AUTONOMOUS_NAVIGATION.md)** - 🆕 **Complete guide to autonomous navigation with YOLO object detection**
- **[FEATURES_GUIDE.md](FEATURES_GUIDE.md)** - 🆕 **Complete guide to new features (Port 3000, Emotion Detection, etc.)**
- **[ROS_SETUP_GUIDE.md](ROS_SETUP_GUIDE.md)** - 🆕 **Complete ROS installation and usage guide**
- **[SPEAKER_SETUP_GUIDE.md](SPEAKER_SETUP_GUIDE.md)** - 🆕 **Hardware guide for 40mm 8Ω speaker**
- **[SETUP_GUIDE.md](SETUP_GUIDE.md)** - 🆕 **Complete setup guide for multi-port architecture**
- **[Integration Update](INTEGRATION_UPDATE.md)** - Latest features: Camera, Speech, ROS, Mental Health Monitoring
- **[Mental Health Features](MENTAL_HEALTH_FEATURES.md)** - Comprehensive mental health support documentation
- **[Multimodal API Guide](downloads/multimodal-api-guide.md)** - Complete guide for multimodal interactions
- **[4-Wheel Setup Guide](downloads/4-wheel-setup-guide.md)** - Detailed setup guide for 4-wheel motor control
- [Setup Guide](downloads/setup-guide.md) - Legacy setup instructions
- [Architecture](downloads/robot-architecture.md) - System architecture details
- [API Integration](downloads/api-integration.md) - API documentation
- [Hardware Code](downloads/hardware-code.md) - Hardware implementation
- [Face Display](downloads/face-display.md) - Frontend documentation

## 🆘 Crisis Support

**If you or someone you know is in crisis, please reach out immediately:**

- **USA**: 
  - 988 Suicide & Crisis Lifeline
  - Crisis Text Line: Text HOME to 741741
  - Emergency: 911

- **International**: 
  - Find resources at https://www.iasp.info/resources/Crisis_Centres/

This robot can provide emotional support, but **professional help is essential** for mental health crises.

## 🔒 Security

- Never commit `.env` file with real API keys
- Use environment variables for sensitive data
- Enable firewall on Raspberry Pi
- Use HTTPS/WSS for production deployments
- Implement authentication for production use

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## 📄 License

This project is open source and available under the MIT License.

## 👥 Authors

- Eshwar Pawan Peddi

## 🙏 Acknowledgments

- Google Gemini AI for voice processing
- FastAPI for the web framework
- Raspberry Pi Foundation
- ESP8266 Community

## 📞 Support

For issues and questions:
- Open an issue on GitHub
- Check the troubleshooting guide
- Review documentation in `/downloads`

---

**Note**: This is an educational/hobbyist project. For production use, additional security, error handling, and testing are recommended.
