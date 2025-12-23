# AI Pet Robot for Mental Health and Personal Assistance

An empathetic AI-powered companion robot designed specifically for **mental health support** and **personal assistance**. This robot uses advanced AI, emotional intelligence, and compassionate interaction to provide daily support, companionship, and wellness assistance for individuals managing their mental health.

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
- **Multimodal Interactions**: Text, voice, and image processing for richer communication
- **Mood Tracking**: Log and monitor emotional patterns
- **Crisis Resources**: Immediate access to professional help resources
- **Breathing Exercises**: Guided relaxation techniques
- **Positive Affirmations**: Daily encouragement and support
- **Emotion Recognition**: Detects anxiety, stress, sadness, and more

### Personal Assistance
- **Medication Reminders**: Never miss important doses
- **Routine Building**: Establish healthy daily habits
- **Self-Care Prompts**: Hydration, movement, rest reminders
- **Goal Tracking**: Set and achieve wellness goals
- **Social Connection**: Encouragement for healthy relationships
- **Image Analysis**: Visual context understanding for better assistance

### Technical Features
- **Voice Interaction**: Natural conversation via Gemini AI
- **Multimodal API**: Process text, images, and audio simultaneously
- **Animated Face Display**: Expressive, comforting presence
- **Emotion Synchronization**: Smooth face/audio alignment
- **Autonomous Movement**: Physical robot capabilities
- **WebSocket Communication**: Real-time responsiveness with authentication
- **ROS Integration**: Full Robot Operating System support
- **Mobile App**: Android/iOS app for remote control (Flutter)
- **Security**: Token-based authentication and encrypted connections
- **Privacy-First**: All data stays on your device

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                  MOBILE APP / WEB CLIENT                         │
│  ┌──────────────┐ ┌──────────────┐ ┌─────────────────────────┐ │
│  │ Voice Input  │ │ Manual Ctrl  │ │ Animated Face Display   │ │
│  │ (Gemini AI)  │ │ (Joystick)   │ │ (Shows Emotions)        │ │
│  └──────┬───────┘ └──────┬───────┘ └────────────┬────────────┘ │
│         └────────────────┼──────────────────────┘               │
│                 WebSocket/HTTP (Real-time)                       │
└─────────────────────────┼─────────────────────────────────────┘
                          │
         ┌────────────────┼────────────────┐
         │                │                │
         ▼                ▼                ▼
┌──────────────────────────────────────────────┐
│     CENTRAL SERVER (Processing Core)          │
│  ┌────────────────────────────────────────┐  │
│  │ Gemini AI Integration                   │  │
│  │ - Voice Processing                      │  │
│  │ - LLM Response Generation               │  │
│  │ - Emotion Detection & Synthesis         │  │
│  └────────────────────────────────────────┘  │
│  ┌────────────────────────────────────────┐  │
│  │ WebSocket Hub & Command Routing         │  │
│  │ - Raspberry Pi Motor & Face (Wi-Fi)     │  │
│  │ - Mobile App Control (WebSocket)        │  │
│  │ - Mode Toggle (Manual/Autonomous)       │  │
│  └────────────────────────────────────────┘  │
└──────────────┬───────────────────────────────┘
               │
      WebSocket│
       (Wi-Fi)│
               │
       ┌───────▼────────────────────┐
       │   RASPBERRY PI 4           │
       │  (Motor Control & Display) │
       │  ┌──────────────────────┐  │
       │  │ GPIO Motor Control   │  │
       │  │ 2x L298N Drivers     │  │
       │  │ 4-Wheel Setup        │  │
       │  └──────────────────────┘  │
       │  ┌──────────────────────┐  │
       │  │ HDMI Face Display    │  │
       │  │ Real-time Render     │  │
       │  └──────────────────────┘  │
       │  ┌──────────────────────┐  │
       │  │ Audio Output         │  │
       │  │ (Audio Jack)         │  │
       │  └──────────────────────┘  │
       └─────────┬──────────────────┘
                 │
          ┌──────┴──────┐
          │             │
    ┌─────▼────┐  ┌────▼─────┐
    │ L298N #1 │  │ L298N #2 │
    └─────┬────┘  └────┬─────┘
          │            │
      ┌───┴───┐    ┌───┴───┐
      │       │    │       │
      ▼       ▼    ▼       ▼
   Motor A Motor B Motor C Motor D
   (Front L)(Front R)(Rear L)(Rear R)
```

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

### Software Requirements
- Python 3.9+
- Docker and Docker Compose (for server - optional)
- Flutter SDK (for mobile app - optional)
- Node.js 18+ (for mobile app development - optional)
- **ROS Noetic** (Ubuntu 20.04) or **ROS Melodic** (Ubuntu 18.04) - **Optional for autonomous mode**

## 🚀 Quick Start

### Option 1: Standard Setup (FastAPI Server)

### 1. Clone the Repository

```bash
git clone https://github.com/Eshwarpawanpeddi/AI-Pet-robot-for-mental-health-and-personal-assistance.git
cd AI-Pet-robot-for-mental-health-and-personal-assistance
```

### 2. Server Setup (Laptop)

#### Option A: Using Docker (Recommended)

```bash
# Copy environment template
cp .env.example .env

# Edit .env and add your Gemini API key
nano .env

# Start the server
docker-compose up -d

# View logs
docker-compose logs -f robot-server
```

#### Option B: Local Installation

```bash
cd server
pip install -r requirements.txt

# Set environment variables
export GEMINI_API_KEY="your-api-key-here"

# Run server
python server.py
```

The server will be available at `http://localhost:8000`

### 3. Raspberry Pi Setup

```bash
# On Raspberry Pi
cd hardware/raspberry_pi

# Install dependencies
sudo apt-get update
sudo apt-get install -y python3-pip python3-websockets python3-rpi.gpio
pip3 install websockets asyncio

# Update the SERVER_URL in raspberry_pi_controller.py
# Edit the file and set SERVER_URL to your server's IP
nano raspberry_pi_controller.py
# Change: SERVER_URL = "ws://YOUR_SERVER_IP:8000/ws/raspberry_pi"

# Run controller
python3 raspberry_pi_controller.py
```

**Note**: The Raspberry Pi will:
- Connect to the central server via WebSocket
- Control 4 DC motors via 2 L298N motor drivers using GPIO
- Render face animations on HDMI display
- Play audio through the audio jack
- Receive emotion and motor control updates in real-time

**GPIO Pin Configuration**:
The Raspberry Pi uses the following GPIO pins for motor control:

**Motor Driver 1 (Motors A & B - Front Wheels)**:
- GPIO17 → Motor A IN1 (Direction 1)
- GPIO27 → Motor A IN2 (Direction 2)
- GPIO22 → Motor A ENA (PWM Speed Control)
- GPIO23 → Motor B IN3 (Direction 1)
- GPIO24 → Motor B IN4 (Direction 2)
- GPIO25 → Motor B ENB (PWM Speed Control)

**Motor Driver 2 (Motors C & D - Rear Wheels)**:
- GPIO5 → Motor C IN1 (Direction 1)
- GPIO6 → Motor C IN2 (Direction 2)
- GPIO13 → Motor C ENA (PWM Speed Control)
- GPIO19 → Motor D IN3 (Direction 1)
- GPIO26 → Motor D IN4 (Direction 2)
- GPIO12 → Motor D ENB (PWM Speed Control)

**Wiring Instructions**:
1. Connect Raspberry Pi GPIO pins to L298N driver inputs as shown above
2. Connect L298N OUT1/OUT2 to Motor A, OUT3/OUT4 to Motor B (Driver 1)
3. Connect L298N OUT1/OUT2 to Motor C, OUT3/OUT4 to Motor D (Driver 2)
4. Power both L298N drivers with 7-12V supply
5. Connect all grounds together (Pi GND, both L298N GND, power supply GND)

### 4. Access the Web Interface

Open your browser and navigate to:
```
http://localhost:8000
```

You should see the animated robot face with control buttons.

### Option 2: ROS Integration Setup

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
│   ├── raspberry_pi/          # Raspberry Pi controller
│   │   ├── raspberry_pi_controller.py
│   │   ├── config.yaml        # Hardware configuration
│   │   └── requirements.txt
│   └── esp12e/                # ESP12E firmware
│       ├── motor_controller.ino
│       └── config.h           # Pin definitions
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

# Network Configuration
RASPBERRY_PI_IP=192.168.1.100
ROBOT_NAME=MyPetRobot
```

### Raspberry Pi Configuration

Edit `hardware/raspberry_pi/config.yaml` to configure GPIO pins, I2C settings, and server connection.

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

### Raspberry Pi Not Connecting

- Verify server URL in `raspberry_pi_controller.py`
- Check network connectivity: `ping <server_ip>`
- Ensure WebSocket port 8000 is accessible
- Check server logs for Raspberry Pi connection attempts
- Verify RPi.GPIO library is installed: `python3 -c "import RPi.GPIO"`

### Motors Not Moving

- Check Raspberry Pi WebSocket connection status
- Verify motor driver (L298N) connections to GPIO pins
- Check power supply voltage (7-12V) and current capacity
- Test motors directly with power supply
- Verify GPIO pin numbers match configuration
- Check server logs for motor command transmission
- Monitor Raspberry Pi logs for command receipt
- Ensure all grounds are connected properly

### WebSocket Connection Failed

- Check firewall settings on server
- Verify server is running: `curl http://<server_ip>:8000/health`
- Ensure all devices are on same network
- Use correct server IP address (not localhost from remote devices)
- Check router allows WebSocket connections

## 📚 Additional Documentation

- **[Mental Health Features](MENTAL_HEALTH_FEATURES.md)** - Comprehensive mental health support documentation
- **[Multimodal API Guide](downloads/multimodal-api-guide.md)** - Complete guide for multimodal interactions
- [Setup Guide](downloads/setup-guide.md) - Detailed setup instructions
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
