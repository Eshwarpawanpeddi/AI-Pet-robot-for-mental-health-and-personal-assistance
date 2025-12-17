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

### Technical Features
- **Voice Interaction**: Natural conversation via Gemini AI
- **Animated Face Display**: Expressive, comforting presence
- **Autonomous Movement**: Physical robot capabilities
- **WebSocket Communication**: Real-time responsiveness
- **Privacy-First**: All data stays on your device

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                     PHONE/WEB CLIENT                             │
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
│     LAPTOP SERVER (Processing Core)           │
│  ┌────────────────────────────────────────┐  │
│  │ Gemini AI Integration                   │  │
│  │ - Voice Processing                      │  │
│  │ - LLM Response Generation               │  │
│  │ - Emotion Detection & Synthesis         │  │
│  └────────────────────────────────────────┘  │
│  ┌────────────────────────────────────────┐  │
│  │ Command Orchestration & Routing         │  │
│  │ - Parse voice commands                  │  │
│  │ - Generate motor commands               │  │
│  │ - Manage face animations                │  │
│  └────────────────────────────────────────┘  │
└──────────────────┬───────────────────────────┘
                   │
         I2C/MQTT  │  WebSocket
                   │
         ┌─────────▼─────────────────┐
         │                           │
         ▼                           ▼
┌─────────────────────────────────────┐
│   RASPBERRY PI 4 (Central Hub)      │
│  ┌────────────────────────────────┐ │
│  │ GPIO/I2C/SPI Management         │ │
│  │ - Sensor data collection        │ │
│  │ - Hardware state management     │ │
│  │ - Command parsing               │ │
│  └────────────────────────────────┘ │
└──┬───────────┬─────────────────────┘
   │           │
I2C│           │GPIO/PWM
   │           │
┌──▼──┐    ┌───▼────────────┐
│     │    │                │
▼     ▼    ▼                ▼
┌─────────────┐ ┌────────┐  ┌─────────┐
│   ESP12E    │ │Display │  │ Speaker │
│(Motor Ctrl) │ │ (LCD)  │  │(Audio)  │
└──────┬──────┘ └────────┘  └─────────┘
       │
   PWM │
       │
┌──────▼──────────────┐
│  L298N Motor Driver │
└─────────┬───────────┘
          │
    ┌─────┴─────┬─────────┐
    │           │         │
    ▼           ▼         ▼
  Motor A    Motor B   Servo(s)
```

## 📋 Prerequisites

### Hardware Requirements
- Raspberry Pi 4 (4GB+ recommended)
- ESP12E (NodeMCU) microcontroller
- L298N Motor Driver
- DC Motors (2x)
- Touch sensors
- Ultrasonic distance sensor
- Power supply (5V for Pi, appropriate voltage for motors)
- Laptop/Desktop for server

### Software Requirements
- Python 3.9+
- Docker and Docker Compose (for server)
- Arduino IDE (for ESP12E programming)
- Node.js 18+ (optional, for mobile app)

## 🚀 Quick Start

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
sudo apt-get install -y python3-pip python3-smbus python3-rpi.gpio i2c-tools
pip3 install -r requirements.txt

# Enable I2C
sudo raspi-config nonint do_i2c 0

# Update config.yaml with your server IP
nano config.yaml

# Run controller
python3 raspberry_pi_controller.py
```

### 4. ESP12E Setup

1. Open Arduino IDE
2. Install ESP8266 board support:
   - Go to **File → Preferences**
   - Add to "Additional Boards Manager URLs": 
     ```
     http://arduino.esp8266.com/stable/package_esp8266com_index.json
     ```
3. Install ESP8266 from **Tools → Board → Boards Manager**
4. Open `hardware/esp12e/motor_controller.ino`
5. Select **Tools → Board → NodeMCU 1.0 (ESP-12E Module)**
6. Upload the sketch to ESP12E

### 5. Access the Web Interface

Open your browser and navigate to:
```
http://localhost:8000
```

You should see the animated robot face with control buttons.

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

### ESP12E Not Responding

- Verify I2C connection: `i2cdetect -y 1`
- Check I2C pull-up resistors (4.7kΩ on SDA/SCL)
- Ensure ESP12E is powered correctly

### Motors Not Moving

- Check motor driver connections
- Verify GPIO pins are correctly configured
- Test motor driver with known good motors
- Check power supply voltage and current

### WebSocket Connection Failed

- Check firewall settings
- Verify server is running
- Check network connectivity
- Use correct server IP address

## 📚 Additional Documentation

- **[Mental Health Features](MENTAL_HEALTH_FEATURES.md)** - Comprehensive mental health support documentation
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
