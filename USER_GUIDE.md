# AI Pet Robot - Complete User Setup Guide

**Version:** 2.2  
**Last Updated:** January 2026

---

## 📋 Table of Contents

1. [Introduction](#introduction)
2. [System Requirements](#system-requirements)
3. [Quick Start](#quick-start)
4. [Detailed Setup](#detailed-setup)
5. [Port Architecture](#port-architecture)
6. [TTS Voice Configuration](#tts-voice-configuration)
7. [Running the System](#running-the-system)
8. [API Reference](#api-reference)
9. [Troubleshooting](#troubleshooting)
10. [Advanced Configuration](#advanced-configuration)

---

## 🎯 Introduction

The AI Pet Robot is an empathetic AI-powered companion designed for mental health support and personal assistance. This guide will help you set up and configure the complete system.

### Key Features
- **Multi-Port Architecture**: 4 separate servers for different functions
- **Voice Input/Output**: Customizable TTS with voice broadcast to all ports
- **Emotion Detection**: Real-time facial emotion recognition
- **Autonomous Navigation**: YOLO-based obstacle detection and avoidance
- **Mental Health Monitoring**: Crisis detection and emotional tracking
- **Smart Home Integration**: IoT device control via MQTT/HTTP

---

## 💻 System Requirements

### Software Requirements

#### Required
- **Python 3.9+** (Python 3.12.3 recommended)
- **Git** for cloning the repository
- **pip** for Python package management

#### Optional
- **Docker** and **Docker Compose** (for containerized deployment)
- **ROS Noetic** (Ubuntu 20.04) or **ROS Melodic** (Ubuntu 18.04) - for autonomous navigation
- **Flutter SDK** - for mobile app development
- **Node.js 18+** - for frontend development

### Hardware Requirements

#### For Development/Testing
- Computer with 4GB+ RAM
- Webcam (for emotion detection testing)

#### For Full Robot Deployment
- **Raspberry Pi 4** (4GB+ RAM recommended)
- **Raspberry Pi Camera Module** or USB webcam
- **Motor Driver**: L298N H-Bridge (1-2 units)
- **DC Motors**: 4x motors for movement
- **Speaker**: 40mm 8Ω speaker (3.5mm, GPIO PWM, or I2S DAC)
- **Power Supply**: Appropriate for motors and Pi
- **Chassis**: Robot frame with wheels

---

## 🚀 Quick Start

### 1. Clone the Repository

```bash
git clone https://github.com/Eshwarpawanpeddi/AI-Pet-robot-for-mental-health-and-personal-assistance.git
cd AI-Pet-robot-for-mental-health-and-personal-assistance
```

### 2. Set Up Virtual Environment

The repository includes a pre-configured virtual environment setup:

```bash
# Activate the virtual environment
source venv/bin/activate  # Linux/MacOS
# or
venv\Scripts\activate  # Windows

# If venv doesn't exist, create it:
python3 -m venv venv
source venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
```

> **Note**: See [VENV_SETUP.md](VENV_SETUP.md) for detailed virtual environment documentation.

### 3. Configure Environment Variables

```bash
# Copy the environment template
cp ".env copy" .env

# Edit the .env file
nano .env  # or use your preferred editor
```

**Required Configuration:**
```env
# Get your API key from: https://makersuite.google.com/app/apikey
GEMINI_API_KEY=your-actual-api-key-here
```

**Optional Configuration:**
```env
# TTS Voice Settings
TTS_VOICE=en+f3        # Female voice (default)
TTS_SPEED=150          # Words per minute
TTS_PITCH=50           # Voice pitch (0-99)

# Gemini Port Control
GEMINI_ENABLED_PORT_8000=true
GEMINI_ENABLED_PORT_3000=true
```

### 4. Launch All Servers

```bash
# From the repository root
python server/launch_all.py
```

This will start all four servers:
- Port 8000: Primary control server
- Port 3000: Mobile web interface
- Port 9999: Emotion detection server
- Port 10000: Emotion display server

### 5. Access the Interfaces

- **Primary Control**: http://localhost:8000
- **Mobile Interface**: http://localhost:3000
- **Emotion Detection**: http://localhost:9999
- **Emotion Display**: http://localhost:10000

---

## 📦 Detailed Setup

### Step 1: Install System Dependencies

#### Ubuntu/Debian

```bash
# Update package list
sudo apt update

# Install Python and essential tools
sudo apt install -y python3 python3-pip python3-venv git

# Install espeak for TTS (if using Raspberry Pi)
sudo apt install -y espeak espeak-data

# Optional: Install ROS (for autonomous navigation)
# Follow instructions at: http://wiki.ros.org/noetic/Installation
```

#### macOS

```bash
# Install Homebrew (if not installed)
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

# Install Python
brew install python@3.12

# Install espeak
brew install espeak
```

#### Windows

1. Download and install Python 3.12+ from [python.org](https://www.python.org/downloads/)
2. Ensure "Add Python to PATH" is checked during installation
3. Install Git from [git-scm.com](https://git-scm.com/)

### Step 2: Install Python Dependencies

```bash
cd AI-Pet-robot-for-mental-health-and-personal-assistance

# Activate virtual environment
source venv/bin/activate

# Install all dependencies
pip install -r requirements.txt
```

**Core Dependencies Installed:**
- `fastapi==0.109.1` - Web framework
- `uvicorn==0.27.0` - ASGI server
- `websockets>=13.0.0` - WebSocket support
- `google-generativeai==0.8.3` - Gemini AI
- `pydantic>=2.9.0` - Data validation
- `aiohttp==3.9.1` - Async HTTP client
- `python-dotenv==1.0.0` - Environment management

### Step 3: Configure Gemini API

1. Visit https://makersuite.google.com/app/apikey
2. Sign in with your Google account
3. Create an API key
4. Copy the API key to your `.env` file

```env
GEMINI_API_KEY=your-key-here
```

### Step 4: Test the Installation

```bash
# Test Python imports
python -c "
import fastapi
import uvicorn
import google.generativeai
print('All imports successful!')
"

# Check server files
ls server/
# Should see: server.py, mobile_web_server.py, emotion_display_server.py, emotion_detection_server.py
```

---

## 🔌 Port Architecture

The AI Pet Robot uses a **multi-port architecture** where different services run on separate ports. This design provides modularity, scalability, and isolation of concerns.

### Port Overview

| Port | Service | Purpose | Primary Functions |
|------|---------|---------|-------------------|
| **8000** | Primary Control Server | Main control hub | Movement control, AI processing, camera management, voice input |
| **3000** | Mobile Web Interface | User interaction | Mobile-friendly controls, joystick, status display |
| **9999** | Emotion Detection | Face analysis | Real-time emotion detection from camera |
| **10000** | Emotion Display | Visual feedback | Animated robot face/emotions |

### Data Flow Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Port 8000 (Primary)                      │
│  - Voice Input Reception                                    │
│  - AI Response Generation (Gemini)                          │
│  - TTS Broadcast Orchestration                              │
│  - Camera Stream Management                                 │
│  - Movement Control                                         │
└──────────────┬──────────────────────────────────────────────┘
               │
               │ Broadcasts to all ports
               ├─────────────┬─────────────┬──────────────┐
               │             │             │              │
               ▼             ▼             ▼              ▼
         ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐
         │Port 3000 │  │Port 9999 │  │Port 10000│  │Raspberry │
         │Mobile UI │  │Emotion   │  │Display   │  │Pi        │
         │          │  │Detection │  │Server    │  │Hardware  │
         └──────────┘  └──────────┘  └──────────┘  └──────────┘
```

### Voice Input/Output Flow

**Voice Input**: Port 8000 ONLY
**Voice Output**: ALL ports receive broadcasts

1. User sends voice input to Port 8000 (`POST /api/speak`)
2. Port 8000 processes the text through Gemini AI
3. AI response is broadcast to ALL ports:
   - **Port 3000**: Displays TTS text on mobile interface
   - **Port 9999**: Receives TTS for emotion context
   - **Port 10000**: Syncs with display animations
   - **Raspberry Pi**: Plays audio through speaker

### Inter-Port Communication

#### Synchronization
- **Emotions**: Port 8000 → Ports 3000, 9999, 10000, Raspberry Pi
- **TTS Output**: Port 8000 → Ports 3000, 9999, 10000, Raspberry Pi
- **Camera Feed**: Port 8000 → Port 9999 (for emotion detection)
- **Status Updates**: Port 8000 broadcasts state to all clients

#### Endpoint Mapping

##### Port 8000 Endpoints
```
POST /api/speak           - Send TTS (broadcasts to all ports)
GET  /api/tts/voices      - List available voices
POST /api/tts/settings    - Update default voice settings
POST /api/move            - Control robot movement
GET  /api/state           - Get current system state
POST /api/emotion         - Set robot emotion
```

##### Port 3000 Endpoints
```
GET  /                    - Mobile web interface
POST /api/tts             - Receive TTS broadcast from port 8000
WS   /ws/mobile           - WebSocket for mobile clients
```

##### Port 9999 Endpoints
```
GET  /                    - Emotion detection interface
POST /api/tts             - Receive TTS broadcast from port 8000
GET  /api/emotion         - Get detected emotion
WS   /ws/emotion          - WebSocket for emotion stream
```

##### Port 10000 Endpoints
```
GET  /                    - Emotion display interface
POST /api/tts             - Receive TTS broadcast from port 8000
POST /api/emotion         - Receive emotion updates
GET  /api/emotion         - Get current display emotion
```

---

## 🎙️ TTS Voice Configuration

The AI Pet Robot uses **espeak** for text-to-speech with extensive customization options.

### Available Voices

| Voice ID | Name | Gender | Description |
|----------|------|--------|-------------|
| `en` | English Male | Male | Standard English male voice (default espeak) |
| `en+f1` | English Female 1 | Female | Light female voice |
| `en+f2` | English Female 2 | Female | Medium female voice |
| `en+f3` | **English Female 3** | Female | **Standard female voice (recommended)** |
| `en+f4` | English Female 4 | Female | Deeper female voice |
| `en+m1` | English Male 1 | Male | Light male voice |
| `en+m2` | English Male 2 | Male | Medium male voice |
| `en+m3` | English Male 3 | Male | Standard male voice |
| `en+m4` | English Male 4 | Male | Deep male voice |
| `en+m7` | English Male 7 | Male | Very deep male voice |
| `en-us` | American English | Male | American accent |
| `en-uk` | British English | Male | British accent |
| `en-scottish` | Scottish English | Male | Scottish accent |

### Voice Parameters

#### Speed (80-450 words per minute)
- **80-120**: Very slow, clear enunciation
- **120-150**: Slow, comfortable listening
- **150-175**: Normal speed (default: 150)
- **175-250**: Fast, efficient
- **250+**: Very fast

#### Pitch (0-99)
- **0-30**: Very low pitch
- **30-45**: Low pitch
- **45-55**: Normal pitch (default: 50)
- **55-70**: Higher pitch
- **70-99**: Very high pitch

### Configuration Methods

#### Method 1: Environment Variables (.env file)

```env
TTS_VOICE=en+f3    # Female voice 3
TTS_SPEED=150      # 150 words per minute
TTS_PITCH=50       # Normal pitch
```

#### Method 2: API Configuration

```bash
# Get available voices
curl http://localhost:8000/api/tts/voices

# Update default settings
curl -X POST http://localhost:8000/api/tts/settings \
  -H "Content-Type: application/json" \
  -d '{"voice": "en+m3", "speed": 160, "pitch": 45}'
```

#### Method 3: Per-Request Override

```bash
# Send TTS with custom voice
curl -X POST http://localhost:8000/api/speak \
  -H "Content-Type: application/json" \
  -d '{
    "text": "Hello, how are you today?",
    "voice": "en+f3",
    "speed": 140,
    "pitch": 55
  }'
```

### Raspberry Pi Voice Configuration

On the Raspberry Pi, edit the configuration in `hardware/raspberry_pi/raspberry_pi_controller.py`:

```python
# Audio Configuration
AUDIO_OUTPUT_MODE = "default"  # Options: "default", "gpio_pwm", "i2s"
AUDIO_VOLUME = 80  # Volume percentage (0-100)

# TTS Voice Configuration
TTS_VOICE = "en+f3"  # Change voice here
TTS_SPEED = 150      # Adjust speed
TTS_PITCH = 50       # Adjust pitch
```

### Testing Voices

Test different voices using the API:

```python
import requests

voices_to_test = ["en", "en+f3", "en+m3", "en-us"]
test_text = "Hello, this is a voice test"

for voice in voices_to_test:
    response = requests.post(
        "http://localhost:8000/api/speak",
        json={"text": test_text, "voice": voice}
    )
    print(f"Tested voice: {voice}")
    # Wait for speech to complete before testing next voice
    import time
    time.sleep(3)
```

---

## 🎮 Running the System

### Option 1: Launch All Services (Recommended)

```bash
# From repository root
python server/launch_all.py

# Or with options:
python server/launch_all.py --full          # All components including hardware
python server/launch_all.py --with-pi       # Servers + Pi simulation
python server/launch_all.py --server-only   # Just primary server (8000)
```

### Option 2: Launch Individual Servers

Useful for development or debugging specific components:

```bash
# Activate virtual environment first
source venv/bin/activate

# Port 8000 - Primary Control Server
cd server
python server.py

# Port 3000 - Mobile Web Interface (in new terminal)
cd server
python mobile_web_server.py

# Port 9999 - Emotion Detection (in new terminal)
cd server
python emotion_detection_server.py

# Port 10000 - Emotion Display (in new terminal)
cd server
python emotion_display_server.py
```

### Option 3: Docker Deployment (Coming Soon)

```bash
docker-compose up -d
```

### Raspberry Pi Setup

On your Raspberry Pi, run the hardware controller:

```bash
# SSH into Raspberry Pi
ssh pi@raspberrypi.local

# Navigate to project
cd AI-Pet-robot-for-mental-health-and-personal-assistance

# Update SERVER_URL in raspberry_pi_controller.py
nano hardware/raspberry_pi/raspberry_pi_controller.py
# Change SERVER_URL to your computer's IP address

# Run the controller
python3 hardware/raspberry_pi/raspberry_pi_controller.py
```

---

## 📚 API Reference

### TTS (Text-to-Speech) APIs

#### List Available Voices

```http
GET /api/tts/voices
```

**Response:**
```json
{
  "available_voices": [
    {
      "id": "en+f3",
      "name": "English Female 3",
      "gender": "female",
      "description": "Standard female voice (recommended)"
    }
  ],
  "current_settings": {
    "voice": "en+f3",
    "speed": 150,
    "pitch": 50
  },
  "parameter_ranges": {
    "speed": {"min": 80, "max": 450, "default": 150},
    "pitch": {"min": 0, "max": 99, "default": 50}
  }
}
```

#### Update TTS Settings

```http
POST /api/tts/settings
Content-Type: application/json

{
  "voice": "en+m3",
  "speed": 160,
  "pitch": 45
}
```

#### Send TTS

```http
POST /api/speak
Content-Type: application/json

{
  "text": "Hello, how can I help you?",
  "voice": "en+f3",    // Optional
  "speed": 150,        // Optional
  "pitch": 50          // Optional
}
```

**Response:**
```json
{
  "status": "ok",
  "text": "Hello, how can I help you?",
  "broadcast": "all_ports",
  "voice_params": {
    "voice": "en+f3",
    "speed": 150,
    "pitch": 50
  }
}
```

### Control APIs

#### Move Robot

```http
POST /api/move
Content-Type: application/json

{
  "direction": "forward",  // forward, backward, left, right, stop
  "speed": 50              // 0-100
}
```

#### Set Emotion

```http
POST /api/emotion
Content-Type: application/json

{
  "emotion": "happy"  // happy, sad, angry, neutral, etc.
}
```

#### Get System State

```http
GET /api/state
```

**Response:**
```json
{
  "emotion": "neutral",
  "is_listening": false,
  "is_speaking": false,
  "battery_level": 100,
  "control_mode": "manual",
  "camera_enabled": true,
  "raspberry_pi_connected": true
}
```

### Gemini Control APIs

#### Get Gemini Status

```http
GET /api/gemini/status
```

#### Toggle Gemini on Specific Port

```http
POST /api/gemini/port_control
Content-Type: application/json

{
  "port": 8000,
  "enabled": true
}
```

---

## 🔧 Troubleshooting

### Common Issues

#### 1. "ModuleNotFoundError: No module named 'fastapi'"

**Solution:**
```bash
# Ensure virtual environment is activated
source venv/bin/activate

# Reinstall dependencies
pip install -r requirements.txt
```

#### 2. "Port already in use"

**Solution:**
```bash
# Find process using the port
lsof -i :8000  # Replace 8000 with your port

# Kill the process
kill -9 <PID>

# Or use a different port by editing the server file
```

#### 3. "GEMINI_API_KEY not found"

**Solution:**
```bash
# Ensure .env file exists
ls -la .env

# If not, copy from template
cp ".env copy" .env

# Edit and add your API key
nano .env
```

#### 4. TTS Not Working

**Solutions:**

**On Linux/Raspberry Pi:**
```bash
# Install espeak
sudo apt install espeak espeak-data

# Test espeak
espeak "Hello, this is a test"

# Check audio output
speaker-test -t wav -c 2
```

**On macOS:**
```bash
# Install espeak
brew install espeak

# Test
espeak "Hello"
```

**On Windows:**
- espeak is not natively supported
- Consider using alternative TTS libraries or WSL

#### 5. Camera Not Detected (Port 9999)

**Solution:**
```bash
# Check connected cameras
ls /dev/video*

# Test camera with OpenCV
python -c "import cv2; cap = cv2.VideoCapture(0); print('Camera works' if cap.isOpened() else 'Camera failed')"

# Grant camera permissions if needed
sudo usermod -a -G video $USER
```

#### 6. "Connection refused" to Primary Server

**Solution:**
```bash
# Ensure primary server (8000) is running
netstat -tuln | grep 8000

# Check firewall
sudo ufw allow 8000

# Verify SERVER_URL in dependent servers
grep "localhost" server/*.py
```

### Logging and Debugging

#### Enable Debug Logging

Edit `server/server.py`:
```python
logging.basicConfig(level=logging.DEBUG)  # Changed from INFO
```

#### View Server Logs

```bash
# Real-time logs
tail -f server/logs/*.log

# Emotion detection logs
cat server/logs/emotion_detection_server.log
```

#### Test Individual Components

```python
# Test Gemini API
python -c "
import os
from dotenv import load_dotenv
import google.generativeai as genai

load_dotenv()
api_key = os.getenv('GEMINI_API_KEY')
genai.configure(api_key=api_key)
model = genai.GenerativeModel('gemini-1.5-flash')
response = model.generate_content('Hello')
print(response.text)
"
```

---

## ⚙️ Advanced Configuration

### Custom Port Configuration

Edit the port numbers in respective server files:

```python
# server/server.py
uvicorn.run(app, host="0.0.0.0", port=8000)  # Change port here

# server/mobile_web_server.py
uvicorn.run(app, host="0.0.0.0", port=3000)

# server/emotion_detection_server.py
uvicorn.run(app, host="0.0.0.0", port=9999)

# server/emotion_display_server.py
uvicorn.run(app, host="0.0.0.0", port=10000)
```

### Audio Output Configuration (Raspberry Pi)

In `hardware/raspberry_pi/raspberry_pi_controller.py`:

```python
# Audio Configuration
AUDIO_OUTPUT_MODE = "default"  # Options:
# "default" - 3.5mm audio jack or HDMI
# "gpio_pwm" - GPIO PWM audio output
# "i2s" - I2S DAC modules (better quality)

AUDIO_VOLUME = 80  # Volume percentage (0-100)
```

### Network Configuration

For remote access, update `.env`:

```env
# Replace localhost with your server IP
RASPBERRY_PI_IP=192.168.1.100

# Update URLs in server files
# server/mobile_web_server.py
primary_server_url = "http://YOUR_IP:8000"
```

### Smart Home Integration

Enable and configure smart home features:

```env
SMART_HOME_ENABLED=true
MQTT_BROKER_URL=mqtt://localhost:1883
MQTT_USERNAME=your_username
MQTT_PASSWORD=your_password
```

### Performance Tuning

#### Adjust Camera FPS

```python
# server/camera_stream_manager.py
FPS = 10  # Adjust based on your needs (5-30)
```

#### Emotion Detection Frequency

```python
# server/emotion_detection_server.py
DETECTION_INTERVAL = 1.0  # Seconds between detections
```

---

## 📞 Support

- **GitHub Issues**: https://github.com/Eshwarpawanpeddi/AI-Pet-robot-for-mental-health-and-personal-assistance/issues
- **Documentation**: Check README.md and other guides in the repository
- **Community**: Discussions and questions on GitHub Discussions

---

## 📄 License

This project is open source. See LICENSE file for details.

---

## 🙏 Acknowledgments

- Google Gemini AI for conversational intelligence
- FastAPI for the web framework
- OpenCV for computer vision
- espeak for text-to-speech
- ROS community for autonomous navigation support

---

**Last Updated**: January 2026  
**Version**: 2.2
