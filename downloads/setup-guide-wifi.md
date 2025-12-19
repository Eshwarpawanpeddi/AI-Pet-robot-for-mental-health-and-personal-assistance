# Complete Setup Guide - Wi-Fi Based Architecture

## Overview

This guide walks you through setting up the AI Pet Robot with the new Wi-Fi-based architecture where all components communicate through a central server.

## Architecture Summary

- **Central Server**: Coordinates all components, processes AI, routes commands
- **ESP12E**: Motor controller, connects via Wi-Fi WebSocket
- **Raspberry Pi**: Face display and audio, connects via Wi-Fi WebSocket
- **Mobile App**: User interface, connects via Wi-Fi WebSocket

## Prerequisites

### Hardware
- Central Server (Laptop/Desktop or Cloud)
- Raspberry Pi 4 (4GB+ RAM recommended)
- ESP12E (NodeMCU)
- L298N Motor Driver
- 2x DC Motors
- HDMI Display for Raspberry Pi
- Speaker/Audio output
- Wi-Fi Router (all devices on same network)
- Power supplies

### Software
- Python 3.9+
- Arduino IDE 1.8+ or 2.x
- Flutter SDK (for mobile app)
- Git

## Network Configuration

### Step 1: Prepare Network

1. **Wi-Fi Network**: All devices must be on the same Wi-Fi network
2. **Server Static IP**: Recommended to assign static IP to server
   - Option A: Configure in router (DHCP reservation)
   - Option B: Set static IP on server

**Example Network Setup:**
- Router: 192.168.1.1
- Server: 192.168.1.100 (static)
- ESP12E: 192.168.1.101 (DHCP)
- Raspberry Pi: 192.168.1.102 (DHCP)
- Mobile: 192.168.1.xxx (DHCP)

## Server Setup

### Step 1: Clone Repository

```bash
git clone https://github.com/Eshwarpawanpeddi/AI-Pet-robot-for-mental-health-and-personal-assistance.git
cd AI-Pet-robot-for-mental-health-and-personal-assistance
```

### Step 2: Configure Environment

```bash
# Copy environment template
cp .env.example .env

# Edit and add your Gemini API key
nano .env
```

Set in `.env`:
```env
GEMINI_API_KEY=your-api-key-here
LOG_LEVEL=INFO
```

### Step 3: Install Dependencies

**Option A: Using Docker (Recommended)**

```bash
docker-compose up -d
docker-compose logs -f robot-server
```

**Option B: Local Installation**

```bash
cd server
pip install -r requirements.txt

# Run server
python server.py
```

### Step 4: Verify Server

```bash
# Check health endpoint
curl http://localhost:8000/health

# Expected response:
# {
#   "status": "healthy",
#   "gemini_initialized": true,
#   "robot_state": {...}
# }
```

**Note Server IP**: You'll need this IP for ESP12E and Raspberry Pi configuration.

## ESP12E Setup (Motor Controller)

### Step 1: Install Arduino IDE

1. Download Arduino IDE from https://www.arduino.cc/en/software
2. Install and launch Arduino IDE

### Step 2: Configure Arduino for ESP12E

1. Open **File → Preferences**
2. Add to "Additional Boards Manager URLs":
   ```
   http://arduino.esp8266.com/stable/package_esp8266com_index.json
   ```
3. Click OK
4. Open **Tools → Board → Boards Manager**
5. Search for "ESP8266"
6. Install "esp8266 by ESP8266 Community"

### Step 3: Install Required Libraries

1. Open **Sketch → Include Library → Manage Libraries**
2. Search and install:
   - **WebSocketsClient** by Markus Sattler
   - **ArduinoJson** by Benoit Blanchon (version 6.x)

### Step 4: Configure ESP12E Code

1. Open `hardware/esp12e/motor_controller.ino`
2. Open `hardware/esp12e/config.h`
3. Edit configuration:

```cpp
// Wi-Fi Configuration
#define WIFI_SSID "YourWiFiName"
#define WIFI_PASSWORD "YourWiFiPassword"
#define SERVER_HOST "192.168.1.100"  // Your server IP
#define SERVER_PORT 8000
```

### Step 5: Upload to ESP12E

1. Connect ESP12E to computer via USB
2. Select **Tools → Board → NodeMCU 1.0 (ESP-12E Module)**
3. Select **Tools → Port → [Your ESP12E Port]**
4. Click **Upload**
5. Open **Tools → Serial Monitor** (115200 baud)
6. Verify connection:
   ```
   ESP12E Motor Controller Initialized
   Connecting to WiFi...
   WiFi connected
   IP address: 192.168.1.101
   WebSocket Connected
   ```

### Step 6: Hardware Wiring

**ESP12E to L298N Motor Driver:**
```
ESP12E Pin  →  L298N Pin
D0 (GPIO16) →  IN1
D1 (GPIO5)  →  IN2
D2 (GPIO4)  →  IN3
D3 (GPIO0)  →  IN4
D5 (GPIO14) →  ENA (PWM Motor A)
D6 (GPIO12) →  ENB (PWM Motor B)
GND         →  GND
```

**L298N to Motors:**
```
OUT1, OUT2  →  Motor A
OUT3, OUT4  →  Motor B
12V Input   →  External Power Supply
```

## Raspberry Pi Setup (Face Display & Audio)

### Step 1: Install Raspberry Pi OS

1. Download Raspberry Pi OS (Desktop version recommended)
2. Flash to SD card using Raspberry Pi Imager
3. Boot Raspberry Pi and complete initial setup
4. Connect HDMI display and speakers/headphones

### Step 2: Install Dependencies

```bash
# Update system
sudo apt-get update && sudo apt-get upgrade -y

# Install Python and required packages
sudo apt-get install -y \
    python3-pip \
    git \
    python3-websockets

# Install Python websockets library
pip3 install websockets asyncio
```

### Step 3: Clone Repository on Pi

```bash
git clone https://github.com/Eshwarpawanpeddi/AI-Pet-robot-for-mental-health-and-personal-assistance.git
cd AI-Pet-robot-for-mental-health-and-personal-assistance/hardware/raspberry_pi
```

### Step 4: Configure Raspberry Pi Controller

Edit `raspberry_pi_controller.py`:

```python
# Server Configuration
SERVER_URL = "ws://192.168.1.100:8000/ws/raspberry_pi"  # Use your server IP
RECONNECT_DELAY = 5
```

### Step 5: Test Connection

```bash
python3 raspberry_pi_controller.py
```

Expected output:
```
INFO - Raspberry Pi Controller Initialized
INFO - Connected to server
```

### Step 6: Create Auto-Start Service

```bash
# Create service file
sudo nano /etc/systemd/system/robot-display.service
```

Add:
```ini
[Unit]
Description=AI Pet Robot Display Controller
After=network.target

[Service]
ExecStart=/usr/bin/python3 /home/pi/AI-Pet-robot-for-mental-health-and-personal-assistance/hardware/raspberry_pi/raspberry_pi_controller.py
WorkingDirectory=/home/pi/AI-Pet-robot-for-mental-health-and-personal-assistance/hardware/raspberry_pi
StandardOutput=inherit
StandardError=inherit
Restart=always
User=pi

[Install]
WantedBy=multi-user.target
```

Enable service:
```bash
sudo systemctl enable robot-display.service
sudo systemctl start robot-display.service
sudo systemctl status robot-display.service
```

## Mobile App Setup (Optional)

### Step 1: Install Flutter

Follow instructions at: https://flutter.dev/docs/get-started/install

### Step 2: Setup Mobile App

```bash
cd mobile_app
flutter pub get
```

### Step 3: Configure Server URL

The app allows setting server URL in the settings screen, or you can pre-configure:

Edit `lib/config/app_config.dart`:
```dart
static const String defaultServerUrl = '192.168.1.100:8000';
```

### Step 4: Run on Device

```bash
# Connect Android device via USB or start iOS simulator
flutter run

# Or build APK
flutter build apk
# APK will be in: build/app/outputs/flutter-apk/app-release.apk
```

## Testing the Complete System

### Test 1: Server Health

```bash
curl http://192.168.1.100:8000/health
```

Should return robot state with connections.

### Test 2: Check Connections

```bash
curl http://192.168.1.100:8000/api/state
```

Verify:
```json
{
  "esp_connected": true,
  "raspberry_pi_connected": true,
  "control_mode": "manual"
}
```

### Test 3: Motor Control

From web browser, open: `http://192.168.1.100:8000`

Or use API:
```bash
curl -X POST http://192.168.1.100:8000/api/command \
  -H "Content-Type: application/json" \
  -d '{
    "type": "move",
    "direction": "forward",
    "speed": 200
  }'
```

Motors should move forward.

### Test 4: Emotion Control

```bash
curl -X POST http://192.168.1.100:8000/api/command \
  -H "Content-Type: application/json" \
  -d '{
    "type": "emotion",
    "emotion": "happy"
  }'
```

Raspberry Pi display should update emotion.

### Test 5: Control Mode Toggle

```bash
# Set to autonomous mode
curl -X POST http://192.168.1.100:8000/api/mode \
  -H "Content-Type: application/json" \
  -d '{"mode": "autonomous"}'

# Get current mode
curl http://192.168.1.100:8000/api/mode
```

### Test 6: Mobile App

1. Open mobile app
2. Go to Settings
3. Enter server IP: `192.168.1.100:8000`
4. Connect
5. Verify connection status shows green
6. Test control buttons

## Troubleshooting

### ESP12E Won't Connect

1. Check serial monitor for error messages
2. Verify Wi-Fi credentials in `config.h`
3. Ping server from another device on network
4. Check firewall on server allows port 8000

**Debug Steps:**
```bash
# On server, check if port is listening
netstat -an | grep 8000

# Check server logs
docker-compose logs -f robot-server
# or if running locally
tail -f server.log
```

### Raspberry Pi Connection Failed

1. Verify server URL is correct
2. Test network connectivity:
   ```bash
   ping 192.168.1.100
   curl http://192.168.1.100:8000/health
   ```
3. Check Python websockets installed:
   ```bash
   python3 -c "import websockets; print('OK')"
   ```

### No Motor Movement

1. Check ESP12E serial output for command receipt
2. Verify L298N connections
3. Check motor power supply (adequate voltage/current)
4. Test motors directly with power supply

### Server Gemini API Errors

1. Verify API key is correct in `.env`
2. Check API quota: https://makersuite.google.com/
3. Test API key:
   ```bash
   curl -H "Authorization: Bearer YOUR_API_KEY" \
     https://generativelanguage.googleapis.com/v1/models
   ```

## Advanced Configuration

### Enable HTTPS/WSS

For production, use HTTPS and secure WebSocket:

1. Obtain SSL certificate (Let's Encrypt)
2. Configure nginx as reverse proxy
3. Update all endpoints to use `wss://` and `https://`

### Firewall Configuration

```bash
# Allow port 8000 (adjust for your firewall)
sudo ufw allow 8000/tcp
sudo ufw enable
```

### Network Isolation

Consider separate IoT network for robot components:
- Create VLAN for robot devices
- Configure firewall rules for isolation
- Allow only necessary traffic between networks

## Maintenance

### Update Components

**Server:**
```bash
git pull
docker-compose down
docker-compose up -d --build
```

**ESP12E:**
- Re-upload firmware via Arduino IDE

**Raspberry Pi:**
```bash
git pull
sudo systemctl restart robot-display
```

### Backup Configuration

```bash
# Backup server .env
cp .env .env.backup

# Backup ESP config
cp hardware/esp12e/config.h hardware/esp12e/config.h.backup
```

### Monitor System

```bash
# Server logs
docker-compose logs -f

# Raspberry Pi logs
sudo journalctl -u robot-display -f

# ESP12E logs
# Connect serial monitor to ESP12E
```

## Next Steps

- [ ] Configure autonomous mode with ROS integration
- [ ] Add camera for vision features
- [ ] Implement voice recognition
- [ ] Add more sensors (ultrasonic, IMU)
- [ ] Create custom face animations
- [ ] Develop advanced mental health features

## Support

For issues or questions:
- GitHub Issues: https://github.com/Eshwarpawanpeddi/AI-Pet-robot-for-mental-health-and-personal-assistance/issues
- Check documentation in `/downloads` folder
- Review architecture: `downloads/robot-architecture.md`
