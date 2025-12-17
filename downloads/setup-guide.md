# Complete Setup & Deployment Guide
# git config --global user.email "you@example.com"
# git config --global user.name "Your Name"

## Directory Structure

```
pet-robot-os/
├── hardware/
│   ├── esp12e/
│   │   ├── motor_controller.ino
│   │   └── config.h
│   └── raspberry_pi/
│       ├── raspberry_pi_controller.py
│       ├── requirements.txt
│       └── config.yaml
├── server/
│   ├── server.py
│   ├── gemini_integration.py
│   ├── requirements.txt
│   └── Dockerfile
├── frontend/
│   ├── face_display.html
│   ├── mobile_app/ (React Native / Flutter)
│   └── styles.css
├── docker-compose.yml
└── README.md
```

## Step 1: Raspberry Pi Setup

### 1.1 Install OS & Dependencies

```bash
# Update system
sudo apt-get update && sudo apt-get upgrade -y

# Install Python and essential packages
sudo apt-get install -y \
    python3-pip \
    python3-smbus \
    python3-rpi.gpio \
    i2c-tools \
    build-essential \
    git \
    libopenjp2-7 \
    libtiff5 \
    libjasper-dev \
    libharfbuzz0b \
    libwebp6

# Install Python packages
pip3 install --upgrade pip
pip3 install \
    smbus-cffi \
    RPi.GPIO \
    PyYAML \
    aiosmbus \
    Adafruit-ADS1x15
```

### 1.2 Enable I2C & UART

```bash
# Enable I2C
sudo raspi-config nonint do_i2c 0

# Enable UART (if using serial communication)
sudo raspi-config nonint do_serial_hw 0

# Verify I2C
i2cdetect -y 1
```

### 1.3 Create Raspberry Pi Service

```bash
# Create systemd service for auto-start
sudo nano /etc/systemd/system/robot-controller.service
```

Add the following:

```ini
[Unit]
Description=Pet Robot Raspberry Pi Controller
After=network.target

[Service]
ExecStart=/usr/bin/python3 /home/pi/pet-robot-os/hardware/raspberry_pi/raspberry_pi_controller.py
WorkingDirectory=/home/pi/pet-robot-os/hardware/raspberry_pi/
StandardOutput=inherit
StandardError=inherit
Restart=always
User=pi

[Install]
WantedBy=multi-user.target
```

```bash
# Enable service
sudo systemctl enable robot-controller
sudo systemctl start robot-controller
```

## Step 2: ESP12E Arduino Setup

### 2.1 Install Arduino IDE

```bash
# Download Arduino IDE for Raspberry Pi / your development machine
wget https://downloads.arduino.cc/arduino-1.8.19-LinuxARM.tar.xz
tar xf arduino-1.8.19-LinuxARM.tar.xz
cd arduino-1.8.19
sudo ./install.sh
```

### 2.2 Install ESP8266 Board Support

1. Open Arduino IDE
2. Go to **File → Preferences**
3. Add to "Additional Boards Manager URLs":
   ```
   http://arduino.esp8266.com/stable/package_esp8266com_index.json
   ```
4. Go to **Tools → Board → Boards Manager**
5. Search and install **esp8266**
6. Select **Tools → Board → NodeMCU 1.0 (ESP-12E Module)**

### 2.3 Upload Firmware

```bash
# Copy the motor_controller.ino to Arduino sketches folder
cp motor_controller.ino ~/Arduino/sketches/

# Connect ESP12E via USB
# In Arduino IDE:
# - Select correct board and port
# - Click Upload
# - Monitor Serial output at 115200 baud
```

## Step 3: Laptop Server Setup

### 3.1 Install Docker

```bash
# Ubuntu/Debian
sudo apt-get install docker.io docker-compose

# Add user to docker group
sudo usermod -aG docker $USER
```

### 3.2 Build Docker Image

```bash
cd pet-robot-os/server
docker build -t robot-server:latest .
```

### 3.3 Create docker-compose.yml

```yaml
version: '3.8'

services:
  robot-server:
    build:
      context: ./server
      dockerfile: Dockerfile
    ports:
      - "8000:8000"
      - "8001:8001"
    environment:
      - GEMINI_API_KEY=${GEMINI_API_KEY}
      - REDIS_URL=redis://redis:6379
      - LOG_LEVEL=INFO
    volumes:
      - ./server:/app
    depends_on:
      - redis
    networks:
      - robot-network

  redis:
    image: redis:7-alpine
    ports:
      - "6379:6379"
    volumes:
      - redis_data:/data
    networks:
      - robot-network

  nginx:
    image: nginx:alpine
    ports:
      - "80:80"
      - "443:443"
    volumes:
      - ./nginx.conf:/etc/nginx/nginx.conf
    depends_on:
      - robot-server
    networks:
      - robot-network

volumes:
  redis_data:

networks:
  robot-network:
    driver: bridge
```

### 3.4 Run Server

```bash
# Set Gemini API Key
export GEMINI_API_KEY="your-api-key-here"

# Start services
docker-compose up -d

# View logs
docker-compose logs -f robot-server
```

## Step 4: Phone App Setup

### 4.1 For React Native

```bash
# Install Node.js and npm
curl https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.0/install.sh | bash
nvm install 18

# Create React Native app
npx create-expo-app PetRobotApp
cd PetRobotApp

# Install WebSocket library
npm install ws react-native-audio-toolkit

# Start development server
npm start
```

### 4.2 For Flutter

```bash
# Install Flutter
git clone https://github.com/flutter/flutter.git
export PATH="$PATH:`pwd`/flutter/bin"
flutter doctor

# Create Flutter app
flutter create pet_robot_app
cd pet_robot_app

# Add dependencies to pubspec.yaml
dependencies:
  web_socket_channel: ^2.4.0
  record: ^5.0.0
  just_audio: ^0.9.30

# Run app
flutter run
```

## Step 5: Network Configuration

### 5.1 Static IP Assignment

```bash
# On Raspberry Pi
sudo nano /etc/dhcpcd.conf

# Add at end:
interface wlan0
static ip_address=192.168.1.100/24
static routers=192.168.1.1
static domain_name_servers=8.8.8.8 8.8.4.4
```

### 5.2 Network Bridge

```bash
# Connect all devices on same network:
# Phone: WiFi or hotspot
# Raspberry Pi: WiFi or Ethernet
# Laptop: WiFi or Ethernet
# ESP12E: WiFi (optional - can use I2C only)

# Test connectivity from phone:
# adb shell
# ping 192.168.1.100
```

## Step 6: Verification & Testing

### 6.1 Test I2C Communication

```bash
# From Raspberry Pi
i2cdetect -y 1

# Should show ESP12E at address 0x08
```

### 6.2 Test Motor Control

```bash
# From Raspberry Pi
python3 -c "
from hardware.raspberry_pi.raspberry_pi_controller import RaspberryPiController
controller = RaspberryPiController()
controller.move_forward(200)
import time
time.sleep(2)
controller.stop()
"
```

### 6.3 Test Server

```bash
# Check server health
curl http://localhost:8000/health

# Should return:
# {
#   "status": "healthy",
#   "timestamp": "2024-XX-XX...",
#   "robot_state": {...}
# }
```

### 6.4 Test WebSocket

```bash
# Use websocat
cargo install websocat
websocat ws://localhost:8000/ws/control

# Send test message
{"type": "move", "direction": "forward", "speed": 200}
```

## Step 7: Production Deployment

### 7.1 Environment Variables

```bash
# Create .env file
cat > .env << EOF
GEMINI_API_KEY=your-key-here
RASPBERRY_PI_IP=192.168.1.100
ROBOT_NAME=MyPetRobot
LOG_LEVEL=INFO
DATABASE_URL=mongodb://mongo:27017/robot_db
REDIS_URL=redis://redis:6379
EOF
```

### 7.2 SSL/TLS Setup

```bash
# Install Certbot
sudo apt-get install certbot python3-certbot-nginx

# Generate certificate
sudo certbot certonly --standalone -d your-domain.com

# Update nginx config with SSL
```

### 7.3 Systemd Services

```bash
# Robot Controller (Raspberry Pi)
sudo systemctl enable robot-controller
sudo systemctl start robot-controller

# Check status
sudo systemctl status robot-controller
```

### 7.4 Monitoring & Logging

```bash
# Install logging stack (optional)
docker-compose up -d logstash elasticsearch kibana

# View all logs
docker-compose logs -f --tail=100
```

## Step 8: Troubleshooting

### Issue: ESP12E not responding

```bash
# Check I2C connection
i2cdetect -y 1

# Check power and I2C pull-ups
# ESP12E needs 4.7kΩ pull-ups on SDA/SCL

# Check Arduino upload
# Open Serial Monitor at 115200 baud
```

### Issue: Motors not moving

```bash
# Check motor driver connections
# Verify PWM pins are configured
# Test motor driver with known good motor

# Debug from Raspberry Pi:
# GPIO.setmode(GPIO.BCM)
# GPIO.setup(17, GPIO.OUT)
# GPIO.output(17, GPIO.HIGH)  # Should enable motor
```

### Issue: WebSocket connection failed

```bash
# Check server is running
docker-compose ps

# Check firewall
sudo ufw allow 8000/tcp

# Test connectivity
netstat -an | grep 8000
```

### Issue: Gemini API error

```bash
# Verify API key
echo $GEMINI_API_KEY

# Test API directly
python3 -c "
import google.generativeai as genai
genai.configure(api_key='$GEMINI_API_KEY')
print(genai.list_models())
"
```

## Performance Optimization

1. **Reduce latency**: Use MQTT instead of HTTP for faster messaging
2. **Optimize face rendering**: Use WebGL instead of Canvas 2D
3. **Audio compression**: Use Opus codec instead of PCM for voice
4. **Edge processing**: Run lightweight models on Raspberry Pi

## Security Considerations

1. Enable SSH key-based authentication
2. Use VPN for remote access
3. Implement API rate limiting
4. Add authentication to WebSocket
5. Use encrypted connections (HTTPS/WSS)
6. Keep API keys in environment variables

## Additional Resources

- [ESP8266 Documentation](https://docs.espressif.com/projects/esp8266-rtos-sdk/en/latest/)
- [Raspberry Pi GPIO Documentation](https://www.raspberrypi.org/documentation/computers/os.html)
- [Gemini Live API Guide](https://ai.google.dev/gemini-api/docs/live-guide)
- [FastAPI Documentation](https://fastapi.tiangolo.com/)
