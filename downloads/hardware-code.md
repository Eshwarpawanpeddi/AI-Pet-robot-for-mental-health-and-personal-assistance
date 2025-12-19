# Hardware Code Documentation - Wi-Fi Architecture

## Overview

This document describes the hardware code for the AI Pet Robot using the new Wi-Fi-based architecture. All hardware components communicate with the central server via WebSocket connections.

## ESP12E Motor Controller (Wi-Fi Version)

### Features

- Wi-Fi WebSocket connection to central server
- PWM motor control with L298N driver
- Automatic fallback mode on connection loss
- Real-time status reporting via heartbeat
- Sensor data transmission
- Command acknowledgment

### Hardware Configuration

#### Pin Mapping

```cpp
// Motor Control Pins (L298N Motor Driver)
#define MOTOR_A_PIN1 D0  // GPIO16 - Motor A Direction 1
#define MOTOR_A_PIN2 D1  // GPIO5  - Motor A Direction 2
#define MOTOR_B_PIN1 D2  // GPIO4  - Motor B Direction 1
#define MOTOR_B_PIN2 D3  // GPIO0  - Motor B Direction 2

// PWM Control Pins (Speed Control)
#define MOTOR_A_PWM D5   // GPIO14 - Motor A Speed (PWM)
#define MOTOR_B_PWM D6   // GPIO12 - Motor B Speed (PWM)

// Optional Sensor Pins
#define TOUCH_SENSOR_PIN D7   // GPIO13
```

#### Wiring Diagram

```
ESP12E          L298N Motor Driver
------          ------------------
D0 (GPIO16) --> IN1
D1 (GPIO5)  --> IN2
D2 (GPIO4)  --> IN3
D3 (GPIO0)  --> IN4
D5 (GPIO14) --> ENA (Enable A / PWM)
D6 (GPIO12) --> ENB (Enable B / PWM)
GND         --> GND

L298N           Motors & Power
-----           --------------
OUT1, OUT2  --> DC Motor A
OUT3, OUT4  --> DC Motor B
12V         --> External Power Supply (7-12V)
GND         --> Power Supply GND & ESP GND
```

### Configuration (config.h)

```cpp
// Wi-Fi Configuration
#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"
#define SERVER_HOST "192.168.1.100"  // Server IP address
#define SERVER_PORT 8000
#define WEBSOCKET_PATH "/ws/esp"

// Network Configuration
#define RECONNECT_DELAY_MS 5000
#define HEARTBEAT_INTERVAL_MS 10000
#define COMMAND_TIMEOUT_MS 5000

// Motor Configuration
#define DEFAULT_SPEED 200
```

### Required Libraries

Install via Arduino Library Manager:
- **WebSocketsClient** by Markus Sattler
- **ArduinoJson** by Benoit Blanchon (version 6.x)

### Key Functions

#### Wi-Fi Connection

```cpp
void connectWiFi() {
  Serial.println("Connecting to WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  }
}
```

#### WebSocket Event Handler

```cpp
void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      isConnected = false;
      fallbackMode = true;
      stopAllMotors();
      break;
      
    case WStype_CONNECTED:
      isConnected = true;
      fallbackMode = false;
      webSocket.sendTXT("{\"type\":\"esp_connected\"}");
      break;
      
    case WStype_TEXT:
      handleCommand(payload, length);
      break;
  }
}
```

#### Command Handler

```cpp
void handleCommand(uint8_t * payload, size_t length) {
  StaticJsonDocument<200> doc;
  deserializeJson(doc, payload, length);
  
  const char* type = doc["type"];
  
  if (strcmp(type, "move") == 0) {
    const char* direction = doc["direction"];
    int speed = doc["speed"] | DEFAULT_SPEED;
    
    if (strcmp(direction, "forward") == 0) {
      moveForward(speed);
    } else if (strcmp(direction, "backward") == 0) {
      moveBackward(speed);
    } else if (strcmp(direction, "left") == 0) {
      turnLeft(speed);
    } else if (strcmp(direction, "right") == 0) {
      turnRight(speed);
    } else if (strcmp(direction, "stop") == 0) {
      stopAllMotors();
    }
  }
}
```

#### Motor Control

```cpp
void moveForward(int speed) {
  digitalWrite(MOTOR_A_PIN1, HIGH);
  digitalWrite(MOTOR_A_PIN2, LOW);
  digitalWrite(MOTOR_B_PIN1, HIGH);
  digitalWrite(MOTOR_B_PIN2, LOW);
  
  analogWrite(MOTOR_A_PWM, speed);
  analogWrite(MOTOR_B_PWM, speed);
}

void moveBackward(int speed) {
  digitalWrite(MOTOR_A_PIN1, LOW);
  digitalWrite(MOTOR_A_PIN2, HIGH);
  digitalWrite(MOTOR_B_PIN1, LOW);
  digitalWrite(MOTOR_B_PIN2, HIGH);
  
  analogWrite(MOTOR_A_PWM, speed);
  analogWrite(MOTOR_B_PWM, speed);
}

void turnLeft(int speed) {
  digitalWrite(MOTOR_A_PIN1, LOW);
  digitalWrite(MOTOR_A_PIN2, HIGH);
  digitalWrite(MOTOR_B_PIN1, HIGH);
  digitalWrite(MOTOR_B_PIN2, LOW);
  
  analogWrite(MOTOR_A_PWM, speed);
  analogWrite(MOTOR_B_PWM, speed);
}

void turnRight(int speed) {
  digitalWrite(MOTOR_A_PIN1, HIGH);
  digitalWrite(MOTOR_A_PIN2, LOW);
  digitalWrite(MOTOR_B_PIN1, LOW);
  digitalWrite(MOTOR_B_PIN2, HIGH);
  
  analogWrite(MOTOR_A_PWM, speed);
  analogWrite(MOTOR_B_PWM, speed);
}

void stopAllMotors() {
  digitalWrite(MOTOR_A_PIN1, LOW);
  digitalWrite(MOTOR_A_PIN2, LOW);
  digitalWrite(MOTOR_B_PIN1, LOW);
  digitalWrite(MOTOR_B_PIN2, LOW);
  
  analogWrite(MOTOR_A_PWM, 0);
  analogWrite(MOTOR_B_PWM, 0);
}
```

### Communication Protocol

#### Messages from Server to ESP12E

**Movement Command:**
```json
{
  "type": "move",
  "direction": "forward",
  "speed": 200
}
```

Directions: `forward`, `backward`, `left`, `right`, `stop`

#### Messages from ESP12E to Server

**Heartbeat:**
```json
{
  "type": "heartbeat",
  "device": "ESP12E",
  "status": "normal",
  "uptime": 12345
}
```

**Sensor Data:**
```json
{
  "type": "sensor",
  "sensor": "touch",
  "value": true
}
```

**Status:**
```json
{
  "type": "status",
  "device": "ESP12E",
  "connected": true
}
```

### Fallback Mode

When connection is lost or no commands received for 10+ seconds:
- Automatically stops all motors
- Sets `fallbackMode = true`
- Continues attempting reconnection
- Resumes normal operation when connection restored

---

## Raspberry Pi Face Display & Audio Controller

### Features

- WebSocket connection to central server
- Real-time face display rendering on HDMI
- Audio output via audio jack
- Emotion synchronization
- Status reporting

### Dependencies

```bash
# Install required packages
sudo apt-get install -y python3-pip python3-websockets
pip3 install websockets asyncio
```

### Configuration

```python
# Server Configuration
SERVER_URL = "ws://192.168.1.100:8000/ws/raspberry_pi"
RECONNECT_DELAY = 5

# GPIO Pin Definitions (for audio output)
GPIO_I2S_BCK = 18
GPIO_I2S_LRCK = 13
GPIO_I2S_DATA = 12
```

### Key Functions

#### WebSocket Connection

```python
async def connect_to_server(self):
    """Connect to central server via WebSocket"""
    while True:
        try:
            async with websockets.connect(SERVER_URL) as websocket:
                self.websocket = websocket
                logger.info("Connected to server")
                
                # Send initial connection message
                await websocket.send(json.dumps({
                    "type": "raspberry_pi_connected",
                    "device": "RaspberryPi"
                }))
                
                # Listen for commands
                async for message in websocket:
                    await self.handle_server_message(message)
                    
        except Exception as e:
            logger.error(f"WebSocket connection error: {e}")
            await asyncio.sleep(RECONNECT_DELAY)
```

#### Message Handler

```python
async def handle_server_message(self, message: str):
    """Handle incoming messages from server"""
    data = json.loads(message)
    msg_type = data.get("type")
    
    if msg_type == "emotion":
        emotion = data.get("emotion", "neutral")
        await self.update_emotion(emotion)
        
    elif msg_type == "face_animation":
        animation = data.get("animation")
        await self.display_face_animation(animation)
        
    elif msg_type == "play_audio":
        audio_data = data.get("audio")
        await self.play_audio(audio_data)
```

#### Emotion & Face Updates

```python
async def update_emotion(self, emotion: str):
    """Update robot's emotion and sync with display"""
    self.current_emotion = emotion
    logger.info(f"Emotion updated to: {emotion}")
    
    # Update face display
    await self.display_face_animation({"emotion": emotion})
```

### Communication Protocol

#### Messages from Server to Raspberry Pi

**Emotion Update:**
```json
{
  "type": "emotion",
  "emotion": "happy"
}
```

**Face Animation:**
```json
{
  "type": "face_animation",
  "animation": {
    "emotion": "excited",
    "duration": 2000
  }
}
```

**Audio Playback:**
```json
{
  "type": "play_audio",
  "audio": {
    "text": "Hello! How are you?",
    "emotion": "friendly"
  }
}
```

#### Messages from Raspberry Pi to Server

**Status Update:**
```json
{
  "type": "status",
  "device": "RaspberryPi",
  "emotion": "happy",
  "timestamp": "2024-01-01T12:00:00"
}
```

---

## Testing Hardware Code

### ESP12E Testing

1. **Upload and Monitor:**
   ```bash
   # In Arduino IDE
   - Upload sketch
   - Open Serial Monitor (115200 baud)
   - Watch for connection messages
   ```

2. **Expected Output:**
   ```
   ESP12E Motor Controller Initialized
   Connecting to WiFi...
   WiFi connected
   IP address: 192.168.1.101
   WebSocket client initialized
   WebSocket Connected
   ```

3. **Test Commands:**
   Send from server or use API:
   ```bash
   curl -X POST http://192.168.1.100:8000/api/command \
     -H "Content-Type: application/json" \
     -d '{"type":"move","direction":"forward","speed":200}'
   ```

### Raspberry Pi Testing

1. **Run Controller:**
   ```bash
   python3 raspberry_pi_controller.py
   ```

2. **Expected Output:**
   ```
   INFO - Raspberry Pi Controller Initialized
   INFO - Connected to server
   ```

3. **Test Emotion Update:**
   ```bash
   curl -X POST http://192.168.1.100:8000/api/command \
     -H "Content-Type: application/json" \
     -d '{"type":"emotion","emotion":"happy"}'
   ```

---

## Troubleshooting

### ESP12E Issues

**Won't Connect to Wi-Fi:**
- Check SSID and password in config.h
- Verify ESP12E is in range of Wi-Fi
- Check serial monitor for error messages

**Motors Don't Move:**
- Verify L298N connections
- Check power supply (7-12V, sufficient current)
- Test motors directly with power supply
- Monitor serial output for command receipt

**WebSocket Won't Connect:**
- Verify server IP in config.h
- Check server is running: `curl http://server:8000/health`
- Check firewall allows port 8000
- Monitor server logs for connection attempts

### Raspberry Pi Issues

**Connection Failed:**
- Verify SERVER_URL is correct
- Test network: `ping server-ip`
- Check websockets library: `python3 -c "import websockets"`
- Check server logs

**Audio Not Working:**
- Verify audio jack is connected
- Test with: `speaker-test -t wav -c 2`
- Check audio output device: `aplay -l`

**Display Issues:**
- Verify HDMI connection
- Check display resolution settings
- Test with simple graphics library

---

## Future Enhancements

### Ultrasonic Sensor Support

Can be added to ESP12E for obstacle detection:

```cpp
#define DISTANCE_TRIG D8
#define DISTANCE_ECHO D4

float readDistance() {
  digitalWrite(DISTANCE_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(DISTANCE_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(DISTANCE_TRIG, LOW);
  
  long duration = pulseIn(DISTANCE_ECHO, HIGH);
  float distance = (duration * 0.034) / 2;
  
  return distance;
}
```

Report to server:
```json
{
  "type": "sensor",
  "sensor": "distance",
  "value": 25.5
}
```

### IMU/Gyroscope

Could be added for orientation sensing and better navigation.

### Camera Integration

Could be added to Raspberry Pi for vision-based features.

---

## Complete Code Files

The complete, up-to-date code is available in the repository:

- **ESP12E**: `hardware/esp12e/motor_controller.ino` and `config.h`
- **Raspberry Pi**: `hardware/raspberry_pi/raspberry_pi_controller.py`
- **Server**: `server/server.py`

Refer to these files for the most current implementation.
