# Hardware Code Documentation - Direct Pi Motor Control

## Overview

This document describes the hardware code for the AI Pet Robot using the new direct Raspberry Pi motor control architecture. The Raspberry Pi controls 4 DC motors via 2 L298N motor drivers using GPIO pins, eliminating the need for a separate ESP12E module.

## Raspberry Pi Motor Controller

### Features

- WebSocket connection to central server for command reception
- Direct GPIO-based motor control for 4 wheels
- PWM speed control (0-100% duty cycle)
- Synchronized 4-wheel movement
- Real-time face display and audio output
- Status reporting to server

### Hardware Configuration

#### GPIO Pin Mapping

**Motor Driver 1 (Front Wheels - Motors A & B)**:
```python
MOTOR_A_IN1 = 17   # Motor A Direction 1
MOTOR_A_IN2 = 27   # Motor A Direction 2
MOTOR_A_ENA = 22   # Motor A Speed/PWM Enable

MOTOR_B_IN3 = 23   # Motor B Direction 1
MOTOR_B_IN4 = 24   # Motor B Direction 2
MOTOR_B_ENB = 25   # Motor B Speed/PWM Enable
```

**Motor Driver 2 (Rear Wheels - Motors C & D)**:
```python
MOTOR_C_IN1 = 5    # Motor C Direction 1
MOTOR_C_IN2 = 6    # Motor C Direction 2
MOTOR_C_ENA = 13   # Motor C Speed/PWM Enable

MOTOR_D_IN3 = 19   # Motor D Direction 1
MOTOR_D_IN4 = 26   # Motor D Direction 2
MOTOR_D_ENB = 12   # Motor D Speed/PWM Enable
```

**PWM Configuration**:
```python
PWM_FREQUENCY = 1000  # 1kHz for smooth motor operation
```

#### Wiring Diagram

```
Raspberry Pi GPIO    L298N Motor Driver 1 (Front Wheels)
-----------------    -----------------------------------
GPIO17           --> IN1
GPIO27           --> IN2
GPIO22           --> ENA (Enable A / PWM)
GPIO23           --> IN3
GPIO24           --> IN4
GPIO25           --> ENB (Enable B / PWM)
GND              --> GND

L298N Driver 1      Motors & Power
--------------      --------------
OUT1, OUT2      --> DC Motor A (Front Left)
OUT3, OUT4      --> DC Motor B (Front Right)
12V             --> External Power Supply (7-12V)
GND             --> Power Supply GND & Pi GND (common ground)


Raspberry Pi GPIO    L298N Motor Driver 2 (Rear Wheels)
-----------------    ----------------------------------
GPIO5            --> IN1
GPIO6            --> IN2
GPIO13           --> ENA (Enable C / PWM)
GPIO19           --> IN3
GPIO26           --> IN4
GPIO12           --> ENB (Enable D / PWM)
GND              --> GND

L298N Driver 2      Motors & Power
--------------      --------------
OUT1, OUT2      --> DC Motor C (Rear Left)
OUT3, OUT4      --> DC Motor D (Rear Right)
12V             --> External Power Supply (7-12V)
GND             --> Power Supply GND & Pi GND (common ground)
```

**CRITICAL**: Ensure all grounds are connected together:
- Raspberry Pi GND
- L298N Driver 1 GND
- L298N Driver 2 GND
- Power Supply GND

This common ground is essential for proper signal communication.

### Configuration

Update the `SERVER_URL` in `raspberry_pi_controller.py`:

```python
# Server Configuration
SERVER_URL = "ws://192.168.1.100:8000/ws/raspberry_pi"
RECONNECT_DELAY = 5

# PWM Configuration
PWM_FREQUENCY = 1000  # 1kHz for smooth motor operation
```

### Dependencies

```bash
# Install required packages
sudo apt-get update
sudo apt-get install -y python3-pip python3-websockets python3-rpi.gpio
pip3 install websockets asyncio
```

### Key Functions

#### GPIO Initialization

```python
def _init_gpio(self):
    """Initialize GPIO pins"""
    if not self.GPIO:
        return
        
    GPIO = self.GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    # Motor Driver 1 (Motors A & B) - Setup as outputs
    GPIO.setup(MOTOR_A_IN1, GPIO.OUT)
    GPIO.setup(MOTOR_A_IN2, GPIO.OUT)
    GPIO.setup(MOTOR_A_ENA, GPIO.OUT)
    GPIO.setup(MOTOR_B_IN3, GPIO.OUT)
    GPIO.setup(MOTOR_B_IN4, GPIO.OUT)
    GPIO.setup(MOTOR_B_ENB, GPIO.OUT)
    
    # Motor Driver 2 (Motors C & D) - Setup as outputs
    GPIO.setup(MOTOR_C_IN1, GPIO.OUT)
    GPIO.setup(MOTOR_C_IN2, GPIO.OUT)
    GPIO.setup(MOTOR_C_ENA, GPIO.OUT)
    GPIO.setup(MOTOR_D_IN3, GPIO.OUT)
    GPIO.setup(MOTOR_D_IN4, GPIO.OUT)
    GPIO.setup(MOTOR_D_ENB, GPIO.OUT)
```

#### PWM Initialization

```python
def _init_motors(self):
    """Initialize PWM for all 4 motors"""
    if not self.GPIO:
        return
        
    GPIO = self.GPIO
    
    # Create PWM instances for all motor enable pins
    self.pwm_motors['A'] = GPIO.PWM(MOTOR_A_ENA, PWM_FREQUENCY)
    self.pwm_motors['B'] = GPIO.PWM(MOTOR_B_ENB, PWM_FREQUENCY)
    self.pwm_motors['C'] = GPIO.PWM(MOTOR_C_ENA, PWM_FREQUENCY)
    self.pwm_motors['D'] = GPIO.PWM(MOTOR_D_ENB, PWM_FREQUENCY)
    
    # Start all PWM at 0% duty cycle (stopped)
    for motor_name, pwm in self.pwm_motors.items():
        pwm.start(0)
```

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
    
    elif msg_type == "move":
        direction = data.get("direction", "stop")
        speed = data.get("speed", 75)
        await self.handle_motor_command(direction, speed)
```

#### Motor Control Functions

**Forward Movement**:
```python
def _move_forward(self, duty_cycle: int):
    """Move all 4 wheels forward"""
    GPIO = self.GPIO
    
    # All motors forward
    GPIO.output(MOTOR_A_IN1, GPIO.HIGH)
    GPIO.output(MOTOR_A_IN2, GPIO.LOW)
    GPIO.output(MOTOR_B_IN3, GPIO.HIGH)
    GPIO.output(MOTOR_B_IN4, GPIO.LOW)
    GPIO.output(MOTOR_C_IN1, GPIO.HIGH)
    GPIO.output(MOTOR_C_IN2, GPIO.LOW)
    GPIO.output(MOTOR_D_IN3, GPIO.HIGH)
    GPIO.output(MOTOR_D_IN4, GPIO.LOW)
    
    # Set speed for all motors
    for pwm in self.pwm_motors.values():
        pwm.ChangeDutyCycle(duty_cycle)
```

**Backward Movement**:
```python
def _move_backward(self, duty_cycle: int):
    """Move all 4 wheels backward"""
    GPIO = self.GPIO
    
    # All motors backward
    GPIO.output(MOTOR_A_IN1, GPIO.LOW)
    GPIO.output(MOTOR_A_IN2, GPIO.HIGH)
    GPIO.output(MOTOR_B_IN3, GPIO.LOW)
    GPIO.output(MOTOR_B_IN4, GPIO.HIGH)
    GPIO.output(MOTOR_C_IN1, GPIO.LOW)
    GPIO.output(MOTOR_C_IN2, GPIO.HIGH)
    GPIO.output(MOTOR_D_IN3, GPIO.LOW)
    GPIO.output(MOTOR_D_IN4, GPIO.HIGH)
    
    # Set speed for all motors
    for pwm in self.pwm_motors.values():
        pwm.ChangeDutyCycle(duty_cycle)
```

**Left Turn** (Tank-style):
```python
def _turn_left(self, duty_cycle: int):
    """Turn left - left wheels backward, right wheels forward"""
    GPIO = self.GPIO
    
    # Left motors backward
    GPIO.output(MOTOR_A_IN1, GPIO.LOW)
    GPIO.output(MOTOR_A_IN2, GPIO.HIGH)
    GPIO.output(MOTOR_C_IN1, GPIO.LOW)
    GPIO.output(MOTOR_C_IN2, GPIO.HIGH)
    
    # Right motors forward
    GPIO.output(MOTOR_B_IN3, GPIO.HIGH)
    GPIO.output(MOTOR_B_IN4, GPIO.LOW)
    GPIO.output(MOTOR_D_IN3, GPIO.HIGH)
    GPIO.output(MOTOR_D_IN4, GPIO.LOW)
    
    # Set speed for all motors
    for pwm in self.pwm_motors.values():
        pwm.ChangeDutyCycle(duty_cycle)
```

**Right Turn** (Tank-style):
```python
def _turn_right(self, duty_cycle: int):
    """Turn right - right wheels backward, left wheels forward"""
    GPIO = self.GPIO
    
    # Left motors forward
    GPIO.output(MOTOR_A_IN1, GPIO.HIGH)
    GPIO.output(MOTOR_A_IN2, GPIO.LOW)
    GPIO.output(MOTOR_C_IN1, GPIO.HIGH)
    GPIO.output(MOTOR_C_IN2, GPIO.LOW)
    
    # Right motors backward
    GPIO.output(MOTOR_B_IN3, GPIO.LOW)
    GPIO.output(MOTOR_B_IN4, GPIO.HIGH)
    GPIO.output(MOTOR_D_IN3, GPIO.LOW)
    GPIO.output(MOTOR_D_IN4, GPIO.HIGH)
    
    # Set speed for all motors
    for pwm in self.pwm_motors.values():
        pwm.ChangeDutyCycle(duty_cycle)
```

**Stop All Motors**:
```python
def _stop_all_motors(self):
    """Stop all 4 motors"""
    GPIO = self.GPIO
    
    # Set all direction pins LOW
    GPIO.output(MOTOR_A_IN1, GPIO.LOW)
    GPIO.output(MOTOR_A_IN2, GPIO.LOW)
    GPIO.output(MOTOR_B_IN3, GPIO.LOW)
    GPIO.output(MOTOR_B_IN4, GPIO.LOW)
    GPIO.output(MOTOR_C_IN1, GPIO.LOW)
    GPIO.output(MOTOR_C_IN2, GPIO.LOW)
    GPIO.output(MOTOR_D_IN3, GPIO.LOW)
    GPIO.output(MOTOR_D_IN4, GPIO.LOW)
    
    # Set all PWM to 0%
    for pwm in self.pwm_motors.values():
        pwm.ChangeDutyCycle(0)
```

### Communication Protocol

#### Messages from Server to Raspberry Pi

**Movement Command:**
```json
{
  "type": "move",
  "direction": "forward",
  "speed": 75
}
```

Directions: `forward`, `backward`, `left`, `right`, `stop`
Speed: 0-100 (percentage, mapped to PWM duty cycle)

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

**Connection:**
```json
{
  "type": "raspberry_pi_connected",
  "device": "RaspberryPi"
}
```

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

### Raspberry Pi Motor Control Testing

1. **Run Controller:**
   ```bash
   cd hardware/raspberry_pi
   python3 raspberry_pi_controller.py
   ```

2. **Expected Output:**
   ```
   INFO - Raspberry Pi Controller Initialized with 4-wheel motor control
   INFO - GPIO pins initialized for 4-wheel motor control
   INFO - PWM initialized for all 4 motors at 1000Hz
   INFO - Connected to server
   ```

3. **Test Motor Commands:**
   Send from server or use API:
   ```bash
   # Test forward movement
   curl -X POST http://192.168.1.100:8000/api/command \
     -H "Content-Type: application/json" \
     -d '{"type":"move","direction":"forward","speed":75}'
   
   # Test backward movement
   curl -X POST http://192.168.1.100:8000/api/command \
     -H "Content-Type: application/json" \
     -d '{"type":"move","direction":"backward","speed":75}'
   
   # Test left turn
   curl -X POST http://192.168.1.100:8000/api/command \
     -H "Content-Type: application/json" \
     -d '{"type":"move","direction":"left","speed":75}'
   
   # Test right turn
   curl -X POST http://192.168.1.100:8000/api/command \
     -H "Content-Type: application/json" \
     -d '{"type":"move","direction":"right","speed":75}'
   
   # Stop motors
   curl -X POST http://192.168.1.100:8000/api/command \
     -H "Content-Type: application/json" \
     -d '{"type":"move","direction":"stop","speed":0}'
   ```

4. **Test Emotion Update:**
   ```bash
   curl -X POST http://192.168.1.100:8000/api/command \
     -H "Content-Type: application/json" \
     -d '{"type":"emotion","emotion":"happy"}'
   ```

### PWM Testing

Test different speed levels to ensure smooth operation:

```bash
# Low speed (25%)
curl -X POST http://192.168.1.100:8000/api/command \
  -H "Content-Type: application/json" \
  -d '{"type":"move","direction":"forward","speed":25}'

# Medium speed (50%)
curl -X POST http://192.168.1.100:8000/api/command \
  -H "Content-Type: application/json" \
  -d '{"type":"move","direction":"forward","speed":50}'

# High speed (75%)
curl -X POST http://192.168.1.100:8000/api/command \
  -H "Content-Type: application/json" \
  -d '{"type":"move","direction":"forward","speed":75}'

# Maximum speed (100%)
curl -X POST http://192.168.1.100:8000/api/command \
  -H "Content-Type: application/json" \
  -d '{"type":"move","direction":"forward","speed":100}'
```

---

## Troubleshooting

### Raspberry Pi Issues

**Connection Failed:**
- Verify SERVER_URL is correct
- Test network: `ping server-ip`
- Check websockets library: `python3 -c "import websockets"`
- Check server logs
- Ensure server is running: `curl http://server:8000/health`

**GPIO Errors:**
- Verify RPi.GPIO is installed: `python3 -c "import RPi.GPIO"`
- Install if missing: `sudo apt-get install python3-rpi.gpio`
- Check GPIO permissions: `sudo usermod -a -G gpio $USER`
- Run with sudo if needed: `sudo python3 raspberry_pi_controller.py`

**Motors Don't Move:**
- Verify L298N connections to GPIO pins
- Check power supply (7-12V, sufficient current - at least 2A per driver)
- Test motors directly with power supply
- Verify all grounds are connected together
- Check GPIO pin numbers match configuration
- Monitor Raspberry Pi logs for command receipt
- Use multimeter to check GPIO output voltage

**Motors Move Erratically:**
- Check power supply capacity (motors may draw high current)
- Verify PWM frequency (should be 1kHz)
- Check for loose connections
- Ensure common ground connection
- Test with lower speeds first

**One or More Motors Not Working:**
- Check individual L298N driver connections
- Verify motor wiring (OUT1/OUT2 pairs)
- Test motor directly with power supply
- Check specific GPIO pins with `gpio readall`
- Verify PWM signal on ENA/ENB pins

**Audio Not Working:**
- Verify audio jack is connected
- Test with: `speaker-test -t wav -c 2`
- Check audio output device: `aplay -l`
- Adjust volume: `alsamixer`

**Display Issues:**
- Verify HDMI connection
- Check display resolution settings
- Test with simple graphics: `fbset`

---

## Safety Considerations

1. **Power Supply**: Use appropriate voltage (7-12V) and sufficient current capacity for motors
2. **Common Ground**: Always connect all grounds together
3. **Motor Stall**: Avoid stalling motors for extended periods (overheating risk)
4. **Emergency Stop**: Implement physical emergency stop button
5. **Connection Loss**: Motors automatically stop if connection is lost
6. **Testing**: Start with low speeds (25-50%) during initial testing

---

## Future Enhancements

### Ultrasonic Sensor Support

Can be added to Raspberry Pi GPIO for obstacle detection:

```python
import time

TRIG_PIN = 16
ECHO_PIN = 20

def setup_ultrasonic():
    GPIO.setup(TRIG_PIN, GPIO.OUT)
    GPIO.setup(ECHO_PIN, GPIO.IN)

def read_distance():
    # Send trigger pulse
    GPIO.output(TRIG_PIN, GPIO.LOW)
    time.sleep(0.000002)
    GPIO.output(TRIG_PIN, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(TRIG_PIN, GPIO.LOW)
    
    # Measure echo time
    while GPIO.input(ECHO_PIN) == 0:
        pulse_start = time.time()
    
    while GPIO.input(ECHO_PIN) == 1:
        pulse_end = time.time()
    
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150  # Speed of sound
    distance = round(distance, 2)
    
    return distance
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

Could be added via I2C for orientation sensing and better navigation.

### Camera Integration

Could be added to Raspberry Pi via CSI or USB for vision-based features.

### Encoder Support

Add rotary encoders to motors for precise position control and odometry.

---

## Complete Code Files

The complete, up-to-date code is available in the repository:

- **Raspberry Pi Controller**: `hardware/raspberry_pi/raspberry_pi_controller.py`
- **Server**: `server/server.py`

Refer to these files for the most current implementation.
