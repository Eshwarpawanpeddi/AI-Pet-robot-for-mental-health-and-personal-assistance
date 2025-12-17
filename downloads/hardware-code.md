# ESP12E Motor & Hardware Controller
# Arduino Sketch for Motor Control and Hardware I/O

```cpp
#include <Wire.h>
#include <ArduinoJson.h>

// I2C Configuration
#define SLAVE_ADDRESS 0x08
#define BAUD_RATE 115200

// Motor Control Pins (L298N Motor Driver)
#define MOTOR_A_PIN1 D0  // GPIO16
#define MOTOR_A_PIN2 D1  // GPIO5
#define MOTOR_B_PIN1 D2  // GPIO4
#define MOTOR_B_PIN2 D3  // GPIO0

// PWM Control Pins (Speed)
#define MOTOR_A_PWM D5   // GPIO14
#define MOTOR_B_PWM D6   // GPIO12

// Sensor Pins
#define TOUCH_SENSOR_PIN D7   // GPIO13
#define DISTANCE_TRIG D8      // GPIO15
#define DISTANCE_ECHO D4      // GPIO2

// Servo Control Pins (Head, Eyes)
#define SERVO_HEAD_PIN D7
#define SERVO_EYES_PIN D8

// Command Structure
struct MotorCommand {
  int motor;           // 0 = Motor A, 1 = Motor B, 2 = Both
  int direction;       // 0 = Stop, 1 = Forward, 2 = Backward, 3 = Left, 4 = Right
  int speed;           // 0-255
};

volatile byte requestedBytes = 0;
volatile MotorCommand currentCommand = {0, 0, 0};

void setup() {
  Serial.begin(BAUD_RATE);
  Wire.begin(SLAVE_ADDRESS);
  
  // Set motor pins as outputs
  pinMode(MOTOR_A_PIN1, OUTPUT);
  pinMode(MOTOR_A_PIN2, OUTPUT);
  pinMode(MOTOR_B_PIN1, OUTPUT);
  pinMode(MOTOR_B_PIN2, OUTPUT);
  
  // PWM configuration
  pinMode(MOTOR_A_PWM, OUTPUT);
  pinMode(MOTOR_B_PWM, OUTPUT);
  
  // Sensor pins
  pinMode(TOUCH_SENSOR_PIN, INPUT);
  pinMode(DISTANCE_TRIG, OUTPUT);
  pinMode(DISTANCE_ECHO, INPUT);
  
  // I2C callbacks
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);
  
  Serial.println("ESP12E Motor Controller Initialized");
  Serial.println("Waiting for I2C commands...");
  
  // Stop all motors by default
  stopAllMotors();
}

void loop() {
  // Check for touch sensor
  if (digitalRead(TOUCH_SENSOR_PIN) == HIGH) {
    Serial.println("Touch detected!");
    delay(50); // Debounce
  }
  
  delay(10);
}

// I2C Data Reception Handler
void receiveData(int byteCount) {
  if (byteCount >= 3) {
    int motor = Wire.read();      // Motor selection
    int direction = Wire.read();  // Direction
    int speed = Wire.read();      // Speed 0-255
    
    currentCommand.motor = motor;
    currentCommand.direction = direction;
    currentCommand.speed = speed;
    
    executeMotorCommand(currentCommand);
    
    Serial.print("Motor: ");
    Serial.print(motor);
    Serial.print(" | Direction: ");
    Serial.print(direction);
    Serial.print(" | Speed: ");
    Serial.println(speed);
  }
}

// I2C Data Transmission Handler
void sendData() {
  Wire.write(currentCommand.direction);  // Send current state
}

// Motor Control Functions
void executeMotorCommand(MotorCommand cmd) {
  if (cmd.direction == 0) {
    stopAllMotors();
    return;
  }
  
  switch (cmd.direction) {
    case 1: // Forward
      moveForward(cmd.speed);
      break;
    case 2: // Backward
      moveBackward(cmd.speed);
      break;
    case 3: // Left
      turnLeft(cmd.speed);
      break;
    case 4: // Right
      turnRight(cmd.speed);
      break;
  }
}

void moveForward(int speed) {
  // Motor A: Forward
  digitalWrite(MOTOR_A_PIN1, HIGH);
  digitalWrite(MOTOR_A_PIN2, LOW);
  
  // Motor B: Forward
  digitalWrite(MOTOR_B_PIN1, HIGH);
  digitalWrite(MOTOR_B_PIN2, LOW);
  
  // Set speed
  analogWrite(MOTOR_A_PWM, speed);
  analogWrite(MOTOR_B_PWM, speed);
}

void moveBackward(int speed) {
  // Motor A: Backward
  digitalWrite(MOTOR_A_PIN1, LOW);
  digitalWrite(MOTOR_A_PIN2, HIGH);
  
  // Motor B: Backward
  digitalWrite(MOTOR_B_PIN1, LOW);
  digitalWrite(MOTOR_B_PIN2, HIGH);
  
  // Set speed
  analogWrite(MOTOR_A_PWM, speed);
  analogWrite(MOTOR_B_PWM, speed);
}

void turnLeft(int speed) {
  // Motor A: Backward (right side)
  digitalWrite(MOTOR_A_PIN1, LOW);
  digitalWrite(MOTOR_A_PIN2, HIGH);
  
  // Motor B: Forward (left side)
  digitalWrite(MOTOR_B_PIN1, HIGH);
  digitalWrite(MOTOR_B_PIN2, LOW);
  
  analogWrite(MOTOR_A_PWM, speed);
  analogWrite(MOTOR_B_PWM, speed);
}

void turnRight(int speed) {
  // Motor A: Forward (right side)
  digitalWrite(MOTOR_A_PIN1, HIGH);
  digitalWrite(MOTOR_A_PIN2, LOW);
  
  // Motor B: Backward (left side)
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

// Sensor Reading Functions
float readDistance() {
  digitalWrite(DISTANCE_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(DISTANCE_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(DISTANCE_TRIG, LOW);
  
  long duration = pulseIn(DISTANCE_ECHO, HIGH);
  float distance = (duration * 0.034) / 2; // Convert to cm
  
  return distance;
}

void servoPulse(int pin, int angle) {
  // Simple servo control (1ms = 0°, 1.5ms = 90°, 2ms = 180°)
  int pulseWidth = map(angle, 0, 180, 1000, 2000);
  
  digitalWrite(pin, HIGH);
  delayMicroseconds(pulseWidth);
  digitalWrite(pin, LOW);
  delayMicroseconds(20000 - pulseWidth);
}
```

## Raspberry Pi 4 Hardware Controller (Python)

```python
# raspberry_pi_controller.py
import smbus
import time
import json
import logging
from typing import Dict, List
from enum import Enum
from dataclasses import dataclass
from threading import Thread, Lock
import RPi.GPIO as GPIO
from datetime import datetime

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# I2C Configuration
I2C_BUS = 1
ESP12E_ADDRESS = 0x08

# GPIO Pin Definitions
GPIO_MOTOR_EN_A = 17
GPIO_MOTOR_EN_B = 27
GPIO_MOTOR_DIR_A1 = 22
GPIO_MOTOR_DIR_A2 = 23
GPIO_MOTOR_DIR_B1 = 24
GPIO_MOTOR_DIR_B2 = 25

# Touch Sensor GPIO
GPIO_TOUCH = 26

# Display GPIO (SPI)
GPIO_SPI_CE = 8
GPIO_SPI_MOSI = 10
GPIO_SPI_MISO = 9
GPIO_SPI_CLK = 11

# Audio GPIO (I2S)
GPIO_I2S_BCK = 18
GPIO_I2S_LRCK = 13
GPIO_I2S_DATA = 12

class Direction(Enum):
    STOP = 0
    FORWARD = 1
    BACKWARD = 2
    LEFT = 3
    RIGHT = 4

@dataclass
class SensorData:
    timestamp: float
    touch_detected: bool
    distance: float
    battery_level: int
    temperature: float

class RaspberryPiController:
    """Central Hardware Controller for Raspberry Pi 4"""
    
    def __init__(self):
        self.bus = smbus.SMBus(I2C_BUS)
        self.gpio_lock = Lock()
        self.sensor_data_lock = Lock()
        self.current_sensor_data = SensorData(
            timestamp=time.time(),
            touch_detected=False,
            distance=0.0,
            battery_level=100,
            temperature=25.0
        )
        
        self._init_gpio()
        self._init_i2c()
        
        logger.info("Raspberry Pi Controller Initialized")
    
    def _init_gpio(self):
        """Initialize GPIO pins"""
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Motor pins
        motor_pins = [
            GPIO_MOTOR_EN_A, GPIO_MOTOR_EN_B,
            GPIO_MOTOR_DIR_A1, GPIO_MOTOR_DIR_A2,
            GPIO_MOTOR_DIR_B1, GPIO_MOTOR_DIR_B2
        ]
        
        for pin in motor_pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)
        
        # PWM Setup for variable speed
        self.pwm_a = GPIO.PWM(GPIO_MOTOR_EN_A, 100)  # 100Hz frequency
        self.pwm_b = GPIO.PWM(GPIO_MOTOR_EN_B, 100)
        self.pwm_a.start(0)
        self.pwm_b.start(0)
        
        # Touch sensor input
        GPIO.setup(GPIO_TOUCH, GPIO.IN)
        GPIO.add_event_detect(GPIO_TOUCH, GPIO.RISING, 
                             callback=self._touch_callback, 
                             bouncetime=200)
        
        # SPI Setup for display
        spi_pins = [GPIO_SPI_CE, GPIO_SPI_MOSI, GPIO_SPI_MISO, GPIO_SPI_CLK]
        for pin in spi_pins:
            GPIO.setup(pin, GPIO.OUT)
        
        # I2S Setup for audio
        audio_pins = [GPIO_I2S_BCK, GPIO_I2S_LRCK, GPIO_I2S_DATA]
        for pin in audio_pins:
            GPIO.setup(pin, GPIO.OUT)
        
        logger.info("GPIO pins initialized")
    
    def _init_i2c(self):
        """Initialize I2C communication"""
        try:
            # Test I2C connection with ESP12E
            data = self.bus.read_byte(ESP12E_ADDRESS)
            logger.info(f"ESP12E connected at address 0x{ESP12E_ADDRESS:02x}")
        except Exception as e:
            logger.error(f"Failed to connect to ESP12E: {e}")
    
    def _touch_callback(self, channel):
        """Callback for touch sensor"""
        with self.sensor_data_lock:
            self.current_sensor_data.touch_detected = True
        logger.info("Touch sensor activated")
    
    def send_motor_command(self, motor: int, direction: Direction, speed: int):
        """Send motor command to ESP12E via I2C"""
        if not (0 <= speed <= 255):
            logger.warning(f"Invalid speed {speed}, clamping to 0-255")
            speed = max(0, min(255, speed))
        
        try:
            # Send command: motor, direction, speed
            command = [motor, direction.value, speed]
            self.bus.write_i2c_block_data(ESP12E_ADDRESS, 0x00, command)
            logger.debug(f"Motor command sent: motor={motor}, direction={direction.name}, speed={speed}")
        except Exception as e:
            logger.error(f"Failed to send motor command: {e}")
    
    def move_forward(self, speed: int = 200):
        """Move robot forward"""
        self.send_motor_command(motor=2, direction=Direction.FORWARD, speed=speed)
    
    def move_backward(self, speed: int = 200):
        """Move robot backward"""
        self.send_motor_command(motor=2, direction=Direction.BACKWARD, speed=speed)
    
    def turn_left(self, speed: int = 200):
        """Turn robot left"""
        self.send_motor_command(motor=2, direction=Direction.LEFT, speed=speed)
    
    def turn_right(self, speed: int = 200):
        """Turn robot right"""
        self.send_motor_command(motor=2, direction=Direction.RIGHT, speed=speed)
    
    def stop(self):
        """Stop all motors"""
        self.send_motor_command(motor=2, direction=Direction.STOP, speed=0)
    
    def get_sensor_data(self) -> Dict:
        """Get current sensor data"""
        with self.sensor_data_lock:
            return {
                'timestamp': self.current_sensor_data.timestamp,
                'touch_detected': self.current_sensor_data.touch_detected,
                'distance': self.current_sensor_data.distance,
                'battery_level': self.current_sensor_data.battery_level,
                'temperature': self.current_sensor_data.temperature
            }
    
    def display_text(self, text: str, x: int = 0, y: int = 0):
        """Display text on LCD/OLED"""
        logger.info(f"Display: {text} at ({x}, {y})")
        # TODO: Implement SPI communication with display
    
    def play_sound(self, frequency: int, duration: int):
        """Play sound through speaker"""
        logger.info(f"Playing sound: {frequency}Hz for {duration}ms")
        # TODO: Implement I2S audio output
    
    def cleanup(self):
        """Cleanup GPIO"""
        self.pwm_a.stop()
        self.pwm_b.stop()
        GPIO.cleanup()
        logger.info("GPIO cleaned up")
```

## FastAPI Server (Laptop Processing Core)

```python
# server.py
from fastapi import FastAPI, WebSocket, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
import asyncio
import json
import logging
from typing import Dict, List, Optional
from datetime import datetime
import google.generativeai as genai
from google.generativeai.types import LiveConnectorConfig
import uvicorn

app = FastAPI(title="Pet Robot Server", version="1.0.0")

# CORS Configuration
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Gemini API Configuration
GEMINI_API_KEY = "YOUR_GEMINI_API_KEY"
genai.configure(api_key=GEMINI_API_KEY)

class RobotState:
    """Global robot state management"""
    def __init__(self):
        self.emotion = "neutral"
        self.is_speaking = False
        self.is_listening = False
        self.battery_level = 100
        self.connected_clients: List[WebSocket] = []
        self.sensor_data = {}
        self.gemini_session = None

robot_state = RobotState()

@app.on_event("startup")
async def startup():
    """Initialize server on startup"""
    logger.info("Robot Server Starting...")
    # Initialize Gemini Live API session
    await initialize_gemini()

@app.on_event("shutdown")
async def shutdown():
    """Cleanup on shutdown"""
    logger.info("Robot Server Shutting Down...")
    if robot_state.gemini_session:
        await robot_state.gemini_session.close()

async def initialize_gemini():
    """Initialize Gemini Live API connection"""
    try:
        model = genai.GenerativeModel(
            model_name="gemini-2.5-flash-native-audio-preview-12-2025",
            system_instruction="You are a friendly, emotionally intelligent pet robot. Keep responses concise and engaging. Express emotions through your speech patterns.",
            generation_config={
                'response_modalities': ['audio'],
                'realtime_input_config': {'automatic_activity_detection': {'disabled': False}}
            }
        )
        robot_state.gemini_session = model
        logger.info("Gemini Live API initialized")
    except Exception as e:
        logger.error(f"Failed to initialize Gemini: {e}")

@app.websocket("/ws/control")
async def websocket_control(websocket: WebSocket):
    """WebSocket endpoint for real-time control"""
    await websocket.accept()
    robot_state.connected_clients.append(websocket)
    
    try:
        while True:
            data = await websocket.receive_json()
            
            # Route command
            command_type = data.get('type')
            
            if command_type == 'move':
                await handle_move_command(data)
            elif command_type == 'voice':
                await handle_voice_command(data)
            elif command_type == 'emotion':
                await handle_emotion_command(data)
            elif command_type == 'get_state':
                await websocket.send_json(get_robot_state())
            
            # Broadcast state to all clients
            await broadcast_state()
    
    except Exception as e:
        logger.error(f"WebSocket error: {e}")
    finally:
        robot_state.connected_clients.remove(websocket)

async def handle_move_command(data: Dict):
    """Handle movement commands"""
    direction = data.get('direction')  # forward, backward, left, right, stop
    speed = data.get('speed', 200)
    
    logger.info(f"Move command: {direction} at speed {speed}")
    # Send to Raspberry Pi via MQTT/gRPC
    # await send_to_pi({"type": "motor", "direction": direction, "speed": speed})

async def handle_voice_command(data: Dict):
    """Handle voice commands via Gemini Live API"""
    audio_data = data.get('audio')
    
    robot_state.is_listening = True
    await broadcast_state()
    
    try:
        # Send audio to Gemini Live API
        response = await process_voice_with_gemini(audio_data)
        
        # Extract emotion and response
        emotion = detect_emotion(response)
        robot_state.emotion = emotion
        robot_state.is_speaking = True
        
        await broadcast_state()
        
        # Send response back to clients
        await broadcast_message({
            'type': 'response',
            'text': response,
            'emotion': emotion,
            'audio': response  # Gemini returns audio
        })
        
        robot_state.is_speaking = False
    except Exception as e:
        logger.error(f"Voice processing error: {e}")
    finally:
        robot_state.is_listening = False
        await broadcast_state()

async def handle_emotion_command(data: Dict):
    """Handle emotion change commands"""
    emotion = data.get('emotion')
    robot_state.emotion = emotion
    logger.info(f"Emotion changed to: {emotion}")

async def process_voice_with_gemini(audio_data: bytes) -> str:
    """Process voice with Gemini Live API"""
    try:
        # Implement Gemini Live API audio processing
        # This will send PCM audio and receive responses
        response = await asyncio.to_thread(
            lambda: robot_state.gemini_session.send_message(audio_data)
        )
        return response
    except Exception as e:
        logger.error(f"Gemini processing error: {e}")
        return "Sorry, I couldn't process that."

def detect_emotion(response: str) -> str:
    """Simple emotion detection based on response content"""
    response_lower = response.lower()
    
    if any(word in response_lower for word in ['happy', 'great', 'awesome', '!']):
        return 'happy'
    elif any(word in response_lower for word in ['sad', 'sorry', 'bad']):
        return 'sad'
    elif any(word in response_lower for word in ['angry', 'hate', 'wrong']):
        return 'angry'
    else:
        return 'neutral'

def get_robot_state() -> Dict:
    """Get current robot state"""
    return {
        'emotion': robot_state.emotion,
        'is_speaking': robot_state.is_speaking,
        'is_listening': robot_state.is_listening,
        'battery_level': robot_state.battery_level,
        'sensor_data': robot_state.sensor_data
    }

async def broadcast_state():
    """Broadcast state to all connected clients"""
    state = get_robot_state()
    for client in robot_state.connected_clients:
        try:
            await client.send_json({'type': 'state', 'data': state})
        except Exception as e:
            logger.error(f"Broadcast error: {e}")

async def broadcast_message(message: Dict):
    """Broadcast message to all connected clients"""
    for client in robot_state.connected_clients:
        try:
            await client.send_json(message)
        except Exception as e:
            logger.error(f"Broadcast error: {e}")

@app.get("/api/state")
async def get_state():
    """Get robot state via HTTP"""
    return get_robot_state()

@app.post("/api/command")
async def send_command(command: Dict):
    """Send command via HTTP"""
    await handle_move_command(command)
    return {"status": "ok"}

@app.get("/health")
async def health_check():
    """Health check endpoint"""
    return {
        "status": "healthy",
        "timestamp": datetime.now().isoformat(),
        "robot_state": get_robot_state()
    }

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
```

## Installation & Setup

### 1. Raspberry Pi Setup
```bash
# Update system
sudo apt-get update && sudo apt-get upgrade -y

# Install dependencies
sudo apt-get install -y python3-pip python3-smbus python3-rpi.gpio i2c-tools

# Install Python packages
pip3 install smbus-cffi PyYAML

# Enable I2C
sudo raspi-config nonint do_i2c 0
```

### 2. ESP12E Arduino Setup
- Install Arduino IDE
- Add ESP8266 board manager: http://arduino.esp8266.com/stable/package_esp8266com_index.json
- Select Board: NodeMCU 1.0 (ESP-12E Module)
- Upload the ESP12E firmware

### 3. Laptop Server Setup
```bash
pip install fastapi uvicorn google-generativeai websockets python-multipart

# Run server
python server.py
```

### 4. Phone App (React Native/Flutter)
- Create mobile app connecting to WebSocket at ws://server_ip:8000/ws/control
- Implement Gemini Live API for voice input
- Display animated face based on emotion state
