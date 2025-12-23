import time
import json
import logging
from typing import Dict, List
from enum import Enum
from dataclasses import dataclass
from threading import Thread, Lock
import asyncio
import websockets
from datetime import datetime

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Server Configuration
SERVER_URL = "ws://192.168.1.100:8000/ws/raspberry_pi"
RECONNECT_DELAY = 5

# Motor Driver 1 GPIO Pin Definitions (Motors A & B - Front wheels)
MOTOR_A_IN1 = 17   # Motor A Direction 1
MOTOR_A_IN2 = 27   # Motor A Direction 2
MOTOR_A_ENA = 22   # Motor A Speed/PWM Enable

MOTOR_B_IN3 = 23   # Motor B Direction 1
MOTOR_B_IN4 = 24   # Motor B Direction 2
MOTOR_B_ENB = 25   # Motor B Speed/PWM Enable

# Motor Driver 2 GPIO Pin Definitions (Motors C & D - Rear wheels)
MOTOR_C_IN1 = 5    # Motor C Direction 1
MOTOR_C_IN2 = 6    # Motor C Direction 2
MOTOR_C_ENA = 13   # Motor C Speed/PWM Enable

MOTOR_D_IN3 = 19   # Motor D Direction 1
MOTOR_D_IN4 = 26   # Motor D Direction 2
MOTOR_D_ENB = 12   # Motor D Speed/PWM Enable

# PWM Configuration
PWM_FREQUENCY = 1000  # 1kHz for smooth motor operation

# Touch Sensor GPIO
GPIO_TOUCH = 16

# Audio GPIO (I2S)
GPIO_I2S_BCK = 18
GPIO_I2S_LRCK = 20
GPIO_I2S_DATA = 21

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
    """Central Hardware Controller for Raspberry Pi 4 - WebSocket based with Motor Control"""
    
    def __init__(self):
        self.gpio_lock = Lock()
        self.sensor_data_lock = Lock()
        self.websocket = None
        self.running = False
        self.current_emotion = "neutral"
        self.current_speed = 0
        self.current_direction = "stop"
        self.pwm_motors = {}  # Store PWM instances for motors
        self.current_sensor_data = SensorData(
            timestamp=time.time(),
            touch_detected=False,
            distance=0.0,
            battery_level=100,
            temperature=25.0
        )
        
        try:
            import RPi.GPIO as GPIO
            self.GPIO = GPIO
            self._init_gpio()
            self._init_motors()
        except ImportError:
            logger.warning("RPi.GPIO not available - running in simulation mode")
            self.GPIO = None
        
        logger.info("Raspberry Pi Controller Initialized with 4-wheel motor control")
    
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
        
        # Audio pins
        GPIO.setup(GPIO_I2S_BCK, GPIO.OUT)
        GPIO.setup(GPIO_I2S_LRCK, GPIO.OUT)
        GPIO.setup(GPIO_I2S_DATA, GPIO.OUT)
        
        logger.info("GPIO pins initialized for 4-wheel motor control")
    
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
        
        logger.info("PWM initialized for all 4 motors at {}Hz".format(PWM_FREQUENCY))
    
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
                logger.info(f"Reconnecting in {RECONNECT_DELAY} seconds...")
                await asyncio.sleep(RECONNECT_DELAY)
    
    async def handle_server_message(self, message: str):
        """Handle incoming messages from server"""
        try:
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
                
            elif msg_type == "get_status":
                await self.send_status()
                
        except json.JSONDecodeError as e:
            logger.error(f"JSON decode error: {e}")
    
    async def update_emotion(self, emotion: str):
        """Update robot's emotion and sync with display"""
        self.current_emotion = emotion
        logger.info(f"Emotion updated to: {emotion}")
        
        # Update face display
        await self.display_face_animation({"emotion": emotion})
    
    async def display_face_animation(self, animation_data: Dict):
        """Display face animation on HDMI output"""
        logger.info(f"Displaying face animation: {animation_data}")
        # This would be handled by a separate display process/service
        # that renders the face on HDMI output
    
    async def play_audio(self, audio_data: Dict):
        """Play audio through audio jack"""
        logger.info(f"Playing audio: {audio_data}")
        # Audio playback via system audio (e.g., using pygame, pyaudio, etc.)
    
    async def send_status(self):
        """Send status update to server"""
        if not self.websocket:
            return
            
        status = {
            "type": "status",
            "device": "RaspberryPi",
            "emotion": self.current_emotion,
            "sensor_data": self.get_sensor_data(),
            "timestamp": datetime.now().isoformat()
        }
        
        await self.websocket.send(json.dumps(status))
    
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
    
    async def handle_motor_command(self, direction: str, speed: int):
        """
        Handle motor control commands for 4-wheel setup.
        
        Args:
            direction (str): Movement direction - 'forward', 'backward', 'left', 'right', or 'stop'
            speed (int): Speed percentage (0-100), mapped to PWM duty cycle
        """
        if not self.GPIO:
            logger.warning("GPIO not available - motor command ignored")
            return
        
        with self.gpio_lock:
            self.current_direction = direction
            self.current_speed = speed
            
            # Convert speed (0-100) to PWM duty cycle (0-100)
            duty_cycle = max(0, min(100, speed))
            
            logger.info(f"Motor command: {direction} at speed {speed}% (duty cycle: {duty_cycle}%)")
            
            if direction == "forward":
                self._move_forward(duty_cycle)
            elif direction == "backward":
                self._move_backward(duty_cycle)
            elif direction == "left":
                self._turn_left(duty_cycle)
            elif direction == "right":
                self._turn_right(duty_cycle)
            elif direction == "stop":
                self._stop_all_motors()
            else:
                logger.warning(f"Unknown direction: {direction}")
    
    def _move_forward(self, duty_cycle: int):
        """
        Move all 4 wheels forward.
        
        Args:
            duty_cycle (int): PWM duty cycle percentage (0-100)
        """
        if not self.GPIO:
            return
        
        GPIO = self.GPIO
        
        # Motor A - Forward
        GPIO.output(MOTOR_A_IN1, GPIO.HIGH)
        GPIO.output(MOTOR_A_IN2, GPIO.LOW)
        
        # Motor B - Forward
        GPIO.output(MOTOR_B_IN3, GPIO.HIGH)
        GPIO.output(MOTOR_B_IN4, GPIO.LOW)
        
        # Motor C - Forward
        GPIO.output(MOTOR_C_IN1, GPIO.HIGH)
        GPIO.output(MOTOR_C_IN2, GPIO.LOW)
        
        # Motor D - Forward
        GPIO.output(MOTOR_D_IN3, GPIO.HIGH)
        GPIO.output(MOTOR_D_IN4, GPIO.LOW)
        
        # Set speed for all motors
        for pwm in self.pwm_motors.values():
            pwm.ChangeDutyCycle(duty_cycle)
        
        logger.debug(f"Moving forward at {duty_cycle}% speed")
    
    def _move_backward(self, duty_cycle: int):
        """
        Move all 4 wheels backward.
        
        Args:
            duty_cycle (int): PWM duty cycle percentage (0-100)
        """
        if not self.GPIO:
            return
        
        GPIO = self.GPIO
        
        # Motor A - Backward
        GPIO.output(MOTOR_A_IN1, GPIO.LOW)
        GPIO.output(MOTOR_A_IN2, GPIO.HIGH)
        
        # Motor B - Backward
        GPIO.output(MOTOR_B_IN3, GPIO.LOW)
        GPIO.output(MOTOR_B_IN4, GPIO.HIGH)
        
        # Motor C - Backward
        GPIO.output(MOTOR_C_IN1, GPIO.LOW)
        GPIO.output(MOTOR_C_IN2, GPIO.HIGH)
        
        # Motor D - Backward
        GPIO.output(MOTOR_D_IN3, GPIO.LOW)
        GPIO.output(MOTOR_D_IN4, GPIO.HIGH)
        
        # Set speed for all motors
        for pwm in self.pwm_motors.values():
            pwm.ChangeDutyCycle(duty_cycle)
        
        logger.debug(f"Moving backward at {duty_cycle}% speed")
    
    def _turn_left(self, duty_cycle: int):
        """
        Turn left using tank-style steering - left wheels backward, right wheels forward.
        
        Args:
            duty_cycle (int): PWM duty cycle percentage (0-100)
        """
        if not self.GPIO:
            return
        
        GPIO = self.GPIO
        
        # Motor A (Left Front) - Backward
        GPIO.output(MOTOR_A_IN1, GPIO.LOW)
        GPIO.output(MOTOR_A_IN2, GPIO.HIGH)
        
        # Motor B (Right Front) - Forward
        GPIO.output(MOTOR_B_IN3, GPIO.HIGH)
        GPIO.output(MOTOR_B_IN4, GPIO.LOW)
        
        # Motor C (Left Rear) - Backward
        GPIO.output(MOTOR_C_IN1, GPIO.LOW)
        GPIO.output(MOTOR_C_IN2, GPIO.HIGH)
        
        # Motor D (Right Rear) - Forward
        GPIO.output(MOTOR_D_IN3, GPIO.HIGH)
        GPIO.output(MOTOR_D_IN4, GPIO.LOW)
        
        # Set speed for all motors
        for pwm in self.pwm_motors.values():
            pwm.ChangeDutyCycle(duty_cycle)
        
        logger.debug(f"Turning left at {duty_cycle}% speed")
    
    def _turn_right(self, duty_cycle: int):
        """
        Turn right using tank-style steering - right wheels backward, left wheels forward.
        
        Args:
            duty_cycle (int): PWM duty cycle percentage (0-100)
        """
        if not self.GPIO:
            return
        
        GPIO = self.GPIO
        
        # Motor A (Left Front) - Forward
        GPIO.output(MOTOR_A_IN1, GPIO.HIGH)
        GPIO.output(MOTOR_A_IN2, GPIO.LOW)
        
        # Motor B (Right Front) - Backward
        GPIO.output(MOTOR_B_IN3, GPIO.LOW)
        GPIO.output(MOTOR_B_IN4, GPIO.HIGH)
        
        # Motor C (Left Rear) - Forward
        GPIO.output(MOTOR_C_IN1, GPIO.HIGH)
        GPIO.output(MOTOR_C_IN2, GPIO.LOW)
        
        # Motor D (Right Rear) - Backward
        GPIO.output(MOTOR_D_IN3, GPIO.LOW)
        GPIO.output(MOTOR_D_IN4, GPIO.HIGH)
        
        # Set speed for all motors
        for pwm in self.pwm_motors.values():
            pwm.ChangeDutyCycle(duty_cycle)
        
        logger.debug(f"Turning right at {duty_cycle}% speed")
    
    def _stop_all_motors(self):
        """Stop all 4 motors"""
        if not self.GPIO:
            return
        
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
        
        logger.debug("All motors stopped")
    
    def run(self):
        """Run the controller"""
        self.running = True
        asyncio.run(self.connect_to_server())
    
    def cleanup(self):
        """Cleanup GPIO and stop motors"""
        self.running = False
        
        # Stop all motors before cleanup
        if self.GPIO:
            try:
                self._stop_all_motors()
                
                # Stop all PWM
                for pwm in self.pwm_motors.values():
                    pwm.stop()
                
                # Cleanup GPIO
                self.GPIO.cleanup()
                logger.info("GPIO cleaned up and motors stopped")
            except Exception as e:
                logger.error(f"Error during cleanup: {e}")
        else:
            logger.info("Cleanup called (simulation mode)")

if __name__ == "__main__":
    controller = RaspberryPiController()
    try:
        controller.run()
    except KeyboardInterrupt:
        logger.info("Shutting down...")
        controller.cleanup()