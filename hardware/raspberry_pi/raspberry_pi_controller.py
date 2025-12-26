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
SERVER_URL = "ws://10.44.35.29:8000/ws/raspberry_pi"
RECONNECT_DELAY = 5

# Motor Driver GPIO Pin Definitions (1 x L298N Driver - Parallel Wiring)
# Left Motors (Front & Rear) connected to OUT1/OUT2
MOTOR_LEFT_IN1 = 17   
MOTOR_LEFT_IN2 = 27   
MOTOR_LEFT_ENA = 22   

# Right Motors (Front & Rear) connected to OUT3/OUT4
MOTOR_RIGHT_IN3 = 23  
MOTOR_RIGHT_IN4 = 24  
MOTOR_RIGHT_ENB = 25  

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
    """Central Hardware Controller for Raspberry Pi 4 - 1 x L298N Parallel Motor Setup"""
    
    def __init__(self):
        self.gpio_lock = Lock()
        self.sensor_data_lock = Lock()
        self.websocket = None
        self.running = False
        self.current_emotion = "neutral"
        self.current_speed = 0
        self.current_direction = "stop"
        self.pwm_motors = {}  # Store PWM instances
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
        
        logger.info("Raspberry Pi Controller Initialized with 1-motor driver parallel setup")
    
    def _init_gpio(self):
        """Initialize GPIO pins for single driver"""
        if not self.GPIO: return
            
        GPIO = self.GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Setup motor control pins
        motor_pins = [MOTOR_LEFT_IN1, MOTOR_LEFT_IN2, MOTOR_LEFT_ENA,
                      MOTOR_RIGHT_IN3, MOTOR_RIGHT_IN4, MOTOR_RIGHT_ENB]
        GPIO.setup(motor_pins, GPIO.OUT)
        
        # Audio and Touch pins
        GPIO.setup([GPIO_I2S_BCK, GPIO_I2S_LRCK, GPIO_I2S_DATA], GPIO.OUT)
        GPIO.setup(GPIO_TOUCH, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        
        logger.info("GPIO pins initialized for single-driver setup")
    
    def _init_motors(self):
        """Initialize PWM for Left and Right channels"""
        if not self.GPIO: return
            
        GPIO = self.GPIO
        self.pwm_motors['LEFT'] = GPIO.PWM(MOTOR_LEFT_ENA, PWM_FREQUENCY)
        self.pwm_motors['RIGHT'] = GPIO.PWM(MOTOR_RIGHT_ENB, PWM_FREQUENCY)
        
        for pwm in self.pwm_motors.values():
            pwm.start(0)
        
        logger.info(f"PWM initialized at {PWM_FREQUENCY}Hz for Left/Right channels")
    
    async def connect_to_server(self):
        """Connect to central server via WebSocket"""
        while True:
            try:
                async with websockets.connect(SERVER_URL) as websocket:
                    self.websocket = websocket
                    logger.info("Connected to server")
                    
                    await websocket.send(json.dumps({
                        "type": "raspberry_pi_connected",
                        "device": "RaspberryPi"
                    }))
                    
                    async for message in websocket:
                        await self.handle_server_message(message)
                        
            except Exception as e:
                logger.error(f"WebSocket connection error: {e}")
                await asyncio.sleep(RECONNECT_DELAY)
    
    async def handle_server_message(self, message: str):
        """Handle incoming messages from server"""
        try:
            data = json.loads(message)
            msg_type = data.get("type")
            
            if msg_type == "emotion":
                await self.update_emotion(data.get("emotion", "neutral"))
            elif msg_type == "move":
                await self.handle_motor_command(data.get("direction", "stop"), data.get("speed", 75))
            elif msg_type == "play_audio":
                await self.play_audio(data.get("audio"))
            elif msg_type == "get_status":
                await self.send_status()
                
        except json.JSONDecodeError as e:
            logger.error(f"JSON decode error: {e}")

    async def handle_motor_command(self, direction: str, speed: int):
        if not self.GPIO: return
        
        with self.gpio_lock:
            self.current_direction = direction
            self.current_speed = speed
            duty_cycle = max(0, min(100, speed))
            
            if direction == "forward":
                self._move_forward(duty_cycle)
            elif direction == "backward":
                self._move_backward(duty_cycle)
            elif direction == "left":
                self._turn_left(duty_cycle)
            elif direction == "right":
                self._turn_right(duty_cycle)
            else:
                self._stop_all_motors()

    def _move_forward(self, duty_cycle: int):
        self.GPIO.output(MOTOR_LEFT_IN1, self.GPIO.HIGH)
        self.GPIO.output(MOTOR_LEFT_IN2, self.GPIO.LOW)
        self.GPIO.output(MOTOR_RIGHT_IN3, self.GPIO.HIGH)
        self.GPIO.output(MOTOR_RIGHT_IN4, self.GPIO.LOW)
        self._set_pwm(duty_cycle)

    def _move_backward(self, duty_cycle: int):
        self.GPIO.output(MOTOR_LEFT_IN1, self.GPIO.LOW)
        self.GPIO.output(MOTOR_LEFT_IN2, self.GPIO.HIGH)
        self.GPIO.output(MOTOR_RIGHT_IN3, self.GPIO.LOW)
        self.GPIO.output(MOTOR_RIGHT_IN4, self.GPIO.HIGH)
        self._set_pwm(duty_cycle)

    def _turn_left(self, duty_cycle: int):
        # Tank turn: Left backward, Right forward
        self.GPIO.output(MOTOR_LEFT_IN1, self.GPIO.LOW)
        self.GPIO.output(MOTOR_LEFT_IN2, self.GPIO.HIGH)
        self.GPIO.output(MOTOR_RIGHT_IN3, self.GPIO.HIGH)
        self.GPIO.output(MOTOR_RIGHT_IN4, self.GPIO.LOW)
        self._set_pwm(duty_cycle)

    def _turn_right(self, duty_cycle: int):
        # Tank turn: Left forward, Right backward
        self.GPIO.output(MOTOR_LEFT_IN1, self.GPIO.HIGH)
        self.GPIO.output(MOTOR_LEFT_IN2, self.GPIO.LOW)
        self.GPIO.output(MOTOR_RIGHT_IN3, self.GPIO.LOW)
        self.GPIO.output(MOTOR_RIGHT_IN4, self.GPIO.HIGH)
        self._set_pwm(duty_cycle)

    def _stop_all_motors(self):
        self.GPIO.output([MOTOR_LEFT_IN1, MOTOR_LEFT_IN2, MOTOR_RIGHT_IN3, MOTOR_RIGHT_IN4], self.GPIO.LOW)
        self._set_pwm(0)

    def _set_pwm(self, duty_cycle: int):
        for pwm in self.pwm_motors.values():
            pwm.ChangeDutyCycle(duty_cycle)

    async def update_emotion(self, emotion: str):
        self.current_emotion = emotion
        logger.info(f"Emotion updated to: {emotion}")

    async def play_audio(self, audio_data: Dict):
        logger.info(f"Playing audio: {audio_data}")

    async def send_status(self):
        if self.websocket:
            status = {"type": "status", "device": "RaspberryPi", "emotion": self.current_emotion}
            await self.websocket.send(json.dumps(status))

    def run(self):
        self.running = True
        asyncio.run(self.connect_to_server())

    def cleanup(self):
        if self.GPIO:
            self._stop_all_motors()
            for pwm in self.pwm_motors.values(): pwm.stop()
            self.GPIO.cleanup()

if __name__ == "__main__":
    controller = RaspberryPiController()
    try:
        controller.run()
    except KeyboardInterrupt:
        controller.cleanup()
