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