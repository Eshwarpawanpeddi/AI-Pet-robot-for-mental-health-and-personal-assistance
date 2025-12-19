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

# GPIO Pin Definitions (for audio output)
GPIO_I2S_BCK = 18
GPIO_I2S_LRCK = 13
GPIO_I2S_DATA = 12

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
    """Central Hardware Controller for Raspberry Pi 4 - WebSocket based"""
    
    def __init__(self):
        self.gpio_lock = Lock()
        self.sensor_data_lock = Lock()
        self.websocket = None
        self.running = False
        self.current_emotion = "neutral"
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
        except ImportError:
            logger.warning("RPi.GPIO not available - running in simulation mode")
            self.GPIO = None
        
        logger.info("Raspberry Pi Controller Initialized")
    
    def _init_gpio(self):
        """Initialize GPIO pins"""
        if not self.GPIO:
            return
            
        GPIO = self.GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Motor pins (not used directly anymore - handled via server)
        # PWM Setup for audio output
        if self.GPIO:
            GPIO.setup(GPIO_I2S_BCK, GPIO.OUT)
            GPIO.setup(GPIO_I2S_LRCK, GPIO.OUT)
            GPIO.setup(GPIO_I2S_DATA, GPIO.OUT)
        
        logger.info("GPIO pins initialized")
    
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
    
    def run(self):
        """Run the controller"""
        self.running = True
        asyncio.run(self.connect_to_server())
    
    def cleanup(self):
        """Cleanup GPIO"""
        self.running = False
        if self.GPIO:
            self.GPIO.cleanup()
        logger.info("GPIO cleaned up")

if __name__ == "__main__":
    controller = RaspberryPiController()
    try:
        controller.run()
    except KeyboardInterrupt:
        logger.info("Shutting down...")
        controller.cleanup()