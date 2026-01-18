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
import base64
import io
import subprocess
import os

# Try to load environment variables
try:
    from dotenv import load_dotenv
    load_dotenv()
except ImportError:
    pass

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Server Configuration - Use environment variable or default to localhost
SERVER_HOST = os.getenv("SERVER_HOST", "localhost")
SERVER_PORT = os.getenv("SERVER_PORT", "8000")
SERVER_URL = f"ws://{SERVER_HOST}:{SERVER_PORT}/ws/raspberry_pi"
RECONNECT_DELAY = 5

logger.info(f"Raspberry Pi Controller will connect to: {SERVER_URL}")

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

# Audio Configuration
AUDIO_OUTPUT_MODE = "default"  # Options: "default" (3.5mm jack), "gpio_pwm", "i2s"
AUDIO_VOLUME = 80  # Volume percentage (0-100)

# TTS Voice Configuration
# Available espeak voices: en (default), en-us, en-uk, en-scottish, en+f1-f5 (female), en+m1-m7 (male)
# Speed: 80-450 words per minute (default: 175)
# Pitch: 0-99 (default: 50)
TTS_VOICE = "en+f3"  # Options: "en" (male), "en+f3" (female), "en+m3" (male deep), etc.
TTS_SPEED = 150  # Words per minute
TTS_PITCH = 50  # Voice pitch (0-99)

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
        self.camera = None
        self.camera_enabled = False
        self.camera_streaming = False
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
        
        # Initialize camera
        self._init_camera()
        
        logger.info("Raspberry Pi Controller Initialized with 1-motor driver parallel setup")
    
    def _init_gpio(self):
        """Initialize GPIO pins for single driver with validation"""
        if not self.GPIO: return
            
        GPIO = self.GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Validate GPIO pin numbers (BCM valid range: 2-27)
        motor_pins = [MOTOR_LEFT_IN1, MOTOR_LEFT_IN2, MOTOR_LEFT_ENA,
                      MOTOR_RIGHT_IN3, MOTOR_RIGHT_IN4, MOTOR_RIGHT_ENB]
        
        for pin in motor_pins:
            if not (2 <= pin <= 27):
                logger.error(f"Invalid GPIO pin number: {pin}. Must be between 2-27 (BCM mode).")
                raise ValueError(f"Invalid GPIO pin: {pin}")
        
        # Setup motor control pins
        try:
            GPIO.setup(motor_pins, GPIO.OUT)
            logger.info(f"Motor GPIO pins initialized: Left({MOTOR_LEFT_IN1},{MOTOR_LEFT_IN2},{MOTOR_LEFT_ENA}), Right({MOTOR_RIGHT_IN3},{MOTOR_RIGHT_IN4},{MOTOR_RIGHT_ENB})")
        except Exception as e:
            logger.error(f"Failed to setup motor GPIO pins: {e}")
            raise
        
        # Audio and Touch pins
        try:
            GPIO.setup([GPIO_I2S_BCK, GPIO_I2S_LRCK, GPIO_I2S_DATA], GPIO.OUT)
            GPIO.setup(GPIO_TOUCH, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
            logger.info(f"Audio/Touch GPIO pins initialized: I2S({GPIO_I2S_BCK},{GPIO_I2S_LRCK},{GPIO_I2S_DATA}), Touch({GPIO_TOUCH})")
        except Exception as e:
            logger.warning(f"Failed to setup audio/touch GPIO pins: {e}")
    
    def _init_motors(self):
        """Initialize PWM for Left and Right channels with validation"""
        if not self.GPIO: return
            
        GPIO = self.GPIO
        
        # Validate PWM-capable pins (ENA and ENB)
        pwm_pins = [MOTOR_LEFT_ENA, MOTOR_RIGHT_ENB]
        for pin in pwm_pins:
            if not (2 <= pin <= 27):
                logger.error(f"Invalid PWM GPIO pin: {pin}")
                raise ValueError(f"Invalid PWM GPIO pin: {pin}")
        
        try:
            self.pwm_motors['LEFT'] = GPIO.PWM(MOTOR_LEFT_ENA, PWM_FREQUENCY)
            self.pwm_motors['RIGHT'] = GPIO.PWM(MOTOR_RIGHT_ENB, PWM_FREQUENCY)
            
            # Pin mapping for logging
            pin_map = {'LEFT': MOTOR_LEFT_ENA, 'RIGHT': MOTOR_RIGHT_ENB}
            
            for channel, pwm in self.pwm_motors.items():
                pwm.start(0)
                logger.info(f"{channel} motor PWM initialized on pin {pin_map[channel]}")
            
            logger.info(f"PWM initialized at {PWM_FREQUENCY}Hz for Left/Right channels")
        except Exception as e:
            logger.error(f"Failed to initialize PWM: {e}")
            raise
    
    def _init_camera(self):
        """Initialize Pi camera with detailed logging"""
        try:
            logger.info("Attempting to initialize camera...")
            from picamera2 import Picamera2
            self.camera = Picamera2()
            config = self.camera.create_preview_configuration(
                main={"size": (640, 480), "format": "RGB888"}
            )
            self.camera.configure(config)
            self.camera_enabled = True
            logger.info("✓ Camera initialized successfully (640x480, RGB888)")
        except ImportError as e:
            logger.warning(f"picamera2 not available - camera features disabled: {e}")
            self.camera_enabled = False
            self.camera = None
        except RuntimeError as e:
            logger.error(f"Camera hardware error - check connection: {e}")
            self.camera_enabled = False
            self.camera = None
        except Exception as e:
            logger.error(f"Failed to initialize camera: {type(e).__name__}: {e}")
            self.camera_enabled = False
            self.camera = None
    
    async def start_camera_stream(self):
        """Start streaming camera frames to server with thread safety"""
        if not self.camera_enabled or not self.camera:
            logger.warning("Camera not available for streaming")
            return
        
        # Prevent multiple streaming sessions
        if self.camera_streaming:
            logger.warning("Camera streaming already active")
            return
        
        self.camera_streaming = True
        camera_started = False
        
        try:
            self.camera.start()
            camera_started = True
            logger.info("✓ Camera streaming started (10 FPS, JPEG quality 70)")
            
            frame_count = 0
            while self.camera_streaming and self.websocket:
                try:
                    # Capture frame
                    frame = self.camera.capture_array()
                    frame_count += 1
                    
                    # Convert to JPEG
                    from PIL import Image
                    img = Image.fromarray(frame)
                    buffer = io.BytesIO()
                    img.save(buffer, format='JPEG', quality=70)
                    buffer.seek(0)
                    
                    # Encode to base64
                    img_base64 = base64.b64encode(buffer.read()).decode('utf-8')
                    
                    # Send to server
                    await self.websocket.send(json.dumps({
                        "type": "camera_frame",
                        "frame": img_base64
                    }))
                    
                    # Limit frame rate to ~10 FPS
                    await asyncio.sleep(0.1)
                    
                except asyncio.CancelledError:
                    logger.info("Camera streaming task cancelled")
                    break
                except Exception as e:
                    logger.error(f"Error streaming camera frame #{frame_count}: {e}")
                    await asyncio.sleep(1)
                    
        except Exception as e:
            logger.error(f"Camera streaming error: {e}")
        finally:
            if camera_started and self.camera:
                try:
                    self.camera.stop()
                    logger.info(f"Camera streaming stopped (streamed {frame_count} frames)")
                except Exception as e:
                    logger.warning(f"Error stopping camera: {e}")
            self.camera_streaming = False
    
    async def stop_camera_stream(self):
        """Stop camera streaming"""
        self.camera_streaming = False
        if self.camera:
            try:
                self.camera.stop()
            except:
                pass

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
            elif msg_type == "start_camera":
                asyncio.create_task(self.start_camera_stream())
            elif msg_type == "stop_camera":
                await self.stop_camera_stream()
            elif msg_type == "speak":
                # Extract voice parameters if provided
                text = data.get("text", "")
                voice = data.get("voice")  # Optional voice parameter
                speed = data.get("speed")  # Optional speed parameter
                pitch = data.get("pitch")  # Optional pitch parameter
                await self.speak_text(text, voice=voice, speed=speed, pitch=pitch)
                
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
        """Play audio (text-to-speech) on Raspberry Pi"""
        try:
            text = audio_data.get('text', '')
            if not text:
                return
            
            logger.info(f"Playing audio: {text}")
            
            # Use espeak for simple TTS (should be installed on Pi)
            # Non-blocking call
            subprocess.Popen(['espeak', text])
            
        except Exception as e:
            logger.error(f"Error playing audio: {e}")
    
    async def speak_text(self, text: str, voice: str = None, speed: int = None, pitch: int = None):
        """
        Text-to-speech on Raspberry Pi with audio output configuration
        
        Args:
            text: Text to speak
            voice: espeak voice (e.g., 'en', 'en+f3', 'en+m3'). Defaults to TTS_VOICE config
            speed: Words per minute (80-450). Defaults to TTS_SPEED config
            pitch: Voice pitch (0-99). Defaults to TTS_PITCH config
        """
        try:
            # Use provided values or defaults from configuration
            voice = voice or TTS_VOICE
            speed = speed or TTS_SPEED
            pitch = pitch or TTS_PITCH
            
            logger.info(f"Speaking: {text} [Voice: {voice}, Speed: {speed}, Pitch: {pitch}]")
            
            # Build espeak command with voice parameters
            espeak_cmd = ['espeak', '-v', voice, '-s', str(speed), '-p', str(pitch), text]
            
            # Set audio output based on configuration
            if AUDIO_OUTPUT_MODE == "gpio_pwm":
                # For GPIO PWM audio output (via transistor circuit)
                # espeak can output to ALSA, which can be configured for GPIO
                subprocess.Popen(espeak_cmd)
            elif AUDIO_OUTPUT_MODE == "i2s":
                # For I2S DAC modules (better quality)
                # Requires I2S configuration in /boot/config.txt
                subprocess.Popen(espeak_cmd)
            else:
                # Default: Use 3.5mm audio jack or HDMI
                # Ensure audio output is set to headphones (not HDMI)
                subprocess.Popen(espeak_cmd)
            
            # Set volume using amixer
            subprocess.Popen(['amixer', 'set', 'Master', f'{AUDIO_VOLUME}%'])
            
        except Exception as e:
            logger.error(f"Error in text-to-speech: {e}")

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
