#!/usr/bin/env python3
"""
AI Pet Robot - Primary Control Server (Port 8000)
Handles robot control, camera, AI integration, and WebSocket connections.
"""

import sys
import os

# Check critical dependencies before importing
try:
    from fastapi import FastAPI, WebSocket
    from fastapi.middleware.cors import CORSMiddleware
    from fastapi.staticfiles import StaticFiles
    from fastapi.responses import FileResponse
except ImportError as e:
    print("\n" + "="*70)
    print("ERROR: Missing required dependencies for Primary Control Server")
    print("="*70)
    print(f"\nMissing module: {e.name if hasattr(e, 'name') else 'unknown'}")
    print("\nTo install required dependencies, run:")
    print("  pip3 install fastapi uvicorn aiohttp python-dotenv")
    print("\nOr install all dependencies:")
    print("  pip3 install -r requirements.txt")
    print("\nFor a complete dependency check, run:")
    print("  python3 check_dependencies.py 8000")
    print("="*70 + "\n")
    sys.exit(1)

try:
    import uvicorn
    import aiohttp
except ImportError as e:
    print("\n" + "="*70)
    print("ERROR: Missing required dependencies for Primary Control Server")
    print("="*70)
    print(f"\nMissing module: {e.name if hasattr(e, 'name') else 'unknown'}")
    print("\nTo install required dependencies, run:")
    print("  pip3 install uvicorn aiohttp")
    print("="*70 + "\n")
    sys.exit(1)

from contextlib import asynccontextmanager
import asyncio
import logging
import uuid
from typing import Dict, Optional
from datetime import datetime

try:
    from dotenv import load_dotenv
except ImportError:
    print("Warning: python-dotenv not installed. Environment variables must be set manually.")
    print("Install with: pip3 install python-dotenv")
    # Define a no-op load_dotenv function
    def load_dotenv():
        pass

try:
    import google.generativeai as genai
except ImportError:
    print("Warning: google-generativeai not installed. AI features will be disabled.")
    print("Install with: pip3 install google-generativeai")
    genai = None

try:
    from gemini_integration import GeminiMultimodalIntegration
    from camera_stream_manager import camera_stream_manager
    from autonomous_navigation import autonomous_navigator
    from smart_home_integration import smart_home, register_example_devices
except ImportError as e:
    print(f"\n" + "="*70)
    print("ERROR: Missing local module dependencies")
    print("="*70)
    print(f"\nFailed to import: {e}")
    print("\nThis usually means missing packages like:")
    print("  - numpy (pip3 install numpy)")
    print("  - opencv-python (pip3 install opencv-python)")
    print("\nRun the dependency checker:")
    print("  python3 check_dependencies.py 8000")
    print("="*70 + "\n")
    sys.exit(1)

load_dotenv()
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
SERVER_HOST = os.getenv("SERVER_HOST", "localhost")

class RobotState:
    def __init__(self):
        self.emotion = "neutral"        # Robot face emotion
        self.user_emotion = "neutral"   # Tracked sentiment of the user (deprecated in v2.2, use current_emotion)
        self.current_emotion = "unknown"  # Current detected user emotion from port 9999
        self.is_listening = False
        self.is_speaking = False
        self.battery_level = 100
        self.connected_clients = []
        self.emotion_display_clients = []  # Clients connected to emotion display
        self.raspberry_pi_client = None
        self.ros_client = None  # ROS bridge client
        self.gemini_session = None
        self.last_transcript = ""
        self.camera_enabled = False
        self.camera_clients = []  # Clients viewing camera feed
        self.control_mode = "manual"  # manual, autonomous, or autonomous_navigation
        self.latest_camera_frame = None
        # Mental health monitoring
        self.user_emotion_history = []  # Track emotion over time
        self.mental_health_insights = []
        self.concern_level = 0  # 0-10 scale
        # Autonomous navigation
        self.navigation_enabled = False
        self.navigation_status = {}
        # Port-specific Gemini control
        self.gemini_enabled_port_8000 = os.getenv("GEMINI_ENABLED_PORT_8000", "true").lower() == "true"
        self.gemini_enabled_port_3000 = os.getenv("GEMINI_ENABLED_PORT_3000", "true").lower() == "true"
        # Task scheduling and reminders
        self.scheduled_tasks = []
        self.reminders = []
        # TTS voice preferences with error handling
        try:
            self.tts_voice = os.getenv("TTS_VOICE", "en+f3")
            self.tts_speed = int(os.getenv("TTS_SPEED", "150"))
            self.tts_pitch = int(os.getenv("TTS_PITCH", "50"))
        except ValueError as e:
            logger.warning(f"Invalid TTS configuration in .env, using defaults: {e}")
            self.tts_voice = "en+f3"
            self.tts_speed = 150
            self.tts_pitch = 50

robot_state = RobotState()

async def initialize_gemini():
    if not GEMINI_API_KEY:
        logger.error("GEMINI_API_KEY not found in environment")
        return
    gemini_integration = GeminiMultimodalIntegration(GEMINI_API_KEY)
    if await gemini_integration.initialize_session():
        robot_state.gemini_session = gemini_integration
        logger.info("Gemini Multimodal API initialized successfully")

async def initialize_navigation():
    """Initialize autonomous navigation system"""
    try:
        # Initialize YOLO model
        success = await autonomous_navigator.initialize_model()
        if success:
            logger.info("Autonomous navigation system initialized successfully")
            
            # Set motor command callback
            async def motor_command_callback(command: str, speed: int):
                if robot_state.raspberry_pi_client:
                    await robot_state.raspberry_pi_client.send_json({
                        "type": "move",
                        "direction": command,
                        "speed": speed
                    })
            
            autonomous_navigator.set_motor_command_callback(motor_command_callback)
            
            # Subscribe to camera stream
            camera_stream_manager.subscribe(autonomous_navigator.process_frame)
            logger.info("Navigation subscribed to camera stream")
        else:
            logger.warning("Failed to initialize navigation - will be available when ultralytics is installed")
    except Exception as e:
        logger.error(f"Error initializing navigation: {e}")

@asynccontextmanager
async def lifespan(app: FastAPI):
    # Initialize Gemini
    await initialize_gemini()
    
    # Initialize autonomous navigation
    await initialize_navigation()
    
    # Initialize smart home integration
    try:
        await smart_home.initialize()
        # Register example devices (for demonstration)
        await register_example_devices()
        logger.info("Smart home integration initialized")
    except Exception as e:
        logger.warning(f"Smart home integration not available: {e}")
    
    # Start background status logging
    async def log_status():
        await asyncio.sleep(10)  # Wait for initial connections
        while True:
            logger.info("📊 Connection Status:")
            logger.info(f"   - Web Clients: {len(robot_state.connected_clients)}")
            logger.info(f"   - Raspberry Pi: {'✓ Connected' if robot_state.raspberry_pi_client else '✗ Not connected'}")
            logger.info(f"   - ROS Bridge: {'✓ Connected' if robot_state.ros_client else '✗ Not connected'}")
            logger.info(f"   - Camera: {'✓ Active' if robot_state.camera_enabled else '○ Inactive'}")
            logger.info(f"   - Control Mode: {robot_state.control_mode.upper()}")
            logger.info(f"   - Navigation: {'✓ Active' if robot_state.navigation_enabled else '○ Inactive'}")
            logger.info(f"   - Mental Health: Concern Level {robot_state.concern_level}/10")
            logger.info(f"   - Smart Home: {'✓ Enabled' if smart_home.enabled else '○ Disabled'}")
            logger.info("")
            await asyncio.sleep(30)  # Log status every 30 seconds
    
    status_task = asyncio.create_task(log_status())
    
    yield
    
    # Cleanup
    status_task.cancel()
    try:
        await status_task
    except asyncio.CancelledError:
        pass
    
    # Close smart home connections
    await smart_home.close()

app = FastAPI(lifespan=lifespan)
app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"])

frontend_dir = os.path.join(os.path.dirname(__file__), "../frontend")
if os.path.exists(frontend_dir):
    app.mount("/static", StaticFiles(directory=frontend_dir), name="static")

@app.websocket("/ws/control")
async def websocket_control(websocket: WebSocket):
    await websocket.accept()
    robot_state.connected_clients.append(websocket)
    try:
        while True:
            data = await websocket.receive_json()
            command_type = data.get('type')
            
            if command_type == 'text':
                await handle_text_command(data)
            elif command_type == 'emotion':
                robot_state.emotion = data.get('emotion', 'neutral')
                await sync_emotion_to_display(robot_state.emotion)
            elif command_type == 'move':
                # Route movement commands based on control mode
                if robot_state.control_mode == 'autonomous' and robot_state.ros_client:
                    # Send to ROS in autonomous mode
                    await robot_state.ros_client.send_json(data)
                elif robot_state.raspberry_pi_client:
                    # Send directly to Pi in manual mode
                    await robot_state.raspberry_pi_client.send_json(data)
            elif command_type == 'start_camera':
                await start_camera()
            elif command_type == 'stop_camera':
                await stop_camera()
            elif command_type == 'subscribe_camera':
                if websocket not in robot_state.camera_clients:
                    robot_state.camera_clients.append(websocket)
            elif command_type == 'unsubscribe_camera':
                if websocket in robot_state.camera_clients:
                    robot_state.camera_clients.remove(websocket)
            elif command_type == 'set_mode':
                mode = data.get('mode', 'manual')
                robot_state.control_mode = mode
                # Notify ROS bridge of mode change
                if robot_state.ros_client:
                    await robot_state.ros_client.send_json({
                        'type': 'set_mode',
                        'mode': mode
                    })
            elif command_type == 'start_listening':
                robot_state.is_listening = True
                await broadcast_state()
            elif command_type == 'stop_listening':
                robot_state.is_listening = False
                await broadcast_state()
            elif command_type == 'start_navigation':
                # Start autonomous navigation
                if await autonomous_navigator.start_navigation():
                    robot_state.navigation_enabled = True
                    robot_state.control_mode = 'autonomous_navigation'
                    logger.info("Autonomous navigation started")
                await broadcast_state()
            elif command_type == 'stop_navigation':
                # Stop autonomous navigation
                await autonomous_navigator.stop_navigation()
                robot_state.navigation_enabled = False
                if robot_state.control_mode == 'autonomous_navigation':
                    robot_state.control_mode = 'manual'
                logger.info("Autonomous navigation stopped")
                await broadcast_state()
            
            await broadcast_state()
    except:
        if websocket in robot_state.connected_clients:
            robot_state.connected_clients.remove(websocket)
        if websocket in robot_state.camera_clients:
            robot_state.camera_clients.remove(websocket)

@app.websocket("/ws/raspberry_pi")
async def websocket_raspberry_pi(websocket: WebSocket):
    await websocket.accept()
    robot_state.raspberry_pi_client = websocket
    logger.info("Raspberry Pi hardware connected")
    try:
        while True:
            data = await websocket.receive_json()
            msg_type = data.get('type')
            
            if msg_type == 'camera_frame':
                # Store frame in robot state
                robot_state.latest_camera_frame = data.get('frame')
                
                # Publish to camera stream manager for shared consumption
                await camera_stream_manager.publish_frame({
                    'frame': data.get('frame'),
                    'timestamp': datetime.now().isoformat()
                })
                
                # Broadcast camera frame to subscribed clients (legacy support)
                for client in robot_state.camera_clients:
                    try:
                        await client.send_json({
                            'type': 'camera_frame',
                            'frame': data.get('frame')
                        })
                    except:
                        pass
    except:
        robot_state.raspberry_pi_client = None
        logger.info("Raspberry Pi disconnected")

@app.websocket("/ws/ros")
async def websocket_ros(websocket: WebSocket):
    """WebSocket endpoint for ROS bridge connection"""
    await websocket.accept()
    robot_state.ros_client = websocket
    logger.info("ROS bridge connected")
    try:
        while True:
            data = await websocket.receive_json()
            msg_type = data.get('type')
            
            # Handle ROS state updates
            if msg_type == 'ros_state':
                # Broadcast ROS state to connected clients
                for client in robot_state.connected_clients:
                    try:
                        await client.send_json({
                            'type': 'ros_update',
                            'data': data.get('data')
                        })
                    except:
                        pass
    except:
        robot_state.ros_client = None
        logger.info("ROS bridge disconnected")

@app.websocket("/ws/emotion_display")
async def websocket_emotion_display(websocket: WebSocket):
    """WebSocket endpoint for emotion display clients on port 8000"""
    await websocket.accept()
    robot_state.emotion_display_clients.append(websocket)
    logger.info(f"Emotion display client connected. Total: {len(robot_state.emotion_display_clients)}")
    
    # Send current emotion immediately
    try:
        await websocket.send_json({
            'type': 'emotion_update',
            'emotion': robot_state.emotion
        })
    except Exception as e:
        logger.error(f"Failed to send initial emotion to client: {e}")
    
    try:
        while True:
            # Keep connection alive and listen for any messages
            await websocket.receive_text()
    except asyncio.CancelledError:
        logger.info("WebSocket connection cancelled")
        raise
    except Exception as e:
        logger.debug(f"WebSocket disconnected: {e}")
    finally:
        # Ensure proper cleanup
        if websocket in robot_state.emotion_display_clients:
            robot_state.emotion_display_clients.remove(websocket)
        logger.info(f"Emotion display client disconnected. Total: {len(robot_state.emotion_display_clients)}")

async def start_camera():
    """Start camera streaming from Raspberry Pi"""
    if robot_state.raspberry_pi_client:
        await robot_state.raspberry_pi_client.send_json({"type": "start_camera"})
        robot_state.camera_enabled = True
        logger.info("Camera streaming started")

async def stop_camera():
    """Stop camera streaming from Raspberry Pi"""
    if robot_state.raspberry_pi_client:
        await robot_state.raspberry_pi_client.send_json({"type": "stop_camera"})
        robot_state.camera_enabled = False
        logger.info("Camera streaming stopped")

async def query_emotion_detection() -> str:
    """Query emotion detection server (port 9999) for current user emotion"""
    try:
        async with aiohttp.ClientSession() as session:
            async with session.get(
                f'http://{SERVER_HOST}:9999/api/emotion',
                timeout=aiohttp.ClientTimeout(total=2)
            ) as resp:
                if resp.status == 200:
                    data = await resp.json()
                    emotion = data.get('current_emotion', 'unknown')
                    logger.info(f"Detected user emotion from port 9999: {emotion}")
                    return emotion
                else:
                    logger.warning(f"Emotion detection API returned status {resp.status}")
                    return "unknown"
    except asyncio.TimeoutError:
        logger.warning("Emotion detection query timed out")
        return "unknown"
    except Exception as e:
        logger.debug(f"Could not query emotion detection: {e}")
        return "unknown"

async def handle_text_command(data: Dict):
    text = data.get('text', '')
    if not text: return
    
    # Query emotion detection from port 9999
    detected_emotion = await query_emotion_detection()
    robot_state.current_emotion = detected_emotion
    
    # Update legacy field for backward compatibility
    if detected_emotion != "unknown":
        robot_state.user_emotion = detected_emotion
    else:
        # Fallback to text-based emotion detection
        robot_state.user_emotion = detect_emotion(text)
        robot_state.current_emotion = robot_state.user_emotion
    
    # Track emotion history for mental health monitoring
    robot_state.user_emotion_history.append(robot_state.current_emotion)
    if len(robot_state.user_emotion_history) > 50:  # Keep last 50 emotions
        robot_state.user_emotion_history.pop(0)
    
    # Analyze mental state
    mental_state = analyze_mental_state()
    if mental_state == "concerning" and robot_state.concern_level >= 7:
        robot_state.mental_health_insights.append({
            'timestamp': datetime.now().isoformat(),
            'level': 'high',
            'message': 'User showing consistent negative emotions - consider professional support'
        })
    
    robot_state.is_listening = True
    await broadcast_state()
    
    # Check if Gemini is enabled for port 8000
    if robot_state.gemini_session and robot_state.gemini_enabled_port_8000:
        robot_state.is_speaking = True
        await broadcast_state()
        
        # Update Gemini with current user emotion
        robot_state.gemini_session.set_user_emotion(robot_state.current_emotion)
        
        # Send text with emotion context
        response = await robot_state.gemini_session.send_text(text, include_emotion_context=True)
        if response:
            response_text = response['text']
            # Robot reacts with its own detected emotion
            robot_state.emotion = detect_emotion(response_text)
            robot_state.last_transcript = f"User: {text}\nRobot: {response_text}"
            await sync_emotion_to_display(robot_state.emotion)
            
            # Broadcast speech to all ports and devices
            await broadcast_tts_to_all_ports(response_text)
        
        robot_state.is_speaking = False
    elif not robot_state.gemini_enabled_port_8000:
        logger.info("Gemini disabled on port 8000, skipping AI response")
    
    robot_state.is_listening = False
    await broadcast_state()

async def sync_emotion_to_display(emotion: str):
    """Synchronize emotion to Raspberry Pi, local emotion display clients, and emotion display server (port 10000)"""
    # Send to Raspberry Pi
    if robot_state.raspberry_pi_client:
        await robot_state.raspberry_pi_client.send_json({"type": "emotion", "emotion": emotion})
    
    # Broadcast to local emotion display clients (port 8000)
    disconnected_clients = []
    for client in robot_state.emotion_display_clients:
        try:
            await client.send_json({
                'type': 'emotion_update',
                'emotion': emotion
            })
        except Exception as e:
            logger.error(f"Failed to send to emotion display client: {e}")
            disconnected_clients.append(client)
    
    # Clean up disconnected clients
    for client in disconnected_clients:
        if client in robot_state.emotion_display_clients:
            robot_state.emotion_display_clients.remove(client)
    
    # Send to emotion display server (port 10000)
    try:
        async with aiohttp.ClientSession() as session:
            async with session.post(
                f'http://{SERVER_HOST}:10000/api/emotion',
                json={'emotion': emotion},
                timeout=aiohttp.ClientTimeout(total=1)
            ) as resp:
                if resp.status == 200:
                    logger.debug(f"Emotion synced to display server: {emotion}")
    except Exception as e:
        logger.debug(f"Could not sync to emotion display server: {e}")

async def broadcast_tts_to_all_ports(text: str, voice: str = None, speed: int = None, pitch: int = None):
    """
    Broadcast TTS (text-to-speech) to all connected ports and devices
    - Sends to Raspberry Pi for hardware TTS output
    - Sends to emotion detection server (port 9999)
    - Sends to mobile web server (port 3000)
    - Sends to emotion display server (port 10000)
    - Broadcasts to all connected WebSocket clients on port 8000
    """
    tts_message = {
        "type": "speak",
        "text": text
    }
    
    # Add voice parameters using defaults from robot_state if not provided
    tts_message['voice'] = voice or robot_state.tts_voice
    tts_message['speed'] = speed or robot_state.tts_speed
    tts_message['pitch'] = pitch or robot_state.tts_pitch
    
    # Send to Raspberry Pi (hardware speaker)
    if robot_state.raspberry_pi_client:
        try:
            await robot_state.raspberry_pi_client.send_json(tts_message)
            logger.info(f"TTS sent to Raspberry Pi: {text}")
        except Exception as e:
            logger.error(f"Failed to send TTS to Raspberry Pi: {e}")
    
    # Broadcast to all connected WebSocket clients on port 8000
    disconnected_clients = []
    for client in robot_state.connected_clients:
        try:
            await client.send_json({
                'type': 'tts_output',
                'text': text,
                'voice': tts_message['voice'],
                'speed': tts_message['speed'],
                'pitch': tts_message['pitch']
            })
        except Exception as e:
            logger.debug(f"Failed to send TTS to client: {e}")
            disconnected_clients.append(client)
    
    # Clean up disconnected clients
    for client in disconnected_clients:
        if client in robot_state.connected_clients:
            robot_state.connected_clients.remove(client)
    
    # Send to emotion detection server (port 9999)
    try:
        async with aiohttp.ClientSession() as session:
            async with session.post(
                f'http://{SERVER_HOST}:9999/api/tts',
                json=tts_message,
                timeout=aiohttp.ClientTimeout(total=1)
            ) as resp:
                if resp.status == 200:
                    logger.debug(f"TTS sent to emotion detection server (9999)")
    except Exception as e:
        logger.debug(f"Could not send TTS to emotion detection server: {e}")
    
    # Send to emotion display server (port 10000)
    try:
        async with aiohttp.ClientSession() as session:
            async with session.post(
                f'http://{SERVER_HOST}:10000/api/tts',
                json=tts_message,
                timeout=aiohttp.ClientTimeout(total=1)
            ) as resp:
                if resp.status == 200:
                    logger.debug(f"TTS sent to emotion display server (10000)")
    except Exception as e:
        logger.debug(f"Could not send TTS to emotion display server: {e}")
    
    # Send to mobile web server (port 3000)
    try:
        async with aiohttp.ClientSession() as session:
            async with session.post(
                f'http://{SERVER_HOST}:3000/api/tts',
                json=tts_message,
                timeout=aiohttp.ClientTimeout(total=1)
            ) as resp:
                if resp.status == 200:
                    logger.debug(f"TTS sent to mobile web server (3000)")
    except Exception as e:
        logger.debug(f"Could not send TTS to mobile web server: {e}")

def detect_emotion(text: str) -> str:
    """Detect emotion from text with enhanced mental health awareness"""
    text = text.lower()
    
    # Check for crisis keywords
    crisis_keywords = ['suicide', 'kill myself', 'want to die', 'end it all', 'no reason to live']
    if any(keyword in text for keyword in crisis_keywords):
        robot_state.concern_level = 10
        robot_state.mental_health_insights.append({
            'timestamp': datetime.now().isoformat(),
            'level': 'critical',
            'message': 'Crisis keywords detected - immediate intervention needed'
        })
        return 'sad'
    
    # Detect various emotions with mental health tracking
    if any(w in text for w in ['sad', 'cry', 'crying', 'lonely', 'hurt', 'depressed', 'unhappy', 'hopeless', 'worthless']):
        robot_state.concern_level = min(robot_state.concern_level + 1, 10)
        return 'sad'
    if any(w in text for w in ['anxious', 'anxiety', 'worried', 'scared', 'panic', 'stressed', 'overwhelmed']):
        robot_state.concern_level = min(robot_state.concern_level + 1, 10)
        return 'sad'
    if any(w in text for w in ['angry', 'mad', 'hate', 'furious', 'annoyed', 'frustrated']):
        return 'angry'
    if any(w in text for w in ['happy', 'good', 'yay', 'great', 'awesome', 'excited', 'wonderful', 'amazing']):
        # Positive emotion - reduce concern level
        robot_state.concern_level = max(robot_state.concern_level - 1, 0)
        return 'happy'
    
    return 'neutral'

def analyze_mental_state():
    """Analyze user's mental state from emotion history"""
    if len(robot_state.user_emotion_history) < 5:
        return "insufficient_data"
    
    # Check recent emotions
    recent = robot_state.user_emotion_history[-10:]
    negative_count = sum(1 for e in recent if e in ['sad', 'angry'])
    
    if negative_count >= 7:
        return "concerning"
    elif negative_count >= 5:
        return "monitoring"
    else:
        return "stable"

async def broadcast_state():
    state = {
        'emotion': robot_state.emotion,
        'user_emotion': robot_state.user_emotion,
        'battery_level': robot_state.battery_level,
        'is_listening': robot_state.is_listening,
        'is_speaking': robot_state.is_speaking,
        'last_transcript': robot_state.last_transcript,
        'camera_enabled': robot_state.camera_enabled,
        'control_mode': robot_state.control_mode,
        'raspberry_pi_connected': robot_state.raspberry_pi_client is not None,
        'ros_connected': robot_state.ros_client is not None,
        'navigation_enabled': robot_state.navigation_enabled,
        'navigation_status': autonomous_navigator.get_status() if robot_state.navigation_enabled else {}
    }
    for client in robot_state.connected_clients:
        try:
            await client.send_json({'type': 'state', 'data': state})
        except:
            pass

@app.get("/")
async def root():
    """Serve the emotion display page (emotion-only, no controls)"""
    emotion_display_path = os.path.join(frontend_dir, "emotion_display.html")
    if os.path.exists(emotion_display_path):
        return FileResponse(emotion_display_path)
    # Fallback to face_display.html if emotion_display.html doesn't exist
    return FileResponse(os.path.join(frontend_dir, "face_display.html"))

@app.get("/control")
async def control_panel():
    """Serve the full control panel with camera, audio controls, etc."""
    return FileResponse(os.path.join(frontend_dir, "face_display.html"))

@app.get("/health")
async def health_check():
    """Health check endpoint for monitoring"""
    return {
        'status': 'healthy',
        'server': 'running',
        'components': {
            'gemini_ai': robot_state.gemini_session is not None,
            'raspberry_pi': robot_state.raspberry_pi_client is not None,
            'ros_bridge': robot_state.ros_client is not None,
            'camera': robot_state.camera_enabled,
            'web_clients': len(robot_state.connected_clients)
        }
    }

@app.get("/api/status")
async def get_full_status():
    """Get comprehensive system status"""
    return {
        'server': {
            'running': True,
            'version': '2.0',
            'uptime': 'running'
        },
        'connections': {
            'web_clients': len(robot_state.connected_clients),
            'raspberry_pi': {
                'connected': robot_state.raspberry_pi_client is not None,
                'camera_enabled': robot_state.camera_enabled,
                'features': ['motor_control', 'camera', 'tts']
            },
            'ros_bridge': {
                'connected': robot_state.ros_client is not None,
                'mode': robot_state.control_mode,
                'features': ['autonomous_navigation', 'obstacle_avoidance']
            }
        },
        'ai': {
            'gemini': {
                'initialized': robot_state.gemini_session is not None,
                'status': 'ready' if robot_state.gemini_session else 'not_configured'
            }
        },
        'mental_health': {
            'tracking_enabled': True,
            'concern_level': robot_state.concern_level,
            'emotions_tracked': len(robot_state.user_emotion_history),
            'insights_available': len(robot_state.mental_health_insights)
        },
        'features': {
            'camera_streaming': {
                'available': robot_state.raspberry_pi_client is not None,
                'active': robot_state.camera_enabled,
                'clients': len(robot_state.camera_clients)
            },
            'speech': {
                'tts_available': robot_state.raspberry_pi_client is not None,
                'listening': robot_state.is_listening,
                'speaking': robot_state.is_speaking
            },
            'control': {
                'mode': robot_state.control_mode,
                'manual_available': robot_state.raspberry_pi_client is not None,
                'autonomous_available': robot_state.ros_client is not None
            }
        },
        'setup_instructions': {
            'raspberry_pi': 'Run: cd hardware/raspberry_pi && python3 raspberry_pi_controller.py' if not robot_state.raspberry_pi_client else 'Connected ✓',
            'ros_bridge': 'Run: roslaunch pet_robot_ros ros_bridge.launch' if not robot_state.ros_client else 'Connected ✓'
        }
    }

@app.get("/api/state")
async def get_state():
    """Get current robot state"""
    return {
        'emotion': robot_state.emotion,
        'user_emotion': robot_state.user_emotion,
        'battery_level': robot_state.battery_level,
        'is_listening': robot_state.is_listening,
        'is_speaking': robot_state.is_speaking,
        'last_transcript': robot_state.last_transcript,
        'camera_enabled': robot_state.camera_enabled,
        'control_mode': robot_state.control_mode,
        'raspberry_pi_connected': robot_state.raspberry_pi_client is not None,
        'ros_connected': robot_state.ros_client is not None
    }

@app.post("/api/control_mode")
async def set_control_mode(data: Dict):
    """Set control mode (manual or autonomous)"""
    mode = data.get('mode', 'manual')
    robot_state.control_mode = mode
    await broadcast_state()
    return {"status": "ok", "mode": mode}

@app.post("/api/speak")
async def speak_command(data: Dict):
    """
    Send text to be spoken by robot with optional voice customization
    Voice input is received on port 8000 and broadcast to all ports
    
    Parameters:
    - text (str): Text to speak (required)
    - voice (str, optional): Voice type (e.g., 'en', 'en+f3', 'en+m3')
    - speed (int, optional): Speech speed in words per minute (80-450)
    - pitch (int, optional): Voice pitch (0-99)
    
    Examples:
    - {"text": "Hello", "voice": "en+f3", "speed": 150, "pitch": 50}
    - {"text": "Hello"} # Uses default voice settings
    """
    text = data.get('text', '')
    if text:
        # Extract voice parameters
        voice = data.get('voice')
        speed = data.get('speed')
        pitch = data.get('pitch')
        
        # Broadcast TTS to all ports and devices
        await broadcast_tts_to_all_ports(text, voice=voice, speed=speed, pitch=pitch)
        
        return {
            "status": "ok",
            "text": text,
            "broadcast": "all_ports",
            "voice_params": {
                "voice": voice or robot_state.tts_voice,
                "speed": speed or robot_state.tts_speed,
                "pitch": pitch or robot_state.tts_pitch
            }
        }
    return {"status": "error", "message": "No text provided"}

@app.get("/api/tts/voices")
async def get_available_voices():
    """
    Get available TTS voices and current settings
    Returns list of supported espeak voices and current configuration
    """
    available_voices = [
        {"id": "en", "name": "English Male (Default)", "gender": "male", "description": "Standard English male voice"},
        {"id": "en+f1", "name": "English Female 1", "gender": "female", "description": "Light female voice"},
        {"id": "en+f2", "name": "English Female 2", "gender": "female", "description": "Medium female voice"},
        {"id": "en+f3", "name": "English Female 3", "gender": "female", "description": "Standard female voice (recommended)"},
        {"id": "en+f4", "name": "English Female 4", "gender": "female", "description": "Deeper female voice"},
        {"id": "en+m1", "name": "English Male 1", "gender": "male", "description": "Light male voice"},
        {"id": "en+m2", "name": "English Male 2", "gender": "male", "description": "Medium male voice"},
        {"id": "en+m3", "name": "English Male 3", "gender": "male", "description": "Standard male voice"},
        {"id": "en+m4", "name": "English Male 4", "gender": "male", "description": "Deep male voice"},
        {"id": "en+m7", "name": "English Male 7", "gender": "male", "description": "Very deep male voice"},
        {"id": "en-us", "name": "English US", "gender": "male", "description": "American English accent"},
        {"id": "en-uk", "name": "English UK", "gender": "male", "description": "British English accent"},
        {"id": "en-scottish", "name": "English Scottish", "gender": "male", "description": "Scottish English accent"}
    ]
    
    return {
        "available_voices": available_voices,
        "current_settings": {
            "voice": robot_state.tts_voice,
            "speed": robot_state.tts_speed,
            "pitch": robot_state.tts_pitch
        },
        "parameter_ranges": {
            "speed": {"min": 80, "max": 450, "default": 150, "description": "Words per minute"},
            "pitch": {"min": 0, "max": 99, "default": 50, "description": "Voice pitch"}
        }
    }

@app.post("/api/tts/settings")
async def update_tts_settings(data: Dict):
    """
    Update default TTS voice settings
    These settings will be used for all future TTS requests unless overridden
    
    Parameters:
    - voice (str, optional): Default voice ID
    - speed (int, optional): Default speed (80-450)
    - pitch (int, optional): Default pitch (0-99)
    """
    updated = {}
    
    if 'voice' in data:
        robot_state.tts_voice = data['voice']
        updated['voice'] = robot_state.tts_voice
    
    if 'speed' in data:
        try:
            speed = max(80, min(450, int(data['speed'])))
            robot_state.tts_speed = speed
            updated['speed'] = robot_state.tts_speed
        except (ValueError, TypeError) as e:
            return {"status": "error", "message": f"Invalid speed value: {data['speed']}"}
    
    if 'pitch' in data:
        try:
            pitch = max(0, min(99, int(data['pitch'])))
            robot_state.tts_pitch = pitch
            updated['pitch'] = robot_state.tts_pitch
        except (ValueError, TypeError) as e:
            return {"status": "error", "message": f"Invalid pitch value: {data['pitch']}"}
    
    if updated:
        logger.info(f"TTS settings updated: {updated}")
        return {
            "status": "ok",
            "updated": updated,
            "current_settings": {
                "voice": robot_state.tts_voice,
                "speed": robot_state.tts_speed,
                "pitch": robot_state.tts_pitch
            }
        }
    
    return {"status": "error", "message": "No valid parameters provided"}

@app.get("/api/mental_health/insights")
async def get_mental_health_insights():
    """Get mental health monitoring insights"""
    mental_state = analyze_mental_state()
    return {
        'mental_state': mental_state,
        'concern_level': robot_state.concern_level,
        'recent_emotions': robot_state.user_emotion_history[-10:] if robot_state.user_emotion_history else [],
        'insights': robot_state.mental_health_insights[-5:] if robot_state.mental_health_insights else [],
        'recommendation': get_recommendation(mental_state, robot_state.concern_level)
    }

@app.post("/api/navigation/start")
async def start_autonomous_navigation():
    """Start autonomous navigation mode"""
    if not autonomous_navigator.model:
        return {
            'success': False,
            'message': 'Navigation model not loaded. Please ensure ultralytics is installed.'
        }
    
    success = await autonomous_navigator.start_navigation()
    if success:
        robot_state.navigation_enabled = True
        robot_state.control_mode = 'autonomous_navigation'
        # Ensure camera is enabled
        if not robot_state.camera_enabled:
            await start_camera()
    
    return {
        'success': success,
        'message': 'Autonomous navigation started' if success else 'Failed to start navigation',
        'status': autonomous_navigator.get_status()
    }

@app.post("/api/navigation/stop")
async def stop_autonomous_navigation():
    """Stop autonomous navigation mode"""
    await autonomous_navigator.stop_navigation()
    robot_state.navigation_enabled = False
    if robot_state.control_mode == 'autonomous_navigation':
        robot_state.control_mode = 'manual'
    
    return {
        'success': True,
        'message': 'Autonomous navigation stopped',
        'status': autonomous_navigator.get_status()
    }

@app.get("/api/navigation/status")
async def get_navigation_status():
    """Get autonomous navigation status"""
    return {
        'navigation': autonomous_navigator.get_status(),
        'parameters': autonomous_navigator.get_parameters(),
        'camera_stream': camera_stream_manager.get_stats()
    }

@app.post("/api/navigation/parameters")
async def update_navigation_parameters(params: Dict):
    """Update navigation parameters"""
    autonomous_navigator.update_parameters(params)
    return {
        'success': True,
        'message': 'Parameters updated',
        'parameters': autonomous_navigator.get_parameters()
    }

def get_recommendation(mental_state: str, concern_level: int) -> str:
    """Get recommendation based on mental state"""
    if concern_level >= 8:
        return "High concern detected. Please consider reaching out to a mental health professional or crisis hotline immediately."
    elif concern_level >= 5:
        return "Moderate concern. Consider talking to someone you trust or a mental health professional."
    elif mental_state == "concerning":
        return "Showing signs of distress. Self-care activities and reaching out to friends may help."
    elif mental_state == "monitoring":
        return "Keep monitoring your mood. Engage in activities you enjoy."
    else:
        return "You're doing well! Keep up the positive habits."

# New API endpoints for industrial-level features

@app.post("/api/gemini/control")
async def control_gemini(data: Dict):
    """Enable or disable Gemini on specific ports"""
    port = data.get('port')
    enabled = data.get('enabled', True)
    
    if port == 8000:
        robot_state.gemini_enabled_port_8000 = enabled
        logger.info(f"Gemini on port 8000: {'enabled' if enabled else 'disabled'}")
    elif port == 3000:
        robot_state.gemini_enabled_port_3000 = enabled
        logger.info(f"Gemini on port 3000: {'enabled' if enabled else 'disabled'}")
    else:
        return {"status": "error", "message": "Invalid port. Use 8000 or 3000"}
    
    await broadcast_state()
    return {
        "status": "ok",
        "port": port,
        "enabled": enabled,
        "message": f"Gemini {'enabled' if enabled else 'disabled'} on port {port}"
    }

@app.get("/api/gemini/status")
async def get_gemini_status():
    """Get Gemini status for all ports"""
    return {
        "initialized": robot_state.gemini_session is not None,
        "port_8000_enabled": robot_state.gemini_enabled_port_8000,
        "port_3000_enabled": robot_state.gemini_enabled_port_3000,
        "current_user_emotion": robot_state.current_emotion
    }

@app.post("/api/tasks/schedule")
async def schedule_task(data: Dict):
    """Schedule a task or reminder"""
    task = {
        'id': str(uuid.uuid4()),
        'type': data.get('type', 'task'),
        'description': data.get('description', ''),
        'time': data.get('time', ''),
        'created_at': datetime.now().isoformat(),
        'status': 'pending'
    }
    
    if task['type'] == 'reminder':
        robot_state.reminders.append(task)
    else:
        robot_state.scheduled_tasks.append(task)
    
    logger.info(f"Scheduled {task['type']}: {task['description']}")
    return {"status": "ok", "task": task}

@app.get("/api/tasks/list")
async def list_tasks():
    """List all scheduled tasks and reminders"""
    return {
        "tasks": robot_state.scheduled_tasks,
        "reminders": robot_state.reminders,
        "total": len(robot_state.scheduled_tasks) + len(robot_state.reminders)
    }

@app.delete("/api/tasks/{task_id}")
async def delete_task(task_id: str):
    """Delete a scheduled task or reminder"""
    # Remove from both lists
    initial_task_count = len(robot_state.scheduled_tasks)
    initial_reminder_count = len(robot_state.reminders)
    
    robot_state.scheduled_tasks = [t for t in robot_state.scheduled_tasks if t.get('id') != task_id]
    robot_state.reminders = [r for r in robot_state.reminders if r.get('id') != task_id]
    
    deleted = (len(robot_state.scheduled_tasks) < initial_task_count or 
               len(robot_state.reminders) < initial_reminder_count)
    
    if deleted:
        logger.info(f"Deleted task: {task_id}")
        return {"status": "ok", "message": "Task deleted"}
    else:
        return {"status": "error", "message": "Task not found"}

@app.post("/api/query")
async def information_query(data: Dict):
    """Process information retrieval query through Gemini"""
    query = data.get('query', '')
    if not query:
        return {"status": "error", "message": "No query provided"}
    
    if not robot_state.gemini_session or not robot_state.gemini_enabled_port_8000:
        return {"status": "error", "message": "Gemini not available"}
    
    try:
        # Format as information retrieval request
        formatted_query = f"[Information Request] {query}"
        response = await robot_state.gemini_session.send_text(formatted_query, include_emotion_context=False)
        
        if response:
            return {
                "status": "ok",
                "query": query,
                "response": response['text']
            }
        else:
            return {"status": "error", "message": "Failed to get response"}
    except Exception as e:
        logger.error(f"Error processing query: {e}")
        return {"status": "error", "message": str(e)}

@app.get("/api/emotion/current")
async def get_current_emotion():
    """Get current detected user emotion"""
    return {
        "current_emotion": robot_state.current_emotion,
        "robot_emotion": robot_state.emotion,
        "last_updated": datetime.now().isoformat()
    }

# Smart Home Integration API Endpoints

@app.get("/api/smarthome/devices")
async def list_smart_devices():
    """List all registered smart home devices"""
    try:
        devices = smart_home.list_devices()
        return {
            "status": "ok",
            "devices": devices,
            "count": len(devices),
            "enabled": smart_home.enabled
        }
    except Exception as e:
        logger.error(f"Error listing devices: {e}")
        return {"status": "error", "message": str(e)}

@app.get("/api/smarthome/device/{device_id}")
async def get_device_state(device_id: str):
    """Get state of a specific smart home device"""
    try:
        state = await smart_home.get_device_state(device_id)
        if state:
            return {"status": "ok", "device": state}
        else:
            return {"status": "error", "message": "Device not found"}
    except Exception as e:
        logger.error(f"Error getting device state: {e}")
        return {"status": "error", "message": str(e)}

@app.post("/api/smarthome/control")
async def control_smart_device(data: Dict):
    """
    Control a smart home device
    
    Body:
        device_id: str - Device identifier
        command: str - Command to execute
        parameters: dict - Optional command parameters
    """
    try:
        device_id = data.get("device_id")
        command = data.get("command")
        parameters = data.get("parameters", {})
        
        if not device_id or not command:
            return {
                "status": "error",
                "message": "Missing device_id or command"
            }
        
        result = await smart_home.control_device(device_id, command, parameters)
        logger.info(f"Smart home control: {device_id} -> {command}")
        return result
        
    except Exception as e:
        logger.error(f"Error controlling device: {e}")
        return {"status": "error", "message": str(e)}

@app.post("/api/smarthome/scene")
async def execute_smart_scene(data: Dict):
    """
    Execute a smart home scene
    
    Body:
        scene_name: str - Name of the scene
        devices: dict - Device commands mapping
    """
    try:
        scene_name = data.get("scene_name", "unnamed_scene")
        devices = data.get("devices", {})
        
        if not devices:
            return {
                "status": "error",
                "message": "No devices specified in scene"
            }
        
        result = await smart_home.execute_scene(scene_name, devices)
        logger.info(f"Executed scene: {scene_name}")
        return result
        
    except Exception as e:
        logger.error(f"Error executing scene: {e}")
        return {"status": "error", "message": str(e)}

if __name__ == "__main__":
    import sys
    
    logger.info("=" * 60)
    logger.info("AI Pet Robot Server Starting...")
    logger.info("=" * 60)
    logger.info("")
    logger.info("🚀 Server Configuration:")
    logger.info(f"   - Host: 0.0.0.0")
    logger.info(f"   - Port: 8000")
    logger.info(f"   - Gemini API: {'✓ Configured' if GEMINI_API_KEY else '✗ Not configured'}")
    logger.info("")
    logger.info("📡 WebSocket Endpoints:")
    logger.info("   - /ws/control     → Web/Mobile clients")
    logger.info("   - /ws/raspberry_pi → Raspberry Pi hardware")
    logger.info("   - /ws/ros         → ROS bridge (autonomous mode)")
    logger.info("")
    logger.info("🌐 Web Interface:")
    logger.info("   - Main UI: http://localhost:8000")
    logger.info("   - Features: Camera, Speech, Emotions, Controls")
    logger.info("")
    logger.info("📹 Camera Streaming:")
    logger.info("   - Requires Raspberry Pi connection")
    logger.info("   - Toggle in web interface or mobile app")
    logger.info("   - Real-time JPEG streaming at ~10 FPS")
    logger.info("")
    logger.info("🔊 Audio/Speech:")
    logger.info("   - Text-to-Speech on Raspberry Pi (espeak)")
    logger.info("   - Automatic speech when AI responds")
    logger.info("   - Toggle controls in web interface")
    logger.info("")
    logger.info("🤖 ROS Integration:")
    logger.info("   - Launch ROS bridge separately:")
    logger.info("     roslaunch pet_robot_ros ros_bridge.launch")
    logger.info("   - Switch modes in web interface:")
    logger.info("     • Manual → Direct Pi control")
    logger.info("     • Autonomous → ROS navigation")
    logger.info("")
    logger.info("🧠 Mental Health Monitoring:")
    logger.info("   - Real-time emotion tracking")
    logger.info("   - Crisis detection enabled")
    logger.info("   - API: GET /api/mental_health/insights")
    logger.info("")
    logger.info("📱 Mobile App:")
    logger.info("   - Configure server IP in app settings")
    logger.info("   - All features available remotely")
    logger.info("")
    logger.info("⚙️  To Enable All Features:")
    logger.info("   1. Start this server (you're doing it now!)")
    logger.info("   2. On Raspberry Pi:")
    logger.info("      cd hardware/raspberry_pi")
    logger.info("      python3 raspberry_pi_controller.py")
    logger.info("   3. For autonomous mode (optional):")
    logger.info("      roslaunch pet_robot_ros ros_bridge.launch")
    logger.info("")
    logger.info("=" * 60)
    logger.info("Server is ready! Open http://localhost:8000 in your browser")
    logger.info("=" * 60)
    logger.info("")
    
    uvicorn.run(app, host="0.0.0.0", port=8000)