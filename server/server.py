from fastapi import FastAPI, WebSocket
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse
from contextlib import asynccontextmanager
import asyncio
import logging
import os
from typing import Dict
from dotenv import load_dotenv
import google.generativeai as genai
import uvicorn
from gemini_integration import GeminiMultimodalIntegration

load_dotenv()
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")

class RobotState:
    def __init__(self):
        self.emotion = "neutral"        # Robot face emotion
        self.user_emotion = "neutral"   # Tracked sentiment of the user
        self.is_listening = False
        self.is_speaking = False
        self.battery_level = 100
        self.connected_clients = []
        self.raspberry_pi_client = None
        self.ros_client = None  # ROS bridge client
        self.gemini_session = None
        self.last_transcript = ""
        self.camera_enabled = False
        self.camera_clients = []  # Clients viewing camera feed
        self.control_mode = "manual"  # manual or autonomous
        self.latest_camera_frame = None
        # Mental health monitoring
        self.user_emotion_history = []  # Track emotion over time
        self.mental_health_insights = []
        self.concern_level = 0  # 0-10 scale

robot_state = RobotState()

async def initialize_gemini():
    if not GEMINI_API_KEY:
        logger.error("GEMINI_API_KEY not found in environment")
        return
    gemini_integration = GeminiMultimodalIntegration(GEMINI_API_KEY)
    if await gemini_integration.initialize_session():
        robot_state.gemini_session = gemini_integration
        logger.info("Gemini Multimodal API initialized successfully")

@asynccontextmanager
async def lifespan(app: FastAPI):
    await initialize_gemini()
    yield

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
                # Broadcast camera frame to subscribed clients
                robot_state.latest_camera_frame = data.get('frame')
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

async def handle_text_command(data: Dict):
    text = data.get('text', '')
    if not text: return
    
    # Detect User Emotion from their input
    robot_state.user_emotion = detect_emotion(text)
    
    # Track emotion history for mental health monitoring
    from datetime import datetime
    robot_state.user_emotion_history.append(robot_state.user_emotion)
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
    
    if robot_state.gemini_session:
        robot_state.is_speaking = True
        await broadcast_state()
        
        response = await robot_state.gemini_session.send_text(text)
        if response:
            response_text = response['text']
            # Robot reacts with its own detected emotion
            robot_state.emotion = detect_emotion(response_text)
            robot_state.last_transcript = f"User: {text}\nRobot: {response_text}"
            await sync_emotion_to_display(robot_state.emotion)
            
            # Send speech to Raspberry Pi if enabled
            if robot_state.raspberry_pi_client:
                await robot_state.raspberry_pi_client.send_json({
                    "type": "speak",
                    "text": response_text
                })
        
        robot_state.is_speaking = False
    
    robot_state.is_listening = False
    await broadcast_state()

async def sync_emotion_to_display(emotion: str):
    if robot_state.raspberry_pi_client:
        await robot_state.raspberry_pi_client.send_json({"type": "emotion", "emotion": emotion})

def detect_emotion(text: str) -> str:
    """Detect emotion from text with enhanced mental health awareness"""
    text = text.lower()
    
    # Check for crisis keywords
    crisis_keywords = ['suicide', 'kill myself', 'want to die', 'end it all', 'no reason to live']
    if any(keyword in text for keyword in crisis_keywords):
        robot_state.concern_level = 10
        robot_state.mental_health_insights.append({
            'timestamp': asyncio.get_event_loop().time(),
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
        'ros_connected': robot_state.ros_client is not None
    }
    for client in robot_state.connected_clients:
        try:
            await client.send_json({'type': 'state', 'data': state})
        except:
            pass

@app.get("/")
async def root():
    return FileResponse(os.path.join(frontend_dir, "face_display.html"))

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
    """Send text to be spoken by robot"""
    text = data.get('text', '')
    if text and robot_state.raspberry_pi_client:
        await robot_state.raspberry_pi_client.send_json({
            "type": "speak",
            "text": text
        })
        return {"status": "ok", "text": text}
    return {"status": "error", "message": "No text or Pi not connected"}

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

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)