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