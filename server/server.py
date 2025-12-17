from fastapi import FastAPI, WebSocket, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse
from contextlib import asynccontextmanager
import asyncio
import json
import logging
import os
from typing import Dict, List, Optional
from datetime import datetime
from dotenv import load_dotenv
import google.generativeai as genai
import uvicorn

# Load environment variables
load_dotenv()

# Configure logging
log_level = os.getenv("LOG_LEVEL", "INFO")
logging.basicConfig(
    level=getattr(logging, log_level),
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Gemini API Configuration
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
if not GEMINI_API_KEY:
    logger.warning("GEMINI_API_KEY not set. Gemini API features will be disabled.")
else:
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

async def initialize_gemini():
    """Initialize Gemini Live API connection"""
    if not GEMINI_API_KEY:
        logger.warning("Skipping Gemini initialization: API key not set")
        return
    
    try:
        model = genai.GenerativeModel(
            model_name="gemini-2.0-flash-exp",
            system_instruction="You are a friendly, emotionally intelligent pet robot. Keep responses concise and engaging. Express emotions through your speech patterns.",
            generation_config={
                'temperature': 0.9,
                'top_p': 1.0,
                'top_k': 40,
                'max_output_tokens': 200,
            }
        )
        robot_state.gemini_session = model
        logger.info("Gemini API initialized successfully")
    except Exception as e:
        logger.error(f"Failed to initialize Gemini: {e}")
        robot_state.gemini_session = None

@asynccontextmanager
async def lifespan(app: FastAPI):
    """Lifespan event handler"""
    # Startup
    logger.info("Robot Server Starting...")
    await initialize_gemini()
    yield
    # Shutdown
    logger.info("Robot Server Shutting Down...")
    if robot_state.gemini_session:
        # Clean up if needed
        pass

app = FastAPI(title="Pet Robot Server", version="1.0.0", lifespan=lifespan)

# CORS Configuration
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

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
        "robot_state": get_robot_state(),
        "gemini_initialized": robot_state.gemini_session is not None
    }

@app.get("/")
async def read_root():
    """Serve the face display HTML"""
    frontend_path = os.path.join(os.path.dirname(__file__), "../frontend/face_display.html")
    if os.path.exists(frontend_path):
        return FileResponse(frontend_path)
    return {"message": "AI Pet Robot Server is running. Access /health for status."}

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)