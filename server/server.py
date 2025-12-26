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
        self.battery_level = 100
        self.connected_clients = []
        self.raspberry_pi_client = None
        self.gemini_session = None
        self.last_transcript = ""

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
                # Route movement commands to Raspberry Pi
                if robot_state.raspberry_pi_client:
                    await robot_state.raspberry_pi_client.send_json(data)
            
            await broadcast_state()
    except:
        if websocket in robot_state.connected_clients:
            robot_state.connected_clients.remove(websocket)

@app.websocket("/ws/raspberry_pi")
async def websocket_raspberry_pi(websocket: WebSocket):
    await websocket.accept()
    robot_state.raspberry_pi_client = websocket
    logger.info("Raspberry Pi hardware connected")
    try:
        while True:
            await websocket.receive_json()
    except:
        robot_state.raspberry_pi_client = None
        logger.info("Raspberry Pi disconnected")

async def handle_text_command(data: Dict):
    text = data.get('text', '')
    if not text: return
    
    # Detect User Emotion from their input
    robot_state.user_emotion = detect_emotion(text)
    robot_state.is_listening = True
    await broadcast_state()
    
    if robot_state.gemini_session:
        response = await robot_state.gemini_session.send_text(text)
        if response:
            # Robot reacts with its own detected emotion
            robot_state.emotion = detect_emotion(response['text'])
            robot_state.last_transcript = f"User: {text}\nRobot: {response['text']}"
            await sync_emotion_to_display(robot_state.emotion)
    
    robot_state.is_listening = False
    await broadcast_state()

async def sync_emotion_to_display(emotion: str):
    if robot_state.raspberry_pi_client:
        await robot_state.raspberry_pi_client.send_json({"type": "emotion", "emotion": emotion})

def detect_emotion(text: str) -> str:
    text = text.lower()
    if any(w in text for w in ['sad', 'cry', 'lonely', 'hurt', 'depressed', 'unhappy']): return 'sad'
    if any(w in text for w in ['angry', 'mad', 'hate', 'stop', 'annoyed']): return 'angry'
    if any(w in text for w in ['happy', 'good', 'yay', 'great', 'awesome', 'excited']): return 'happy'
    return 'neutral'

async def broadcast_state():
    state = {
        'emotion': robot_state.emotion,
        'user_emotion': robot_state.user_emotion,
        'battery_level': robot_state.battery_level,
        'is_listening': robot_state.is_listening,
        'last_transcript': robot_state.last_transcript
    }
    for client in robot_state.connected_clients:
        try:
            await client.send_json({'type': 'state', 'data': state})
        except:
            pass

@app.get("/")
async def root():
    return FileResponse(os.path.join(frontend_dir, "face_display.html"))

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)