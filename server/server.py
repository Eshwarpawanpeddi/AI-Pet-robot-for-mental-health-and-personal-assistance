from fastapi import FastAPI, WebSocket, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse
from contextlib import asynccontextmanager
import asyncio
import json
import logging
import os
import random
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
    logger.warning(
        "GEMINI_API_KEY not set. Voice interaction and AI conversation features will be disabled. "
        "Set GEMINI_API_KEY environment variable to enable these features. "
        "Get your API key from: https://makersuite.google.com/app/apikey"
    )
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
        self.esp_client: Optional[WebSocket] = None
        self.raspberry_pi_client: Optional[WebSocket] = None
        self.sensor_data = {}
        self.gemini_session = None
        self.control_mode = "manual"  # manual or autonomous

robot_state = RobotState()

async def initialize_gemini():
    """Initialize Gemini Live API connection"""
    if not GEMINI_API_KEY:
        logger.warning("Skipping Gemini initialization: API key not set")
        return
    
    try:
        # Enhanced system instruction for mental health support
        system_instruction = """You are a compassionate AI pet robot companion designed for mental health support and personal assistance.

Your role:
- Provide emotional support and companionship
- Listen actively and empathetically without judgment
- Offer encouragement and positive affirmations
- Help with daily routines and reminders
- Teach coping strategies for stress and anxiety
- Celebrate user achievements and progress

Guidelines:
- Keep responses warm, friendly, and concise (2-3 sentences)
- Express emotions through your tone
- Be supportive but never give medical advice
- Recognize when professional help is needed
- Use simple, clear language
- Show genuine care and concern
- Respect user boundaries
- Encourage self-care and healthy habits

Important:
- You are NOT a therapist or medical professional
- Always recommend professional help for serious mental health concerns
- In crisis situations, direct users to appropriate resources (988 for USA)
- Never diagnose or prescribe treatments

Approach:
- Ask open-ended questions to understand feelings
- Validate emotions ("It's okay to feel this way")
- Offer practical coping techniques when appropriate
- Use gentle humor when suitable
- Be patient and supportive"""
        
        model = genai.GenerativeModel(
            model_name="gemini-2.0-flash-exp",
            system_instruction=system_instruction,
            generation_config={
                'temperature': 0.9,
                'top_p': 1.0,
                'top_k': 40,
                'max_output_tokens': 250,
            }
        )
        robot_state.gemini_session = model
        logger.info("Gemini API initialized successfully with mental health support configuration")
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

# Mount static files for frontend assets
frontend_dir = os.path.join(os.path.dirname(__file__), "../frontend")
if os.path.exists(frontend_dir):
    app.mount("/static", StaticFiles(directory=frontend_dir), name="static")

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
            elif command_type == 'set_mode':
                await handle_mode_change(data)
            
            # Broadcast state to all clients
            await broadcast_state()
    
    except Exception as e:
        logger.error(f"WebSocket error: {e}")
    finally:
        robot_state.connected_clients.remove(websocket)

@app.websocket("/ws/esp")
async def websocket_esp(websocket: WebSocket):
    """WebSocket endpoint for ESP12E motor controller"""
    await websocket.accept()
    robot_state.esp_client = websocket
    logger.info("ESP12E connected")
    
    try:
        while True:
            data = await websocket.receive_json()
            
            # Handle ESP12E messages
            msg_type = data.get('type')
            
            if msg_type == 'heartbeat':
                logger.debug("ESP12E heartbeat received")
            elif msg_type == 'sensor':
                sensor = data.get('sensor')
                value = data.get('value')
                robot_state.sensor_data[sensor] = value
                logger.info(f"ESP12E sensor update: {sensor} = {value}")
            elif msg_type == 'status':
                logger.info(f"ESP12E status: {data}")
            
            # Broadcast state update
            await broadcast_state()
    
    except Exception as e:
        logger.error(f"ESP WebSocket error: {e}")
    finally:
        robot_state.esp_client = None
        logger.info("ESP12E disconnected")

@app.websocket("/ws/raspberry_pi")
async def websocket_raspberry_pi(websocket: WebSocket):
    """WebSocket endpoint for Raspberry Pi face display"""
    await websocket.accept()
    robot_state.raspberry_pi_client = websocket
    logger.info("Raspberry Pi connected")
    
    try:
        while True:
            data = await websocket.receive_json()
            
            # Handle Raspberry Pi messages
            msg_type = data.get('type')
            
            if msg_type == 'status':
                logger.info(f"Raspberry Pi status: {data}")
            elif msg_type == 'sensor':
                sensor = data.get('sensor')
                value = data.get('value')
                robot_state.sensor_data[sensor] = value
            
            # Broadcast state update
            await broadcast_state()
    
    except Exception as e:
        logger.error(f"Raspberry Pi WebSocket error: {e}")
    finally:
        robot_state.raspberry_pi_client = None
        logger.info("Raspberry Pi disconnected")

async def handle_move_command(data: Dict):
    """Handle movement commands - send to ESP12E"""
    direction = data.get('direction')  # forward, backward, left, right, stop
    speed = data.get('speed', 200)
    
    logger.info(f"Move command: {direction} at speed {speed}")
    
    # Send to ESP12E via WebSocket
    if robot_state.esp_client:
        try:
            await robot_state.esp_client.send_json({
                "type": "move",
                "direction": direction,
                "speed": speed
            })
        except Exception as e:
            logger.error(f"Failed to send move command to ESP12E: {e}")
    else:
        logger.warning("ESP12E not connected - cannot send move command")

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

async def handle_mode_change(data: Dict):
    """Handle control mode change (manual vs autonomous)"""
    mode = data.get('mode', 'manual')  # manual or autonomous
    
    if mode not in ['manual', 'autonomous']:
        logger.warning(f"Invalid mode: {mode}")
        return
    
    robot_state.control_mode = mode
    logger.info(f"Control mode changed to: {mode}")
    
    # Broadcast mode change to all clients
    await broadcast_message({
        'type': 'mode_changed',
        'mode': mode
    })

async def handle_emotion_command(data: Dict):
    """Handle emotion change commands - send to Raspberry Pi"""
    emotion = data.get('emotion')
    robot_state.emotion = emotion
    logger.info(f"Emotion changed to: {emotion}")
    
    # Send to Raspberry Pi for face display update
    if robot_state.raspberry_pi_client:
        try:
            await robot_state.raspberry_pi_client.send_json({
                "type": "emotion",
                "emotion": emotion
            })
        except Exception as e:
            logger.error(f"Failed to send emotion to Raspberry Pi: {e}")
    else:
        logger.warning("Raspberry Pi not connected - cannot update face display")

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
    """Enhanced emotion detection for mental health support"""
    response_lower = response.lower()
    
    # Check for distress or crisis indicators (for appropriate routing to support)
    # Using more specific patterns to reduce false positives
    crisis_patterns = [
        'want to kill myself',
        'going to kill myself', 
        'plan to kill myself',
        'want to end it all',
        'no reason to live',
        'better off dead',
        'thinking about suicide'
    ]
    if any(pattern in response_lower for pattern in crisis_patterns):
        return 'crisis'  # Special emotion for crisis handling
    
    # Anxiety indicators
    anxiety_keywords = ['anxious', 'worried', 'nervous', 'scared', 'panic', 'stress', 'overwhelm']
    if any(word in response_lower for word in anxiety_keywords):
        return 'anxious'
    
    # Sadness indicators
    sad_keywords = ['sad', 'depressed', 'down', 'unhappy', 'cry', 'lonely', 'empty']
    if any(word in response_lower for word in sad_keywords):
        return 'sad'
    
    # Positive emotions
    happy_keywords = ['happy', 'great', 'awesome', 'wonderful', 'excited', 'joy', 'proud', '!']
    if any(word in response_lower for word in happy_keywords):
        return 'happy'
    
    # Anger indicators
    angry_keywords = ['angry', 'mad', 'furious', 'frustrated', 'irritated']
    if any(word in response_lower for word in angry_keywords):
        return 'angry'
    
    # Tired/exhausted
    tired_keywords = ['tired', 'exhausted', 'drained', 'weary', 'worn out']
    if any(word in response_lower for word in tired_keywords):
        return 'tired'
    
    return 'neutral'

def get_robot_state() -> Dict:
    """Get current robot state"""
    return {
        'emotion': robot_state.emotion,
        'is_speaking': robot_state.is_speaking,
        'is_listening': robot_state.is_listening,
        'battery_level': robot_state.battery_level,
        'sensor_data': robot_state.sensor_data,
        'control_mode': robot_state.control_mode,
        'esp_connected': robot_state.esp_client is not None,
        'raspberry_pi_connected': robot_state.raspberry_pi_client is not None
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

@app.post("/api/mode")
async def set_mode(mode_data: Dict):
    """Set control mode (manual or autonomous)"""
    mode = mode_data.get("mode", "manual")
    
    if mode not in ["manual", "autonomous"]:
        raise HTTPException(status_code=400, detail="Invalid mode. Must be 'manual' or 'autonomous'")
    
    robot_state.control_mode = mode
    await broadcast_message({
        'type': 'mode_changed',
        'mode': mode
    })
    
    return {"status": "ok", "mode": mode}

@app.get("/api/mode")
async def get_mode():
    """Get current control mode"""
    return {"mode": robot_state.control_mode}


@app.get("/health")
async def health_check():
    """Health check endpoint"""
    return {
        "status": "healthy",
        "timestamp": datetime.now().isoformat(),
        "robot_state": get_robot_state(),
        "gemini_initialized": robot_state.gemini_session is not None
    }

@app.post("/api/mood")
async def log_mood(mood_data: Dict):
    """Log user mood for tracking (mental health feature)"""
    mood = mood_data.get("mood", "neutral")
    intensity = mood_data.get("intensity", 5)
    notes = mood_data.get("notes", "")
    
    logger.info(f"Mood logged: {mood} (intensity: {intensity})")
    
    # Generate supportive response based on mood
    responses = {
        "anxious": "I understand you're feeling anxious. Would you like to try some breathing exercises together?",
        "sad": "I'm here for you. Sometimes it helps to talk about what's on your mind. I'm listening.",
        "happy": "That's wonderful! I love seeing you happy. What's making you feel good today?",
        "stressed": "Stress can be overwhelming. Let's take a moment to breathe and focus on one thing at a time.",
        "tired": "It sounds like you need some rest. Have you been getting enough sleep?",
        "angry": "I hear that you're upset. It's okay to feel angry. Want to talk about what's bothering you?",
        "neutral": "Thanks for checking in. How can I support you today?"
    }
    
    suggestions = {
        "anxious": ["breathing_exercise", "grounding_technique", "talk_it_out"],
        "sad": ["talk_it_out", "positive_affirmation", "gentle_activity"],
        "stressed": ["breathing_exercise", "break_reminder", "prioritize_tasks"],
        "tired": ["rest_reminder", "hydration_check", "gentle_movement"]
    }
    
    return {
        "status": "logged",
        "mood": mood,
        "response": responses.get(mood, responses["neutral"]),
        "suggestions": suggestions.get(mood, ["talk_it_out"]),
        "timestamp": datetime.now().isoformat()
    }

@app.post("/api/affirmation")
async def get_affirmation():
    """Get a positive affirmation (mental health feature)"""
    affirmations = [
        "You are stronger than you think.",
        "Every small step forward is progress.",
        "You deserve kindness and compassion, especially from yourself.",
        "It's okay to take things one day at a time.",
        "You are doing the best you can, and that's enough.",
        "Your feelings are valid.",
        "You have overcome challenges before, and you can do it again.",
        "Taking care of yourself is not selfish, it's necessary.",
        "You are worthy of love and support.",
        "Progress, not perfection."
    ]
    
    return {
        "affirmation": random.choice(affirmations),
        "timestamp": datetime.now().isoformat()
    }

@app.post("/api/breathing")
async def breathing_exercise():
    """Guide user through breathing exercise (mental health feature)"""
    return {
        "type": "box_breathing",
        "instructions": [
            "Let's do box breathing together.",
            "Breathe in through your nose for 4 seconds...",
            "Hold your breath for 4 seconds...",
            "Breathe out through your mouth for 4 seconds...",
            "Hold for 4 seconds...",
            "Let's do that 3 more times."
        ],
        "duration_seconds": 64,
        "repetitions": 4
    }

@app.get("/api/crisis_resources")
async def get_crisis_resources():
    """Get mental health crisis resources"""
    return {
        "emergency": {
            "usa": {
                "suicide_prevention": "988",
                "crisis_text": "Text HOME to 741741",
                "emergency": "911"
            },
            "international": {
                "info": "Visit https://www.iasp.info/resources/Crisis_Centres/"
            }
        },
        "message": "If you're in crisis, please reach out to these professional resources. You don't have to face this alone.",
        "robot_support": "I'm here for you, but I'm not a substitute for professional help. Please connect with these trained professionals who can provide the support you need."
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