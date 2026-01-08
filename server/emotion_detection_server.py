#!/usr/bin/env python3
"""
Emotion Detection Server - Port 9999
Real-time emotion detection from camera stream using emotion_model.h5
Detects emotions from facial expressions and provides feedback via Gemini API
Uses the same connection logic pattern as server.py for consistency.
"""
from fastapi import FastAPI, WebSocket
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse
from contextlib import asynccontextmanager
import asyncio
import logging
import uvicorn
import cv2
import numpy as np
import base64
from typing import Optional
import os
import json

try:
    import tensorflow as tf
except ImportError:
    tf = None
    
try:
    from dotenv import load_dotenv
    load_dotenv()
except ImportError:
    pass

try:
    import google.generativeai as genai
except ImportError:
    genai = None

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
SERVER_HOST = os.getenv("SERVER_HOST", "localhost")

# Message type constants
MSG_TYPE_CAMERA_FRAME = "camera_frame"
MSG_TYPE_EMOTION_DETECTION = "emotion_detection"

class EmotionDetectionState:
    def __init__(self):
        self.clients = []
        self.emotion_model = None
        self.face_cascade = None
        self.gemini_model = None
        self.primary_server_url = f"http://{SERVER_HOST}:8000"
        self.primary_ws_url = f"ws://{SERVER_HOST}:8000/ws/control"
        self.primary_ws = None
        self.latest_camera_frame = None
        self.current_emotion = "neutral"
        self.emotion_history = []
        self.emotion_labels = ['angry', 'disgust', 'fear', 'happy', 'sad', 'surprise', 'neutral']
        self.is_connected = False
    
    async def load_models(self):
        """Load emotion detection model and face detector"""
        try:
            # Load emotion model
            if tf is not None:
                model_path = os.path.join(os.path.dirname(__file__), "..", "emotion_model.h5")
                if os.path.exists(model_path):
                    self.emotion_model = tf.keras.models.load_model(model_path)
                    logger.info(f"Emotion model loaded successfully from {model_path}")
                else:
                    logger.warning(f"Emotion model not found at {model_path}")
            else:
                logger.warning("TensorFlow not available - emotion model disabled")
            
            # Load face detector
            cascade_path = cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
            self.face_cascade = cv2.CascadeClassifier(cascade_path)
            logger.info("Face cascade loaded successfully")
            
            # Initialize Gemini API for emotion-based responses
            if GEMINI_API_KEY and genai is not None:
                genai.configure(api_key=GEMINI_API_KEY)
                self.gemini_model = genai.GenerativeModel(
                    model_name="gemini-1.5-flash",
                    system_instruction="""You are a compassionate AI companion analyzing emotions.
                    Provide brief, empathetic responses (1-2 sentences) based on detected emotions.
                    Be supportive and encouraging.""",
                    generation_config={
                        'temperature': 0.9,
                        'max_output_tokens': 100,
                    }
                )
                logger.info("Gemini API initialized for emotion responses")
            else:
                logger.warning("GEMINI_API_KEY not found or genai not available - emotion responses disabled")
                
        except Exception as e:
            logger.error(f"Error loading models: {e}")
    
    async def connect_to_primary(self):
        """Connect to primary control server for camera feed with retry logic"""
        while True:
            try:
                import websockets
                logger.info(f"Connecting to primary server at {self.primary_ws_url}...")
                self.primary_ws = await asyncio.wait_for(
                    websockets.connect(self.primary_ws_url),
                    timeout=10
                )
                self.is_connected = True
                logger.info("✓ Connected to primary server (port 8000)")
                
                # Subscribe to camera feed
                await self.primary_ws.send('{"type": "subscribe_camera"}')
                
                # Listen for camera frames
                await self.listen_to_primary()
                
            except asyncio.TimeoutError:
                logger.warning("Connection to primary server timed out, retrying in 5s...")
            except Exception as e:
                logger.warning(f"Failed to connect to primary server: {e}, retrying in 5s...")
            
            self.is_connected = False
            self.primary_ws = None
            await asyncio.sleep(5)
    
    async def listen_to_primary(self):
        """Listen for camera frames from primary server"""
        try:
            while self.primary_ws:
                message = await self.primary_ws.recv()
                data = json.loads(message)
                
                if data.get('type') == MSG_TYPE_CAMERA_FRAME:
                    self.latest_camera_frame = data.get('frame')
                    # Process frame for emotion detection
                    asyncio.create_task(self.process_frame())
        except Exception as e:
            logger.error(f"Error listening to primary: {e}")
            self.is_connected = False
            self.primary_ws = None
    
    async def process_frame(self):
        """Process camera frame for emotion detection"""
        if not self.latest_camera_frame or not self.emotion_model or not self.face_cascade:
            return
        
        try:
            # Decode base64 image
            img_bytes = base64.b64decode(self.latest_camera_frame)
            nparr = np.frombuffer(img_bytes, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            
            if frame is None:
                return
            
            # Convert to grayscale for face detection
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Detect faces
            faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)
            
            detected_emotions = []
            
            for (x, y, w, h) in faces:
                # Extract face ROI
                face_roi = gray[y:y+h, x:x+w]
                
                # Resize to model input size (typically 48x48 for emotion models)
                face_roi = cv2.resize(face_roi, (48, 48))
                face_roi = face_roi.astype('float32') / 255.0
                face_roi = np.expand_dims(face_roi, axis=0)
                face_roi = np.expand_dims(face_roi, axis=-1)
                
                # Predict emotion
                predictions = self.emotion_model.predict(face_roi, verbose=0)
                emotion_idx = np.argmax(predictions[0])
                emotion = self.emotion_labels[emotion_idx]
                confidence = float(predictions[0][emotion_idx])
                
                detected_emotions.append({
                    'emotion': emotion,
                    'confidence': confidence,
                    'bbox': [int(x), int(y), int(w), int(h)]
                })
                
                # Draw rectangle and label on frame
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.putText(frame, f"{emotion} ({confidence:.2f})", 
                           (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 
                           0.9, (0, 255, 0), 2)
            
            # Encode processed frame
            _, buffer = cv2.imencode('.jpg', frame)
            processed_frame = base64.b64encode(buffer).decode('utf-8')
            
            # Update current emotion
            if detected_emotions:
                primary_emotion = detected_emotions[0]['emotion']
                if primary_emotion != self.current_emotion:
                    self.current_emotion = primary_emotion
                    self.emotion_history.append({
                        'emotion': primary_emotion,
                        'timestamp': asyncio.get_event_loop().time()
                    })
                    # Keep only last 100 emotions
                    if len(self.emotion_history) > 100:
                        self.emotion_history.pop(0)
            
            # Broadcast to all clients
            await self.broadcast_results(processed_frame, detected_emotions)
            
        except Exception as e:
            logger.error(f"Error processing frame: {e}")
    
    async def broadcast_results(self, processed_frame: str, emotions: list):
        """Broadcast detection results to all clients"""
        for client in self.clients[:]:
            try:
                await client.send_json({
                    'type': MSG_TYPE_EMOTION_DETECTION,
                    'frame': processed_frame,
                    'emotions': emotions,
                    'current_emotion': self.current_emotion
                })
            except:
                self.clients.remove(client)
    
    async def get_emotion_response(self, emotion: str) -> Optional[str]:
        """Get empathetic response from Gemini based on detected emotion"""
        if not self.gemini_model:
            return None
        
        try:
            prompt = f"The person appears to be feeling {emotion}. Provide a brief, supportive response."
            response = await asyncio.to_thread(
                self.gemini_model.generate_content, prompt
            )
            return response.text
        except Exception as e:
            logger.error(f"Error getting emotion response: {e}")
            return None

emotion_state = EmotionDetectionState()

@asynccontextmanager
async def lifespan(app: FastAPI):
    """Application lifespan manager"""
    logger.info("Emotion Detection Server starting...")
    
    # Load models
    await emotion_state.load_models()
    
    # Start connection task in background (with retry logic)
    connection_task = asyncio.create_task(emotion_state.connect_to_primary())
    
    yield
    
    # Cleanup
    connection_task.cancel()
    try:
        await connection_task
    except asyncio.CancelledError:
        pass
    if emotion_state.primary_ws:
        await emotion_state.primary_ws.close()

app = FastAPI(lifespan=lifespan)
app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"])

@app.get("/")
async def root():
    """Serve the emotion detection interface"""
    return HTMLResponse(content="""
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Emotion Detection - AI Pet Robot</title>
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }
        
        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
            padding: 20px;
            color: white;
        }
        
        .container {
            max-width: 1200px;
            margin: 0 auto;
        }
        
        .header {
            text-align: center;
            padding: 20px 0;
            margin-bottom: 30px;
        }
        
        .header h1 {
            font-size: 36px;
            margin-bottom: 10px;
        }
        
        .status-badge {
            display: inline-block;
            padding: 8px 20px;
            border-radius: 20px;
            font-size: 14px;
            background: rgba(255, 255, 255, 0.2);
        }
        
        .status-badge.connected {
            background: #10b981;
        }
        
        .main-content {
            display: grid;
            grid-template-columns: 2fr 1fr;
            gap: 20px;
            margin-bottom: 20px;
        }
        
        @media (max-width: 768px) {
            .main-content {
                grid-template-columns: 1fr;
            }
        }
        
        .video-panel {
            background: rgba(255, 255, 255, 0.1);
            border-radius: 15px;
            padding: 20px;
            backdrop-filter: blur(10px);
        }
        
        .video-container {
            width: 100%;
            min-height: 480px;
            background: rgba(0, 0, 0, 0.5);
            border-radius: 10px;
            display: flex;
            align-items: center;
            justify-content: center;
            overflow: hidden;
            position: relative;
        }
        
        .video-container img {
            max-width: 100%;
            max-height: 100%;
            border-radius: 10px;
        }
        
        .video-placeholder {
            color: rgba(255, 255, 255, 0.5);
            font-size: 18px;
        }
        
        .info-panel {
            display: flex;
            flex-direction: column;
            gap: 20px;
        }
        
        .card {
            background: rgba(255, 255, 255, 0.1);
            border-radius: 15px;
            padding: 20px;
            backdrop-filter: blur(10px);
        }
        
        .card h2 {
            font-size: 20px;
            margin-bottom: 15px;
            display: flex;
            align-items: center;
            gap: 10px;
        }
        
        .emotion-display {
            text-align: center;
            padding: 20px;
        }
        
        .emotion-icon {
            font-size: 80px;
            margin-bottom: 10px;
        }
        
        .emotion-label {
            font-size: 24px;
            font-weight: bold;
            text-transform: capitalize;
        }
        
        .emotion-list {
            list-style: none;
        }
        
        .emotion-item {
            padding: 10px;
            margin: 5px 0;
            background: rgba(255, 255, 255, 0.1);
            border-radius: 8px;
            display: flex;
            justify-content: space-between;
            align-items: center;
        }
        
        .confidence-bar {
            width: 100px;
            height: 8px;
            background: rgba(255, 255, 255, 0.2);
            border-radius: 4px;
            overflow: hidden;
        }
        
        .confidence-fill {
            height: 100%;
            background: #10b981;
            transition: width 0.3s;
        }
        
        .stats {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 10px;
        }
        
        .stat-item {
            text-align: center;
            padding: 15px;
            background: rgba(255, 255, 255, 0.1);
            border-radius: 8px;
        }
        
        .stat-value {
            font-size: 28px;
            font-weight: bold;
        }
        
        .stat-label {
            font-size: 12px;
            opacity: 0.8;
            margin-top: 5px;
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>😊 Emotion Detection</h1>
            <span class="status-badge" id="statusBadge">Connecting...</span>
        </div>
        
        <div class="main-content">
            <div class="video-panel">
                <h2>📹 Live Camera Feed</h2>
                <div class="video-container" id="videoContainer">
                    <div class="video-placeholder">Waiting for camera feed...</div>
                </div>
            </div>
            
            <div class="info-panel">
                <div class="card">
                    <h2>🎭 Current Emotion</h2>
                    <div class="emotion-display">
                        <div class="emotion-icon" id="emotionIcon">😐</div>
                        <div class="emotion-label" id="emotionLabel">Neutral</div>
                    </div>
                </div>
                
                <div class="card">
                    <h2>📊 Detected Faces</h2>
                    <ul class="emotion-list" id="emotionList">
                        <li style="text-align: center; opacity: 0.5;">No faces detected</li>
                    </ul>
                </div>
                
                <div class="card">
                    <h2>📈 Statistics</h2>
                    <div class="stats">
                        <div class="stat-item">
                            <div class="stat-value" id="faceCount">0</div>
                            <div class="stat-label">Faces Detected</div>
                        </div>
                        <div class="stat-item">
                            <div class="stat-value" id="emotionCount">0</div>
                            <div class="stat-label">Emotions Tracked</div>
                        </div>
                    </div>
                </div>
            </div>
        </div>
    </div>
    
    <script>
        const emotionIcons = {
            'happy': '😊',
            'sad': '😢',
            'angry': '😠',
            'fear': '😨',
            'surprise': '😲',
            'disgust': '🤢',
            'neutral': '😐'
        };
        
        const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const ws = new WebSocket(`${protocol}//${window.location.host}/ws/emotion`);
        
        let totalEmotions = 0;
        
        ws.onopen = () => {
            document.getElementById('statusBadge').textContent = 'Connected';
            document.getElementById('statusBadge').className = 'status-badge connected';
        };
        
        ws.onclose = () => {
            document.getElementById('statusBadge').textContent = 'Disconnected';
            document.getElementById('statusBadge').className = 'status-badge';
        };
        
        ws.onmessage = (event) => {
            const data = JSON.parse(event.data);
            
            if (data.type === 'emotion_detection') {
                // Update video
                if (data.frame) {
                    document.getElementById('videoContainer').innerHTML = 
                        `<img src="data:image/jpeg;base64,${data.frame}" alt="Camera">`;
                }
                
                // Update current emotion
                const emotion = data.current_emotion || 'neutral';
                document.getElementById('emotionIcon').textContent = emotionIcons[emotion] || '😐';
                document.getElementById('emotionLabel').textContent = emotion.charAt(0).toUpperCase() + emotion.slice(1);
                
                // Update detected emotions list
                if (data.emotions && data.emotions.length > 0) {
                    totalEmotions++;
                    document.getElementById('emotionCount').textContent = totalEmotions;
                    document.getElementById('faceCount').textContent = data.emotions.length;
                    
                    const listHtml = data.emotions.map((e, idx) => `
                        <li class="emotion-item">
                            <span>${emotionIcons[e.emotion] || '😐'} ${e.emotion}</span>
                            <div class="confidence-bar">
                                <div class="confidence-fill" style="width: ${e.confidence * 100}%"></div>
                            </div>
                        </li>
                    `).join('');
                    
                    document.getElementById('emotionList').innerHTML = listHtml;
                } else {
                    document.getElementById('faceCount').textContent = '0';
                }
            }
        };
    </script>
</body>
</html>
    """)

@app.websocket("/ws/emotion")
async def websocket_emotion(websocket: WebSocket):
    """WebSocket for emotion detection clients"""
    await websocket.accept()
    emotion_state.clients.append(websocket)
    logger.info(f"Emotion client connected. Total: {len(emotion_state.clients)}")
    
    try:
        while True:
            # Keep connection alive and handle any client messages
            data = await websocket.receive_json()
            
            # Handle requests for emotion-based responses
            if data.get('type') == 'get_emotion_response':
                emotion = data.get('emotion', emotion_state.current_emotion)
                response = await emotion_state.get_emotion_response(emotion)
                if response:
                    await websocket.send_json({
                        'type': 'emotion_response',
                        'emotion': emotion,
                        'response': response
                    })
    except:
        if websocket in emotion_state.clients:
            emotion_state.clients.remove(websocket)
        logger.info(f"Emotion client disconnected. Total: {len(emotion_state.clients)}")

@app.get("/health")
async def health_check():
    """Health check endpoint"""
    return {
        'status': 'healthy',
        'server': 'emotion_detection',
        'port': 9999,
        'clients': len(emotion_state.clients),
        'model_loaded': emotion_state.emotion_model is not None,
        'primary_connected': emotion_state.primary_ws is not None,
        'current_emotion': emotion_state.current_emotion
    }

@app.get("/api/emotion")
async def get_current_emotion():
    """Get current detected emotion"""
    return {
        'current_emotion': emotion_state.current_emotion,
        'emotion': emotion_state.current_emotion,  # For backward compatibility
        'history': emotion_state.emotion_history[-10:]  # Last 10 emotions
    }

@app.post("/api/emotion/response")
async def get_emotion_response_api(emotion: str = "neutral"):
    """Get empathetic response for detected emotion"""
    # Use provided emotion or fall back to current detected emotion
    emotion_to_use = emotion if emotion != "neutral" or not emotion_state.current_emotion else emotion_state.current_emotion
    
    response = await emotion_state.get_emotion_response(emotion_to_use)
    return {
        'emotion': emotion_to_use,
        'response': response
    }

@app.post("/api/tts")
async def receive_tts(data: dict):
    """
    Receive TTS broadcast from port 8000
    Forwards TTS to all connected emotion detection clients
    """
    try:
        # Broadcast to all connected clients
        disconnected_clients = []
        for client in emotion_state.clients:
            try:
                await client.send_json({
                    'type': 'tts_output',
                    'text': data.get('text', ''),
                    'voice': data.get('voice', 'en+f3'),
                    'speed': data.get('speed', 150),
                    'pitch': data.get('pitch', 50)
                })
            except Exception as e:
                logger.error(f"Failed to send TTS to emotion detection client: {e}")
                disconnected_clients.append(client)
        
        # Clean up disconnected clients
        for client in disconnected_clients:
            if client in emotion_state.clients:
                emotion_state.clients.remove(client)
        
        logger.info(f"TTS broadcast to {len(emotion_state.clients)} emotion detection clients: {data.get('text', '')}")
        return {"status": "ok", "broadcast_to": len(emotion_state.clients)}
    except Exception as e:
        logger.error(f"Error in TTS broadcast: {e}")
        return {"status": "error", "message": str(e)}

if __name__ == "__main__":
    logger.info("=" * 60)
    logger.info("Emotion Detection Server Starting...")
    logger.info("=" * 60)
    logger.info("")
    logger.info("😊 Server Configuration:")
    logger.info(f"   - Host: 0.0.0.0")
    logger.info(f"   - Port: 9999")
    logger.info("")
    logger.info("🌐 Access the emotion detection interface at:")
    logger.info("   - http://localhost:9999")
    logger.info("")
    logger.info("📡 WebSocket Endpoint:")
    logger.info("   - /ws/emotion → Emotion detection clients")
    logger.info("")
    logger.info("🔄 Features:")
    logger.info("   - Real-time face detection")
    logger.info("   - Emotion classification (7 emotions)")
    logger.info("   - Gemini-powered responses")
    logger.info("   - Camera feed from port 8000")
    logger.info("")
    logger.info("=" * 60)
    logger.info("")
    
    uvicorn.run(app, host="0.0.0.0", port=9999)
