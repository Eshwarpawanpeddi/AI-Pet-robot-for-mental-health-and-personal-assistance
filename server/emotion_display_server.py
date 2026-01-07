#!/usr/bin/env python3
"""
Emotion Display Server - Port 10000
Dedicated server for displaying robot emotions/animated face.
Receives emotion updates from the primary control server (port 8000).
"""
from fastapi import FastAPI, WebSocket
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse, HTMLResponse
from contextlib import asynccontextmanager
import asyncio
import logging
import os
import uvicorn
import aiohttp

# Configure logging to file and console
log_dir = os.path.join(os.path.dirname(__file__), 'logs')
os.makedirs(log_dir, exist_ok=True)
log_file = os.path.join(log_dir, 'emotion_display_server.log')

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler(log_file),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

class EmotionDisplayState:
    def __init__(self):
        self.emotion = "neutral"
        self.display_clients = []
    
    async def set_emotion(self, emotion: str):
        """Update emotion and broadcast to all connected clients"""
        self.emotion = emotion
        logger.info(f"Emotion display updated to: {emotion}")
        await self.broadcast_emotion()
    
    async def broadcast_emotion(self):
        """Broadcast current emotion to all display clients"""
        disconnected_clients = []
        for client in self.display_clients:
            try:
                await client.send_json({
                    'type': 'emotion_update',
                    'emotion': self.emotion
                })
            except Exception as e:
                logger.error(f"Failed to send to client: {e}")
                disconnected_clients.append(client)
        
        # Clean up disconnected clients
        for client in disconnected_clients:
            if client in self.display_clients:
                self.display_clients.remove(client)
                logger.info(f"Removed disconnected client. Total clients: {len(self.display_clients)}")

display_state = EmotionDisplayState()

@asynccontextmanager
async def lifespan(app: FastAPI):
    """Application lifespan manager"""
    logger.info("Emotion Display Server starting...")
    
    # Start background task to poll primary server for emotion updates
    async def poll_primary_server():
        """Poll port 8000 for emotion state with exponential backoff on errors"""
        await asyncio.sleep(5)  # Wait for server to be ready
        
        retry_delay = 1  # Start with 1 second delay
        max_retry_delay = 30  # Maximum 30 seconds between retries
        consecutive_failures = 0
        
        while True:
            try:
                async with aiohttp.ClientSession() as session:
                    async with session.get(
                        'http://localhost:8000/api/state',
                        timeout=aiohttp.ClientTimeout(total=5)
                    ) as resp:
                        if resp.status == 200:
                            data = await resp.json()
                            new_emotion = data.get('emotion', 'neutral')
                            if new_emotion != display_state.emotion:
                                await display_state.set_emotion(new_emotion)
                            # Reset retry delay on success
                            retry_delay = 1
                            consecutive_failures = 0
                        else:
                            logger.warning(f"Primary server returned status {resp.status}")
                            consecutive_failures += 1
            except asyncio.TimeoutError:
                consecutive_failures += 1
                logger.warning(f"Timeout connecting to primary server (attempt {consecutive_failures})")
            except aiohttp.ClientError as e:
                consecutive_failures += 1
                logger.debug(f"Could not connect to primary server: {e}")
            except Exception as e:
                consecutive_failures += 1
                logger.error(f"Unexpected error polling primary server: {e}")
            
            # Implement exponential backoff for failures
            if consecutive_failures > 0:
                retry_delay = min(retry_delay * 2, max_retry_delay)
                logger.debug(f"Using retry delay of {retry_delay:.1f}s after {consecutive_failures} failures")
            
            await asyncio.sleep(retry_delay)
    
    poll_task = asyncio.create_task(poll_primary_server())
    
    yield
    
    # Cleanup
    poll_task.cancel()
    try:
        await poll_task
    except asyncio.CancelledError:
        pass

app = FastAPI(lifespan=lifespan)

# Configure CORS for production - allow all for emotion display
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Allow all origins for emotion display
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Mount static files
frontend_dir = os.path.join(os.path.dirname(__file__), "../frontend")
if os.path.exists(frontend_dir):
    app.mount("/static", StaticFiles(directory=frontend_dir), name="static")

@app.get("/")
async def root():
    """Serve the emotion display page"""
    # Check if static file exists (try both possible names)
    for filename in ["emotion_display.html", "face_display.html"]:
        file_path = os.path.join(frontend_dir, filename)
        if os.path.exists(file_path):
            logger.debug(f"Serving emotion display from: {file_path}")
            return FileResponse(file_path)
    
    # Return a simple inline version as fallback
    logger.info("emotion_display.html not found, serving inline fallback")
    return HTMLResponse(content="""
<!DOCTYPE html>
<html>
<head>
    <title>Robot Emotion Display</title>
    <style>
        body { margin: 0; background: #000; overflow: hidden; }
        canvas { display: block; width: 100vw; height: 100vh; }
    </style>
</head>
<body>
    <canvas id="faceCanvas"></canvas>
    <script>
        class ExpressiveEyesRenderer {
            constructor(canvasId) {
                this.canvas = document.getElementById(canvasId);
                this.ctx = this.canvas.getContext('2d');
                this.emotion = 'neutral';
                this.eyeOpen = 1.0;
                this.time = 0;
                this.tears = [];
                this.angerShake = 0;
                this.angerIntensity = 0;
                
                this.resize();
                window.addEventListener('resize', () => this.resize());
                this.animate();
            }

            resize() {
                this.canvas.width = window.innerWidth;
                this.canvas.height = window.innerHeight;
                this.centerX = this.canvas.width / 2;
                this.centerY = this.canvas.height / 2;
            }

            animate() {
                this.time += 0.05;
                this.eyeOpen = (this.time % 5 < 0.15) ? 0.1 : 1.0;
                
                if (this.emotion === 'angry') {
                    this.angerShake = Math.sin(this.time * 3) * 5;
                    this.angerIntensity = Math.abs(Math.sin(this.time * 2)) * 0.5 + 0.5;
                }
                
                this.draw();
                requestAnimationFrame(() => this.animate());
            }

            draw() {
                this.ctx.fillStyle = '#000';
                this.ctx.fillRect(0, 0, this.canvas.width, this.canvas.height);
                
                const color = this.getEmotionColor();
                
                if (this.emotion === 'sad') this.drawTears();
                if (this.emotion === 'angry') this.drawAngerEffects();

                const eyeSpacing = this.canvas.width * 0.16;
                const shakeX = this.emotion === 'angry' ? this.angerShake : 0;
                this.drawEye(this.centerX - eyeSpacing + shakeX, this.centerY, color, 'L');
                this.drawEye(this.centerX + eyeSpacing + shakeX, this.centerY, color, 'R');
            }
            
            drawAngerEffects() {
                const alpha = this.angerIntensity * 0.2;
                this.ctx.fillStyle = `rgba(255, 80, 80, ${alpha})`;
                this.ctx.fillRect(0, 0, this.canvas.width, this.canvas.height);
                
                this.ctx.strokeStyle = `rgba(255, 100, 100, ${this.angerIntensity})`;
                this.ctx.lineWidth = 3;
                for (let i = 0; i < 3; i++) {
                    const angle = (this.time * 2 + i * Math.PI * 2 / 3) % (Math.PI * 2);
                    const x = this.centerX + Math.cos(angle) * 150;
                    const y = this.centerY + Math.sin(angle) * 100;
                    this.ctx.beginPath();
                    this.ctx.moveTo(x, y);
                    this.ctx.lineTo(x + Math.cos(angle) * 30, y + Math.sin(angle) * 30);
                    this.ctx.stroke();
                }
            }

            drawTears() {
                const eyeSpacing = this.canvas.width * 0.16;
                if (Math.random() > 0.92) {
                    this.tears.push({x: (this.centerX - eyeSpacing) + (Math.random()*30-15), y: this.centerY + 20, speed: Math.random()*2+2});
                    this.tears.push({x: (this.centerX + eyeSpacing) + (Math.random()*30-15), y: this.centerY + 20, speed: Math.random()*2+2});
                }

                this.ctx.fillStyle = 'rgba(74, 144, 226, 0.7)';
                for (let i = 0; i < this.tears.length; i++) {
                    let t = this.tears[i];
                    t.y += t.speed;
                    this.ctx.beginPath();
                    this.ctx.arc(t.x, t.y, 4, 0, Math.PI * 2);
                    this.ctx.fill();
                    
                    if (t.y > this.canvas.height) {
                        this.tears.splice(i, 1);
                        i--;
                    }
                }
            }

            drawEye(x, y, color, side) {
                this.ctx.save();
                this.ctx.translate(x, y);
                this.ctx.fillStyle = color;
                this.ctx.shadowBlur = 20;
                this.ctx.shadowColor = color;

                const baseSize = this.canvas.height * 0.12;

                if (this.emotion === 'happy') {
                    this.ctx.lineWidth = baseSize * 0.25;
                    this.ctx.strokeStyle = color;
                    this.ctx.lineCap = 'round';
                    this.ctx.beginPath();
                    this.ctx.arc(0, baseSize * 0.2, baseSize * 0.6, Math.PI, 0, false);
                    this.ctx.stroke();
                } else if (this.emotion === 'angry') {
                    this.ctx.strokeStyle = color;
                    this.ctx.lineWidth = baseSize * 0.2;
                    this.ctx.lineCap = 'round';
                    this.ctx.beginPath();
                    if (side === 'L') {
                        this.ctx.moveTo(-baseSize, -baseSize);
                        this.ctx.lineTo(baseSize * 0.5, -baseSize * 0.4);
                    } else {
                        this.ctx.moveTo(baseSize, -baseSize);
                        this.ctx.lineTo(-baseSize * 0.5, -baseSize * 0.4);
                    }
                    this.ctx.stroke();
                    this.drawPill(baseSize * 0.9, baseSize * 1.5);
                } else if (this.emotion === 'sad') {
                    this.ctx.rotate(side === 'L' ? -0.25 : 0.25);
                    this.drawPill(baseSize * 0.8, baseSize * 1.1 * this.eyeOpen);
                } else {
                    this.drawPill(baseSize * 0.9, baseSize * 1.5 * this.eyeOpen);
                }
                this.ctx.restore();
            }

            drawPill(w, h) {
                this.ctx.beginPath();
                this.ctx.roundRect(-w/2, -h/2, w, h, w/2);
                this.ctx.fill();
            }

            getEmotionColor() {
                const colors = {
                    'happy': '#FFD700', 'sad': '#4A90E2', 'angry': '#FF5050', 'neutral': '#4ADE80'
                };
                return colors[this.emotion] || colors['neutral'];
            }
        }

        const eyes = new ExpressiveEyesRenderer('faceCanvas');
        
        const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const ws = new WebSocket(`${protocol}//${window.location.host}/ws/emotion_display`);
        
        ws.onmessage = (event) => {
            const data = JSON.parse(event.data);
            if (data.type === 'emotion_update') {
                eyes.emotion = data.emotion;
                console.log('Emotion updated to:', data.emotion);
            }
        };
        
        ws.onopen = () => console.log('Connected to emotion display server');
        ws.onclose = () => console.log('Disconnected from emotion display server');
    </script>
</body>
</html>
        """)

@app.websocket("/ws/emotion_display")
async def websocket_emotion_display(websocket: WebSocket):
    """WebSocket for emotion display clients with proper disconnect handling"""
    await websocket.accept()
    display_state.display_clients.append(websocket)
    logger.info(f"Emotion display client connected. Total: {len(display_state.display_clients)}")
    
    # Send current emotion immediately
    try:
        await websocket.send_json({
            'type': 'emotion_update',
            'emotion': display_state.emotion
        })
    except Exception as e:
        logger.error(f"Failed to send initial emotion to client: {e}")
    
    try:
        while True:
            # Keep connection alive and listen for any messages
            await websocket.receive_text()
    except asyncio.CancelledError:
        logger.info("WebSocket connection cancelled")
        raise  # Re-raise to allow proper task cancellation
    except Exception as e:
        logger.debug(f"WebSocket disconnected: {e}")
    finally:
        # Ensure proper cleanup
        if websocket in display_state.display_clients:
            display_state.display_clients.remove(websocket)
        logger.info(f"Emotion display client disconnected. Total: {len(display_state.display_clients)}")

@app.post("/api/emotion")
async def update_emotion(data: dict):
    """API endpoint to update emotion (called from port 8000)"""
    emotion = data.get('emotion', 'neutral')
    await display_state.set_emotion(emotion)
    return {"status": "ok", "emotion": emotion}

@app.get("/api/emotion")
async def get_emotion():
    """Get current emotion"""
    return {"emotion": display_state.emotion}

@app.post("/api/tts")
async def receive_tts(data: dict):
    """
    Receive TTS broadcast from port 8000
    Forwards TTS to all connected emotion display clients
    """
    try:
        # Broadcast to all connected display clients
        disconnected_clients = []
        for client in display_state.display_clients:
            try:
                await client.send_json({
                    'type': 'tts_output',
                    'text': data.get('text', ''),
                    'voice': data.get('voice', 'en+f3'),
                    'speed': data.get('speed', 150),
                    'pitch': data.get('pitch', 50)
                })
            except Exception as e:
                logger.error(f"Failed to send TTS to display client: {e}")
                disconnected_clients.append(client)
        
        # Clean up disconnected clients
        for client in disconnected_clients:
            if client in display_state.display_clients:
                display_state.display_clients.remove(client)
        
        logger.info(f"TTS broadcast to {len(display_state.display_clients)} display clients: {data.get('text', '')}")
        return {"status": "ok", "broadcast_to": len(display_state.display_clients)}
    except Exception as e:
        logger.error(f"Error in TTS broadcast: {e}")
        return {"status": "error", "message": str(e)}

@app.get("/health")
async def health_check():
    """Health check endpoint"""
    return {
        'status': 'healthy',
        'server': 'emotion_display',
        'port': 10000,
        'emotion': display_state.emotion,
        'clients': len(display_state.display_clients)
    }

if __name__ == "__main__":
    logger.info("=" * 60)
    logger.info("Emotion Display Server Starting...")
    logger.info("=" * 60)
    logger.info("")
    logger.info("🎨 Server Configuration:")
    logger.info(f"   - Host: 0.0.0.0")
    logger.info(f"   - Port: 10000")
    logger.info(f"   - Log File: {log_file}")
    logger.info("")
    logger.info("🌐 Access the emotion display at:")
    logger.info("   - http://localhost:10000")
    logger.info("")
    logger.info("📡 WebSocket Endpoint:")
    logger.info("   - /ws/emotion_display → Display clients")
    logger.info("")
    logger.info("🔄 Emotion Updates:")
    logger.info("   - Polls port 8000 for emotion state")
    logger.info("   - POST /api/emotion to update directly")
    logger.info("")
    logger.info("=" * 60)
    logger.info("")
    
    uvicorn.run(app, host="0.0.0.0", port=10000)
