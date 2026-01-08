#!/usr/bin/env python3
"""
Emotion Display Server - Port 10000
Dedicated server for displaying robot emotions/animated face.
Receives emotion updates from the primary control server (port 8000).
"""

import sys
import os

# Check critical dependencies before importing
try:
    from fastapi import FastAPI, WebSocket
    from fastapi.middleware.cors import CORSMiddleware
    from fastapi.staticfiles import StaticFiles
    from fastapi.responses import FileResponse, HTMLResponse
except ImportError as e:
    print("\n" + "="*70)
    print("ERROR: Missing required dependencies for Emotion Display Server")
    print("="*70)
    print(f"\nMissing module: {e.name if hasattr(e, 'name') else 'unknown'}")
    print("\nTo install required dependencies, run:")
    print("  pip3 install fastapi uvicorn aiohttp python-dotenv")
    print("\nOr install all dependencies:")
    print("  pip3 install -r requirements.txt")
    print("\nFor a complete dependency check, run:")
    print("  python3 check_dependencies.py 10000")
    print("="*70 + "\n")
    sys.exit(1)

try:
    import uvicorn
    import aiohttp
except ImportError as e:
    print("\n" + "="*70)
    print("ERROR: Missing required dependencies for Emotion Display Server")
    print("="*70)
    print(f"\nMissing module: {e.name if hasattr(e, 'name') else 'unknown'}")
    print("\nTo install required dependencies, run:")
    print("  pip3 install uvicorn aiohttp")
    print("="*70 + "\n")
    sys.exit(1)

from contextlib import asynccontextmanager
import asyncio
import logging
from datetime import datetime

try:
    from dotenv import load_dotenv
except ImportError:
    print("Warning: python-dotenv not installed. Environment variables must be set manually.")
    print("Install with: pip3 install python-dotenv")
    # Define a no-op load_dotenv function
    def load_dotenv():
        pass

load_dotenv()

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

SERVER_HOST = os.getenv("SERVER_HOST", "localhost")

class EmotionDisplayState:
    def __init__(self):
        self.emotion = "neutral"
        self.display_clients = []
        self.emotion_history = []  # Track emotion changes over time
        self.mood_analytics = {
            'happy': 0,
            'sad': 0,
            'angry': 0,
            'neutral': 0,
            'fear': 0,
            'surprise': 0,
            'disgust': 0
        }
        self.total_updates = 0
    
    async def set_emotion(self, emotion: str):
        """Update emotion and broadcast to all connected clients"""
        self.emotion = emotion
        self.total_updates += 1
        
        # Track emotion in history (keep last 100 entries)
        self.emotion_history.append({
            'emotion': emotion,
            'timestamp': datetime.now().isoformat()
        })
        if len(self.emotion_history) > 100:
            self.emotion_history.pop(0)
        
        # Update mood analytics
        if emotion in self.mood_analytics:
            self.mood_analytics[emotion] += 1
        
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
                        f'http://{SERVER_HOST}:8000/api/state',
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

# ============================================================================
# UNIQUE FEATURES FOR PORT 10000 - Emotion Analytics & Visualization
# ============================================================================

@app.get("/api/analytics")
async def get_emotion_analytics():
    """Get mood analytics - unique to port 10000"""
    total = sum(display_state.mood_analytics.values())
    percentages = {}
    if total > 0:
        percentages = {k: round(v / total * 100, 1) for k, v in display_state.mood_analytics.items()}
    
    # Calculate dominant mood
    dominant_mood = max(display_state.mood_analytics, key=display_state.mood_analytics.get) if total > 0 else 'neutral'
    
    return {
        'total_updates': display_state.total_updates,
        'current_emotion': display_state.emotion,
        'mood_counts': display_state.mood_analytics,
        'mood_percentages': percentages,
        'dominant_mood': dominant_mood,
        'connected_displays': len(display_state.display_clients)
    }

@app.get("/api/history")
async def get_emotion_history():
    """Get emotion history timeline - unique to port 10000"""
    return {
        'current_emotion': display_state.emotion,
        'history': display_state.emotion_history[-50:],  # Last 50 entries
        'total_entries': len(display_state.emotion_history)
    }

@app.post("/api/analytics/reset")
async def reset_analytics():
    """Reset mood analytics - unique to port 10000"""
    display_state.mood_analytics = {
        'happy': 0,
        'sad': 0,
        'angry': 0,
        'neutral': 0,
        'fear': 0,
        'surprise': 0,
        'disgust': 0
    }
    display_state.emotion_history = []
    display_state.total_updates = 0
    logger.info("Emotion analytics reset")
    return {"status": "ok", "message": "Analytics reset successfully"}

@app.get("/dashboard")
async def analytics_dashboard():
    """Serve the mood analytics dashboard - unique to port 10000"""
    return HTMLResponse(content="""
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Mood Analytics Dashboard - Port 10000</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
            background: linear-gradient(135deg, #1a1a2e 0%, #16213e 100%);
            min-height: 100vh;
            color: white;
            padding: 20px;
        }
        .header {
            text-align: center;
            padding: 20px 0;
            margin-bottom: 30px;
        }
        .header h1 { font-size: 32px; margin-bottom: 10px; }
        .header .subtitle { color: #888; font-size: 14px; }
        .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); gap: 20px; max-width: 1200px; margin: 0 auto; }
        .card {
            background: rgba(255, 255, 255, 0.05);
            border-radius: 15px;
            padding: 20px;
            backdrop-filter: blur(10px);
            border: 1px solid rgba(255, 255, 255, 0.1);
        }
        .card h2 { font-size: 18px; margin-bottom: 15px; color: #4ade80; }
        .current-emotion {
            text-align: center;
            font-size: 80px;
            padding: 20px;
        }
        .emotion-label { font-size: 24px; text-transform: capitalize; margin-top: 10px; }
        .stat-row { display: flex; justify-content: space-between; padding: 10px 0; border-bottom: 1px solid rgba(255,255,255,0.1); }
        .stat-row:last-child { border-bottom: none; }
        .stat-value { font-weight: bold; color: #4ade80; }
        .bar-chart { margin-top: 15px; }
        .bar-row { display: flex; align-items: center; margin: 8px 0; }
        .bar-label { width: 80px; font-size: 14px; }
        .bar-container { flex: 1; height: 20px; background: rgba(255,255,255,0.1); border-radius: 10px; overflow: hidden; }
        .bar-fill { height: 100%; border-radius: 10px; transition: width 0.5s ease; }
        .bar-value { width: 50px; text-align: right; font-size: 14px; }
        .timeline { max-height: 300px; overflow-y: auto; }
        .timeline-item { display: flex; padding: 8px 0; border-bottom: 1px solid rgba(255,255,255,0.05); }
        .timeline-time { width: 80px; font-size: 12px; color: #888; }
        .timeline-emotion { text-transform: capitalize; }
        .refresh-btn {
            background: #4ade80;
            border: none;
            padding: 10px 20px;
            border-radius: 8px;
            color: #1a1a2e;
            font-weight: bold;
            cursor: pointer;
            margin-top: 15px;
        }
        .refresh-btn:hover { background: #22c55e; }
    </style>
</head>
<body>
    <div class="header">
        <h1>📊 Mood Analytics Dashboard</h1>
        <div class="subtitle">Port 10000 - Emotion Display & Analytics Server</div>
    </div>
    
    <div class="grid">
        <div class="card">
            <h2>🎭 Current Emotion</h2>
            <div class="current-emotion" id="emotionIcon">😐</div>
            <div class="emotion-label" id="emotionLabel">Neutral</div>
        </div>
        
        <div class="card">
            <h2>📈 Statistics</h2>
            <div class="stat-row"><span>Total Updates</span><span class="stat-value" id="totalUpdates">0</span></div>
            <div class="stat-row"><span>Dominant Mood</span><span class="stat-value" id="dominantMood">-</span></div>
            <div class="stat-row"><span>Connected Displays</span><span class="stat-value" id="connectedDisplays">0</span></div>
        </div>
        
        <div class="card" style="grid-column: span 2;">
            <h2>📊 Mood Distribution</h2>
            <div class="bar-chart" id="moodChart"></div>
        </div>
        
        <div class="card" style="grid-column: span 2;">
            <h2>⏱️ Recent History</h2>
            <div class="timeline" id="timeline"></div>
            <button class="refresh-btn" onclick="loadData()">🔄 Refresh</button>
        </div>
    </div>
    
    <script>
        const emotionIcons = {
            'happy': '😊', 'sad': '😢', 'angry': '😠', 'neutral': '😐',
            'fear': '😨', 'surprise': '😲', 'disgust': '🤢'
        };
        const emotionColors = {
            'happy': '#ffd700', 'sad': '#4a90e2', 'angry': '#ff5050', 'neutral': '#4ade80',
            'fear': '#9b59b6', 'surprise': '#f39c12', 'disgust': '#27ae60'
        };
        
        async function loadData() {
            try {
                const [analytics, history] = await Promise.all([
                    fetch('/api/analytics').then(r => r.json()),
                    fetch('/api/history').then(r => r.json())
                ]);
                
                // Update current emotion
                document.getElementById('emotionIcon').textContent = emotionIcons[analytics.current_emotion] || '😐';
                document.getElementById('emotionLabel').textContent = analytics.current_emotion;
                
                // Update stats
                document.getElementById('totalUpdates').textContent = analytics.total_updates;
                document.getElementById('dominantMood').textContent = analytics.dominant_mood;
                document.getElementById('connectedDisplays').textContent = analytics.connected_displays;
                
                // Update chart
                const chartHtml = Object.entries(analytics.mood_percentages).map(([mood, pct]) => `
                    <div class="bar-row">
                        <div class="bar-label">${emotionIcons[mood] || ''} ${mood}</div>
                        <div class="bar-container">
                            <div class="bar-fill" style="width: ${pct}%; background: ${emotionColors[mood] || '#4ade80'}"></div>
                        </div>
                        <div class="bar-value">${pct}%</div>
                    </div>
                `).join('');
                document.getElementById('moodChart').innerHTML = chartHtml;
                
                // Update timeline
                const timelineHtml = history.history.slice(-20).reverse().map(item => `
                    <div class="timeline-item">
                        <div class="timeline-time">${new Date(item.timestamp).toLocaleTimeString()}</div>
                        <div class="timeline-emotion">${emotionIcons[item.emotion] || ''} ${item.emotion}</div>
                    </div>
                `).join('');
                document.getElementById('timeline').innerHTML = timelineHtml || '<div style="color: #888;">No history yet</div>';
                
            } catch (error) {
                console.error('Error loading data:', error);
            }
        }
        
        // WebSocket for real-time updates
        const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const ws = new WebSocket(`${protocol}//${window.location.host}/ws/emotion_display`);
        ws.onmessage = (event) => {
            const data = JSON.parse(event.data);
            if (data.type === 'emotion_update') {
                document.getElementById('emotionIcon').textContent = emotionIcons[data.emotion] || '😐';
                document.getElementById('emotionLabel').textContent = data.emotion;
                loadData(); // Refresh all data
            }
        };
        
        loadData();
        setInterval(loadData, 10000); // Refresh every 10 seconds
    </script>
</body>
</html>
    """)

if __name__ == "__main__":
    logger.info("=" * 60)
    logger.info("Emotion Display & Analytics Server Starting...")
    logger.info("=" * 60)
    logger.info("")
    logger.info("🎨 Server Configuration:")
    logger.info(f"   - Host: 0.0.0.0")
    logger.info(f"   - Port: 10000")
    logger.info(f"   - Log File: {log_file}")
    logger.info("")
    logger.info("🌐 Access Points:")
    logger.info("   - http://localhost:10000           → Emotion Display")
    logger.info("   - http://localhost:10000/dashboard → Analytics Dashboard")
    logger.info("")
    logger.info("📡 API Endpoints (Unique to Port 10000):")
    logger.info("   - GET  /api/analytics  → Mood statistics")
    logger.info("   - GET  /api/history    → Emotion timeline")
    logger.info("   - POST /api/analytics/reset → Reset analytics")
    logger.info("")
    logger.info("🔄 WebSocket:")
    logger.info("   - /ws/emotion_display → Real-time emotion updates")
    logger.info("")
    logger.info("=" * 60)
    logger.info("")
    
    uvicorn.run(app, host="0.0.0.0", port=10000)
