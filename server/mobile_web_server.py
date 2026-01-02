#!/usr/bin/env python3
"""
Mobile Web Control Server - Port 3000
Mobile-friendly web-based control interface for basic robot movement,
camera view, and emotion changes. Communicates with primary server (port 8000).
"""
from fastapi import FastAPI, WebSocket
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse
from contextlib import asynccontextmanager
import asyncio
import logging
import uvicorn
import aiohttp
from typing import Optional

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class MobileControlState:
    def __init__(self):
        self.mobile_clients = []
        self.primary_server_url = "http://localhost:8000"
        self.primary_ws_url = "ws://localhost:8000/ws/control"
        self.primary_ws = None
        self.current_state = {}
        self.latest_camera_frame = None
    
    async def connect_to_primary(self):
        """Connect to primary control server via WebSocket"""
        import websockets
        try:
            logger.info("Connecting to primary server...")
            self.primary_ws = await websockets.connect(self.primary_ws_url)
            logger.info("Connected to primary server")
            
            # Subscribe to camera feed
            await self.primary_ws.send('{"type": "subscribe_camera"}')
            
            # Start listening for updates
            asyncio.create_task(self.listen_to_primary())
        except Exception as e:
            logger.error(f"Failed to connect to primary server: {e}")
            self.primary_ws = None
    
    async def listen_to_primary(self):
        """Listen for state updates from primary server"""
        try:
            while self.primary_ws:
                message = await self.primary_ws.recv()
                import json
                data = json.loads(message)
                
                if data.get('type') == 'state':
                    self.current_state = data.get('data', {})
                    await self.broadcast_state()
                elif data.get('type') == 'camera_frame':
                    self.latest_camera_frame = data.get('frame')
                    await self.broadcast_camera_frame()
        except Exception as e:
            logger.error(f"Error listening to primary: {e}")
            self.primary_ws = None
    
    async def broadcast_state(self):
        """Broadcast state to all mobile clients"""
        for client in self.mobile_clients[:]:
            try:
                await client.send_json({
                    'type': 'state',
                    'data': self.current_state
                })
            except:
                self.mobile_clients.remove(client)
    
    async def broadcast_camera_frame(self):
        """Broadcast camera frame to all mobile clients"""
        if not self.latest_camera_frame:
            return
        
        for client in self.mobile_clients[:]:
            try:
                await client.send_json({
                    'type': 'camera_frame',
                    'frame': self.latest_camera_frame
                })
            except:
                self.mobile_clients.remove(client)
    
    async def send_command(self, command: dict):
        """Send command to primary server"""
        if self.primary_ws:
            import json
            await self.primary_ws.send(json.dumps(command))
        else:
            logger.warning("Not connected to primary server")

mobile_state = MobileControlState()

@asynccontextmanager
async def lifespan(app: FastAPI):
    """Application lifespan manager"""
    logger.info("Mobile Web Control Server starting...")
    
    # Connect to primary server
    await mobile_state.connect_to_primary()
    
    yield
    
    # Cleanup
    if mobile_state.primary_ws:
        await mobile_state.primary_ws.close()

app = FastAPI(lifespan=lifespan)
app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"])

@app.get("/")
async def root():
    """Serve the mobile control interface"""
    return HTMLResponse(content="""
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
    <title>Robot Mobile Control</title>
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
            padding: 10px;
            color: white;
        }
        
        .container {
            max-width: 600px;
            margin: 0 auto;
        }
        
        .header {
            text-align: center;
            padding: 20px 0;
        }
        
        .header h1 {
            font-size: 24px;
            margin-bottom: 10px;
        }
        
        .status-badge {
            display: inline-block;
            padding: 5px 15px;
            border-radius: 20px;
            font-size: 12px;
            background: rgba(255, 255, 255, 0.2);
        }
        
        .status-badge.connected {
            background: #10b981;
        }
        
        .camera-view {
            width: 100%;
            height: 250px;
            background: rgba(0, 0, 0, 0.3);
            border-radius: 15px;
            margin-bottom: 20px;
            display: flex;
            align-items: center;
            justify-content: center;
            overflow: hidden;
        }
        
        .camera-view img {
            max-width: 100%;
            max-height: 100%;
            border-radius: 15px;
        }
        
        .camera-placeholder {
            color: rgba(255, 255, 255, 0.5);
        }
        
        .control-section {
            background: rgba(255, 255, 255, 0.1);
            border-radius: 15px;
            padding: 20px;
            margin-bottom: 15px;
        }
        
        .control-section h2 {
            font-size: 18px;
            margin-bottom: 15px;
        }
        
        .direction-pad {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 10px;
            max-width: 300px;
            margin: 0 auto;
        }
        
        .direction-btn {
            aspect-ratio: 1;
            border: none;
            border-radius: 12px;
            background: rgba(255, 255, 255, 0.2);
            color: white;
            font-size: 24px;
            cursor: pointer;
            transition: all 0.2s;
            backdrop-filter: blur(10px);
        }
        
        .direction-btn:active {
            transform: scale(0.95);
            background: rgba(255, 255, 255, 0.3);
        }
        
        .direction-btn:disabled {
            opacity: 0.3;
            cursor: not-allowed;
        }
        
        .emotion-buttons {
            display: grid;
            grid-template-columns: repeat(2, 1fr);
            gap: 10px;
        }
        
        .emotion-btn {
            padding: 15px;
            border: none;
            border-radius: 12px;
            background: rgba(255, 255, 255, 0.2);
            color: white;
            font-size: 16px;
            cursor: pointer;
            transition: all 0.2s;
        }
        
        .emotion-btn:active {
            transform: scale(0.95);
            background: rgba(255, 255, 255, 0.3);
        }
        
        .toggle-btn {
            width: 100%;
            padding: 15px;
            border: none;
            border-radius: 12px;
            font-size: 16px;
            cursor: pointer;
            transition: all 0.2s;
            margin-top: 10px;
        }
        
        .toggle-btn.active {
            background: #10b981;
            color: white;
        }
        
        .toggle-btn.inactive {
            background: rgba(255, 255, 255, 0.2);
            color: white;
        }
        
        .info-row {
            display: flex;
            justify-content: space-between;
            padding: 10px 0;
            border-bottom: 1px solid rgba(255, 255, 255, 0.1);
        }
        
        .info-row:last-child {
            border-bottom: none;
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>🤖 Robot Control</h1>
            <span class="status-badge" id="statusBadge">Connecting...</span>
        </div>
        
        <div class="camera-view" id="cameraView">
            <div class="camera-placeholder">Camera Off</div>
        </div>
        
        <div class="control-section">
            <h2>📹 Camera</h2>
            <button class="toggle-btn inactive" id="cameraBtn" onclick="toggleCamera()">Start Camera</button>
        </div>
        
        <div class="control-section">
            <h2>🎮 Movement</h2>
            <div class="direction-pad">
                <div></div>
                <button class="direction-btn" onclick="move('forward')">⬆️</button>
                <div></div>
                <button class="direction-btn" onclick="move('left')">⬅️</button>
                <button class="direction-btn" onclick="move('stop')">⏹️</button>
                <button class="direction-btn" onclick="move('right')">➡️</button>
                <div></div>
                <button class="direction-btn" onclick="move('backward')">⬇️</button>
                <div></div>
            </div>
        </div>
        
        <div class="control-section">
            <h2>😊 Emotions</h2>
            <div class="emotion-buttons">
                <button class="emotion-btn" onclick="setEmotion('happy')">😊 Happy</button>
                <button class="emotion-btn" onclick="setEmotion('sad')">😢 Sad</button>
                <button class="emotion-btn" onclick="setEmotion('angry')">😠 Angry</button>
                <button class="emotion-btn" onclick="setEmotion('neutral')">😐 Neutral</button>
            </div>
        </div>
        
        <div class="control-section">
            <h2>ℹ️ Status</h2>
            <div class="info-row">
                <span>Robot Emotion</span>
                <strong id="robotEmotion">neutral</strong>
            </div>
            <div class="info-row">
                <span>Battery</span>
                <strong id="battery">100%</strong>
            </div>
            <div class="info-row">
                <span>Control Mode</span>
                <strong id="controlMode">Manual</strong>
            </div>
        </div>
    </div>
    
    <script>
        let cameraActive = false;
        
        const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const ws = new WebSocket(`${protocol}//${window.location.host}/ws/mobile`);
        
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
            
            if (data.type === 'state') {
                const state = data.data;
                document.getElementById('robotEmotion').textContent = state.emotion || 'neutral';
                document.getElementById('battery').textContent = (state.battery_level || 100) + '%';
                document.getElementById('controlMode').textContent = state.control_mode || 'Manual';
            } else if (data.type === 'camera_frame') {
                const cameraView = document.getElementById('cameraView');
                cameraView.innerHTML = `<img src="data:image/jpeg;base64,${data.frame}" alt="Camera">`;
            }
        };
        
        function move(direction) {
            ws.send(JSON.stringify({ type: 'move', direction: direction, speed: 200 }));
        }
        
        function setEmotion(emotion) {
            ws.send(JSON.stringify({ type: 'emotion', emotion: emotion }));
        }
        
        function toggleCamera() {
            cameraActive = !cameraActive;
            const btn = document.getElementById('cameraBtn');
            
            if (cameraActive) {
                ws.send(JSON.stringify({ type: 'start_camera' }));
                btn.textContent = 'Stop Camera';
                btn.className = 'toggle-btn active';
            } else {
                ws.send(JSON.stringify({ type: 'stop_camera' }));
                btn.textContent = 'Start Camera';
                btn.className = 'toggle-btn inactive';
                document.getElementById('cameraView').innerHTML = '<div class="camera-placeholder">Camera Off</div>';
            }
        }
    </script>
</body>
</html>
    """)

@app.websocket("/ws/mobile")
async def websocket_mobile(websocket: WebSocket):
    """WebSocket for mobile clients"""
    await websocket.accept()
    mobile_state.mobile_clients.append(websocket)
    logger.info(f"Mobile client connected. Total: {len(mobile_state.mobile_clients)}")
    
    # Send current state immediately
    if mobile_state.current_state:
        try:
            await websocket.send_json({
                'type': 'state',
                'data': mobile_state.current_state
            })
        except:
            pass
    
    try:
        while True:
            # Receive commands from mobile client and forward to primary
            data = await websocket.receive_json()
            await mobile_state.send_command(data)
    except:
        if websocket in mobile_state.mobile_clients:
            mobile_state.mobile_clients.remove(websocket)
        logger.info(f"Mobile client disconnected. Total: {len(mobile_state.mobile_clients)}")

@app.get("/health")
async def health_check():
    """Health check endpoint"""
    return {
        'status': 'healthy',
        'server': 'mobile_web_control',
        'port': 3000,
        'clients': len(mobile_state.mobile_clients),
        'primary_connected': mobile_state.primary_ws is not None
    }

@app.get("/api/status")
async def get_status():
    """Get current status"""
    return {
        'connected_to_primary': mobile_state.primary_ws is not None,
        'mobile_clients': len(mobile_state.mobile_clients),
        'current_state': mobile_state.current_state
    }

if __name__ == "__main__":
    logger.info("=" * 60)
    logger.info("Mobile Web Control Server Starting...")
    logger.info("=" * 60)
    logger.info("")
    logger.info("📱 Server Configuration:")
    logger.info(f"   - Host: 0.0.0.0")
    logger.info(f"   - Port: 3000")
    logger.info("")
    logger.info("🌐 Access the mobile interface at:")
    logger.info("   - http://localhost:3000")
    logger.info("")
    logger.info("📡 WebSocket Endpoint:")
    logger.info("   - /ws/mobile → Mobile clients")
    logger.info("")
    logger.info("🔄 Primary Server Connection:")
    logger.info("   - Connects to port 8000 for commands/state")
    logger.info("   - Forwards all mobile commands")
    logger.info("")
    logger.info("=" * 60)
    logger.info("")
    
    uvicorn.run(app, host="0.0.0.0", port=3000)
