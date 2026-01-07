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
            height: 400px;
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
        
        .speed-control {
            margin-top: 15px;
        }
        
        .speed-slider {
            width: 100%;
            height: 8px;
            border-radius: 5px;
            background: rgba(255, 255, 255, 0.2);
            outline: none;
            -webkit-appearance: none;
        }
        
        .speed-slider::-webkit-slider-thumb {
            -webkit-appearance: none;
            appearance: none;
            width: 20px;
            height: 20px;
            border-radius: 50%;
            background: white;
            cursor: pointer;
        }
        
        .speed-slider::-moz-range-thumb {
            width: 20px;
            height: 20px;
            border-radius: 50%;
            background: white;
            cursor: pointer;
            border: none;
        }
        
        .joystick-container {
            display: flex;
            justify-content: center;
            align-items: center;
            margin: 20px auto;
            width: 200px;
            height: 200px;
            background: rgba(255, 255, 255, 0.1);
            border-radius: 50%;
            position: relative;
        }
        
        .joystick-handle {
            width: 80px;
            height: 80px;
            background: white;
            border-radius: 50%;
            position: absolute;
            cursor: grab;
            box-shadow: 0 4px 10px rgba(0, 0, 0, 0.3);
        }
        
        .joystick-handle:active {
            cursor: grabbing;
        }
        
        .mode-tabs {
            display: flex;
            gap: 10px;
            margin-bottom: 15px;
        }
        
        .mode-tab {
            flex: 1;
            padding: 10px;
            border: none;
            border-radius: 12px;
            background: rgba(255, 255, 255, 0.2);
            color: white;
            cursor: pointer;
            transition: all 0.2s;
        }
        
        .mode-tab.active {
            background: white;
            color: #667eea;
            font-weight: bold;
        }
        
        .control-mode-content {
            display: none;
        }
        
        .control-mode-content.active {
            display: block;
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
            <div class="mode-tabs">
                <button class="mode-tab active" onclick="switchControlMode('buttons')">Button Mode</button>
                <button class="mode-tab" onclick="switchControlMode('joystick')">Joystick Mode</button>
            </div>
            
            <div class="control-mode-content active" id="buttonMode">
                <div class="direction-pad">
                    <div></div>
                    <button class="direction-btn" onclick="move('forward')" onmousedown="startMove('forward')" onmouseup="stopMove()" ontouchstart="startMove('forward')" ontouchend="stopMove()">⬆️</button>
                    <div></div>
                    <button class="direction-btn" onclick="move('left')" onmousedown="startMove('left')" onmouseup="stopMove()" ontouchstart="startMove('left')" ontouchend="stopMove()">⬅️</button>
                    <button class="direction-btn" onclick="move('stop')">⏹️</button>
                    <button class="direction-btn" onclick="move('right')" onmousedown="startMove('right')" onmouseup="stopMove()" ontouchstart="startMove('right')" ontouchend="stopMove()">➡️</button>
                    <div></div>
                    <button class="direction-btn" onclick="move('backward')" onmousedown="startMove('backward')" onmouseup="stopMove()" ontouchstart="startMove('backward')" ontouchend="stopMove()">⬇️</button>
                    <div></div>
                </div>
            </div>
            
            <div class="control-mode-content" id="joystickMode">
                <div class="joystick-container" id="joystickContainer">
                    <div class="joystick-handle" id="joystickHandle"></div>
                </div>
            </div>
            
            <div class="speed-control">
                <label>Speed: <span id="speedValue">50</span>%</label>
                <input type="range" class="speed-slider" id="speedSlider" min="0" max="100" value="50" oninput="updateSpeed(this.value)">
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
        
        <div class="control-section">
            <h2>🎙️ Voice & TTS</h2>
            <div style="margin-bottom: 15px;">
                <label style="display: block; margin-bottom: 5px; font-size: 14px;">Voice:</label>
                <select id="voiceSelect" style="width: 100%; padding: 10px; border-radius: 8px; border: 2px solid rgba(255,255,255,0.2); background: rgba(255,255,255,0.1); color: white; font-size: 14px;">
                    <option value="en">English Male (Default)</option>
                    <option value="en+f1">English Female 1</option>
                    <option value="en+f2">English Female 2</option>
                    <option value="en+f3" selected>English Female 3</option>
                    <option value="en+f4">English Female 4</option>
                    <option value="en+m1">English Male 1</option>
                    <option value="en+m2">English Male 2</option>
                    <option value="en+m3">English Male 3</option>
                    <option value="en+m4">English Male 4 (Deep)</option>
                    <option value="en+m7">English Male 7 (Very Deep)</option>
                    <option value="en-us">American English</option>
                    <option value="en-uk">British English</option>
                    <option value="en-scottish">Scottish English</option>
                </select>
            </div>
            <div style="margin-bottom: 15px;">
                <label style="display: block; margin-bottom: 5px; font-size: 14px;">Speed: <span id="ttsSpeedValue">150</span> wpm</label>
                <input type="range" id="ttsSpeedSlider" min="80" max="450" value="150" style="width: 100%;" oninput="document.getElementById('ttsSpeedValue').textContent = this.value">
            </div>
            <div style="margin-bottom: 15px;">
                <label style="display: block; margin-bottom: 5px; font-size: 14px;">Pitch: <span id="ttsPitchValue">50</span></label>
                <input type="range" id="ttsPitchSlider" min="0" max="99" value="50" style="width: 100%;" oninput="document.getElementById('ttsPitchValue').textContent = this.value">
            </div>
            <textarea id="ttsInput" placeholder="Type text to speak..." style="width: 100%; padding: 10px; border-radius: 8px; border: 2px solid rgba(255,255,255,0.2); background: rgba(255,255,255,0.1); color: white; font-size: 14px; min-height: 80px; resize: vertical;"></textarea>
            <button onclick="sendTTS()" style="width: 100%; padding: 15px; margin-top: 10px; border: none; border-radius: 8px; background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); color: white; font-size: 16px; font-weight: bold; cursor: pointer;">🔊 Speak & Broadcast</button>
        </div>
    </div>
    
    <script>
        let cameraActive = false;
        let currentSpeed = 50;
        let moveInterval = null;
        let currentDirection = null;
        let joystickActive = false;
        
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
            } else if (data.type === 'tts_output') {
                // Play TTS audio on port 3000
                playTTSAudio(data.text);
            }
        };
        
        // Web Speech API for TTS playback on port 3000
        function playTTSAudio(text) {
            if ('speechSynthesis' in window) {
                window.speechSynthesis.cancel();
                
                const utterance = new SpeechSynthesisUtterance(text);
                const voices = window.speechSynthesis.getVoices();
                const voiceSelect = document.getElementById('voiceSelect').value;
                
                // Try to match voice
                if (voiceSelect.includes('f')) {
                    const femaleVoice = voices.find(v => v.lang.startsWith('en') && v.name.toLowerCase().includes('female'));
                    if (femaleVoice) utterance.voice = femaleVoice;
                } else if (voiceSelect.includes('m')) {
                    const maleVoice = voices.find(v => v.lang.startsWith('en') && v.name.toLowerCase().includes('male'));
                    if (maleVoice) utterance.voice = maleVoice;
                }
                
                const speed = parseInt(document.getElementById('ttsSpeedSlider').value);
                const pitch = parseInt(document.getElementById('ttsPitchSlider').value);
                
                utterance.rate = speed / 150;
                utterance.pitch = pitch / 50;
                
                window.speechSynthesis.speak(utterance);
                console.log(`Playing TTS on port 3000: "${text}"`);
            }
        }
        
        async function sendTTS() {
            const text = document.getElementById('ttsInput').value.trim();
            if (!text) {
                alert('Please enter text to speak');
                return;
            }
            
            const voice = document.getElementById('voiceSelect').value;
            const speed = parseInt(document.getElementById('ttsSpeedSlider').value);
            const pitch = parseInt(document.getElementById('ttsPitchSlider').value);
            
            try {
                // Send to primary server (port 8000) which will broadcast to all ports
                const response = await fetch('http://localhost:8000/api/speak', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ text, voice, speed, pitch })
                });
                
                if (response.ok) {
                    console.log('TTS sent successfully');
                    document.getElementById('ttsInput').value = '';
                } else {
                    console.error('Failed to send TTS');
                }
            } catch (error) {
                console.error('Error sending TTS:', error);
                alert('Failed to send TTS. Make sure port 8000 is running.');
            }
        }
        
        function move(direction) {
            const speed = Math.floor((currentSpeed / 100) * 255);
            ws.send(JSON.stringify({ type: 'move', direction: direction, speed: speed }));
        }
        
        function startMove(direction) {
            currentDirection = direction;
            move(direction);
        }
        
        function stopMove() {
            currentDirection = null;
            move('stop');
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
        
        function updateSpeed(value) {
            currentSpeed = parseInt(value);
            document.getElementById('speedValue').textContent = currentSpeed;
        }
        
        function switchControlMode(mode) {
            // Update tabs
            const tabs = document.querySelectorAll('.mode-tab');
            tabs.forEach(tab => tab.classList.remove('active'));
            
            // Find and activate the clicked tab
            const clickedTab = Array.from(tabs).find(tab => 
                (mode === 'buttons' && tab.textContent.includes('Button')) ||
                (mode === 'joystick' && tab.textContent.includes('Joystick'))
            );
            if (clickedTab) clickedTab.classList.add('active');
            
            // Update content
            const contents = document.querySelectorAll('.control-mode-content');
            contents.forEach(content => content.classList.remove('active'));
            
            if (mode === 'buttons') {
                document.getElementById('buttonMode').classList.add('active');
            } else {
                document.getElementById('joystickMode').classList.add('active');
            }
        }
        
        // Joystick functionality
        const joystickContainer = document.getElementById('joystickContainer');
        const joystickHandle = document.getElementById('joystickHandle');
        let isDragging = false;
        let centerX = 0;
        let centerY = 0;
        
        function initJoystick() {
            const rect = joystickContainer.getBoundingClientRect();
            centerX = rect.width / 2;
            centerY = rect.height / 2;
            joystickHandle.style.left = `${centerX - 40}px`;
            joystickHandle.style.top = `${centerY - 40}px`;
        }
        
        setTimeout(initJoystick, 100);
        
        function handleJoystickStart(e) {
            isDragging = true;
            e.preventDefault();
        }
        
        function handleJoystickMove(e) {
            if (!isDragging) return;
            e.preventDefault();
            
            const rect = joystickContainer.getBoundingClientRect();
            let clientX, clientY;
            
            if (e.type.includes('touch')) {
                clientX = e.touches[0].clientX;
                clientY = e.touches[0].clientY;
            } else {
                clientX = e.clientX;
                clientY = e.clientY;
            }
            
            let x = clientX - rect.left - centerX;
            let y = clientY - rect.top - centerY;
            
            // Limit to circle
            const distance = Math.sqrt(x * x + y * y);
            const maxDistance = 60;
            
            if (distance > maxDistance) {
                x = (x / distance) * maxDistance;
                y = (y / distance) * maxDistance;
            }
            
            joystickHandle.style.left = `${centerX + x - 40}px`;
            joystickHandle.style.top = `${centerY + y - 40}px`;
            
            // Determine direction
            const angle = Math.atan2(y, x) * (180 / Math.PI);
            let direction = 'stop';
            
            if (distance > 10) {
                if (angle >= -45 && angle < 45) direction = 'right';
                else if (angle >= 45 && angle < 135) direction = 'backward';
                else if (angle >= -135 && angle < -45) direction = 'forward';
                else direction = 'left';
            }
            
            if (direction !== currentDirection) {
                currentDirection = direction;
                move(direction);
            }
        }
        
        function handleJoystickEnd(e) {
            isDragging = false;
            e.preventDefault();
            
            // Return to center
            joystickHandle.style.left = `${centerX - 40}px`;
            joystickHandle.style.top = `${centerY - 40}px`;
            
            currentDirection = 'stop';
            move('stop');
        }
        
        joystickHandle.addEventListener('mousedown', handleJoystickStart);
        joystickHandle.addEventListener('touchstart', handleJoystickStart);
        document.addEventListener('mousemove', handleJoystickMove);
        document.addEventListener('touchmove', handleJoystickMove);
        document.addEventListener('mouseup', handleJoystickEnd);
        document.addEventListener('touchend', handleJoystickEnd);
        
        // WASD keyboard controls
        document.addEventListener('keydown', (e) => {
            if (e.repeat) return;
            
            let direction = null;
            switch(e.key.toLowerCase()) {
                case 'w': direction = 'forward'; break;
                case 'a': direction = 'left'; break;
                case 's': direction = 'backward'; break;
                case 'd': direction = 'right'; break;
            }
            
            if (direction && direction !== currentDirection) {
                currentDirection = direction;
                move(direction);
            }
        });
        
        document.addEventListener('keyup', (e) => {
            const key = e.key.toLowerCase();
            if (['w', 'a', 's', 'd'].includes(key)) {
                currentDirection = null;
                move('stop');
            }
        });
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

@app.post("/api/tts")
async def receive_tts(data: dict):
    """
    Receive TTS broadcast from port 8000
    Forwards TTS to all connected mobile clients for display/output
    """
    try:
        # Broadcast to all connected mobile clients
        disconnected_clients = []
        for client in mobile_state.mobile_clients:
            try:
                await client.send_json({
                    'type': 'tts_output',
                    'text': data.get('text', ''),
                    'voice': data.get('voice', 'en+f3'),
                    'speed': data.get('speed', 150),
                    'pitch': data.get('pitch', 50)
                })
            except Exception as e:
                logger.error(f"Failed to send TTS to mobile client: {e}")
                disconnected_clients.append(client)
        
        # Clean up disconnected clients
        for client in disconnected_clients:
            if client in mobile_state.mobile_clients:
                mobile_state.mobile_clients.remove(client)
        
        logger.info(f"TTS broadcast to {len(mobile_state.mobile_clients)} mobile clients: {data.get('text', '')}")
        return {"status": "ok", "broadcast_to": len(mobile_state.mobile_clients)}
    except Exception as e:
        logger.error(f"Error in TTS broadcast: {e}")
        return {"status": "error", "message": str(e)}

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
