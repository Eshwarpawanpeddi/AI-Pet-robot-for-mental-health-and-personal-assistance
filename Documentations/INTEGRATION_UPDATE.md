# Integration Update: Camera, Speech, and ROS Bridge

## Overview
This update adds comprehensive camera streaming, bidirectional audio/speech, enhanced ROS integration, and mental health monitoring to the AI Pet Robot system.

## New Features

### 1. Camera Streaming
**From Raspberry Pi to Server and Mobile App**

#### Implementation:
- **Raspberry Pi**: Uses `picamera2` to capture frames at ~10 FPS
- **Encoding**: Frames converted to JPEG and base64 encoded
- **Transport**: WebSocket streaming to server
- **Display**: 
  - Web interface: Split-panel layout with live camera view
  - Mobile app: CameraView widget with toggle control

#### Usage:
```python
# On Raspberry Pi - automatically initialized
# Camera starts when button clicked on web or app

# Server endpoint
POST /api/camera/start
POST /api/camera/stop
```

#### Web Interface:
- Camera view in right panel
- Toggle button: "Start Camera" / "Stop Camera"
- Real-time JPEG streaming

#### Mobile App:
- CameraView widget with toggle switch
- Base64 image decoding
- Automatic subscription to camera frames

### 2. Bidirectional Audio/Speech

#### Text-to-Speech on Raspberry Pi:
- **Engine**: espeak (lightweight, offline)
- **Voice**: English female voice (en+f3)
- **Speed**: 150 words per minute
- **Trigger**: Automatic when Gemini AI responds

#### Implementation:
```python
# Raspberry Pi
async def speak_text(self, text: str):
    subprocess.Popen(['espeak', '-v', 'en+f3', '-s', '150', text])

# Server
await raspberry_pi_client.send_json({
    "type": "speak",
    "text": response_text
})
```

#### Controls:
- **Web Interface**: 
  - "Start Listening" button (toggleable)
  - "Robot Speak" button
  - Visual feedback (red = active, green = idle)

- **Server API**:
```bash
POST /api/speak
{
  "text": "Hello, how are you feeling today?"
}
```

### 3. ROS Integration Bridge

#### Architecture:
```
FastAPI Server (port 8000)
    ↕ WebSocket (/ws/ros)
ROS Bridge Node
    ↕ ROS Topics
Robot Controller / Navigation
```

#### Features:
- **Autonomous Mode**: ROS takes control of navigation
- **Manual Mode**: Direct control from web/app
- **Mode Toggle**: Seamless switching via web interface
- **Obstacle Avoidance**: Uses LaserScan data in autonomous mode

#### ROS Bridge Node:
```bash
# Launch ROS bridge
roslaunch pet_robot_ros ros_bridge.launch server_url:=ws://YOUR_SERVER_IP:8000/ws/ros
```

#### Topics:
- **Subscribe**: `/cmd_vel`, `/robot/state`, `/scan`
- **Publish**: `/robot/set_emotion`

#### Mode Switching:
```javascript
// Web interface
function setMode(mode) {
    ws.send(JSON.stringify({ 
        type: 'set_mode', 
        mode: mode  // 'manual' or 'autonomous'
    }));
}
```

### 4. Mental Health Monitoring

#### Real-time Emotion Tracking:
- Tracks last 50 user emotions
- Detects patterns in emotional state
- Assigns concern level (0-10 scale)

#### Crisis Detection:
- Monitors for crisis keywords
- Immediate concern level escalation
- Recommendation for professional help

#### Mental State Analysis:
```python
def analyze_mental_state():
    recent = user_emotion_history[-10:]
    negative_count = sum(1 for e in recent if e in ['sad', 'angry'])
    
    if negative_count >= 7: return "concerning"
    elif negative_count >= 5: return "monitoring"
    else: return "stable"
```

#### API Endpoint:
```bash
GET /api/mental_health/insights

Response:
{
  "mental_state": "stable",
  "concern_level": 2,
  "recent_emotions": ["happy", "neutral", "happy"],
  "insights": [...],
  "recommendation": "You're doing well! Keep up the positive habits."
}
```

### 5. Enhanced Emotion Detection

#### Expanded Keywords:
- **Sad**: sad, cry, crying, lonely, hurt, depressed, unhappy, hopeless, worthless
- **Anxious**: anxious, anxiety, worried, scared, panic, stressed, overwhelmed
- **Angry**: angry, mad, hate, furious, annoyed, frustrated
- **Happy**: happy, good, yay, great, awesome, excited, wonderful, amazing

#### Mental Health Awareness:
- Positive emotions reduce concern level
- Negative emotions increase concern level
- Crisis keywords trigger immediate high concern

### 6. UI/UX Enhancements

#### Web Interface:
**Split-Panel Layout:**
- Left: Robot face with animated emotions
- Right: Camera, audio controls, mode toggle

**Animated Anger Emotion:**
- Shaking effect (sin wave motion)
- Pulsing red background
- Rotating spark effects
- Intensity-based animations

**Connection Status:**
- Visual indicators for Pi and ROS connections
- Real-time status updates
- Color-coded (green = connected, red = disconnected)

#### Mobile App:
- Camera view widget
- Control mode toggle (Manual/ROS)
- Enhanced WebSocket reconnection
- Camera frame display with base64 decoding

## Installation

### Raspberry Pi Dependencies:
```bash
pip3 install picamera2 pillow websockets
sudo apt-get install espeak
```

### ROS Dependencies:
```bash
cd ros_workspace/src/pet_robot_ros
pip install -r requirements.txt
```

### Server (Already included):
- FastAPI with WebSocket support
- Gemini AI integration

## Configuration

### Raspberry Pi:
Edit `raspberry_pi_controller.py`:
```python
SERVER_URL = "ws://YOUR_SERVER_IP:8000/ws/raspberry_pi"
```

### ROS Bridge:
```bash
roslaunch pet_robot_ros ros_bridge.launch server_url:=ws://YOUR_SERVER_IP:8000/ws/ros
```

### Mobile App:
Edit `lib/config/app_config.dart`:
```dart
static const String baseUrl = "http://YOUR_SERVER_IP:8000";
```

## Usage Examples

### 1. Start Complete System:

**Terminal 1 - Server:**
```bash
cd server
python server.py
```

**Terminal 2 - Raspberry Pi:**
```bash
cd hardware/raspberry_pi
python3 raspberry_pi_controller.py
```

**Terminal 3 - ROS (Optional for Autonomous Mode):**
```bash
roslaunch pet_robot_ros ros_bridge.launch
```

### 2. Web Interface:
Navigate to `http://localhost:8000`

**Features:**
- View robot face with emotions
- See camera feed from Pi
- Toggle listening/speaking
- Switch between Manual/ROS mode
- View connection status

### 3. Mobile App:
```bash
cd mobile_app
flutter run
```

**Features:**
- Robot face display
- Camera view with toggle
- Movement controls
- Mental health support features
- Mode switching

## API Reference

### WebSocket Endpoints:
- `/ws/control` - Web clients and mobile apps
- `/ws/raspberry_pi` - Raspberry Pi hardware
- `/ws/ros` - ROS bridge

### REST Endpoints:
- `GET /api/state` - Get current robot state
- `POST /api/control_mode` - Set control mode
- `POST /api/speak` - Trigger speech on Pi
- `GET /api/mental_health/insights` - Get mental health analysis

## Testing

### Camera:
1. Start server and Pi controller
2. Open web interface
3. Click "Start Camera"
4. Verify camera feed displays

### Speech:
1. Ensure espeak installed on Pi
2. Send text command via web interface
3. Verify audio output from Pi

### ROS:
1. Launch ROS bridge
2. Switch to autonomous mode via web
3. Verify ROS topics receiving commands
4. Test obstacle avoidance with laser scanner

### Mental Health:
1. Send various emotional messages
2. Check `/api/mental_health/insights`
3. Verify emotion tracking and recommendations

## Troubleshooting

### Camera Issues:
- Verify picamera2 installed: `pip3 show picamera2`
- Check camera enabled: `sudo raspi-config` → Interface Options → Camera
- Test camera: `libcamera-hello`

### Speech Issues:
- Test espeak: `espeak "Hello world"`
- Check audio output: `aplay -l`
- Verify audio device configured

### ROS Connection:
- Check server URL in launch file
- Verify WebSocket port 8000 open
- Test connection: `rostopic list`

### Mobile App:
- Check server IP in app_config.dart
- Verify same network as server
- Check WebSocket connection logs

## Performance Notes

### Camera Streaming:
- Frame Rate: ~10 FPS
- Resolution: 640x480
- Format: JPEG (quality 70)
- Bandwidth: ~50-100 KB/s

### Speech:
- Latency: <500ms
- Offline processing (no internet needed)
- CPU usage: Minimal

### Mental Health:
- History: Last 50 emotions
- Insights: Last 5 stored
- Update frequency: Per interaction

## Security Considerations

- Camera feed transmitted over local network
- No recording or storage of video
- Mental health data stored in memory only
- WebSocket connections can be secured with WSS
- Consider authentication for production use

## Future Enhancements

- Video recording capability
- Speech-to-text for voice input
- Advanced autonomous navigation
- Multi-user mental health tracking
- Cloud-based emotion analysis
- Encrypted camera streaming

## Contributing

When adding new features:
1. Update this documentation
2. Add API endpoints to server
3. Test on Raspberry Pi hardware
4. Verify mobile app compatibility
5. Update requirements.txt files

## Support

For issues:
- Check troubleshooting section
- Review server logs
- Test individual components
- Open GitHub issue with details
