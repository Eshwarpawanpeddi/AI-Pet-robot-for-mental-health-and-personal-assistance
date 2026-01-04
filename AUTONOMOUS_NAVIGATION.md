# Autonomous Navigation Feature Guide

## Overview

The AI Pet Robot now includes an **autonomous navigation system** that uses YOLO (You Only Look Once) object detection for real-time obstacle avoidance and intelligent movement. The robot can navigate independently while avoiding obstacles detected through its camera.

## Features

### 🤖 Intelligent Navigation
- **Real-time Object Detection**: Uses YOLOv8 Nano for fast, accurate object detection
- **Obstacle Avoidance**: Automatically detects and avoids obstacles in the robot's path
- **Dynamic Speed Adjustment**: Adjusts speed based on obstacle proximity
- **Smart Decision Making**: Chooses optimal paths based on detected obstacles
- **Smooth Transitions**: Gradual changes in direction and speed

### 📹 Shared Camera Stream
- **Multi-Consumer Architecture**: Camera stream shared between navigation and emotion detection
- **Non-Blocking Design**: Navigation and emotion detection run in parallel without interference
- **Efficient Frame Distribution**: Publisher-subscriber pattern for optimal performance
- **Thread-Safe Operations**: Concurrent access to camera frames without conflicts

### 🎮 Control Modes
- **Manual Mode**: Direct user control via web interface or mobile app
- **Autonomous Navigation**: Robot navigates independently with obstacle avoidance
- **ROS Autonomous**: Full ROS-based autonomous navigation (existing feature)

## Architecture

### System Components

```
┌─────────────────────────────────────────────────────────────┐
│                    Camera Stream Flow                        │
└─────────────────────────────────────────────────────────────┘

Raspberry Pi Camera
        │
        ▼
WebSocket (camera_frame) ──> Server (Port 8000)
                                      │
                                      ▼
                          Camera Stream Manager
                            (publisher-subscriber)
                                      │
                    ┌─────────────────┴─────────────────┐
                    ▼                                   ▼
        Autonomous Navigator               Emotion Detection Server
          (YOLOv8 Detection)                      (Port 9999)
                    │                                   │
                    ▼                                   ▼
            Motor Commands                    Facial Emotion Analysis
```

### Module Overview

#### 1. **camera_stream_manager.py**
Manages shared access to camera frames:
- **CameraStreamManager**: Central hub for camera frame distribution
- **Subscriber Pattern**: Multiple services can subscribe to frames
- **Thread-Safe**: Locks protect concurrent access
- **Buffering**: Maintains recent frame history
- **Statistics**: Tracks frame count, subscribers, streaming status

#### 2. **autonomous_navigation.py**
Handles autonomous navigation logic:
- **AutonomousNavigator**: Main navigation controller
- **YOLO Integration**: YOLOv8 object detection
- **Scene Analysis**: Interprets detected objects for navigation decisions
- **Motor Commands**: Generates movement commands based on obstacles
- **Navigation Modes**: IDLE, EXPLORING, AVOIDING, STOPPED
- **Configurable Parameters**: Speed, thresholds, sensitivity

#### 3. **server.py** (Enhanced)
Integrated navigation into main server:
- Navigation initialization on startup
- WebSocket commands for navigation control
- REST API endpoints for navigation
- State broadcasting includes navigation status

## Installation

### Prerequisites

The autonomous navigation system requires:

1. **Python 3.9+** (already required)
2. **OpenCV** (already installed for emotion detection)
3. **Ultralytics YOLOv8** (new dependency)

### Install Dependencies

```bash
cd server
pip install -r requirements.txt
```

This will install:
- `ultralytics==8.0.196` - YOLOv8 object detection

### Download YOLO Model

The YOLOv8 Nano model will be automatically downloaded on first use. No manual download required!

**Model Details:**
- **Model**: YOLOv8n (Nano)
- **Size**: ~6 MB
- **Speed**: ~80 FPS on GPU, ~30 FPS on CPU
- **Accuracy**: Good balance between speed and accuracy
- **Classes**: 80 COCO dataset classes (person, car, chair, etc.)

## Usage

### Starting the System

#### Option 1: Start All Servers
```bash
cd server
python launch_all.py
```

This starts:
- Port 8000: Primary control server (with navigation)
- Port 10000: Emotion display server
- Port 3000: Mobile web interface
- Port 9999: Emotion detection server (if available)

#### Option 2: Start Primary Server Only
```bash
cd server
python server.py
```

### Enabling Autonomous Navigation

#### Via Web Interface

Access `http://localhost:8000` and use the control panel:

1. **Start Camera**: Enable camera streaming
2. **Switch to Autonomous Navigation**: Select mode from dropdown
3. **Start Navigation**: Click the "Start Navigation" button
4. **Monitor Status**: Watch navigation status and detected obstacles
5. **Stop Navigation**: Click "Stop Navigation" when done

#### Via REST API

**Start Navigation:**
```bash
curl -X POST http://localhost:8000/api/navigation/start
```

Response:
```json
{
  "success": true,
  "message": "Autonomous navigation started",
  "status": {
    "enabled": true,
    "running": true,
    "mode": "exploring",
    "model_loaded": true
  }
}
```

**Stop Navigation:**
```bash
curl -X POST http://localhost:8000/api/navigation/stop
```

**Get Status:**
```bash
curl http://localhost:8000/api/navigation/status
```

Response:
```json
{
  "navigation": {
    "enabled": true,
    "running": true,
    "mode": "avoiding",
    "last_command": "left",
    "detection_history_size": 10
  },
  "parameters": {
    "model_name": "yolov8n",
    "confidence_threshold": 0.5,
    "obstacle_distance_threshold": 0.3,
    "safe_speed": 50,
    "caution_speed": 30
  },
  "camera_stream": {
    "frame_count": 1523,
    "subscribers": 2,
    "is_streaming": true
  }
}
```

#### Via WebSocket

Connect to `ws://localhost:8000/ws/control`:

**Start Navigation:**
```json
{
  "type": "start_navigation"
}
```

**Stop Navigation:**
```json
{
  "type": "stop_navigation"
}
```

### Configuration

#### Adjust Navigation Parameters

```bash
curl -X POST http://localhost:8000/api/navigation/parameters \
  -H "Content-Type: application/json" \
  -d '{
    "confidence_threshold": 0.6,
    "obstacle_distance_threshold": 0.4,
    "safe_speed": 60,
    "caution_speed": 35
  }'
```

**Parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `confidence_threshold` | 0.5 | Minimum confidence for object detection (0.0-1.0) |
| `obstacle_distance_threshold` | 0.3 | Fraction of image height for "close" objects (0.0-1.0) |
| `safe_speed` | 50 | Speed when path is clear (0-100) |
| `caution_speed` | 30 | Speed when obstacles detected (0-100) |
| `min_safe_width` | 0.2 | Minimum clear width fraction to proceed (0.0-1.0) |
| `frame_skip` | 2 | Process every Nth frame (1=all frames, higher=better performance) |

## How It Works

### Navigation Logic

1. **Frame Reception**: Camera frames arrive via WebSocket from Raspberry Pi
2. **Frame Distribution**: Camera Stream Manager publishes frames to subscribers
3. **Object Detection**: YOLO model detects objects in frame
4. **Scene Analysis**: Navigator analyzes object positions
5. **Decision Making**: Determines optimal movement command
6. **Command Execution**: Sends motor commands to Raspberry Pi

### Obstacle Detection

Objects are classified by position in the frame:

```
┌─────────────────────────────────┐
│           Camera View           │
│                                 │
│  Left Region  │ Center │ Right │
│      33%      │  33%   │  33%  │
│               │        │       │
│               │        │       │
│  ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─  │ ← Distance Threshold (70% down)
│               │        │       │
│  [Obstacle]   │  [Obs] │       │ ← Close obstacles
│               │        │       │
└─────────────────────────────────┘
```

### Decision Tree

```
Is path clear?
├─ YES → Move forward at safe speed
└─ NO
   ├─ Obstacle ahead?
   │  ├─ YES → Turn toward side with fewer obstacles
   │  └─ NO → Check sides
   │     ├─ Left blocked → Steer right
   │     ├─ Right blocked → Steer left
   │     └─ Both sides blocked
   │        ├─ Center clear → Proceed slowly
   │        └─ Surrounded → STOP
```

### Speed Control

- **Safe Speed (50)**: Used when path is clear
- **Caution Speed (30)**: Used when obstacles detected
- **Slow Speed (15)**: Used in narrow passages
- **Stop (0)**: Used when completely blocked

## Compatibility

### With Emotion Detection (Port 9999)

✅ **Fully Compatible**

The emotion detection server continues to work normally:
- Connects to port 8000 via WebSocket
- Receives camera frames independently
- No interference with navigation
- Both systems process frames in parallel

### With ROS Navigation

✅ **Complementary**

Three control modes are available:
- **Manual**: User controls robot directly
- **Autonomous Navigation**: YOLO-based obstacle avoidance (new)
- **ROS Autonomous**: Full ROS SLAM and navigation (existing)

Switch between modes via the control interface or API.

### With Existing Camera Streaming

✅ **Enhanced**

All existing camera features still work:
- Web interface camera view
- Mobile app camera stream
- Manual camera start/stop
- Camera frame subscriptions

The Camera Stream Manager enhances these features with better multi-consumer support.

## Testing and Debugging

### Test Navigation System

#### 1. Check Model Loading
```bash
curl http://localhost:8000/api/navigation/status
```

Expected output should include:
```json
{
  "navigation": {
    "model_loaded": true
  }
}
```

#### 2. Test with Static Object

Place an object in front of the camera and observe:
- Detection in navigation status
- Motor command changes
- Speed adjustments

#### 3. Monitor Logs

Server logs show navigation activity:
```
INFO - Navigation: Avoiding obstacle ahead - turning left - left at speed 30
INFO - Navigation: Path clear - forward at speed 50
```

### Debug Common Issues

#### Model Not Loading

**Issue**: Navigation fails to start with model error

**Solution**:
```bash
# Install ultralytics
pip install ultralytics

# Test model download
python -c "from ultralytics import YOLO; YOLO('yolov8n.pt')"
```

#### Camera Not Streaming

**Issue**: No frames reaching navigation

**Check**:
```bash
# Check camera stream stats
curl http://localhost:8000/api/navigation/status | jq '.camera_stream'

# Expected output:
{
  "frame_count": 123,
  "subscribers": 2,
  "is_streaming": true
}
```

**Solution**: Ensure Raspberry Pi is connected and camera is enabled

#### Navigation Not Responding

**Issue**: Robot doesn't move despite navigation running

**Check**:
1. Raspberry Pi connected: `curl http://localhost:8000/api/status`
2. Navigation enabled: `curl http://localhost:8000/api/navigation/status`
3. No objects detected initially: May be waiting for obstacles

**Solution**: Place object in camera view to trigger navigation response

### Performance Optimization

#### On Limited Hardware

If running on CPU with limited resources:

1. **Lower Frame Rate**: Raspberry Pi camera can stream at 5 FPS instead of 10 FPS
2. **Reduce Detection Frequency**: Process every 2nd or 3rd frame
3. **Use YOLOv8n**: Already the smallest model (nano)
4. **Limit Subscribers**: Only enable needed services

#### Recommended Hardware

- **CPU**: 4+ cores recommended
- **RAM**: 4GB+ 
- **For GPU**: NVIDIA GPU with CUDA for 5x faster detection
- **Raspberry Pi**: 4GB model for camera streaming

## API Reference

### REST Endpoints

#### POST /api/navigation/start
Start autonomous navigation mode.

**Response:**
```json
{
  "success": true,
  "message": "Autonomous navigation started",
  "status": { /* navigation status */ }
}
```

#### POST /api/navigation/stop
Stop autonomous navigation mode.

**Response:**
```json
{
  "success": true,
  "message": "Autonomous navigation stopped"
}
```

#### GET /api/navigation/status
Get current navigation status and statistics.

**Response:**
```json
{
  "navigation": {
    "enabled": true,
    "running": true,
    "mode": "exploring",
    "last_command": "forward",
    "model_loaded": true
  },
  "parameters": { /* current parameters */ },
  "camera_stream": { /* stream statistics */ }
}
```

#### POST /api/navigation/parameters
Update navigation parameters.

**Request Body:**
```json
{
  "confidence_threshold": 0.6,
  "safe_speed": 60
}
```

**Response:**
```json
{
  "success": true,
  "message": "Parameters updated",
  "parameters": { /* updated parameters */ }
}
```

### WebSocket Commands

#### Start Navigation
```json
{
  "type": "start_navigation"
}
```

#### Stop Navigation
```json
{
  "type": "stop_navigation"
}
```

## Safety Considerations

### Operational Safety

1. **Supervised Testing**: Always supervise the robot during autonomous navigation
2. **Safe Environment**: Test in open areas without fragile objects
3. **Emergency Stop**: Keep manual control available to stop the robot
4. **Height Awareness**: Camera-based navigation may not detect low obstacles
5. **Speed Limits**: Keep speeds reasonable for safe stopping

### Technical Limitations

- **2D Vision Only**: Cannot detect holes, stairs, or height differences
- **Lighting Dependent**: Requires adequate lighting for camera
- **Processing Delay**: ~100-300ms delay between detection and action
- **False Positives**: May detect non-obstacles as obstacles
- **Network Dependent**: Requires stable connection to Raspberry Pi

## Troubleshooting

### Navigation Commands Not Executing

**Symptoms**: Navigation running but robot not moving

**Checks**:
1. Raspberry Pi connected
2. Motors responding to manual commands
3. Navigation mode active
4. Camera streaming enabled

**Fix**: Restart Raspberry Pi controller and server

### High CPU Usage

**Symptoms**: Server using excessive CPU

**Cause**: YOLO detection on every frame

**Solutions**:
1. Increase frame skip (default is 2):
```bash
# Process every 3rd frame instead of every 2nd
curl -X POST http://localhost:8000/api/navigation/parameters \
  -H "Content-Type: application/json" \
  -d '{"frame_skip": 3}'
```
2. Reduce camera frame rate on Raspberry Pi (edit raspberry_pi_controller.py)
3. Use GPU if available (CUDA-enabled GPU provides 5x speed boost)
4. Lower camera resolution if possible (in Raspberry Pi config)

### Erratic Movement

**Symptoms**: Robot frequently changing direction

**Cause**: High sensitivity or noisy detections

**Fix**:
```bash
# Increase confidence threshold
curl -X POST http://localhost:8000/api/navigation/parameters \
  -H "Content-Type: application/json" \
  -d '{"confidence_threshold": 0.7}'
```

### Model Download Fails

**Symptoms**: Cannot download YOLOv8 model

**Cause**: Network restrictions or firewall

**Solution**:
```bash
# Manual download
wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt

# Place in project root or specify path in code
```

## Future Enhancements

Potential improvements for future versions:

1. **Path Planning**: Add memory of explored areas
2. **Goal-Based Navigation**: Navigate to specific targets
3. **Multi-Sensor Fusion**: Integrate ultrasonic sensors
4. **Learning**: Learn optimal paths over time
5. **3D Mapping**: Add depth perception with stereo camera
6. **Voice Commands**: "Go to the kitchen"
7. **Person Following**: Follow a specific person
8. **Integration with ROS**: Bridge to ROS navigation stack

## Support

### Getting Help

- **Documentation**: This guide and README.md
- **GitHub Issues**: Report bugs and request features
- **Logs**: Check server logs for detailed error messages

### Reporting Issues

When reporting navigation issues, include:

1. Server logs (last 50 lines)
2. Navigation status output
3. Camera stream statistics
4. Hardware specifications
5. Steps to reproduce

### Contributing

Contributions welcome! Areas needing improvement:

- Algorithm optimization
- Additional navigation strategies
- Better obstacle classification
- Performance enhancements
- Documentation improvements

---

**Version**: 1.0.0  
**Last Updated**: January 2026  
**Compatible With**: AI Pet Robot v2.1+
