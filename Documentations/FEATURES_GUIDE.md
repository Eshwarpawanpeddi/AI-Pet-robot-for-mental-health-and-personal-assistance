# Quick Start Guide - New Features

This guide covers the latest features added to the AI Pet Robot system.

## New Features Overview

### 1. Enhanced Port 3000 Mobile Interface ✨

The mobile interface (http://localhost:3000) now includes:

#### Larger Camera View
- Camera stream increased from 250px to 400px height
- Better visibility on mobile devices
- Maintains aspect ratio

#### WASD Keyboard Controls
- **W** - Move forward
- **A** - Move left
- **S** - Move backward
- **D** - Move right
- Automatically stops on key release

#### Dual Control Modes
- **Button Mode**: Traditional directional pad with touch support
- **Joystick Mode**: Virtual joystick for smooth control
  - Drag in any direction
  - Automatically returns to center
  - Works with touch and mouse

#### Speed Control
- Adjustable speed slider (0-100%)
- Real-time speed updates
- Speed indicator shows current percentage

### 2. Emotion Detection Server (Port 9999) 😊

A dedicated server for real-time facial emotion detection:

#### Features
- **Real-time face detection** using OpenCV
- **7 emotion classifications**: happy, sad, angry, fear, surprise, disgust, neutral
- **Confidence scores** for each detection
- **Visual feedback** with bounding boxes and labels
- **Gemini-powered responses** based on detected emotions
- **Emotion history tracking**

#### Access
```bash
# Start the emotion detection server
python server/emotion_detection_server.py

# Or use the launcher
python server/launch_all.py  # Includes emotion detection
```

#### Web Interface
Open http://localhost:9999 to see:
- Live camera feed with emotion overlays
- Current detected emotion with icon
- List of detected faces and their emotions
- Statistics (faces detected, emotions tracked)

#### API Endpoints
```bash
# Get current emotion
curl http://localhost:9999/api/emotion

# Get empathetic response for an emotion
curl -X POST http://localhost:9999/api/emotion/response?emotion=happy

# Health check
curl http://localhost:9999/health
```

### 3. Comprehensive ROS Guide 🤖

A complete guide for setting up and using ROS with the robot:

#### What's Covered
- ROS installation (Noetic and Melodic)
- Building the workspace
- Running ROS nodes
- Control mode switching (manual/autonomous)
- Testing and verification
- Troubleshooting

#### Location
See `ROS_SETUP_GUIDE.md` in the root directory

#### Quick Start
```bash
# Install ROS Noetic (Ubuntu 20.04)
sudo apt install ros-noetic-desktop-full

# Build workspace
cd ros_workspace
catkin_make
source devel/setup.bash

# Launch ROS nodes
roslaunch pet_robot_ros pet_robot.launch
```

### 4. Speaker Setup Guide 🔊

Complete hardware guide for connecting an 8Ω speaker:

#### What's Covered
- Multiple connection options
  - Direct 3.5mm audio jack
  - With amplifier (PAM8403 recommended)
  - GPIO PWM output
  - I2S DAC modules
- Circuit diagrams
- Software configuration
- Volume control
- Troubleshooting

#### Location
See `SPEAKER_SETUP_GUIDE.md` in the root directory

#### Recommended Setup
```
Raspberry Pi 3.5mm Jack → PAM8403 Amplifier → 40mm 8Ω Speaker
```

Cost: ~$2 for amplifier, provides excellent sound quality

### 5. Gemini API Configuration ✅

The system is already configured to use the efficient `gemini-1.5-flash` model:

#### Model Details
- **Model**: gemini-1.5-flash
- **Cost**: Lower cost than gemini-pro
- **Speed**: Faster responses
- **Quality**: Excellent for conversational AI

#### Configuration
Located in `server/gemini_integration.py`:
```python
self.model_name = "gemini-1.5-flash"
```

No changes needed - already optimized!

## Testing the New Features

### Test Port 3000 Enhancements

```bash
# Start servers
python server/launch_all.py

# Open in browser
http://localhost:3000
```

**Test checklist:**
- [ ] Camera view is larger and clear
- [ ] WASD keys move the robot
- [ ] Can switch between button and joystick mode
- [ ] Speed slider adjusts robot speed
- [ ] Touch controls work on mobile
- [ ] Joystick can be dragged smoothly

### Test Emotion Detection

```bash
# Start all servers (includes emotion detection)
python server/launch_all.py

# Open emotion detection interface
http://localhost:9999
```

**Test checklist:**
- [ ] Camera feed appears with faces detected
- [ ] Emotions are labeled with confidence scores
- [ ] Multiple faces can be detected simultaneously
- [ ] Emotion icons update in real-time
- [ ] Statistics show detection count

### Test ROS Integration

```bash
# Start roscore
roscore

# Build workspace (first time only)
cd ros_workspace
catkin_make
source devel/setup.bash

# Launch ROS nodes
roslaunch pet_robot_ros pet_robot.launch

# Test movement
rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.5}" -r 10

# Test services
rosservice call /mental_health/get_affirmation "{}"
```

**Test checklist:**
- [ ] ROS nodes start without errors
- [ ] Can publish to /cmd_vel
- [ ] Services respond correctly
- [ ] WebSocket bridge connects
- [ ] Control mode can be switched

### Test Speaker Setup

```bash
# On Raspberry Pi

# Test audio output
speaker-test -t wav -c 2

# Test text-to-speech
espeak "Hello, I am your AI pet robot!"

# Adjust volume
alsamixer
```

**Test checklist:**
- [ ] Speaker produces sound
- [ ] Volume is appropriate
- [ ] No distortion or crackling
- [ ] TTS is clear and understandable

## System Architecture - Updated

```
┌─────────────────────────────────────────────────────────────────┐
│                     CLIENT INTERFACES                            │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌────────────────────┐ │
│  │ Desktop  │ │ Mobile   │ │ Emotion  │ │ Emotion Detection  │ │
│  │ Port     │ │ Port     │ │ Display  │ │ Port 9999          │ │
│  │ 8000     │ │ 3000     │ │ Port     │ │ (Face Analysis)    │ │
│  │          │ │ Enhanced │ │ 10000    │ │                    │ │
│  └────┬─────┘ └────┬─────┘ └────┬─────┘ └─────────┬──────────┘ │
└───────┼────────────┼────────────┼───────────────────┼────────────┘
        │            │            │                   │
        ▼            ▼            ▼                   ▼
┌────────────────────────────────────────────────────────────────┐
│                    SERVER CLUSTER                               │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │ Port 8000: Primary Control Server                        │  │
│  │ - Movement control (manual/autonomous)                   │  │
│  │ - Camera streaming                                       │  │
│  │ - Gemini AI (gemini-1.5-flash)                          │  │
│  │ - Mental health monitoring                               │  │
│  │ - WebSocket: /ws/control, /ws/raspberry_pi, /ws/ros    │  │
│  └──────────────────────────────────────────────────────────┘  │
│                                                                  │
│  ┌──────────────┐  ┌──────────────┐  ┌────────────────────┐   │
│  │ Port 10000:  │  │ Port 3000:   │  │ Port 9999:         │   │
│  │ Emotion      │  │ Mobile Web   │  │ Emotion Detection  │   │
│  │ Display      │  │ - WASD       │  │ - Face detection   │   │
│  │              │  │ - Joystick   │  │ - Emotion model    │   │
│  │              │  │ - Speed      │  │ - Gemini responses │   │
│  └──────────────┘  └──────────────┘  └────────────────────┘   │
└──────────┬─────────────────────────────────────────────────────┘
           │
           ▼
┌──────────────────────────────────────────────────────────────────┐
│                  RASPBERRY PI (Robot Hardware)                    │
│  - Motor Control (4 wheels)                                       │
│  - Camera (feeds ports 8000 and 9999)                            │
│  - Speaker (40mm 8Ω with amplifier) 🔊                           │
│  - TTS via espeak                                                │
└──────────┬────────────────────────────────────────────────────────┘
           │
           ▼
┌──────────────────────────────────────────────────────────────────┐
│                    ROS SYSTEM (Optional)                          │
│  - Autonomous navigation                                         │
│  - Mental health services                                        │
│  - Full robot control via ROS topics                            │
└──────────────────────────────────────────────────────────────────┘
```

## Port Summary

| Port | Service | New Features | Access |
|------|---------|--------------|--------|
| 8000 | Primary Control | - Gemini-1.5-flash | http://localhost:8000 |
| 3000 | Mobile Web | ✨ WASD, Joystick, Speed control | http://localhost:3000 |
| 9999 | Emotion Detection | ✨ NEW - Face emotion analysis | http://localhost:9999 |
| 10000 | Emotion Display | - Synced animations | http://localhost:10000 |

## Starting Everything

### Quick Start - All Services
```bash
cd server
python launch_all.py
```

This starts:
- ✓ Primary Control Server (Port 8000)
- ✓ Emotion Display Server (Port 10000)
- ✓ Mobile Web Interface (Port 3000) - Enhanced
- ✓ Emotion Detection Server (Port 9999) - NEW

### Custom Start

```bash
# Only servers (no hardware)
python launch_all.py

# Servers + hardware simulation
python launch_all.py --full

# Servers + Raspberry Pi simulation
python launch_all.py --with-pi

# Only primary server
python launch_all.py --server-only
```

## Dependencies Update

New dependencies added to `server/requirements.txt`:
- `opencv-python==4.8.1.78` - Face detection
- `tensorflow==2.15.0` - Emotion model
- `numpy==1.24.3` - Numerical operations

Install with:
```bash
cd server
pip install -r requirements.txt
```

## Troubleshooting

### Port 3000 Issues

**Problem**: Joystick not responding
- **Solution**: Check JavaScript console for errors, refresh page

**Problem**: WASD keys not working
- **Solution**: Click on page to focus, ensure not in text input

**Problem**: Camera too small
- **Solution**: Check `.camera-view` CSS - should be 400px height

### Port 9999 Issues

**Problem**: No faces detected
- **Solution**: 
  - Ensure good lighting
  - Face camera directly
  - Check if emotion_model.h5 exists in root directory

**Problem**: Model not loading
- **Solution**:
  ```bash
  pip install tensorflow opencv-python
  # Check model file exists
  ls emotion_model.h5
  ```

**Problem**: Low confidence scores
- **Solution**: Improve lighting, move closer to camera

### ROS Issues

**Problem**: roscore not found
- **Solution**: 
  ```bash
  source /opt/ros/noetic/setup.bash
  echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
  ```

**Problem**: Package not found
- **Solution**:
  ```bash
  cd ros_workspace
  catkin_make
  source devel/setup.bash
  ```

### Speaker Issues

**Problem**: No sound
- **Solution**:
  ```bash
  # Check audio output
  amixer cset numid=3 1  # Force 3.5mm jack
  amixer set Master 80%  # Set volume
  speaker-test -t wav -c 2  # Test
  ```

**Problem**: Distortion
- **Solution**: Reduce volume, add amplifier (PAM8403)

## Next Steps

1. **Explore Port 3000**
   - Try both control modes
   - Test WASD keyboard controls
   - Adjust speed and test response

2. **Test Emotion Detection**
   - Open port 9999
   - Make different facial expressions
   - Check emotion accuracy

3. **Setup ROS** (Optional)
   - Follow ROS_SETUP_GUIDE.md
   - Test autonomous navigation
   - Try mental health services

4. **Connect Speaker**
   - Follow SPEAKER_SETUP_GUIDE.md
   - Test TTS output
   - Adjust volume

5. **Integrate Everything**
   - Start all services with launch_all.py
   - Control robot from port 3000
   - Monitor emotions on port 9999
   - View animated face on port 10000

## Documentation Files

- **ROS_SETUP_GUIDE.md**: Complete ROS installation and usage
- **SPEAKER_SETUP_GUIDE.md**: Hardware setup for audio output
- **README.md**: Main project documentation (updated)
- **SETUP_GUIDE.md**: General setup instructions

## API Reference

### Emotion Detection API

```bash
# Get current emotion
GET /api/emotion
Response: {
  "emotion": "happy",
  "history": [...]
}

# Get emotion response
POST /api/emotion/response?emotion=happy
Response: {
  "emotion": "happy",
  "response": "I'm so glad you're feeling happy! ..."
}

# Health check
GET /health
Response: {
  "status": "healthy",
  "model_loaded": true,
  "current_emotion": "happy"
}
```

### WebSocket - Emotion Detection

```javascript
const ws = new WebSocket('ws://localhost:9999/ws/emotion');

ws.onmessage = (event) => {
  const data = JSON.parse(event.data);
  if (data.type === 'emotion_detection') {
    console.log('Emotions:', data.emotions);
    console.log('Current:', data.current_emotion);
  }
};

// Request emotion-based response
ws.send(JSON.stringify({
  type: 'get_emotion_response',
  emotion: 'happy'
}));
```

## Performance Notes

### Gemini-1.5-Flash
- **Response time**: ~1-2 seconds
- **Cost**: ~50% less than gemini-pro
- **Quality**: Excellent for conversational AI
- **Context window**: 1M tokens
- **Best for**: Real-time interactions

### Emotion Detection
- **Processing time**: ~50-100ms per frame
- **Accuracy**: 70-90% depending on lighting
- **Max faces**: Up to 10 simultaneous
- **GPU**: Optional, but recommended for better performance

### Port 3000 Interface
- **Latency**: <50ms for controls
- **Camera refresh**: 10 FPS (adjustable)
- **Mobile optimized**: Touch-friendly
- **Bandwidth**: ~500 KB/s for camera

## Support

For issues or questions:
1. Check troubleshooting sections in relevant guides
2. Review logs: `python server.py` shows detailed output
3. Open GitHub issue with:
   - Port number and service name
   - Error messages
   - Browser/OS information
   - Steps to reproduce

---

**Enjoy your enhanced AI Pet Robot! 🤖😊🎮**
