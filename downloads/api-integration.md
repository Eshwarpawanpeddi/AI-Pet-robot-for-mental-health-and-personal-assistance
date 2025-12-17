# Pet Robot OS - Complete Integration Guide & API Reference

## System Architecture Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                     PHONE CLIENT (Android/iOS)                   │
│  ┌──────────────┐ ┌──────────────┐ ┌─────────────────────────┐ │
│  │ Voice Input  │ │ Manual Ctrl  │ │ Animated Face Display   │ │
│  │ (Gemini Live)│ │ (Joystick)   │ │ (Shows Emotions)        │ │
│  └──────┬───────┘ └──────┬───────┘ └────────────┬────────────┘ │
│         │                │                      │               │
│         └────────────────┼──────────────────────┘               │
│                          │                                       │
│         WebSocket / HTTP │ (Real-time)                          │
└─────────────────────────┼─────────────────────────────────────┘
                          │
        ┌─────────────────┼─────────────────┐
        │                 │                 │
        ▼                 ▼                 ▼
┌──────────────────────────────────────────────┐
│     LAPTOP SERVER (Processing Core)           │
│  ┌────────────────────────────────────────┐  │
│  │ Gemini Live API Integration             │  │
│  │ - Voice Processing (16kHz PCM)          │  │
│  │ - LLM Response Generation               │  │
│  │ - Emotion Detection & Synthesis         │  │
│  └────────────────────────────────────────┘  │
│  ┌────────────────────────────────────────┐  │
│  │ Command Orchestration & Routing         │  │
│  │ - Parse voice commands                  │  │
│  │ - Generate motor commands               │  │
│  │ - Manage face animations                │  │
│  └────────────────────────────────────────┘  │
└──────────────────────────────────────────────┘
                 │      │      │
        MQTT/    │      │      │   WebSocket
        gRPC     │      │      │
                 │      │      │
        ┌────────┴──┐  ┌┴──────┴─────┐
        │           │  │             │
        ▼           ▼  ▼             ▼
    ┌─────────────────────────────────────┐
    │   RASPBERRY PI 4 (Central Hub)      │
    │  ┌────────────────────────────────┐ │
    │  │ GPIO/I2C/SPI Management         │ │
    │  │ - Sensor data collection        │ │
    │  │ - Hardware state management     │ │
    │  │ - Command parsing               │ │
    │  └────────────────────────────────┘ │
    └──┬───────────┬─────────────────────┘
       │           │
   I2C │           │ GPIO/PWM
       │           │
    ┌──▼──┐    ┌───▼────────────┐
    │     │    │                │
    ▼     ▼    ▼                ▼
┌─────────────┐ ┌────────┐  ┌─────────┐
│   ESP12E    │ │Display │  │ Speaker │
│(Motor Ctrl) │ │ (LCD)  │  │(Speaker)│
└─────────────┘ └────────┘  └─────────┘
     │
   PWM │
     │
┌─────▼──────────────┐
│  L298N Motor       │
│  Driver            │
└────────┬───────────┘
         │
    ┌────┴────┬─────────┐
    │         │         │
    ▼         ▼         ▼
  Motor A  Motor B  Servo(s)
```

## Data Flow - Voice Command Example

```
1. User speaks to phone: "Move forward"
   ↓
2. Phone captures audio (16-bit PCM, 16kHz)
   ↓
3. Sends to server via WebSocket
   ↓
4. Server sends to Gemini Live API
   ↓
5. Gemini processes and returns:
   - Text response: "Moving forward!"
   - Audio response: Synthesized voice
   - Detected intent: MOVE_FORWARD
   ↓
6. Server extracts intent and emotion
   ↓
7. Server routes to Raspberry Pi:
   Command: {motor: 2, direction: 1, speed: 200}
   ↓
8. Raspberry Pi sends via I2C to ESP12E
   ↓
9. ESP12E sets motor pins and PWM
   ↓
10. Robots moves forward!
    ↓
11. Face displays: Happy emotion, moving animation
12. Phone speaker plays: "Moving forward!"
```

## API Endpoints & Protocols

### 1. WebSocket: /ws/control

**Purpose**: Real-time bidirectional communication

**Client → Server Messages**:
```json
{
  "type": "move",
  "direction": "forward|backward|left|right|stop",
  "speed": 0-255
}
```

```json
{
  "type": "voice",
  "audio": "base64_encoded_pcm_audio",
  "format": "audio/pcm;rate=16000"
}
```

```json
{
  "type": "emotion",
  "emotion": "happy|sad|angry|surprised|neutral"
}
```

```json
{
  "type": "get_state"
}
```

**Server → Client Messages**:
```json
{
  "type": "state",
  "data": {
    "emotion": "happy",
    "is_speaking": false,
    "is_listening": true,
    "battery_level": 85,
    "sensor_data": {
      "distance": 45.5,
      "touch_detected": false
    }
  }
}
```

```json
{
  "type": "response",
  "text": "I'm moving forward!",
  "emotion": "happy",
  "audio": "base64_encoded_audio_response"
}
```

### 2. REST API: /api/*

**GET /api/state**
```
Response: {
  "emotion": "neutral",
  "is_speaking": false,
  "is_listening": false,
  "battery_level": 100,
  "sensor_data": { ... }
}
```

**POST /api/command**
```
Body: {
  "type": "move",
  "direction": "forward",
  "speed": 200
}
Response: { "status": "ok" }
```

**GET /health**
```
Response: {
  "status": "healthy",
  "timestamp": "2024-XX-XX...",
  "robot_state": { ... }
}
```

### 3. I2C Protocol: Raspberry Pi ↔ ESP12E

**Format**: 3-byte commands
```
Byte 0: Motor selection (0=A, 1=B, 2=Both)
Byte 1: Direction (0=Stop, 1=Forward, 2=Backward, 3=Left, 4=Right)
Byte 2: Speed (0-255)
```

**Example**: Move forward at full speed
```cpp
i2c_write(ESP12E_ADDRESS, {2, 1, 255})
// Motor=Both, Direction=Forward, Speed=Max
```

## Integration Checklist

### Phase 1: Hardware Layer (Week 1)
- [ ] ESP12E flashed with motor controller firmware
- [ ] I2C communication working (i2cdetect shows 0x08)
- [ ] Motors respond to I2C commands
- [ ] Touch sensors detected
- [ ] Distance sensor reading values

### Phase 2: Raspberry Pi Layer (Week 1-2)
- [ ] GPIO configured correctly
- [ ] PWM working for motor speed control
- [ ] SPI communication with display working
- [ ] I2S audio output tested
- [ ] Service auto-starts on boot

### Phase 3: Server Setup (Week 2)
- [ ] FastAPI server running on port 8000
- [ ] WebSocket endpoint responding
- [ ] Gemini API key configured
- [ ] Audio processing working
- [ ] Emotion detection implemented

### Phase 4: Phone App (Week 3)
- [ ] App connects to WebSocket
- [ ] Voice input captures audio
- [ ] Face display renders properly
- [ ] Manual controls send commands
- [ ] Receives and displays responses

### Phase 5: Integration & Testing (Week 3-4)
- [ ] End-to-end voice command works
- [ ] Robot responds to manual controls
- [ ] Face emotions update correctly
- [ ] Audio playback synchronized
- [ ] All components communicate reliably

## Performance Targets

| Metric | Target | Notes |
|--------|--------|-------|
| Voice latency | <500ms | From speech to motor response |
| WebSocket latency | <100ms | RTT for control commands |
| Gemini response | <1s | Processing + API call |
| Motor response | <50ms | I2C command to movement |
| Face frame rate | 60 FPS | Canvas rendering |
| Audio quality | 16-bit 16kHz | Sufficient for voice |
| Battery life | 4+ hours | Continuous operation |

## Troubleshooting Quick Reference

| Issue | Solution |
|-------|----------|
| ESP12E not responding | Check I2C pull-ups (4.7k), verify address 0x08, check power |
| Motors jittering | Check motor driver enable pins, verify PWM frequency |
| Audio crackling | Verify sample rate (16kHz), check I2S connections |
| Face not rendering | Check canvas size, verify GPU acceleration, reduce animation complexity |
| WebSocket timeout | Check firewall, verify server running, test with websocat |
| Gemini API error | Verify API key, check rate limits, ensure audio format correct |
| Robot not moving | Check motor driver power, verify GPIO connections, test with multimeter |
| Battery draining fast | Check motor duty cycle, reduce animation complexity, optimize WiFi power |

## Code Examples

### Example 1: Send Move Command from Phone

```typescript
// React Native / TypeScript
const socket = new WebSocket('ws://192.168.1.100:8000/ws/control');

socket.onopen = () => {
  const command = {
    type: 'move',
    direction: 'forward',
    speed: 200
  };
  socket.send(JSON.stringify(command));
};

socket.onmessage = (event) => {
  const response = JSON.parse(event.data);
  if (response.type === 'state') {
    updateFaceEmotion(response.data.emotion);
  }
};
```

### Example 2: Process Voice with Gemini

```python
# Server-side
async def process_voice(audio_bytes):
    session = robot_state.gemini_session
    
    # Send audio
    await session.send_message(
        types.Blob(
            mime_type="audio/pcm;rate=16000",
            data=audio_bytes
        )
    )
    
    # Receive response
    async for response in session.receive():
        if response.text:
            text = response.text
        if response.data:
            audio = response.data
    
    return {'text': text, 'audio': audio}
```

### Example 3: Control Motor from Raspberry Pi

```python
# Raspberry Pi
controller = RaspberryPiController()

# Move forward
controller.move_forward(speed=200)

# Turn left
controller.turn_left(speed=180)

# Stop
controller.stop()

# Get sensor data
data = controller.get_sensor_data()
print(f"Distance: {data['distance']} cm")
print(f"Touch: {data['touch_detected']}")
```

### Example 4: Draw Animated Face

```javascript
// Face animation
class RobotFaceRenderer {
  draw() {
    this.ctx.clearRect(0, 0, this.width, this.height);
    
    // Draw head
    this.drawHead();
    
    // Draw eyes with blinking
    this.drawEyes();
    
    // Draw mouth based on emotion
    this.drawMouth();
    
    // Draw eyebrows for expression
    this.drawExpressionIndicators();
  }
  
  setEmotion(emotion) {
    this.emotion = emotion;
    // Face automatically updates on next render
  }
}
```

## Next Steps & Future Enhancements

1. **Computer Vision**: Add camera for gesture recognition
2. **Advanced AI**: Fine-tune Gemini for more natural personality
3. **Memory**: Implement persistent memory of user preferences
4. **Multi-robot**: Support multiple robots communicating with each other
5. **IoT Integration**: Control lights, temperature via robot
6. **Mobile App**: Native iOS/Android with push notifications
7. **Analytics**: Track usage patterns and optimize responses
8. **Game Integration**: Build mini-games the robot can play with user

## Support & Resources

- **GitHub Issues**: Report bugs and request features
- **Documentation**: Full API docs at http://localhost:8000/docs
- **Community Discord**: Join our community for tips and support
- **Video Tutorials**: Check out the setup walkthrough on YouTube

---

**Version**: 1.0.0  
**Last Updated**: December 2024  
**Maintainer**: Your Team
