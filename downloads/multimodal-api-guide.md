# Multimodal API Integration Guide

## Overview

The AI Pet Robot now supports multimodal interactions through Google's Gemini API, enabling rich communication via text, images, and audio. This guide covers the enhanced features and how to use them.

## Features

### 1. Multimodal Input Processing

The robot can now process:
- **Text**: Chat conversations and commands
- **Images**: Visual analysis and context understanding
- **Audio**: Voice interactions (context-based processing)
- **Combined**: Text + Image for richer interactions

### 2. Enhanced Emotion Synchronization

- Emotions are automatically synchronized between:
  - Server state management
  - Raspberry Pi face display (HDMI)
  - Audio output timing
- Smooth transitions ensure face expressions match emotional states

### 3. WebSocket Security

- Token-based authentication for secure connections
- Heartbeat monitoring for connection health
- Automatic reconnection with exponential backoff
- Connection status tracking

## API Endpoints

### Generate Authentication Token

```bash
POST /api/auth/token
```

**Response:**
```json
{
  "token": "secure_token_here",
  "expires": "never",
  "note": "Use this token in WebSocket connection query parameters"
}
```

### WebSocket Connection

Connect with authentication:
```
ws://server:8000/ws/control?token=YOUR_TOKEN
```

## WebSocket Commands

### 1. Text Command

Send text for AI processing:

```json
{
  "type": "text",
  "text": "How are you today?"
}
```

**Response:**
```json
{
  "type": "response",
  "text": "I'm doing great! How can I help you today?",
  "emotion": "happy",
  "transcript": "User: How are you today?\nRobot: I'm doing great! How can I help you today?"
}
```

### 2. Image Analysis Command

Send an image for analysis:

```json
{
  "type": "image",
  "image": "base64_encoded_image_data",
  "prompt": "What do you see in this image?"
}
```

**Response:**
```json
{
  "type": "image_analysis",
  "text": "I see a beautiful sunset over the ocean...",
  "analysis_type": "visual"
}
```

### 3. Multimodal Command

Send text and image together:

```json
{
  "type": "multimodal",
  "text": "What's happening in this scene?",
  "image": "base64_encoded_image_data"
}
```

**Response:**
```json
{
  "type": "multimodal_response",
  "text": "In this scene, I can see...",
  "emotion": "curious"
}
```

### 4. Voice Command

Send voice with context:

```json
{
  "type": "voice",
  "audio": "audio_data",
  "context": "User is asking about their schedule"
}
```

**Response:**
```json
{
  "type": "response",
  "text": "Let me help you with your schedule...",
  "emotion": "helpful",
  "transcript": "Context: User is asking about their schedule\nRobot: Let me help you with your schedule..."
}
```

### 5. Emotion Command

Manually set robot emotion:

```json
{
  "type": "emotion",
  "emotion": "happy"
}
```

Supported emotions:
- `neutral`
- `happy`
- `sad`
- `anxious`
- `excited`
- `tired`
- `angry`
- `crisis` (special handling for crisis situations)

### 6. Mode Change Command

Toggle between manual and autonomous modes:

```json
{
  "type": "set_mode",
  "mode": "autonomous"
}
```

Modes:
- `manual`: Direct control via commands
- `autonomous`: ROS-based autonomous navigation

### 7. Heartbeat

Keep connection alive:

```json
{
  "type": "heartbeat"
}
```

**Response:**
```json
{
  "type": "heartbeat_ack"
}
```

## State Updates

The server broadcasts state updates to all connected clients:

```json
{
  "type": "state",
  "data": {
    "emotion": "happy",
    "is_speaking": false,
    "is_listening": false,
    "battery_level": 85,
    "sensor_data": {},
    "control_mode": "manual",
    "esp_connected": true,
    "raspberry_pi_connected": true,
    "last_transcript": "User: Hello\nRobot: Hi there!"
  }
}
```

## Mobile App Integration

### Connecting with Authentication

```dart
// Generate token from server first
final response = await http.post(
  Uri.parse('http://server:8000/api/auth/token')
);
final token = jsonDecode(response.body)['token'];

// Connect with token
await websocketService.connect(
  'ws://server:8000/ws/control',
  token: token
);
```

### Sending Commands

```dart
// Text command
websocketService.sendTextCommand("Tell me a joke");

// Image analysis
websocketService.sendImageCommand(base64Image, prompt: "What is this?");

// Multimodal
websocketService.sendMultimodalCommand("Describe this", base64Image);

// Mode change
websocketService.sendModeCommand("autonomous");
```

### Listening to Updates

```dart
// State updates
websocketService.stateStream.listen((state) {
  print('Emotion: ${state.emotion}');
  print('ESP Connected: ${state.espConnected}');
  print('Raspberry Pi Connected: ${state.raspberryPiConnected}');
  print('Transcript: ${state.lastTranscript}');
});

// Connection status
websocketService.connectionStream.listen((connected) {
  print('Connected: $connected');
});

// Transcripts
websocketService.transcriptStream.listen((transcript) {
  print('New transcript: $transcript');
});
```

## Emotion Synchronization Flow

1. **Command Received**: User sends text/image/voice command
2. **AI Processing**: Gemini processes input and generates response
3. **Emotion Detection**: Server detects emotion from response
4. **State Update**: Server updates internal emotion state
5. **Sync to Display**: Emotion synced to Raspberry Pi via WebSocket
6. **Face Animation**: Raspberry Pi updates HDMI face display
7. **Audio Preparation**: Audio output prepared with matching emotion
8. **Broadcast**: All clients notified of state change

## Error Handling

### Connection Errors

The WebSocket service automatically:
- Detects connection failures
- Attempts reconnection after 5 seconds
- Maintains heartbeat every 10 seconds
- Notifies clients of connection status

### Authentication Errors

If authentication fails:
```json
{
  "error": "Authentication failed",
  "code": 1008
}
```

Server closes connection with code 1008.

## Best Practices

### 1. Token Management

- Request a token once per session
- Store token securely (not in source code)
- Implement token refresh for production

### 2. Connection Management

- Monitor connection status stream
- Handle reconnection gracefully in UI
- Show connection indicators to users

### 3. Image Processing

- Compress images before sending
- Use JPEG format for photos
- Keep base64 payload under 1MB

### 4. Emotion Handling

- Allow time for emotion transitions
- Don't override emotions too frequently
- Trust the AI-detected emotions

### 5. Performance

- Send heartbeat every 10 seconds
- Don't send commands while processing
- Handle backpressure on state updates

## Testing

### Test Multimodal Processing

```python
import asyncio
import websockets
import json
import base64

async def test_multimodal():
    uri = "ws://localhost:8000/ws/control"
    
    async with websockets.connect(uri) as websocket:
        # Test text
        await websocket.send(json.dumps({
            "type": "text",
            "text": "Hello robot!"
        }))
        
        response = await websocket.recv()
        print(f"Response: {response}")
        
        # Test image (load and encode your image)
        with open("test_image.jpg", "rb") as f:
            image_data = base64.b64encode(f.read()).decode()
        
        await websocket.send(json.dumps({
            "type": "image",
            "image": image_data,
            "prompt": "What do you see?"
        }))
        
        response = await websocket.recv()
        print(f"Image analysis: {response}")

asyncio.run(test_multimodal())
```

### Test Authentication

```bash
# Get token
curl -X POST http://localhost:8000/api/auth/token

# Use token (requires WebSocket client)
# wscat -c "ws://localhost:8000/ws/control?token=YOUR_TOKEN"
```

## Troubleshooting

### Issue: Emotion not syncing to display

**Solution**: Check that Raspberry Pi is connected:
```bash
# Check server logs
docker-compose logs -f robot-server | grep "Raspberry Pi"
```

### Issue: WebSocket connection rejected

**Solution**: Verify authentication token:
```bash
# Generate new token
curl -X POST http://localhost:8000/api/auth/token
```

### Issue: Image analysis not working

**Solution**: 
- Verify image is properly base64 encoded
- Check image size (keep under 1MB)
- Ensure Gemini API key is set

### Issue: Heartbeat timeout

**Solution**:
- Check network stability
- Verify heartbeat is sent every 10 seconds
- Check server logs for connection issues

## Security Considerations

1. **Token Security**
   - Tokens are generated server-side
   - Store tokens securely
   - Implement token expiration in production

2. **Image Data**
   - Validate image format
   - Limit image size
   - Sanitize file uploads

3. **Network Security**
   - Use WSS (WebSocket Secure) in production
   - Implement rate limiting
   - Monitor for unusual activity

4. **API Key Protection**
   - Never expose Gemini API key
   - Use environment variables
   - Rotate keys regularly

## Future Enhancements

- Audio streaming for real-time processing
- Video analysis support
- Enhanced emotion detection with facial recognition
- Multi-language support
- Custom emotion animations
- Advanced context retention

## Resources

- [Gemini API Documentation](https://ai.google.dev/docs)
- [WebSocket Protocol](https://developer.mozilla.org/en-US/docs/Web/API/WebSocket)
- [FastAPI WebSocket Guide](https://fastapi.tiangolo.com/advanced/websockets/)
- [Flutter WebSocket](https://pub.dev/packages/web_socket_channel)

## Support

For issues or questions:
1. Check server logs: `docker-compose logs -f robot-server`
2. Review this documentation
3. Open an issue on GitHub
4. Check the main README for contact information
