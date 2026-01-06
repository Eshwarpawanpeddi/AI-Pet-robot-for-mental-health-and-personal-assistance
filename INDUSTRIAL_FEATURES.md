# Industrial-Level Features Update

This document describes the new industrial-level features added to the AI Pet Robot system for enhanced personal assistance and multimodal communication.

## Table of Contents
1. [Port Communication Control](#port-communication-control)
2. [Emotion Detection Integration](#emotion-detection-integration)
3. [Task Scheduling and Reminders](#task-scheduling-and-reminders)
4. [Information Retrieval](#information-retrieval)
5. [Smart Home Integration](#smart-home-integration)
6. [Raspberry Pi Speaker Configuration](#raspberry-pi-speaker-configuration)
7. [API Reference](#api-reference)

---

## Port Communication Control

### Overview
Gemini AI communication can now be enabled or disabled independently on ports 8000 and 3000 without affecting UI or other operations.

### Configuration

#### Environment Variables
Set these in your `.env` file:
```bash
# Enable/disable Gemini on specific ports (true/false)
GEMINI_ENABLED_PORT_8000=true
GEMINI_ENABLED_PORT_3000=true
```

#### Runtime Control via API

**Enable/Disable Gemini on a Port:**
```bash
curl -X POST http://localhost:8000/api/gemini/control \
  -H "Content-Type: application/json" \
  -d '{
    "port": 8000,
    "enabled": false
  }'
```

**Check Gemini Status:**
```bash
curl http://localhost:8000/api/gemini/status
```

Response:
```json
{
  "initialized": true,
  "port_8000_enabled": true,
  "port_3000_enabled": true,
  "current_user_emotion": "happy"
}
```

### Behavior
- When Gemini is disabled on a port, user messages are still processed
- UI remains functional and responsive
- Emotion detection still works
- No AI responses are generated
- Other ports remain unaffected

---

## Emotion Detection Integration

### Overview
User emotions are now automatically detected from facial expressions via port 9999 and integrated into AI responses.

### How It Works
1. User communicates with the robot (text or voice)
2. System queries emotion detection server on port 9999
3. Current emotion is detected from camera feed
4. Emotion is updated in robot state as `current_emotion`
5. Gemini receives emotion context in prompts
6. AI responds with emotion-aware empathy

### Current Emotion States
- **happy** - User appears joyful
- **sad** - User appears sad or distressed
- **angry** - User appears frustrated or angry
- **fear** - User appears scared or anxious
- **surprise** - User appears surprised
- **disgust** - User appears disgusted
- **neutral** - Default or no clear emotion
- **unknown** - Detection failed or unavailable

### Emotion Context in Prompts
When emotion is detected, Gemini receives context like:
```
[User appears sad] I'm feeling overwhelmed today
```

This allows Gemini to:
- Respond with appropriate empathy
- Adjust tone and suggestions
- Provide mental health support
- Recognize emotional patterns

### API Usage

**Get Current Emotion:**
```bash
curl http://localhost:8000/api/emotion/current
```

Response:
```json
{
  "current_emotion": "happy",
  "robot_emotion": "happy",
  "last_updated": "2026-01-06T13:45:00.000Z"
}
```

### Fallback Behavior
- If emotion detection fails, `current_emotion` is set to "unknown"
- Text-based emotion detection serves as fallback
- System continues normal operation

---

## Task Scheduling and Reminders

### Overview
Schedule tasks and set reminders through the personal assistant AI.

### API Endpoints

**Schedule a Task:**
```bash
curl -X POST http://localhost:8000/api/tasks/schedule \
  -H "Content-Type: application/json" \
  -d '{
    "type": "task",
    "description": "Call doctor for appointment",
    "time": "2026-01-07T14:00:00Z"
  }'
```

**Schedule a Reminder:**
```bash
curl -X POST http://localhost:8000/api/tasks/schedule \
  -H "Content-Type: application/json" \
  -d '{
    "type": "reminder",
    "description": "Take medication",
    "time": "2026-01-06T20:00:00Z"
  }'
```

**List All Tasks:**
```bash
curl http://localhost:8000/api/tasks/list
```

Response:
```json
{
  "tasks": [
    {
      "id": "uuid-1234",
      "type": "task",
      "description": "Call doctor",
      "time": "2026-01-07T14:00:00Z",
      "status": "pending"
    }
  ],
  "reminders": [
    {
      "id": "uuid-5678",
      "type": "reminder",
      "description": "Take medication",
      "time": "2026-01-06T20:00:00Z",
      "status": "pending"
    }
  ],
  "total": 2
}
```

**Delete a Task:**
```bash
curl -X DELETE http://localhost:8000/api/tasks/uuid-1234
```

### Integration with Gemini
Users can schedule tasks through natural conversation:
```
User: "Remind me to take my medication at 8 PM"
Robot: [Creates reminder and confirms]
```

---

## Information Retrieval

### Overview
Query information through the AI assistant for quick answers and research.

### API Usage

**Submit a Query:**
```bash
curl -X POST http://localhost:8000/api/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What are some breathing exercises for anxiety?"
  }'
```

Response:
```json
{
  "status": "ok",
  "query": "What are some breathing exercises for anxiety?",
  "response": "Here are some effective breathing exercises: 1) Box breathing - inhale for 4, hold for 4, exhale for 4, hold for 4. 2) 4-7-8 breathing - inhale for 4, hold for 7, exhale for 8. These can help calm your nervous system."
}
```

### Use Cases
- Mental health tips and techniques
- General knowledge questions
- Daily information needs
- Health and wellness advice
- Educational queries

---

## Smart Home Integration

### Overview
Control smart home devices through standard IoT protocols (MQTT, HTTP, etc.).

### Supported Device Types
- Lights
- Thermostats
- Locks
- Cameras
- Sensors
- Switches
- Smart Plugs

### API Endpoints

**List Devices:**
```bash
curl http://localhost:8000/api/smarthome/devices
```

**Get Device State:**
```bash
curl http://localhost:8000/api/smarthome/device/light_living_room
```

**Control a Device:**
```bash
curl -X POST http://localhost:8000/api/smarthome/control \
  -H "Content-Type: application/json" \
  -d '{
    "device_id": "light_living_room",
    "command": "turn_on",
    "parameters": {
      "brightness": 80
    }
  }'
```

**Execute a Scene:**
```bash
curl -X POST http://localhost:8000/api/smarthome/scene \
  -H "Content-Type: application/json" \
  -d '{
    "scene_name": "Good Night",
    "devices": {
      "light_living_room": {
        "command": "turn_off"
      },
      "thermostat_main": {
        "command": "set_temperature",
        "parameters": {"temperature": 68}
      },
      "lock_front_door": {
        "command": "lock"
      }
    }
  }'
```

### Extending Smart Home Integration

The smart home module is designed to be extended:

1. **Add Protocol Support**: Edit `smart_home_integration.py`
2. **Implement Device Drivers**: Add specific device control logic
3. **Register Devices**: Add your devices through the API or code

Example:
```python
from smart_home_integration import smart_home, SmartDevice, DeviceType, Protocol

# Register your device
smart_home.register_device(SmartDevice(
    id="custom_device_1",
    name="My Smart Device",
    device_type=DeviceType.SWITCH,
    protocol=Protocol.HTTP,
    address="http://192.168.1.100/api",
    state={"power": "off"},
    capabilities=["on", "off"]
))
```

### Voice Control Integration
Smart home commands can be integrated with voice:
```
User: "Turn off the living room lights"
Robot: [Executes command] "I've turned off the living room lights."
```

---

## Raspberry Pi Speaker Configuration

### Overview
Enhanced audio output support for 8-ohm speakers connected to Raspberry Pi GPIO.

### Audio Output Modes

Edit `raspberry_pi_controller.py` to configure:

```python
# Audio Configuration
AUDIO_OUTPUT_MODE = "default"  # Options: "default", "gpio_pwm", "i2s"
AUDIO_VOLUME = 80  # Volume percentage (0-100)
```

### Mode Options

1. **default** - 3.5mm audio jack (recommended for beginners)
   - Simple plug-and-play
   - Works with external speakers or amplifier
   - Best with PAM8403 amplifier module

2. **gpio_pwm** - GPIO PWM audio output
   - Requires transistor circuit (see SPEAKER_SETUP_GUIDE.md)
   - Direct connection to GPIO pins
   - Moderate audio quality

3. **i2s** - I2S DAC modules
   - Best audio quality
   - Requires I2S HAT or module
   - Supports high-quality speakers

### Volume Control
Volume is automatically set when robot speaks:
```python
AUDIO_VOLUME = 80  # Set volume to 80%
```

### Hardware Setup
For detailed wiring diagrams and setup instructions, see:
- **SPEAKER_SETUP_GUIDE.md** - Complete hardware guide
- Includes circuit diagrams for all connection methods
- Troubleshooting tips
- Component recommendations

---

## API Reference

### Gemini Control

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/gemini/control` | POST | Enable/disable Gemini on ports |
| `/api/gemini/status` | GET | Get Gemini configuration status |

### Emotion Detection

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/emotion/current` | GET | Get current user and robot emotions |

### Task Management

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/tasks/schedule` | POST | Schedule a task or reminder |
| `/api/tasks/list` | GET | List all tasks and reminders |
| `/api/tasks/{task_id}` | DELETE | Delete a specific task |

### Information Retrieval

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/query` | POST | Submit information query to Gemini |

### Smart Home

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/smarthome/devices` | GET | List all smart devices |
| `/api/smarthome/device/{id}` | GET | Get device state |
| `/api/smarthome/control` | POST | Control a device |
| `/api/smarthome/scene` | POST | Execute a scene |

---

## Error Handling

All API endpoints return structured error responses:

```json
{
  "status": "error",
  "message": "Description of the error"
}
```

Success responses:
```json
{
  "status": "ok",
  "message": "Operation successful",
  "...": "additional data"
}
```

---

## Logging

Enhanced logging provides visibility into all operations:

```
2026-01-06 13:45:00 - INFO - Gemini on port 8000: enabled
2026-01-06 13:45:01 - INFO - Detected user emotion from port 9999: happy
2026-01-06 13:45:02 - INFO - Scheduled task: Take medication
2026-01-06 13:45:03 - INFO - Smart home control: light_living_room -> turn_on
```

Log levels:
- **INFO** - Normal operations
- **WARNING** - Non-critical issues
- **ERROR** - Critical errors requiring attention
- **DEBUG** - Detailed debugging information

---

## Security Considerations

1. **API Authentication**: Consider adding authentication for production
2. **Network Security**: Use HTTPS/WSS in production environments
3. **Smart Home**: Ensure IoT devices are on secure network
4. **Data Privacy**: User emotions and conversations stay local
5. **Access Control**: Limit API access to trusted devices

---

## Future Enhancements

Planned features:
- Calendar integration for task scheduling
- Weather-based automation
- Voice command parsing for smart home
- Multi-user emotion tracking
- Advanced scene programming
- IFTTT-style automation rules

---

## Support

For issues or questions:
1. Check logs for error messages
2. Review API documentation
3. Test with curl commands
4. Open GitHub issue if needed

---

## Version History

- **v2.1** - Initial industrial-level features release
  - Port-specific Gemini control
  - Emotion detection integration
  - Task scheduling
  - Information retrieval
  - Smart home integration
  - Enhanced audio support
