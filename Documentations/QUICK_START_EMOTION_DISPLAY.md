# Quick Start Guide: Updated Emotion Display System

## What Changed?

### Port 8000 (Primary Server)
- **Root URL** (`http://localhost:8000/`) → Now shows **emotion-only display** (no controls)
- **Control Panel** (`http://localhost:8000/control`) → Full control interface with all features

### Port 10000 (Emotion Display Server)
- Now properly synced with port 8000
- Receives real-time emotion updates
- Can be displayed on separate monitors/devices

## Quick Start

### 1. Start the Servers

**Option A: Start All Servers (Recommended)**
```bash
cd server
python launch_all.py
```

**Option B: Start Individual Servers**
```bash
# Terminal 1 - Primary Server (Port 8000)
cd server
python server.py

# Terminal 2 - Emotion Display Server (Port 10000)
cd server
python emotion_display_server.py
```

### 2. Open the Interfaces

| URL | Purpose | Use Case |
|-----|---------|----------|
| `http://localhost:8000/` | Emotion Display Only | Dedicated emotion screen, TV display |
| `http://localhost:8000/control` | Full Control Panel | Primary control interface |
| `http://localhost:10000/` | Emotion Display (Remote) | Second screen, remote display |

### 3. Test Emotion Changes

The emotions will automatically sync across all displays!

**Via Control Panel:**
1. Open `http://localhost:8000/control`
2. Click emotion buttons: 😊 Happy, 😢 Sad, 💢 Angry, 😐 Neutral
3. Watch all displays update in real-time

**Via WebSocket (for testing):**
```python
import asyncio
import websockets
import json

async def set_emotion(emotion):
    async with websockets.connect('ws://localhost:8000/ws/control') as ws:
        await ws.send(json.dumps({"type": "emotion", "emotion": emotion}))

asyncio.run(set_emotion("happy"))
```

## Enhanced Animations

### 😊 Happy
- Golden sparkle particles that float and rotate
- Glowing aura that pulses gently
- Smile-shaped eyes (upward curves)

### 😐 Neutral
- Gentle breathing animation (pulse effect)
- Soft ambient glow
- Floating particles in a calm pattern

### 💢 Angry
- Red pulsing background
- Shaking/vibration effect
- Animated anger sparks
- Aggressive angled eyebrows

### 😢 Sad
- Drooping eyes
- Falling tears with physics
- Blue color scheme

## Connection Status

All emotion displays show a connection indicator in the top-right:
- 🟢 **Connected** - Green badge, receiving updates
- 🔴 **Disconnected** - Red badge, will auto-reconnect

## Troubleshooting

### "Connection Error" on Port 8000
1. Make sure port 8000 server is running: `python server.py`
2. Check if port is already in use: `lsof -i :8000`
3. Restart the server

### "Connection Error" on Port 10000
1. Make sure port 10000 server is running: `python emotion_display_server.py`
2. Check if port is already in use: `lsof -i :10000`
3. Verify port 8000 is running (port 10000 needs it for syncing)

### Emotions Not Syncing
1. Check both servers are running
2. Look at server logs for errors
3. Verify WebSocket connections in browser console (F12)
4. Try refreshing the emotion display pages

### Animations Not Showing
1. Make sure you're using a modern browser (Chrome, Firefox, Edge)
2. Check browser console (F12) for JavaScript errors
3. Verify the emotion_display.html file exists in frontend/

## Architecture Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                     Port 8000 (Primary)                     │
│  ┌──────────────────┐           ┌──────────────────┐       │
│  │   / (root)       │           │   /control       │       │
│  │ Emotion Display  │           │ Full Interface   │       │
│  │  Only (NEW!)     │           │ with Controls    │       │
│  └──────────────────┘           └──────────────────┘       │
│                                                             │
│  WebSocket Endpoints:                                      │
│  • /ws/control - Control commands                         │
│  • /ws/emotion_display - Emotion updates (NEW!)           │
│  • /ws/raspberry_pi - Hardware connection                 │
│  • /ws/ros - ROS bridge                                   │
└─────────────────────────────────────────────────────────────┘
                              │
                    Emotion Sync (HTTP POST + Poll)
                              │
                              ↓
┌─────────────────────────────────────────────────────────────┐
│                  Port 10000 (Emotion Display)               │
│  ┌──────────────────────────────────────────────────────┐  │
│  │            Dedicated Emotion Display                  │  │
│  │  • Polls port 8000 for updates                       │  │
│  │  • Receives POST updates from port 8000              │  │
│  │  • Broadcasts to all connected clients               │  │
│  └──────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
```

## Files Changed

1. **frontend/emotion_display.html** (NEW)
   - Dedicated emotion-only display
   - Enhanced animations for all emotions
   - Connection status indicator
   - WebSocket integration

2. **server/server.py** (MODIFIED)
   - Added `/ws/emotion_display` WebSocket endpoint
   - Updated root `/` to serve emotion_display.html
   - Added `/control` for full control panel
   - Enhanced emotion synchronization

3. **EMOTION_DISPLAY_UPDATE.md** (NEW)
   - Comprehensive documentation
   - Screenshots and examples
   - Architecture details

## Next Steps

1. ✅ Verify both servers start successfully
2. ✅ Test emotion changes on port 8000 control panel
3. ✅ Verify port 8000 root shows emotion only
4. ✅ Verify port 10000 syncs emotions
5. ✅ Test all four emotions (happy, sad, angry, neutral)
6. ✅ Verify animations are working
7. ✅ Check connection indicators

## Support

If you encounter issues:
1. Check server logs in `server/logs/`
2. Look at browser console (F12)
3. Verify WebSocket connections
4. Restart servers if needed

---

**Updated:** January 3, 2026
**Status:** ✅ Ready to Use
