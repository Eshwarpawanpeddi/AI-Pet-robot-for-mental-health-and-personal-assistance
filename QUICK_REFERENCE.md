# Quick Reference Guide

## 🚀 Quick Commands

### Start the System
```bash
# Activate virtual environment
source venv/bin/activate

# Launch all servers
python server/launch_all.py
```

### Verify Ports
```bash
python verify_ports.py
```

### Access Interfaces
- Primary Control: http://localhost:8000
- Mobile Interface: http://localhost:3000
- Emotion Detection: http://localhost:9999
- Emotion Display: http://localhost:10000

---

## 🎙️ Voice Commands (TTS)

### Send TTS with Default Voice
```bash
curl -X POST http://localhost:8000/api/speak \
  -H "Content-Type: application/json" \
  -d '{"text": "Hello, how are you?"}'
```

### Send TTS with Custom Voice
```bash
curl -X POST http://localhost:8000/api/speak \
  -H "Content-Type: application/json" \
  -d '{
    "text": "Hello with custom voice",
    "voice": "en+f3",
    "speed": 150,
    "pitch": 50
  }'
```

### List Available Voices
```bash
curl http://localhost:8000/api/tts/voices
```

### Update Default Voice Settings
```bash
curl -X POST http://localhost:8000/api/tts/settings \
  -H "Content-Type: application/json" \
  -d '{"voice": "en+m3", "speed": 160, "pitch": 45}'
```

---

## 🔌 Port Architecture

| Port | Service | Voice Input | Voice Output |
|------|---------|-------------|--------------|
| 8000 | Primary Control | ✅ YES | ✅ YES |
| 3000 | Mobile Interface | ❌ NO | ✅ YES (broadcast) |
| 9999 | Emotion Detection | ❌ NO | ✅ YES (broadcast) |
| 10000 | Emotion Display | ❌ NO | ✅ YES (broadcast) |
| Raspberry Pi | Hardware | ❌ NO | ✅ YES (broadcast) |

**Voice Flow**: Port 8000 (input) → Broadcasts to ALL ports (output)

---

## 🎯 Common Tasks

### Move Robot
```bash
curl -X POST http://localhost:8000/api/move \
  -H "Content-Type: application/json" \
  -d '{"direction": "forward", "speed": 50}'
```

### Set Emotion
```bash
curl -X POST http://localhost:8000/api/emotion \
  -H "Content-Type: application/json" \
  -d '{"emotion": "happy"}'
```

### Get System State
```bash
curl http://localhost:8000/api/state
```

### Check Health
```bash
curl http://localhost:8000/health
curl http://localhost:3000/health
curl http://localhost:9999/health
curl http://localhost:10000/health
```

---

## 🐛 Troubleshooting Quick Fixes

### Port Already in Use
```bash
# Find process
lsof -i :8000

# Kill process
kill -9 <PID>
```

### Virtual Environment Not Activated
```bash
source venv/bin/activate
```

### Dependencies Missing
```bash
pip install -r requirements.txt
```

### Test TTS
```bash
# On Linux/Mac
espeak "Hello, this is a test"

# Using API
curl -X POST http://localhost:8000/api/speak \
  -H "Content-Type: application/json" \
  -d '{"text": "Test message"}'
```

---

## 📚 Popular Voice Presets

### Female Voices
```bash
# Standard Female (Recommended)
{"voice": "en+f3", "speed": 150, "pitch": 50}

# Light Female
{"voice": "en+f1", "speed": 160, "pitch": 55}

# Deeper Female
{"voice": "en+f4", "speed": 145, "pitch": 45}
```

### Male Voices
```bash
# Standard Male
{"voice": "en+m3", "speed": 150, "pitch": 50}

# Deep Male
{"voice": "en+m4", "speed": 140, "pitch": 40}

# Very Deep Male
{"voice": "en+m7", "speed": 135, "pitch": 35}
```

### Accent Voices
```bash
# American English
{"voice": "en-us", "speed": 155, "pitch": 50}

# British English
{"voice": "en-uk", "speed": 150, "pitch": 48}

# Scottish English
{"voice": "en-scottish", "speed": 145, "pitch": 50}
```

---

## 📖 Documentation Links

- Full Setup Guide: [USER_GUIDE.md](USER_GUIDE.md)
- Virtual Environment: [VENV_SETUP.md](VENV_SETUP.md)
- Main README: [README.md](README.md)
- Port Verification: Run `python verify_ports.py`

---

## 🆘 Support

If you need help:
1. Check [USER_GUIDE.md](USER_GUIDE.md) for detailed instructions
2. Run `python verify_ports.py` to diagnose connection issues
3. Check server logs in `server/logs/` directory
4. Open an issue on GitHub

---

**Quick Start**: 
```bash
source venv/bin/activate && python server/launch_all.py
```
