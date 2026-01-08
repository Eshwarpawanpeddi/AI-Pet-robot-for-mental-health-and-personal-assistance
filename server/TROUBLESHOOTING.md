# Server Troubleshooting Guide

## Common Issues and Solutions

### Issue: Ports 8000 or 10000 Show "Can't Reach Page" Errors

#### Problem
When trying to access http://localhost:8000 or http://localhost:10000, you see a "can't reach page" error.

#### Root Cause
The servers are not starting due to **missing Python dependencies**.

#### Solution

**Step 1: Check Dependencies**

Run the dependency checker to see what's missing:

```bash
cd server
python3 check_dependencies.py
```

This will show you which dependencies are installed and which are missing.

**Step 2: Install Missing Dependencies**

Option A - Install all dependencies from requirements.txt:
```bash
cd server
pip3 install -r requirements.txt
```

Option B - Install only the core dependencies needed for basic operation:
```bash
pip3 install fastapi uvicorn aiohttp python-dotenv websockets numpy opencv-python google-generativeai
```

**Step 3: Verify Installation**

Run the dependency checker again to confirm all packages are installed:
```bash
python3 check_dependencies.py
```

**Step 4: Start the Servers**

Option A - Use the integrated launcher (recommended):
```bash
python3 launch_all.py
```

Option B - Start servers individually:
```bash
# Terminal 1 - Primary Control Server (Port 8000)
python3 server.py

# Terminal 2 - Emotion Display Server (Port 10000)
python3 emotion_display_server.py

# Terminal 3 - Mobile Web Server (Port 3000)
python3 mobile_web_server.py

# Terminal 4 - Emotion Detection Server (Port 9999)
python3 emotion_detection_server.py
```

**Step 5: Verify Servers Are Running**

Open your browser and check:
- http://localhost:8000 - Primary Control Server ✓
- http://localhost:10000 - Emotion Display Server ✓
- http://localhost:3000 - Mobile Web Interface ✓
- http://localhost:9999 - Emotion Detection Server ✓

You can also check the health endpoints:
```bash
curl http://localhost:8000/health
curl http://localhost:10000/health
```

---

### Issue: Import Errors When Starting Servers

#### Symptoms
You see error messages like:
```
ModuleNotFoundError: No module named 'fastapi'
ModuleNotFoundError: No module named 'numpy'
```

#### Solution
The server startup scripts now include helpful error messages that tell you exactly which packages to install. Follow the instructions in the error message, or use the dependency checker as described above.

---

### Issue: TensorFlow Installation Fails

#### Problem
When installing requirements.txt, you see:
```
ERROR: Could not find a version that satisfies the requirement tensorflow==2.15.0
```

#### Root Cause
TensorFlow 2.15.0 is not compatible with Python 3.12+.

#### Solution
1. Use Python 3.11 or earlier, OR
2. Install a compatible TensorFlow version:
```bash
pip3 install tensorflow>=2.16.0
```

Note: TensorFlow is only required for the emotion detection server (port 9999). The other servers (8000, 10000, 3000) will work without it.

---

### Issue: Servers Start But Show Warnings

#### Symptoms
Servers start but show warnings like:
```
Warning: google-generativeai not installed. AI features will be disabled.
ERROR: GEMINI_API_KEY not found in environment
```

#### Solution
These are warnings, not errors. The servers will still run, but some features will be disabled:

1. **Missing GEMINI_API_KEY**: AI chatbot features won't work. To enable:
   - Get an API key from [Google AI Studio](https://makersuite.google.com/app/apikey)
   - Add to `.env` file: `GEMINI_API_KEY=your_api_key_here`

2. **Missing ultralytics**: Autonomous navigation won't work. To enable:
   ```bash
   pip3 install ultralytics
   ```

3. **Missing tensorflow**: Emotion detection (port 9999) won't work. To enable:
   ```bash
   pip3 install tensorflow>=2.16.0
   ```

---

## Checking Server Status

### Using the Dependency Checker
```bash
cd server
python3 check_dependencies.py all    # Check all servers
python3 check_dependencies.py 8000   # Check port 8000 only
python3 check_dependencies.py 10000  # Check port 10000 only
```

### Using the Health Endpoints
```bash
curl http://localhost:8000/health
curl http://localhost:10000/health
```

### Checking Server Logs
The emotion display server writes logs to:
```
server/logs/emotion_display_server.log
```

View the log:
```bash
tail -f server/logs/emotion_display_server.log
```

---

## Quick Reference

### Port Mapping
- **8000** - Primary Control Server (main interface, AI, camera control)
- **10000** - Emotion Display Server (animated robot face)
- **3000** - Mobile Web Server (mobile-friendly interface)
- **9999** - Emotion Detection Server (facial expression analysis)

### Essential Dependencies
All servers need:
- fastapi
- uvicorn
- aiohttp
- python-dotenv

Additional requirements by server:
- **Port 8000**: google-generativeai, numpy, opencv-python, websockets
- **Port 10000**: websockets
- **Port 3000**: websockets
- **Port 9999**: tensorflow, opencv-python, numpy

### Installation Commands

Minimal setup (all servers except emotion detection):
```bash
pip3 install fastapi uvicorn aiohttp python-dotenv websockets numpy opencv-python google-generativeai
```

Full setup (all features):
```bash
cd server
pip3 install -r requirements.txt
```

### Starting Servers

Easiest (all servers at once):
```bash
cd server
python3 launch_all.py
```

Individual servers:
```bash
python3 server.py              # Port 8000
python3 emotion_display_server.py  # Port 10000
python3 mobile_web_server.py   # Port 3000
python3 emotion_detection_server.py # Port 9999
```

---

## Still Having Issues?

1. **Check Python version**: 
   ```bash
   python3 --version
   ```
   Should be Python 3.8 or higher (3.11 recommended)

2. **Check if ports are already in use**:
   ```bash
   lsof -i :8000
   lsof -i :10000
   ```

3. **Try running with verbose logging**:
   ```bash
   python3 server.py --log-level debug
   ```

4. **Check the dependency checker output**:
   ```bash
   python3 check_dependencies.py all
   ```

5. **Verify installation in the correct environment**:
   If using a virtual environment, make sure it's activated before installing packages.
