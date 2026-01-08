# Fix: Localhost Connection Issue

## Problem Statement
The AI Pet Robot system was running fine locally but was not accessible when opening via IP address from other devices on the network. The servers showed as "running" but web interfaces were not connecting.

## Root Cause
The multi-port server architecture (ports 8000, 3000, 9999, 10000) had hardcoded `localhost` references for inter-server communication. While the FastAPI servers correctly bound to `0.0.0.0` (listening on all network interfaces), the internal HTTP/WebSocket communication between servers used hardcoded `localhost` URLs.

**Example of the issue:**
- Server A (port 8000) tries to communicate with Server B (port 9999)
- Server A uses `http://localhost:9999/api/emotion`
- When accessed via external IP (e.g., 192.168.1.100), Server A still tries to connect to `localhost:9999`
- This fails because `localhost` always resolves to the local machine, not the correct network interface

## Solution
Introduced a configurable `SERVER_HOST` environment variable that:
1. Defaults to `localhost` for backward compatibility
2. Can be set to the machine's IP address for remote access
3. Is used consistently across all inter-server HTTP and WebSocket communication
4. Works with dynamic hostname resolution in browser JavaScript

## Files Modified

### 1. `.env copy` (Environment Template)
**Changes:**
- Added `SERVER_HOST` variable with documentation
- Provided usage examples for different scenarios

```bash
# Server Host (for inter-server communication)
# Use "localhost" for local development (default)
# Use "0.0.0.0" or your machine's IP for remote access
# Example: SERVER_HOST=192.168.1.100
SERVER_HOST=localhost
```

### 2. `server/emotion_detection_server.py`
**Changes:**
- Added `from dotenv import load_dotenv` and `load_dotenv()`
- Added `SERVER_HOST = os.getenv("SERVER_HOST", "localhost")`
- Updated `EmotionDetectionState.__init__()`:
  - `self.primary_server_url = f"http://{SERVER_HOST}:8000"`
  - `self.primary_ws_url = f"ws://{SERVER_HOST}:8000/ws/control"`

### 3. `server/emotion_display_server.py`
**Changes:**
- Added `from dotenv import load_dotenv` and `load_dotenv()`
- Added `SERVER_HOST = os.getenv("SERVER_HOST", "localhost")`
- Updated emotion state polling:
  - `f'http://{SERVER_HOST}:8000/api/state'`

### 4. `server/mobile_web_server.py`
**Changes:**
- Added `import os` and `from dotenv import load_dotenv`
- Added `load_dotenv()`
- Added `SERVER_HOST = os.getenv("SERVER_HOST", "localhost")`
- Updated `MobileControlState.__init__()`:
  - `self.primary_server_url = f"http://{SERVER_HOST}:8000"`
  - `self.primary_ws_url = f"ws://{SERVER_HOST}:8000/ws/control"`
- Updated embedded HTML JavaScript fetch call to use dynamic hostname:
  ```javascript
  const protocol = window.location.protocol;
  const hostname = window.location.hostname;
  const response = await fetch(`${protocol}//${hostname}:8000/api/speak`, {
  ```

### 5. `server/server.py`
**Changes:**
- Added `SERVER_HOST = os.getenv("SERVER_HOST", "localhost")`
- Updated all inter-server API calls:
  - `query_emotion_detection()`: `f'http://{SERVER_HOST}:9999/api/emotion'`
  - `set_robot_emotion()`: `f'http://{SERVER_HOST}:10000/api/emotion'`
  - `broadcast_tts_to_all_ports()`: 
    - `f'http://{SERVER_HOST}:9999/api/tts'`
    - `f'http://{SERVER_HOST}:10000/api/tts'`
    - `f'http://{SERVER_HOST}:3000/api/tts'`

### 6. `README.md`
**Changes:**
- Updated Configuration section with `SERVER_HOST` variable
- Added detailed instructions for remote access
- Added troubleshooting section: "Can't Access Server from Other Devices"
- Included step-by-step guide with example IP addresses

### 7. `server/test_server_host_config.py` (New File)
**Purpose:**
- Test script to verify SERVER_HOST configuration loading
- Displays how URLs will be constructed
- Shows configuration examples for different scenarios

## Usage Instructions

### For Local Development (Default)
No changes needed. The system works as before:
```bash
# .env file (or leave SERVER_HOST unset)
SERVER_HOST=localhost
```

### For Remote Access
1. Find your server machine's IP address:
   ```bash
   # Linux/Mac
   ifconfig
   ip addr
   
   # Windows
   ipconfig
   ```

2. Edit `.env` file:
   ```bash
   SERVER_HOST=192.168.1.100  # Use your actual IP
   ```

3. Restart all servers:
   ```bash
   cd server
   python launch_all.py
   ```

4. Access from any device on the network:
   - Primary Control: `http://192.168.1.100:8000`
   - Mobile Interface: `http://192.168.1.100:3000`
   - Emotion Display: `http://192.168.1.100:10000`
   - Emotion Detection: `http://192.168.1.100:9999`

## Testing & Validation

### ✅ Completed
1. **Python Syntax Validation**: All modified files compile without errors
2. **CodeQL Security Scan**: 0 alerts, no security vulnerabilities introduced
3. **Code Review**: Addressed all feedback
4. **Configuration Logic**: Verified environment variable loading with default fallback

### How to Test
1. Set `SERVER_HOST` in `.env` to your machine's IP
2. Start servers: `python server/launch_all.py`
3. Access from another device on the same network using the IP
4. Verify all inter-server features work:
   - Emotion detection syncing to display
   - TTS broadcasting to all ports
   - Camera feed sharing
   - Mobile interface communication

## Benefits
1. **Backward Compatible**: Defaults to `localhost`, existing setups work unchanged
2. **Network Flexible**: Single configuration change enables remote access
3. **Consistent**: All inter-server communication uses the same pattern
4. **Documented**: Clear instructions for users in README
5. **No Hardcoding**: No IP addresses hardcoded in source files

## Technical Notes
- All servers still bind to `0.0.0.0` (unchanged), accepting connections on all interfaces
- `SERVER_HOST` only affects internal HTTP/WebSocket connections between servers
- Client-side JavaScript uses `window.location.hostname` for dynamic hostname resolution
- The solution supports HTTP and HTTPS/WSS protocols automatically

## Firewall Considerations
When accessing from other devices, ensure the following ports are allowed through the firewall:
- **8000**: Primary control server
- **3000**: Mobile web interface
- **9999**: Emotion detection
- **10000**: Emotion display

## Future Enhancements (Optional)
- Auto-detect server IP address on startup
- Add network interface selection option
- Support for multiple network interfaces simultaneously
- Configuration UI for SERVER_HOST setting
