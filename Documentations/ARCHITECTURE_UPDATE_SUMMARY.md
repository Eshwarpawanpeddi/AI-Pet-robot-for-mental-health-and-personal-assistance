# Architecture Update Summary - Direct Raspberry Pi Motor Control

## Overview

This update transforms the AI Pet Robot from a distributed architecture with separate ESP12E motor controller to a unified architecture where the Raspberry Pi directly controls all 4 motors via GPIO pins.

## Key Changes

### 1. Hardware Architecture

**Before:**
- ESP12E microcontroller for motor control (2 motors)
- L298N motor driver (single unit)
- Raspberry Pi for display and audio only
- Wi-Fi communication between server, ESP12E, and Raspberry Pi

**After:**
- Raspberry Pi 4 handles ALL functions (motors, display, audio)
- 2x L298N motor drivers for 4-wheel control
- Direct GPIO control with PWM
- Simplified architecture with fewer components

### 2. GPIO Pin Configuration

**Motor Driver 1 (Front Wheels - Motors A & B):**
| Pin Function | GPIO Pin | Description |
|--------------|----------|-------------|
| Motor A IN1 | GPIO17 | Direction control 1 |
| Motor A IN2 | GPIO27 | Direction control 2 |
| Motor A ENA | GPIO22 | PWM speed control |
| Motor B IN3 | GPIO23 | Direction control 1 |
| Motor B IN4 | GPIO24 | Direction control 2 |
| Motor B ENB | GPIO25 | PWM speed control |

**Motor Driver 2 (Rear Wheels - Motors C & D):**
| Pin Function | GPIO Pin | Description |
|--------------|----------|-------------|
| Motor C IN1 | GPIO5 | Direction control 1 |
| Motor C IN2 | GPIO6 | Direction control 2 |
| Motor C ENA | GPIO13 | PWM speed control |
| Motor D IN3 | GPIO19 | Direction control 1 |
| Motor D IN4 | GPIO26 | Direction control 2 |
| Motor D ENB | GPIO12 | PWM speed control |

### 3. Motor Control Implementation

**Movement Commands:**
- **Forward**: All 4 motors rotate forward
- **Backward**: All 4 motors rotate backward
- **Left Turn**: Left motors backward, right motors forward (tank-style)
- **Right Turn**: Left motors forward, right motors backward (tank-style)
- **Stop**: All motors stopped, PWM at 0%

**PWM Configuration:**
- Frequency: 1000Hz (1kHz)
- Duty Cycle Range: 0-100%
- Speed Mapping: Direct percentage to duty cycle

### 4. Code Changes

#### `raspberry_pi_controller.py`
**New Features:**
- GPIO initialization for 12 motor control pins
- PWM setup for 4 motors at 1kHz
- Motor control methods: `_move_forward()`, `_move_backward()`, `_turn_left()`, `_turn_right()`, `_stop_all_motors()`
- WebSocket message handler for motor commands
- Thread-safe motor control with locks
- Comprehensive error handling and logging

**Key Methods:**
```python
async def handle_motor_command(direction: str, speed: int)
def _init_motors() -> None
def _move_forward(duty_cycle: int) -> None
def _move_backward(duty_cycle: int) -> None
def _turn_left(duty_cycle: int) -> None
def _turn_right(duty_cycle: int) -> None
def _stop_all_motors() -> None
```

#### `server.py`
**Modified:**
- `handle_move_command()` now routes to Raspberry Pi instead of ESP12E
- Updated logging messages
- Removed ESP12E-specific code paths for motor control

**Communication:**
```json
{
  "type": "move",
  "direction": "forward|backward|left|right|stop",
  "speed": 0-100
}
```

### 5. New Testing Infrastructure

#### `test_motors.py`
**Features:**
- Automated health checks
- Basic movement tests (forward, backward, left, right)
- Speed calibration tests (25%, 50%, 75%, 100%)
- Turn tests (tank-style steering)
- Pattern test (square movement)
- Emergency stop functionality
- Comprehensive error handling

**Usage:**
```bash
cd hardware/raspberry_pi
python3 test_motors.py
```

### 6. Documentation Updates

#### Updated Files:
1. **README.md**
   - Removed ESP12E setup instructions
   - Added 4-wheel GPIO pin configuration
   - Updated architecture diagram
   - Updated hardware requirements
   - Updated troubleshooting section

2. **robot-architecture.md**
   - Updated system overview diagram
   - Removed ESP12E component description
   - Added motor control implementation details
   - Updated communication protocols
   - Added GPIO pin mapping tables
   - Added wiring guide

3. **hardware-code.md**
   - Complete rewrite for Raspberry Pi motor control
   - Removed ESP12E/Arduino code
   - Added Python motor control examples
   - Added GPIO initialization code
   - Added PWM setup documentation
   - Updated testing procedures

#### New Files:
1. **4-wheel-setup-guide.md**
   - Comprehensive step-by-step setup guide
   - Detailed wiring diagrams with ASCII art
   - Power distribution guide
   - GPIO pin connection tables
   - Testing procedures
   - Troubleshooting guide
   - Safety guidelines
   - Performance optimization tips

## Benefits of New Architecture

### Simplified Hardware
✅ **Fewer Components**: Eliminated ESP12E module
✅ **Single Controller**: One Raspberry Pi for everything
✅ **Easier Wiring**: All connections to one device
✅ **Lower Cost**: One less microcontroller needed

### Better Performance
✅ **Direct Control**: No Wi-Fi latency for motor commands
✅ **Synchronized Motors**: All 4 motors controlled in sync
✅ **Higher PWM Frequency**: 1kHz for smoother operation
✅ **Better Responsiveness**: Immediate command execution

### Enhanced Capabilities
✅ **4-Wheel Drive**: Better stability and maneuverability
✅ **Tank-Style Steering**: Tighter turning radius
✅ **Fine Speed Control**: 0-100% PWM duty cycle
✅ **Unified Platform**: Easier to add more sensors/features

### Improved Reliability
✅ **Fewer Network Dependencies**: Direct GPIO control
✅ **Better Error Handling**: Comprehensive exception handling
✅ **Safety Features**: Automatic stop on connection loss
✅ **Testing Framework**: Automated testing suite

## Migration Guide

For users upgrading from ESP12E architecture:

### Hardware Changes Required:
1. **Remove**: ESP12E module
2. **Add**: Second L298N motor driver
3. **Add**: Two additional DC motors
4. **Rewire**: Connect both L298N drivers to Raspberry Pi GPIO
5. **Power**: Ensure adequate power supply (3A+ recommended)

### Software Changes Required:
1. **Update**: Pull latest code from repository
2. **Install**: RPi.GPIO library on Raspberry Pi
3. **Configure**: Update SERVER_URL in raspberry_pi_controller.py
4. **Test**: Run test_motors.py for validation
5. **Remove**: ESP12E firmware no longer needed

### Testing Checklist:
- [ ] Verify GPIO connections
- [ ] Test each motor individually
- [ ] Test all movement directions
- [ ] Test speed control (25%, 50%, 75%, 100%)
- [ ] Test tank-style turning
- [ ] Verify stop command works
- [ ] Check WebSocket communication
- [ ] Validate with pattern test

## System Requirements

### Hardware:
- Raspberry Pi 4 (4GB+ recommended)
- 2x L298N Motor Driver modules
- 4x DC Motors (6V-12V)
- Power supply (7-12V, 3A+ minimum)
- HDMI display
- Audio output device

### Software:
- Python 3.9+
- RPi.GPIO library
- websockets library
- asyncio library

### Network:
- Wi-Fi connection to server
- Same network as central server
- Port 8000 accessible

## Performance Metrics

### PWM Configuration:
- Frequency: 1000Hz (1kHz)
- Resolution: 0-100% duty cycle
- Update Rate: Real-time (limited by network latency)

### Motor Response:
- Command Latency: <100ms (network dependent)
- Speed Change Rate: Immediate (PWM update)
- Stop Response: Immediate (safety critical)

### Network Requirements:
- Bandwidth: Minimal (<10 KB/s)
- Latency: <100ms recommended
- Connection: WebSocket (persistent)

## Security Considerations

✅ **Code Analysis**: CodeQL scan passed with 0 vulnerabilities
✅ **Input Validation**: Speed values clamped to 0-100 range
✅ **Thread Safety**: Motor control protected with locks
✅ **Error Handling**: Comprehensive exception handling
✅ **Safety Features**: Automatic stop on errors/disconnection

## Future Enhancements

### Potential Additions:
1. **Encoders**: Add rotary encoders for precise position control
2. **IMU**: Add gyroscope/accelerometer for better navigation
3. **Ultrasonic Sensors**: Add obstacle detection
4. **Current Sensing**: Monitor motor current for overload detection
5. **Battery Monitoring**: Add voltage sensing for battery level
6. **PID Control**: Implement PID for smoother motor control

### Software Improvements:
1. **Speed Curves**: Non-linear speed mapping for better low-speed control
2. **Motor Trim**: Individual motor calibration for straight driving
3. **Acceleration Ramping**: Gradual speed changes for smoother motion
4. **Dead Reckoning**: Track position using encoder data
5. **Autonomous Navigation**: ROS integration for path planning

## Support and Resources

### Documentation:
- [4-Wheel Setup Guide](downloads/4-wheel-setup-guide.md)
- [Hardware Code Documentation](downloads/hardware-code.md)
- [Robot Architecture](downloads/robot-architecture.md)
- [Main README](README.md)

### Code Files:
- `hardware/raspberry_pi/raspberry_pi_controller.py` - Main controller
- `hardware/raspberry_pi/test_motors.py` - Testing script
- `server/server.py` - Server with motor routing

### Community:
- GitHub Issues: Report bugs or request features
- Pull Requests: Contribute improvements
- Discussions: Share your build and ask questions

## Conclusion

This architecture update significantly simplifies the hardware while improving performance, reliability, and capabilities. The Raspberry Pi now serves as a true all-in-one controller, managing motors, display, and audio from a single platform.

The implementation is production-ready with comprehensive documentation, automated testing, and security validation. Users can confidently build and deploy this updated architecture for a more capable mental health companion robot.

---

**Version**: 2.0  
**Date**: December 2024  
**Status**: Production Ready  
**Security**: ✅ Validated (0 vulnerabilities)  
**Testing**: ✅ Automated test suite included
