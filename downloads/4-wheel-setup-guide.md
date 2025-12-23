# 4-Wheel Motor Control Setup Guide

## Overview

This guide provides detailed instructions for setting up the AI Pet Robot with direct Raspberry Pi motor control using 2 L298N motor drivers for a 4-wheel configuration. This setup eliminates the ESP12E module, simplifying the hardware architecture.

## Hardware Requirements

### Components List

1. **Raspberry Pi 4** (4GB or higher recommended)
2. **2x L298N Motor Driver Modules**
3. **4x DC Motors** (6V-12V recommended)
4. **Power Supply** (7-12V, minimum 3A capacity)
5. **HDMI Display** (for face animations)
6. **Speaker/Audio Output** (3.5mm jack compatible)
7. **Jumper Wires** (Male-to-Female for GPIO connections)
8. **Robot Chassis** with 4-wheel mounting
9. **Optional**: Battery pack (7.4V LiPo or 12V NiMH) for mobile operation

### Tools Needed

- Soldering iron (if permanent connections desired)
- Wire strippers
- Multimeter (for testing connections)
- Screwdriver set

## Wiring Diagram

### Complete System Connection

```
                    RASPBERRY PI 4
                   ┌──────────────┐
                   │              │
    ┌──────────────┤ GPIO 17      │
    │  ┌───────────┤ GPIO 27      │
    │  │  ┌────────┤ GPIO 22      │
    │  │  │  ┌─────┤ GPIO 23      │
    │  │  │  │  ┌──┤ GPIO 24      │
    │  │  │  │  │ ┌┤ GPIO 25      │
    │  │  │  │  │ ││              │
    │  │  │  │  │ ││ GPIO 5  ├────┼───┐
    │  │  │  │  │ ││ GPIO 6  ├────┼─┐ │
    │  │  │  │  │ ││ GPIO 13 ├────┼─┤ │
    │  │  │  │  │ ││ GPIO 19 ├────┼─┤ │
    │  │  │  │  │ ││ GPIO 26 ├────┼─┤ │
    │  │  │  │  │ ││ GPIO 12 ├────┼─┤ │
    │  │  │  │  │ ││         │    │ │ │
    │  │  │  │  │ ││ GND ├────────┼─┤ │
    │  │  │  │  │ │└──────────┘   │ │ │
    │  │  │  │  │ │               │ │ │
    ▼  ▼  ▼  ▼  ▼ ▼               ▼ ▼ ▼
┌─────────────────────┐   ┌─────────────────────┐
│  L298N DRIVER 1     │   │  L298N DRIVER 2     │
│  (Front Wheels)     │   │  (Rear Wheels)      │
├─────────────────────┤   ├─────────────────────┤
│ IN1 ← GPIO 17       │   │ IN1 ← GPIO 5        │
│ IN2 ← GPIO 27       │   │ IN2 ← GPIO 6        │
│ ENA ← GPIO 22 (PWM) │   │ ENA ← GPIO 13 (PWM) │
│ IN3 ← GPIO 23       │   │ IN3 ← GPIO 19       │
│ IN4 ← GPIO 24       │   │ IN4 ← GPIO 26       │
│ ENB ← GPIO 25 (PWM) │   │ ENB ← GPIO 12 (PWM) │
│ GND ─────────────────── │ GND                 │
│ 12V ← Power Supply  │   │ 12V ← Power Supply  │
└──┬──────────┬───────┘   └──┬──────────┬───────┘
   │          │              │          │
   │ OUT1/2   │ OUT3/4       │ OUT1/2   │ OUT3/4
   ▼          ▼              ▼          ▼
┌────────┐ ┌────────┐    ┌────────┐ ┌────────┐
│Motor A │ │Motor B │    │Motor C │ │Motor D │
│(Front  │ │(Front  │    │(Rear   │ │(Rear   │
│ Left)  │ │ Right) │    │ Left)  │ │ Right) │
└────────┘ └────────┘    └────────┘ └────────┘
```

### Power Distribution

```
                    7-12V Power Supply
                           │
                    ┌──────┴──────┐
                    │             │
                    ▼             ▼
            L298N Driver 1   L298N Driver 2
                  12V             12V
                   │               │
                   │               │
            Connected to motors (4x)
                   
        All GND connected together:
        - Pi GND
        - L298N #1 GND
        - L298N #2 GND  
        - Power Supply GND
```

**CRITICAL**: Common ground connection is essential for proper signal communication!

## Step-by-Step Setup

### Step 1: Prepare the Raspberry Pi

1. **Install Raspberry Pi OS**:
   ```bash
   # Use Raspberry Pi Imager to install Raspberry Pi OS (64-bit recommended)
   # Enable SSH and configure Wi-Fi during setup
   ```

2. **Update System**:
   ```bash
   sudo apt-get update
   sudo apt-get upgrade -y
   ```

3. **Install Dependencies**:
   ```bash
   sudo apt-get install -y python3-pip python3-websockets python3-rpi.gpio git
   pip3 install websockets asyncio
   ```

4. **Add User to GPIO Group**:
   ```bash
   sudo usermod -a -G gpio $USER
   # Logout and login again for changes to take effect
   ```

### Step 2: Wire Motor Driver 1 (Front Wheels)

1. **Connect GPIO Pins to L298N Driver 1**:
   - GPIO 17 → IN1 (Motor A Direction 1)
   - GPIO 27 → IN2 (Motor A Direction 2)
   - GPIO 22 → ENA (Motor A PWM Enable)
   - GPIO 23 → IN3 (Motor B Direction 1)
   - GPIO 24 → IN4 (Motor B Direction 2)
   - GPIO 25 → ENB (Motor B PWM Enable)

2. **Connect Motors**:
   - OUT1 & OUT2 → Motor A (Front Left Wheel)
   - OUT3 & OUT4 → Motor B (Front Right Wheel)

3. **Remove ENA/ENB Jumpers**: 
   - Remove the jumper caps from ENA and ENB pins on the L298N
   - This allows PWM control from Raspberry Pi

### Step 3: Wire Motor Driver 2 (Rear Wheels)

1. **Connect GPIO Pins to L298N Driver 2**:
   - GPIO 5 → IN1 (Motor C Direction 1)
   - GPIO 6 → IN2 (Motor C Direction 2)
   - GPIO 13 → ENA (Motor C PWM Enable)
   - GPIO 19 → IN3 (Motor D Direction 1)
   - GPIO 26 → IN4 (Motor D Direction 2)
   - GPIO 12 → ENB (Motor D PWM Enable)

2. **Connect Motors**:
   - OUT1 & OUT2 → Motor C (Rear Left Wheel)
   - OUT3 & OUT4 → Motor D (Rear Right Wheel)

3. **Remove ENA/ENB Jumpers**: 
   - Remove the jumper caps from ENA and ENB pins on the L298N

### Step 4: Power Connections

1. **Connect Power Supply to L298N Drivers**:
   - Connect positive (+) terminal of 7-12V power supply to both L298N 12V inputs
   - You can use a power distribution board or wire in parallel

2. **Connect All Grounds**:
   - Connect GND from Raspberry Pi
   - Connect GND from L298N Driver 1
   - Connect GND from L298N Driver 2
   - Connect GND from Power Supply
   - All grounds MUST be connected together!

3. **Power the Raspberry Pi**:
   - Use separate 5V power supply for Raspberry Pi (official adapter recommended)
   - Do NOT power the Pi from L298N 5V output (insufficient current)

### Step 5: Test GPIO Connections

Before running the motor control software, verify GPIO setup:

1. **Check GPIO Status**:
   ```bash
   gpio readall
   ```

2. **Test Individual GPIO Pins** (optional):
   ```bash
   # Install gpio command if not available
   sudo apt-get install wiringpi
   
   # Set a pin to output and toggle it
   gpio mode 17 out
   gpio write 17 1  # Set HIGH
   gpio write 17 0  # Set LOW
   ```

3. **Verify L298N Connections**:
   - Use multimeter to check continuity
   - Verify voltage levels on 12V input (should read 7-12V)

### Step 6: Install Robot Software

1. **Clone Repository**:
   ```bash
   cd ~
   git clone https://github.com/Eshwarpawanpeddi/AI-Pet-robot-for-mental-health-and-personal-assistance.git
   cd AI-Pet-robot-for-mental-health-and-personal-assistance/hardware/raspberry_pi
   ```

2. **Configure Server URL**:
   ```bash
   nano raspberry_pi_controller.py
   # Update: SERVER_URL = "ws://YOUR_SERVER_IP:8000/ws/raspberry_pi"
   ```

3. **Test Controller** (without motors connected first):
   ```bash
   python3 raspberry_pi_controller.py
   ```

   Expected output:
   ```
   INFO - Raspberry Pi Controller Initialized with 4-wheel motor control
   INFO - GPIO pins initialized for 4-wheel motor control
   INFO - PWM initialized for all 4 motors at 1000Hz
   INFO - Connected to server
   ```

### Step 7: Initial Motor Testing

**IMPORTANT**: Start with motors elevated (wheels not touching ground) for safety.

1. **Test Forward Movement at Low Speed**:
   ```bash
   curl -X POST http://SERVER_IP:8000/api/command \
     -H "Content-Type: application/json" \
     -d '{"type":"move","direction":"forward","speed":25}'
   ```
   - All 4 motors should rotate forward slowly
   - Verify rotation direction is correct

2. **Test Backward Movement**:
   ```bash
   curl -X POST http://SERVER_IP:8000/api/command \
     -H "Content-Type: application/json" \
     -d '{"type":"move","direction":"backward","speed":25}'
   ```
   - All 4 motors should rotate backward

3. **Test Left Turn**:
   ```bash
   curl -X POST http://SERVER_IP:8000/api/command \
     -H "Content-Type: application/json" \
     -d '{"type":"move","direction":"left","speed":25}'
   ```
   - Left motors (A & C) should rotate backward
   - Right motors (B & D) should rotate forward

4. **Test Right Turn**:
   ```bash
   curl -X POST http://SERVER_IP:8000/api/command \
     -H "Content-Type: application/json" \
     -d '{"type":"move","direction":"right","speed":25}'
   ```
   - Left motors (A & C) should rotate forward
   - Right motors (B & D) should rotate backward

5. **Test Stop**:
   ```bash
   curl -X POST http://SERVER_IP:8000/api/command \
     -H "Content-Type: application/json" \
     -d '{"type":"move","direction":"stop","speed":0}'
   ```
   - All motors should stop immediately

### Step 8: Calibrate Motor Directions

If any motor rotates in the wrong direction:

1. **Swap Motor Wires**: Reverse the two wires connected to that motor's OUT terminals
2. **Alternative**: Modify code to invert that motor's direction signals

### Step 9: Speed Calibration

Test different speed levels to find optimal range:

```bash
# Test various speeds
for speed in 25 50 75 100; do
  echo "Testing speed: $speed%"
  curl -X POST http://SERVER_IP:8000/api/command \
    -H "Content-Type: application/json" \
    -d "{\"type\":\"move\",\"direction\":\"forward\",\"speed\":$speed}"
  sleep 3
  curl -X POST http://SERVER_IP:8000/api/command \
    -H "Content-Type: application/json" \
    -d '{"type":"move","direction":"stop","speed":0}'
  sleep 1
done
```

## Troubleshooting

### Motors Don't Move

**Check Power**:
- Verify 7-12V supply is connected and switched on
- Check voltage at L298N 12V input with multimeter
- Ensure power supply can provide sufficient current (minimum 2A per driver)

**Check Connections**:
- Verify all GPIO connections are secure
- Check motor connections to L298N OUT terminals
- Ensure ENA/ENB jumpers are REMOVED

**Check Common Ground**:
- Most common issue! Verify all grounds connected together
- Use multimeter continuity test

**Check GPIO**:
- Run `gpio readall` to verify GPIO setup
- Check for GPIO permission errors in logs
- Try running with sudo: `sudo python3 raspberry_pi_controller.py`

### Motors Move in Wrong Direction

**For Single Motor**:
- Swap the two wires to that motor's OUT terminals on L298N

**For Multiple Motors**:
- Check your wheel arrangement (ensure left/right assignment is correct)
- Verify GPIO pin assignments in code

### Weak or Slow Motors

**Insufficient Power**:
- Check power supply voltage (should be 7-12V)
- Verify power supply can provide adequate current
- Check for voltage drop under load

**PWM Issues**:
- Verify PWM frequency is set to 1000Hz
- Check PWM duty cycle is being applied correctly

### Motors Work Individually But Not Together

**Power Supply Overload**:
- Current demand may exceed supply capacity
- Use higher current power supply (4A+ recommended)
- Check for brownout/reset when all motors run

**Heat Issues**:
- L298N drivers may overheat under continuous load
- Add heatsinks if available
- Reduce duty cycle or add cooling

### Connection Lost / Robot Stops

**Network Issues**:
- Verify Raspberry Pi Wi-Fi connection: `ping SERVER_IP`
- Check server is running: `curl http://SERVER_IP:8000/health`
- Review logs for connection errors

**Safety Feature**:
- Motors automatically stop if WebSocket connection is lost
- This is intentional for safety

## Performance Optimization

### PWM Frequency Tuning

Default is 1kHz, but you can experiment:

```python
# In raspberry_pi_controller.py
PWM_FREQUENCY = 1000  # Try 500, 1000, 2000, 5000
```

Higher frequency = smoother operation but may reduce torque
Lower frequency = more torque but may be noisier

### Speed Curve Adjustment

For finer control at low speeds, implement a non-linear speed curve:

```python
def apply_speed_curve(speed):
    """Apply non-linear speed curve for better low-speed control"""
    # Square root curve gives better control at low speeds
    import math
    return int(math.sqrt(speed / 100) * 100)
```

### Motor Synchronization

If motors have different speeds, add trim values:

```python
MOTOR_TRIM = {
    'A': 1.0,   # No adjustment
    'B': 0.95,  # Slightly slower
    'C': 1.0,   # No adjustment
    'D': 0.98   # Slightly slower
}

# Apply in motor control function
duty_cycle_adjusted = duty_cycle * MOTOR_TRIM['A']
```

## Safety Guidelines

1. **Emergency Stop**: Always have physical access to power switch
2. **Testing**: Start with motors elevated, test at low speeds first
3. **Supervision**: Never leave robot running unattended during testing
4. **Power**: Use appropriate fuses in power supply circuit
5. **Heat**: Monitor L298N temperature during extended use
6. **Clearance**: Ensure robot has adequate clearance and can't fall

## Next Steps

After successful motor testing:

1. **Integrate HDMI Display**: Connect display for face animations
2. **Add Audio Output**: Connect speaker to audio jack
3. **Test Web Interface**: Access robot controls through web browser
4. **Mobile App**: Install and configure mobile app for remote control
5. **Mental Health Features**: Test mood logging, affirmations, etc.
6. **Autonomous Mode**: Set up ROS integration if desired

## Additional Resources

- **Raspberry Pi GPIO Pinout**: https://pinout.xyz/
- **L298N Datasheet**: Search for "L298N datasheet" for detailed specifications
- **Python RPi.GPIO Documentation**: https://sourceforge.net/projects/raspberry-gpio-python/

## Support

For issues or questions:
- Check repository issues: https://github.com/Eshwarpawanpeddi/AI-Pet-robot-for-mental-health-and-personal-assistance/issues
- Review troubleshooting section in README.md
- Check hardware-code.md for detailed implementation

---

**Remember**: This robot is designed as a mental health companion. Handle with care and respect the safety of yourself and others during setup and operation.
