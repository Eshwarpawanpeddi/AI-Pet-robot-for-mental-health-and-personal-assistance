# Speaker Hardware Setup Guide

This guide explains how to connect and configure a 40mm 8Ω speaker with your Raspberry Pi for the AI Pet Robot.

## Hardware Specifications

- **Speaker Type**: 40mm diameter
- **Impedance**: 8 Ohms
- **Power Rating**: Up to 3W (typical for 40mm speakers)
- **Connections**: 2 ports/terminals (positive and negative)

## Connection Options

### Option 1: Direct Connection via Audio Jack (Recommended for Beginners)

The Raspberry Pi has a 3.5mm audio jack that can drive small speakers.

#### Required Components
- 40mm 8Ω speaker
- 3.5mm audio jack to wire adapter (or cut an old 3.5mm cable)
- Optional: Small amplifier module (PAM8403 or similar) for better volume

#### Wiring Steps

1. **Identify Speaker Terminals**
   - Usually marked with + (positive) and - (negative)
   - Or red wire (positive) and black wire (negative)

2. **Prepare 3.5mm Cable**
   ```
   3.5mm Jack Pinout (looking at jack):
   - Tip: Left audio channel
   - Ring: Right audio channel  
   - Sleeve: Ground (common)
   ```

3. **Connect Without Amplifier** (lower volume)
   ```
   Speaker (+) ──────▶ Tip or Ring of 3.5mm jack
   Speaker (-) ──────▶ Sleeve (ground) of 3.5mm jack
   ```

4. **Connect With Amplifier** (RECOMMENDED - better volume)
   ```
   Raspberry Pi 3.5mm Jack ──▶ Amplifier Input
   Amplifier Output (+) ──────▶ Speaker (+)
   Amplifier Output (-) ──────▶ Speaker (-)
   Amplifier VCC ─────────────▶ 5V (from Pi GPIO or external)
   Amplifier GND ─────────────▶ Ground
   ```

#### Recommended Amplifier: PAM8403

- **Cost**: ~$1-3
- **Power**: 3W per channel
- **Voltage**: 5V DC
- **Perfect** for Raspberry Pi and 8Ω speakers

**PAM8403 Wiring:**
```
┌─────────────────────────────────────────────────┐
│         Raspberry Pi 4                           │
│  ┌──────────────────────────────────┐           │
│  │  3.5mm Audio Jack                │           │
│  │  (Tip/Ring/Sleeve)              │           │
│  └───────────┬──────────────────────┘           │
│              │                                   │
│              │ Audio Cable                       │
│              ▼                                   │
│  ┌───────────────────────────────┐              │
│  │   PAM8403 Amplifier           │              │
│  │   ┌─────┐                     │              │
│  │   │ VCC │◀────── 5V Pin 2/4   │              │
│  │   │ GND │◀────── GND Pin 6    │              │
│  │   │ IN  │◀────── Audio Jack   │              │
│  │   │ OUT+│────────────────┐    │              │
│  │   │ OUT-│──────────┐     │    │              │
│  │   └─────┘          │     │    │              │
│  └────────────────────┼─────┼────┘              │
│                       │     │                    │
│                       │     │                    │
│              ┌────────▼─────▼────────┐          │
│              │   40mm Speaker (8Ω)   │          │
│              │         ┌───┐         │          │
│              │     (+) │   │ (-)     │          │
│              └─────────┴───┴─────────┘          │
└──────────────────────────────────────────────────┘
```

### Option 2: GPIO PWM Output (Direct, No Amplifier Needed)

Use GPIO pins with PWM to drive the speaker directly.

#### Required Components
- 40mm 8Ω speaker
- NPN transistor (2N2222 or BC547)
- 1KΩ resistor
- Diode (1N4148 or similar)
- Breadboard and jumper wires

#### Circuit Diagram

```
                     +5V
                      │
                      │
                  ┌───┴───┐
                  │Speaker│
                  │  8Ω   │
                  └───┬───┘
                      │
              Diode   │
          ┌───────────┤
          │  │<──     │
          │   └───────┤
          │           │
          │      ┌────┴────┐
          │      │ NPN     │
          │      │ Trans   │
          │      │  (C)    │
          │      └────┬────┘
          │           │(E)
          │           │
          │         ──┴──  GND
          │           
          │      1KΩ
    GPIO 18 ───/\/\/\───┐
   (PWM Pin)            │(B)
                    ┌───┴───┐
                    │ Trans │
                    └───────┘
```

#### Wiring Steps

1. **Connect Transistor Base** to GPIO 18 through 1KΩ resistor
2. **Connect Transistor Emitter** to Ground
3. **Connect Transistor Collector** to Speaker (-)
4. **Connect Speaker (+)** to 5V through protection diode
5. **Connect Diode** (flyback protection) across speaker

#### GPIO Pin Assignment (Raspberry Pi 4)
```
Pin 12 (GPIO 18) → PWM output to transistor
Pin 2 or 4       → 5V for speaker
Pin 6, 9, 14, 20 → Ground
```

### Option 3: I2S DAC (Best Audio Quality)

For audiophile-quality sound, use an I2S DAC module.

#### Recommended Modules
- **Adafruit I2S 3W Stereo Speaker Bonnet** (~$15)
- **HiFiBerry DAC+ Zero** (~$15)
- **Pimoroni pHAT DAC** (~$20)

These modules:
- Plug directly onto GPIO header
- Include built-in amplifier
- Provide excellent audio quality
- Have speaker terminals built-in

#### Setup
```bash
# For Adafruit I2S Speaker Bonnet
curl -sS https://raw.githubusercontent.com/adafruit/Raspberry-Pi-Installer-Scripts/master/i2samp.sh | bash

# Reboot
sudo reboot

# Your speaker is now ready!
```

## Software Configuration

### 1. Enable Audio on Raspberry Pi

```bash
# Check audio devices
aplay -l

# Should show built-in audio:
# card 0: Headphones [bcm2835 Headphones], device 0: bcm2835 Headphones [bcm2835 Headphones]
```

### 2. Set Audio Output

```bash
# Set to 3.5mm jack (not HDMI)
sudo raspi-config
# Navigate to: System Options → Audio → Headphones
# Select and exit

# Or use command:
amixer cset numid=3 1
# 0=auto, 1=headphones, 2=hdmi
```

### 3. Test Audio Output

```bash
# Install audio tools
sudo apt-get install -y alsa-utils espeak

# Test with beep
speaker-test -t wav -c 2

# Test with text-to-speech (already in project)
espeak "Hello, I am your AI pet robot!"

# Adjust volume
alsamixer
# Use arrow keys to adjust, ESC to exit

# Or set volume directly (0-100%)
amixer set Master 80%
```

### 4. Configure for Project

The project already uses `espeak` for text-to-speech. Make sure it's installed:

```bash
# Install espeak (should already be installed)
sudo apt-get install -y espeak

# Test
espeak -v en "Testing speaker connection"
```

### 5. Python Audio Control (Optional)

If you want to play custom sounds or music:

```bash
# Install pygame for audio playback
pip3 install pygame

# Install pydub for audio processing
pip3 install pydub
```

**Example Python code:**
```python
import pygame
import subprocess

# Initialize mixer
pygame.mixer.init()

# Play sound file
sound = pygame.mixer.Sound('notification.wav')
sound.play()

# Or use espeak from Python (safer method)
subprocess.run(['espeak', 'Hello World'], check=True)

# Alternative with more control
subprocess.run(['espeak', '-v', 'en', '-s', '150', 'Hello World'], check=True)
```

## Audio Settings for Best Quality

### 1. Adjust ALSA Configuration

Edit `/etc/asound.conf` (create if doesn't exist):

```bash
sudo nano /etc/asound.conf
```

Add:
```
pcm.!default {
    type hw
    card 0
}

ctl.!default {
    type hw
    card 0
}
```

### 2. Increase Audio Buffer (if crackling occurs)

Edit `/boot/config.txt`:
```bash
sudo nano /boot/config.txt
```

Add at the end:
```
# Audio improvements
audio_pwm_mode=2
dtparam=audio=on
```

Reboot:
```bash
sudo reboot
```

## Volume Control

### Command Line

```bash
# Set volume to 80%
amixer set Master 80%

# Mute
amixer set Master mute

# Unmute
amixer set Master unmute

# Get current volume
amixer get Master
```

### From Python

```python
import subprocess

def set_volume(volume):
    """Set volume 0-100"""
    subprocess.run(['amixer', 'set', 'Master', f'{volume}%'])

def mute():
    subprocess.run(['amixer', 'set', 'Master', 'mute'])

def unmute():
    subprocess.run(['amixer', 'set', 'Master', 'unmute'])

# Usage
set_volume(75)
```

## Troubleshooting

### Problem: No Sound

**Solutions:**
```bash
# 1. Check connections
# Make sure speaker wires are connected correctly

# 2. Check audio output setting
amixer cset numid=3 1

# 3. Check volume
amixer set Master 80%
amixer set Master unmute

# 4. Test audio
speaker-test -t wav -c 2

# 5. Check for HDMI audio conflict
# Disable HDMI audio in /boot/config.txt:
sudo nano /boot/config.txt
# Comment out: #hdmi_drive=2
```

### Problem: Audio Crackling/Distortion

**Solutions:**
```bash
# 1. Reduce volume
amixer set Master 60%

# 2. Check power supply
# Raspberry Pi needs 5V 3A adapter
# Speakers draw power - insufficient power causes issues

# 3. Use external amplifier
# Direct connection has lower quality

# 4. Adjust buffer size (add to /boot/config.txt)
audio_pwm_mode=2
```

### Problem: Volume Too Low

**Solutions:**
1. **Use amplifier module** (PAM8403 or similar)
2. **Check volume settings:**
   ```bash
   alsamixer  # Increase to 100
   ```
3. **Use powered speakers** instead of passive speaker
4. **Check speaker impedance** - 8Ω is correct for Pi

### Problem: espeak Not Working

**Solutions:**
```bash
# Reinstall espeak
sudo apt-get remove --purge espeak
sudo apt-get install espeak

# Test
espeak "Testing"

# Check audio device
espeak -v en --stdout "test" | aplay
```

## Hardware Safety Tips

1. **Never connect speaker directly to GPIO without transistor**
   - GPIO pins provide only 3.3V and very low current
   - Can damage Pi or speaker

2. **Use appropriate power supply**
   - Pi needs 5V 3A minimum
   - Speakers draw additional current

3. **Add protection diode**
   - Protects Pi from back-EMF when using GPIO

4. **Check polarity**
   - Wrong polarity won't damage 8Ω speaker but reduces quality
   - Correct: Red (+) to positive, Black (-) to negative

5. **Don't exceed volume limits**
   - 40mm speaker rated for ~3W maximum
   - Don't overdrive or distortion occurs

## Recommended Shopping List

For best results, get these components:

1. **PAM8403 Amplifier Module** - $2
2. **40mm 8Ω 3W Speaker** (already have)
3. **3.5mm audio cable** - $1
4. **Jumper wires** - $2
5. **5V Power Supply** (3A minimum) - $10

**Total additional cost**: ~$5 (excluding power supply)

## Integration with Project

The project's `raspberry_pi_controller.py` already includes TTS support:

```python
# Text-to-speech is handled by:
os.system(f'espeak "{text}"')
```

**No code changes needed!** Just:
1. Connect speaker as described above
2. Ensure espeak is installed
3. Adjust volume with alsamixer
4. Robot will speak responses automatically

## Testing Checklist

- [ ] Speaker connected correctly (check polarity)
- [ ] Audio output set to headphones jack
- [ ] espeak installed and working
- [ ] Volume set to appropriate level (60-80%)
- [ ] `speaker-test` produces sound
- [ ] `espeak "test"` produces speech
- [ ] Robot responses are audible
- [ ] No distortion at normal volume
- [ ] No crackling or static

## Advanced: Stereo or Multiple Speakers

Want better sound? Connect two speakers:

```
PAM8403 (Stereo Amplifier)
├── Channel A → Speaker 1 (Left)
└── Channel B → Speaker 2 (Right)
```

Use two 40mm speakers for stereo sound!

## Conclusion

Your 40mm 8Ω speaker is perfect for this project! The recommended setup is:

**Raspberry Pi → PAM8403 Amplifier → Speaker**

This provides:
- ✅ Good volume
- ✅ Clear audio
- ✅ Easy connection
- ✅ Low cost (~$2 for amplifier)
- ✅ Safe for Raspberry Pi

For any questions or issues, refer to the troubleshooting section or consult the Raspberry Pi audio documentation.
