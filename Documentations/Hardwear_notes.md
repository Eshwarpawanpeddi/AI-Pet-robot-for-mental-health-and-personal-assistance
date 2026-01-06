# Hardware Configuration: Single L298N with Four Motors (Parallel per Side)

## L298N to Raspberry Pi (BCM GPIO)
| L298N | RPi GPIO | Function          |
|-------|----------|-------------------|
| IN1   | 17       | Left direction 1  |
| IN2   | 27       | Left direction 2  |
| ENA   | 22       | Left PWM (speed)  |
| IN3   | 23       | Right direction 1 |
| IN4   | 24       | Right direction 2 |
| ENB   | 25       | Right PWM (speed) |

## L298N Outputs to Motors
- **OUT1, OUT2**: Both left side motors (wired in parallel)
- **OUT3, OUT4**: Both right side motors (wired in parallel)

## Power
- **Do NOT power motors from the Pi!** Connect a separate supply to the L298N `VCC` (e.g., 7–12V for motors).
- Common ground between L298N, motors, and Raspberry Pi.

## Software
- Use `raspberry_pi_motor_control.py` for GPIO logic.

## No ESP12E involved. All movement by Pi’s GPIO.
