import RPi.GPIO as GPIO
import time

# Pin definitions (BCM GPIO)
IN1 = 17   # L298N IN1 (Left dir 1)
IN2 = 27   # L298N IN2 (Left dir 2)
ENA = 22   # L298N ENA  (Left PWM)
IN3 = 23   # L298N IN3 (Right dir 1)
IN4 = 24   # L298N IN4 (Right dir 2)
ENB = 25   # L298N ENB  (Right PWM)

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup([IN1, IN2, ENA, IN3, IN4, ENB], GPIO.OUT)

# PWM initialization
pwm_left = GPIO.PWM(ENA, 100)
pwm_right = GPIO.PWM(ENB, 100)
pwm_left.start(0)
pwm_right.start(0)

def set_motors(left_speed=0, left_forward=True, right_speed=0, right_forward=True):
    # Set directions
    GPIO.output(IN1, GPIO.HIGH if left_forward else GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW if left_forward else GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH if right_forward else GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW if right_forward else GPIO.HIGH)
    pwm_left.ChangeDutyCycle(abs(left_speed))
    pwm_right.ChangeDutyCycle(abs(right_speed))

def stop_motors():
    pwm_left.ChangeDutyCycle(0)
    pwm_right.ChangeDutyCycle(0)
    for pin in [IN1, IN2, IN3, IN4]:
        GPIO.output(pin, GPIO.LOW)

def move_forward(speed=75):
    set_motors(speed, True, speed, True)

def move_backward(speed=75):
    set_motors(speed, False, speed, False)

def turn_left(speed=75):
    set_motors(0, True, speed, True)

def turn_right(speed=75):
    set_motors(speed, True, 0, True)

# Example testing logic
if __name__ == "__main__":
    try:
        move_forward(80)
        time.sleep(2)
        turn_left(80)
        time.sleep(1)
        move_backward(80)
        time.sleep(2)
        turn_right(80)
        time.sleep(1)
        stop_motors()
    finally:
        pwm_left.stop()
        pwm_right.stop()
        GPIO.cleanup()
