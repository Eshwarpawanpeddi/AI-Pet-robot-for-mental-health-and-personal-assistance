// ESP12E Configuration Header
// Hardware pin definitions and constants

#ifndef CONFIG_H
#define CONFIG_H

// I2C Configuration
#define SLAVE_ADDRESS 0x08
#define BAUD_RATE 115200

// Motor Control Pins (L298N Motor Driver)
#define MOTOR_A_PIN1 D0  // GPIO16
#define MOTOR_A_PIN2 D1  // GPIO5
#define MOTOR_B_PIN1 D2  // GPIO4
#define MOTOR_B_PIN2 D3  // GPIO0

// PWM Control Pins (Speed Control)
#define MOTOR_A_PWM D5   // GPIO14
#define MOTOR_B_PWM D6   // GPIO12

// Sensor Pins
#define TOUCH_SENSOR_PIN D7   // GPIO13
#define DISTANCE_TRIG D8      // GPIO15
#define DISTANCE_ECHO D4      // GPIO2

// Servo Control Pins (Optional - for head/eye movements)
#define SERVO_HEAD_PIN D7
#define SERVO_EYES_PIN D8

// Motor Configuration
#define PWM_FREQUENCY 1000  // 1kHz PWM frequency
#define MAX_SPEED 255
#define MIN_SPEED 0
#define DEFAULT_SPEED 200

// Sensor Configuration
#define TOUCH_DEBOUNCE_MS 50
#define DISTANCE_TIMEOUT_US 30000  // 30ms timeout for ultrasonic sensor

// Command Definitions
#define CMD_STOP 0
#define CMD_FORWARD 1
#define CMD_BACKWARD 2
#define CMD_LEFT 3
#define CMD_RIGHT 4

// Motor Selection
#define MOTOR_A 0
#define MOTOR_B 1
#define MOTOR_BOTH 2

#endif // CONFIG_H
