#include <Wire.h>
#include <ArduinoJson.h>

// I2C Configuration
#define SLAVE_ADDRESS 0x08
#define BAUD_RATE 115200

// Motor Control Pins (L298N Motor Driver)
#define MOTOR_A_PIN1 D0  // GPIO16
#define MOTOR_A_PIN2 D1  // GPIO5
#define MOTOR_B_PIN1 D2  // GPIO4
#define MOTOR_B_PIN2 D3  // GPIO0

// PWM Control Pins (Speed)
#define MOTOR_A_PWM D5   // GPIO14
#define MOTOR_B_PWM D6   // GPIO12

// Sensor Pins
#define TOUCH_SENSOR_PIN D7   // GPIO13
#define DISTANCE_TRIG D8      // GPIO15
#define DISTANCE_ECHO D4      // GPIO2

// Servo Control Pins (Head, Eyes)
#define SERVO_HEAD_PIN D7
#define SERVO_EYES_PIN D8

// Command Structure
struct MotorCommand {
  int motor;           // 0 = Motor A, 1 = Motor B, 2 = Both
  int direction;       // 0 = Stop, 1 = Forward, 2 = Backward, 3 = Left, 4 = Right
  int speed;           // 0-255
};

volatile byte requestedBytes = 0;
volatile MotorCommand currentCommand = {0, 0, 0};

void setup() {
  Serial.begin(BAUD_RATE);
  Wire.begin(SLAVE_ADDRESS);
  
  // Set motor pins as outputs
  pinMode(MOTOR_A_PIN1, OUTPUT);
  pinMode(MOTOR_A_PIN2, OUTPUT);
  pinMode(MOTOR_B_PIN1, OUTPUT);
  pinMode(MOTOR_B_PIN2, OUTPUT);
  
  // PWM configuration
  pinMode(MOTOR_A_PWM, OUTPUT);
  pinMode(MOTOR_B_PWM, OUTPUT);
  
  // Sensor pins
  pinMode(TOUCH_SENSOR_PIN, INPUT);
  pinMode(DISTANCE_TRIG, OUTPUT);
  pinMode(DISTANCE_ECHO, INPUT);
  
  // I2C callbacks
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);
  
  Serial.println("ESP12E Motor Controller Initialized");
  Serial.println("Waiting for I2C commands...");
  
  // Stop all motors by default
  stopAllMotors();
}

void loop() {
  // Check for touch sensor
  if (digitalRead(TOUCH_SENSOR_PIN) == HIGH) {
    Serial.println("Touch detected!");
    delay(50); // Debounce
  }
  
  delay(10);
}

// I2C Data Reception Handler
void receiveData(int byteCount) {
  if (byteCount >= 3) {
    int motor = Wire.read();      // Motor selection
    int direction = Wire.read();  // Direction
    int speed = Wire.read();      // Speed 0-255
    
    currentCommand.motor = motor;
    currentCommand.direction = direction;
    currentCommand.speed = speed;
    
    executeMotorCommand(currentCommand);
    
    Serial.print("Motor: ");
    Serial.print(motor);
    Serial.print(" | Direction: ");
    Serial.print(direction);
    Serial.print(" | Speed: ");
    Serial.println(speed);
  }
}

// I2C Data Transmission Handler
void sendData() {
  Wire.write(currentCommand.direction);  // Send current state
}

// Motor Control Functions
void executeMotorCommand(MotorCommand cmd) {
  if (cmd.direction == 0) {
    stopAllMotors();
    return;
  }
  
  switch (cmd.direction) {
    case 1: // Forward
      moveForward(cmd.speed);
      break;
    case 2: // Backward
      moveBackward(cmd.speed);
      break;
    case 3: // Left
      turnLeft(cmd.speed);
      break;
    case 4: // Right
      turnRight(cmd.speed);
      break;
  }
}

void moveForward(int speed) {
  // Motor A: Forward
  digitalWrite(MOTOR_A_PIN1, HIGH);
  digitalWrite(MOTOR_A_PIN2, LOW);
  
  // Motor B: Forward
  digitalWrite(MOTOR_B_PIN1, HIGH);
  digitalWrite(MOTOR_B_PIN2, LOW);
  
  // Set speed
  analogWrite(MOTOR_A_PWM, speed);
  analogWrite(MOTOR_B_PWM, speed);
}

void moveBackward(int speed) {
  // Motor A: Backward
  digitalWrite(MOTOR_A_PIN1, LOW);
  digitalWrite(MOTOR_A_PIN2, HIGH);
  
  // Motor B: Backward
  digitalWrite(MOTOR_B_PIN1, LOW);
  digitalWrite(MOTOR_B_PIN2, HIGH);
  
  // Set speed
  analogWrite(MOTOR_A_PWM, speed);
  analogWrite(MOTOR_B_PWM, speed);
}

void turnLeft(int speed) {
  // Motor A: Backward (right side)
  digitalWrite(MOTOR_A_PIN1, LOW);
  digitalWrite(MOTOR_A_PIN2, HIGH);
  
  // Motor B: Forward (left side)
  digitalWrite(MOTOR_B_PIN1, HIGH);
  digitalWrite(MOTOR_B_PIN2, LOW);
  
  analogWrite(MOTOR_A_PWM, speed);
  analogWrite(MOTOR_B_PWM, speed);
}

void turnRight(int speed) {
  // Motor A: Forward (right side)
  digitalWrite(MOTOR_A_PIN1, HIGH);
  digitalWrite(MOTOR_A_PIN2, LOW);
  
  // Motor B: Backward (left side)
  digitalWrite(MOTOR_B_PIN1, LOW);
  digitalWrite(MOTOR_B_PIN2, HIGH);
  
  analogWrite(MOTOR_A_PWM, speed);
  analogWrite(MOTOR_B_PWM, speed);
}

void stopAllMotors() {
  digitalWrite(MOTOR_A_PIN1, LOW);
  digitalWrite(MOTOR_A_PIN2, LOW);
  digitalWrite(MOTOR_B_PIN1, LOW);
  digitalWrite(MOTOR_B_PIN2, LOW);
  
  analogWrite(MOTOR_A_PWM, 0);
  analogWrite(MOTOR_B_PWM, 0);
}

// Sensor Reading Functions
float readDistance() {
  digitalWrite(DISTANCE_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(DISTANCE_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(DISTANCE_TRIG, LOW);
  
  long duration = pulseIn(DISTANCE_ECHO, HIGH);
  float distance = (duration * 0.034) / 2; // Convert to cm
  
  return distance;
}

void servoPulse(int pin, int angle) {
  // Simple servo control (1ms = 0°, 1.5ms = 90°, 2ms = 180°)
  int pulseWidth = map(angle, 0, 180, 1000, 2000);
  
  digitalWrite(pin, HIGH);
  delayMicroseconds(pulseWidth);
  digitalWrite(pin, LOW);
  delayMicroseconds(20000 - pulseWidth);
}