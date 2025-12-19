#include <ESP8266WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include "config.h"

WebSocketsClient webSocket;
unsigned long lastHeartbeat = 0;
unsigned long lastCommandTime = 0;
bool isConnected = false;
bool fallbackMode = false;

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
  
  Serial.println("ESP12E Motor Controller Initialized");
  
  // Stop all motors by default
  stopAllMotors();
  
  // Connect to Wi-Fi
  connectWiFi();
  
  // Setup WebSocket
  webSocket.begin(SERVER_HOST, SERVER_PORT, WEBSOCKET_PATH);
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(RECONNECT_DELAY_MS);
  
  Serial.println("WebSocket client initialized");
}

void loop() {
  webSocket.loop();
  
  // Check for touch sensor
  if (digitalRead(TOUCH_SENSOR_PIN) == HIGH) {
    Serial.println("Touch detected!");
    sendSensorData("touch", true);
    delay(TOUCH_DEBOUNCE_MS);
  }
  
  // Send heartbeat
  if (millis() - lastHeartbeat > HEARTBEAT_INTERVAL_MS) {
    sendHeartbeat();
    lastHeartbeat = millis();
  }
  
  // Check for command timeout and enter fallback mode
  if (isConnected && millis() - lastCommandTime > COMMAND_TIMEOUT_MS * 2) {
    if (!fallbackMode) {
      Serial.println("Entering fallback mode - no commands received");
      fallbackMode = true;
      stopAllMotors();
    }
  }
  
  delay(10);
}

// Wi-Fi Connection
void connectWiFi() {
  Serial.println("Connecting to WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi connection failed");
  }
}

// WebSocket Event Handler
void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.println("WebSocket Disconnected");
      isConnected = false;
      fallbackMode = true;
      stopAllMotors();
      break;
      
    case WStype_CONNECTED:
      Serial.println("WebSocket Connected");
      isConnected = true;
      fallbackMode = false;
      webSocket.sendTXT("{\"type\":\"esp_connected\",\"device\":\"ESP12E\"}");
      break;
      
    case WStype_TEXT:
      handleCommand(payload, length);
      break;
      
    case WStype_ERROR:
      Serial.println("WebSocket Error");
      break;
  }
}

// Handle incoming commands
void handleCommand(uint8_t * payload, size_t length) {
  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, payload, length);
  
  if (error) {
    Serial.println("JSON parsing failed");
    return;
  }
  
  const char* type = doc["type"];
  
  if (strcmp(type, "move") == 0) {
    const char* direction = doc["direction"];
    int speed = doc.containsKey("speed") ? doc["speed"].as<int>() : DEFAULT_SPEED;
    
    lastCommandTime = millis();
    fallbackMode = false;
    
    Serial.print("Move command: ");
    Serial.print(direction);
    Serial.print(" at speed ");
    Serial.println(speed);
    
    if (strcmp(direction, "forward") == 0) {
      moveForward(speed);
    } else if (strcmp(direction, "backward") == 0) {
      moveBackward(speed);
    } else if (strcmp(direction, "left") == 0) {
      turnLeft(speed);
    } else if (strcmp(direction, "right") == 0) {
      turnRight(speed);
    } else if (strcmp(direction, "stop") == 0) {
      stopAllMotors();
    }
    
    // Send acknowledgment
    sendStatus();
  }
}

// Send heartbeat
void sendHeartbeat() {
  if (!isConnected) return;
  
  StaticJsonDocument<200> doc;
  doc["type"] = "heartbeat";
  doc["device"] = "ESP12E";
  doc["status"] = fallbackMode ? "fallback" : "normal";
  doc["uptime"] = millis();
  
  String output;
  serializeJson(doc, output);
  webSocket.sendTXT(output);
}

// Send sensor data
void sendSensorData(const char* sensor, bool value) {
  if (!isConnected) return;
  
  StaticJsonDocument<200> doc;
  doc["type"] = "sensor";
  doc["sensor"] = sensor;
  doc["value"] = value;
  
  String output;
  serializeJson(doc, output);
  webSocket.sendTXT(output);
}

// Send status update
void sendStatus() {
  if (!isConnected) return;
  
  StaticJsonDocument<200> doc;
  doc["type"] = "status";
  doc["device"] = "ESP12E";
  doc["connected"] = true;
  
  String output;
  serializeJson(doc, output);
  webSocket.sendTXT(output);
}

// I2C Data Reception Handler (Deprecated - kept for reference)
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