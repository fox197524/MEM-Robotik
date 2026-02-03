#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// Motor Pin Definitions
const int RL_PWM = 7;
const int RL_IN1 = 8;
const int RL_IN2 = 9;

const int RR_PWM = 12;
const int RR_IN1 = 11;
const int RR_IN2 = 13;

const int FL_PWM = 15;
const int FL_IN1 = 16;
const int FL_IN2 = 17;

const int FR_PWM = 4;
const int FR_IN1 = 5;
const int FR_IN2 = 6;

// Networking
WiFiUDP udp;
unsigned int localPort = 4210;
char packetBuffer[255];

// Input variables
float axis_forward = 0;   // Axis 5 (R2 Trigger)
float axis_backward = 0;  // Axis 4 (L2 Trigger)
float axis_strafe = 0;    // Axis 2 (Right Stick Horizontal)
float axis_turn = 0;      // Axis 0 (Left Stick Horizontal)

// Threshold to prevent accidental movement from stick drift
const float DEADZONE = 0.15; 

void setup() {
  // Pin Modes
  pinMode(RR_IN1, OUTPUT);
  pinMode(RR_IN2, OUTPUT);
  pinMode(RR_PWM, OUTPUT);

  pinMode(RL_IN1, OUTPUT);
  pinMode(RL_IN2, OUTPUT);
  pinMode(RL_PWM, OUTPUT);

  pinMode(FR_IN1, OUTPUT);
  pinMode(FR_IN2, OUTPUT);
  pinMode(FR_PWM, OUTPUT);

  pinMode(FL_IN1, OUTPUT);
  pinMode(FL_IN2, OUTPUT);
  pinMode(FL_PWM, OUTPUT);

  // Initialize serial
  Serial.begin(115200);
  delay(500);
  Serial.println("Mecanum Drive Controller Initialized - Full Power Mode");

  // WiFi Setup
  WiFi.begin("Fox-2", "Kyra2bin9");
  Serial.print("Connecting to WiFi");
  int timeout = 0;
  while (WiFi.status() != WL_CONNECTED && timeout < 20) {
    delay(500);
    Serial.print(".");
    timeout++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi Connected!");
    Serial.print("IP: "); Serial.println(WiFi.localIP());
    udp.begin(localPort);
    Serial.println("UDP Started");
  } else {
    Serial.println("\nWiFi Connection Failed!");
  }
}

void loop() {
  // Check WiFi connection
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi Disconnected, Restarting...");
    delay(1000);
    ESP.restart();
  }

  // Check for UDP packets
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(packetBuffer, 255);
    if (len > 0) {
      packetBuffer[len] = 0;
      parseInput(String(packetBuffer));
      processMecanumMovement();
    }
  } else {
    // No packet received - stop all motors
    stopMotors();
  }
}

void parseInput(String msg) {
  // Parse incoming UDP messages
  int space1 = msg.indexOf(' ');
  int space2 = msg.indexOf(' ', space1 + 1);
  
  if (space1 == -1 || space2 == -1) return;
  
  String type = msg.substring(0, space1);
  String idStr = msg.substring(space1 + 1, space2);
  String valStr = msg.substring(space2 + 1);
  
  int id = idStr.toInt();
  float val = valStr.toFloat();
  
  // Store axis values
  if (type == "AXIS") {
    switch(id) {
      case 4: axis_backward = val; break;
      case 5: axis_forward = val; break;
      case 2: axis_strafe = val; break;
      case 0: axis_turn = val; break;
    }
  }
}

void processMecanumMovement() {
  // Calculate movement vectors
  float forward = axis_forward - axis_backward;
  float strafe = axis_strafe;
  float turn = axis_turn;
  
  // Mecanum wheel formulas:
  float fl_power = forward + strafe + turn;
  float fr_power = forward - strafe - turn;
  float rl_power = forward - strafe + turn;
  float rr_power = forward + strafe - turn;
  
  // Apply "Full Power" logic: 
  // If the calculated power is outside the deadzone, we pass 1 (Forward) or -1 (Backward)
  // The setWheel function will then convert this to 255 PWM.
  setWheel(FL_IN1, FL_IN2, FL_PWM, (abs(fl_power) > DEADZONE) ? (fl_power > 0 ? 1 : -1) : 0);
  setWheel(FR_IN1, FR_IN2, FR_PWM, (abs(fr_power) > DEADZONE) ? (fr_power > 0 ? 1 : -1) : 0);
  setWheel(RL_IN1, RL_IN2, RL_PWM, (abs(rl_power) > DEADZONE) ? (rl_power > 0 ? 1 : -1) : 0);
  setWheel(RR_IN1, RR_IN2, RR_PWM, (abs(rr_power) > DEADZONE) ? (rr_power > 0 ? 1 : -1) : 0);
}

void setWheel(int in1, int in2, int pwm_pin, int direction) {
  // Set direction and FORCE full speed (255) if movement is requested
  if (direction > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(pwm_pin, 255); // Always write 255
  } else if (direction < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(pwm_pin, 255); // Always write 255
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(pwm_pin, 0);   // Stop
  }
}

void stopMotors() {
  // Stop all drive motors
  digitalWrite(RR_IN1, LOW);
  digitalWrite(RR_IN2, LOW);
  digitalWrite(FR_IN1, LOW);
  digitalWrite(FR_IN2, LOW);
  digitalWrite(RL_IN1, LOW);
  digitalWrite(RL_IN2, LOW);
  digitalWrite(FL_IN1, LOW);
  digitalWrite(FL_IN2, LOW);
  
  analogWrite(RR_PWM, 0);
  analogWrite(FR_PWM, 0);
  analogWrite(RL_PWM, 0);
  analogWrite(FL_PWM, 0);
}