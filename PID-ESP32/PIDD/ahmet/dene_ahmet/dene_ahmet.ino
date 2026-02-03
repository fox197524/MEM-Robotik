#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// Motor Pin Definitions
const int RL_PWM = 7, RL_IN1 = 8, RL_IN2 = 9;
const int RR_PWM = 12, RR_IN1 = 11, RR_IN2 = 13;
const int FL_PWM = 15, FL_IN1 = 16, FL_IN2 = 17;
const int FR_PWM = 4, FR_IN1 = 5, FR_IN2 = 6;

// Networking
WiFiUDP udp;
unsigned int localPort = 4210;
char packetBuffer[255];

// Fail-safe variables
unsigned long lastPacketTime = 0;
const unsigned long TIMEOUT_MS = 500; // Stop if no data for 0.5 seconds

// Input variables
float axis_forward = 0, axis_backward = 0, axis_strafe = 0, axis_turn = 0;
const float DEADZONE = 0.15; 

void setup() {
  pinMode(RR_IN1, OUTPUT); pinMode(RR_IN2, OUTPUT); pinMode(RR_PWM, OUTPUT);
  pinMode(RL_IN1, OUTPUT); pinMode(RL_IN2, OUTPUT); pinMode(RL_PWM, OUTPUT);
  pinMode(FR_IN1, OUTPUT); pinMode(FR_IN2, OUTPUT); pinMode(FR_PWM, OUTPUT);
  pinMode(FL_IN1, OUTPUT); pinMode(FL_IN2, OUTPUT); pinMode(FL_PWM, OUTPUT);

  Serial.begin(115200);
  WiFi.begin("Fox-2", "Kyra2bin9");
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  udp.begin(localPort);
  Serial.println("\nUDP Ready");
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) ESP.restart();

  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(packetBuffer, 255);
    if (len > 0) {
      packetBuffer[len] = 0;
      parseInput(String(packetBuffer));
      processMecanumMovement();
      lastPacketTime = millis(); // Update the timer every time a packet arrives
    }
  }

  // FAIL-SAFE: If we haven't heard from the Python script, stop the motors
  if (millis() - lastPacketTime > TIMEOUT_MS) {
    stopMotors();
  }
}

void parseInput(String msg) {
  int space1 = msg.indexOf(' ');
  int space2 = msg.indexOf(' ', space1 + 1);
  if (space1 == -1 || space2 == -1) return;
  
  String type = msg.substring(0, space1);
  int id = msg.substring(space1 + 1, space2).toInt();
  float val = msg.substring(space2 + 1).toFloat();
  
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
  float forward = axis_forward - axis_backward;
  float strafe = axis_strafe;
  float turn = axis_turn;
  
  float fl_power = forward + strafe + turn;
  float fr_power = forward - strafe - turn;
  float rl_power = forward - strafe + turn;
  float rr_power = forward + strafe - turn;
  
  setWheel(FL_IN1, FL_IN2, FL_PWM, (abs(fl_power) > DEADZONE) ? (fl_power > 0 ? 1 : -1) : 0);
  setWheel(FR_IN1, FR_IN2, FR_PWM, (abs(fr_power) > DEADZONE) ? (fr_power > 0 ? 1 : -1) : 0);
  setWheel(RL_IN1, RL_IN2, RL_PWM, (abs(rl_power) > DEADZONE) ? (rl_power > 0 ? 1 : -1) : 0);
  setWheel(RR_IN1, RR_IN2, RR_PWM, (abs(rr_power) > DEADZONE) ? (rr_power > 0 ? 1 : -1) : 0);
}

void setWheel(int in1, int in2, int pwm_pin, int direction) {
  if (direction > 0) {
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW); analogWrite(pwm_pin, 255);
  } else if (direction < 0) {
    digitalWrite(in1, LOW); digitalWrite(in2, HIGH); analogWrite(pwm_pin, 255);
  } else {
    digitalWrite(in1, LOW); digitalWrite(in2, LOW); analogWrite(pwm_pin, 0);
  }
}

void stopMotors() {
  digitalWrite(RR_IN1, LOW); digitalWrite(RR_IN2, LOW); analogWrite(RR_PWM, 0);
  digitalWrite(FR_IN1, LOW); digitalWrite(FR_IN2, LOW); analogWrite(FR_PWM, 0);
  digitalWrite(RL_IN1, LOW); digitalWrite(RL_IN2, LOW); analogWrite(RL_PWM, 0);
  digitalWrite(FL_IN1, LOW); digitalWrite(FL_IN2, LOW); analogWrite(FL_PWM, 0);
}