#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

WiFiUDP udp;
unsigned int localPort = 4210;
char packetBuffer[255];

// Track previous values for change detection
float prev_axis5 = 999, prev_axis2 = 999, prev_axis0 = 999;
int prev_btn0 = -1, prev_btn11 = -1, prev_btn12 = -1;

#define WIFI_SSID "Fox-17"
#define WIFI_PSWD "Kyra2bin9"

const int RL_PWM = 7, RL_IN1 = 8, RL_IN2 = 9;
const int RR_PWM = 12, RR_IN1 = 11, RR_IN2 = 13;
const int FL_PWM = 15, FL_IN1 = 16, FL_IN2 = 17;
const int FR_PWM = 4, FR_IN1 = 5, FR_IN2 = 6;

const int EL_PWM = 18, EL_IN1 = 38, EL_IN2 = 39;
const int ER_PWM = 3, ER_IN1 = 10, ER_IN2 = 42;
const int E_LID = 41;

void setMotor(int in1, int in2, int pwm, int dir, int speed) {
  if (dir > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (dir < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
  analogWrite(pwm, abs(speed));
}

bool hasChanged(float prev, float current, float threshold = 0.05) {
  return abs(prev - current) > threshold;
}

bool hasChanged(int prev, int current) {
  return prev != current;
}

void stopWheels() {
  setMotor(RL_IN1, RL_IN2, RL_PWM, 0, 0);
  setMotor(RR_IN1, RR_IN2, RR_PWM, 0, 0);
  setMotor(FL_IN1, FL_IN2, FL_PWM, 0, 0);
  setMotor(FR_IN1, FR_IN2, FR_PWM, 0, 0);
}

void setup() {
  Serial.begin(115200);
  
  pinMode(RL_IN1, OUTPUT); pinMode(RL_IN2, OUTPUT); pinMode(RL_PWM, OUTPUT);
  pinMode(RR_IN1, OUTPUT); pinMode(RR_IN2, OUTPUT); pinMode(RR_PWM, OUTPUT);
  pinMode(FL_IN1, OUTPUT); pinMode(FL_IN2, OUTPUT); pinMode(FL_PWM, OUTPUT);
  pinMode(FR_IN1, OUTPUT); pinMode(FR_IN2, OUTPUT); pinMode(FR_PWM, OUTPUT);
  
  pinMode(EL_IN1, OUTPUT); pinMode(EL_IN2, OUTPUT); pinMode(EL_PWM, OUTPUT);
  pinMode(ER_IN1, OUTPUT); pinMode(ER_IN2, OUTPUT); pinMode(ER_PWM, OUTPUT);
  pinMode(E_LID, OUTPUT);
  digitalWrite(E_LID, LOW);
  
  stopWheels();
  
  WiFi.begin(WIFI_SSID, WIFI_PSWD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi OK! IP: " + WiFi.localIP().toString());
  udp.begin(localPort);
}

void loop() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(packetBuffer, 255);
    if (len > 0) packetBuffer[len] = 0;
    
    String message = String(packetBuffer);
    
    if (message.startsWith("AXIS 5 ")) {  // FORWARD
      float axis5 = message.substring(7).toFloat();
      if (hasChanged(prev_axis5, axis5)) {
        prev_axis5 = axis5;
        Serial.printf("Axis5(FORWARD): %.3f\n", axis5);
        int fwd_speed = constrain(-axis5 * 255, -255, 255);
        if (abs(fwd_speed) > 20) {
          setMotor(RL_IN1, RL_IN2, RL_PWM, 1, abs(fwd_speed));
          setMotor(RR_IN1, RR_IN2, RR_PWM, 1, abs(fwd_speed));
          setMotor(FL_IN1, FL_IN2, FL_PWM, 1, abs(fwd_speed));
          setMotor(FR_IN1, FR_IN2, FR_PWM, 1, abs(fwd_speed));
        } else {
          stopWheels();
        }
      }
      
    } else if (message.startsWith("AXIS 2 ")) {  // BACKWARD
      float axis2 = message.substring(7).toFloat();
      if (hasChanged(prev_axis2, axis2)) {
        prev_axis2 = axis2;
        Serial.printf("Axis2(BACKWARD): %.3f\n", axis2);
        int back_speed = constrain(-axis2 * 255, -255, 255);
        if (abs(back_speed) > 20) {
          setMotor(RL_IN1, RL_IN2, RL_PWM, -1, abs(back_speed));
          setMotor(RR_IN1, RR_IN2, RR_PWM, -1, abs(back_speed));
          setMotor(FL_IN1, FL_IN2, FL_PWM, -1, abs(back_speed));
          setMotor(FR_IN1, FR_IN2, FR_PWM, -1, abs(back_speed));
        } else {
          stopWheels();
        }
      }
      
    } else if (message.startsWith("AXIS 0 ")) {  // RIGHT/LEFT STRAFE
      float axis0 = message.substring(7).toFloat();
      if (hasChanged(prev_axis0, axis0)) {
        prev_axis0 = axis0;
        Serial.printf("Axis0(STRAFE): %.3f\n", axis0);
        int strafe_speed = constrain(abs(axis0) * 255 * 0.8, 0, 255);
        if (strafe_speed > 20) {
          if (axis0 > 0) {  // RIGHT
            setMotor(FR_IN1, FR_IN2, FR_PWM, 1, strafe_speed);
            setMotor(RR_IN1, RR_IN2, RR_PWM, 1, strafe_speed);
            setMotor(FL_IN1, FL_IN2, FL_PWM, -1, strafe_speed);
            setMotor(RL_IN1, RL_IN2, RL_PWM, -1, strafe_speed);
          } else {  // LEFT
            setMotor(FR_IN1, FR_IN2, FR_PWM, -1, strafe_speed);
            setMotor(RR_IN1, RR_IN2, RR_PWM, -1, strafe_speed);
            setMotor(FL_IN1, FL_IN2, FL_PWM, 1, strafe_speed);
            setMotor(RL_IN1, RL_IN2, RL_PWM, 1, strafe_speed);
          }
        } else {
          stopWheels();
        }
      }
       
  }
  }
}
