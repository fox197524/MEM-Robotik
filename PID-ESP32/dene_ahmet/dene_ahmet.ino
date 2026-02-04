#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

WiFiUDP udp;
unsigned int localPort = 4210;
char packetBuffer[255];
unsigned long lastAxisPacket = 0;
const unsigned long TIMEOUT = 500;

#define WIFI_SSID "Fox-17"
#define WIFI_PSWD "Kyra2bin9"

// Global Axis States
float axis0 = 0.0;
float axis2 = -1.0; 
float axis5 = -1.0;
float axis3 = 0.0;
float axis4 = 0.0;

int speed = 255;

const int RL_PWM = 7, RL_IN1 = 8, RL_IN2 = 9;
const int RR_PWM = 12, RR_IN1 = 13, RR_IN2 = 11;
const int FL_PWM = 15, FL_IN1 = 16, FL_IN2 = 17;
const int FR_PWM = 4, FR_IN1 = 5, FR_IN2 = 6;

void setMotor(int in1, int in2, int pwm, int dir, int speed) {
  if (dir == -1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(pwm, abs(speed));

  } else if (dir == 0){
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(pwm, abs(speed));

  } else if (dir == 1) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(pwm, abs(speed));
  } else if (dir == 2){
    digitalWrite(in1, HIGH); // FOR SUDDENLY BRAKING
    digitalWrite(in2, HIGH); 
    digitalWrite(pwm, LOW); // pwm low for sudden brake
  }
}


void suddenStop() {

  setMotor(RL_IN1, RL_IN2, RL_PWM, 2, 0);
  setMotor(RR_IN1, RR_IN2, RR_PWM, 2, 0);
  setMotor(FL_IN1, FL_IN2, FL_PWM, 2, 0);
  setMotor(FR_IN1, FR_IN2, FR_PWM, 2, 0);
}

void stopAll() {
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
  
  stopAll();
  
  WiFi.begin(WIFI_SSID, WIFI_PSWD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi OK! IP: " + WiFi.localIP().toString());
  udp.begin(localPort);
}

void loop() {
  unsigned long now = millis();
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(packetBuffer, 255);
    if (len > 0) packetBuffer[len] = 0;
    
    String message = String(packetBuffer);
    
    if (message.startsWith("AXIS 5 ")) {
      axis5 = message.substring(7).toFloat();
      lastAxisPacket = now;
    } else if (message.startsWith("AXIS 2 ")) {
      axis2 = message.substring(7).toFloat();
      lastAxisPacket = now;
    } else if (message.startsWith("AXIS 3 ")){
      axis3 = message.substring(7).toFloat();
      lastAxisPacket = now;
    } else if (message.startsWith("AXIS 0 ")){
      axis0 = message.substring(7).toFloat();
      lastAxisPacket = now;
    }
  }
  
  // --- FINAL LOGIC ENGINE ---
  
  // 1. Safety Timeout (If no signal for 500ms, stop)
  if (now - lastAxisPacket > TIMEOUT) {
    stopAll();
    // Optional: Serial.println("TIMEOUT - CONNECTION LOST");
  }
  
  // 2. Drive Forward (Trigger 5)
  else if (axis5 > -0.900) {
      Serial.println("FORWARD");
      setMotor(RL_IN1, RL_IN2, RL_PWM, 1, speed);
      setMotor(RR_IN1, RR_IN2, RR_PWM, 1, speed);
      setMotor(FL_IN1, FL_IN2, FL_PWM, 1, speed);
      setMotor(FR_IN1, FR_IN2, FR_PWM, 1, speed);
  } 
  
  // 3. Drive Reverse (Trigger 2)
  else if (axis2 > -0.900) {
      Serial.println("REVERSE");
      setMotor(RL_IN1, RL_IN2, RL_PWM, -1, speed);
      setMotor(RR_IN1, RR_IN2, RR_PWM, -1, speed);
      setMotor(FL_IN1, FL_IN2, FL_PWM, -1, speed);
      setMotor(FR_IN1, FR_IN2, FR_PWM, -1, speed);
  } 
  
  // 4. Spin Left (Stick pushed far left)
  else if (axis3 < -0.150) { 
    Serial.println("360 LEFT");
    setMotor(RL_IN1, RL_IN2, RL_PWM, -1, speed); // Left side backwards
    setMotor(FL_IN1, FL_IN2, FL_PWM, -1, speed);
    setMotor(RR_IN1, RR_IN2, RR_PWM, 1, speed);  // Right side forwards
    setMotor(FR_IN1, FR_IN2, FR_PWM, 1, speed);
  }
  
  // 5. Spin Right (Stick pushed far right)
  else if (axis3 > 0.150) {
    Serial.println("360 RIGHT");
    setMotor(RL_IN1, RL_IN2, RL_PWM, 1, speed);  // Left side forwards
    setMotor(FL_IN1, FL_IN2, FL_PWM, 1, speed);
    setMotor(RR_IN1, RR_IN2, RR_PWM, -1, speed); // Right side backwards
    setMotor(FR_IN1, FR_IN2, FR_PWM, -1, speed);
  }

  else if (axis0 > -0.010){
    Serial.println("MOVE RIGHT");
    setMotor(RL_IN1, RL_IN2, RL_PWM, -1, speed);  
    setMotor(FL_IN1, FL_IN2, FL_PWM, 1, speed);
    setMotor(RR_IN1, RR_IN2, RR_PWM, 1, speed); 
    setMotor(FR_IN1, FR_IN2, FR_PWM, -1, speed);
  }

  else if (axis0 < -0.030){
    Serial.println("MOVE LEFT");
    setMotor(RL_IN1, RL_IN2, RL_PWM, 1, speed);  
    setMotor(FL_IN1, FL_IN2, FL_PWM, -1, speed);
    setMotor(RR_IN1, RR_IN2, RR_PWM, -1, speed); 
    setMotor(FR_IN1, FR_IN2, FR_PWM, 1, speed);
  } 
  
  else {
    Serial.println("STOPPED");
    stopAll();
  }
}
