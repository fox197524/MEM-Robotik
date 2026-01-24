#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// --- IP CONFIGURATION ---

// --- PIN ASSIGNMENTS (ESP32-S3 SAFE) ---
const int RL_PWM = 1;  const int RL_IN1 = 2;  const int RL_IN2 = 42;
const int RR_PWM = 41; const int RR_IN1 = 40; const int RR_IN2 = 39;
const int FL_PWM = 14; const int FL_IN1 = 13; const int FL_IN2 = 12;
const int FR_PWM = 11; const int FR_IN1 = 10; const int FR_IN2 = 9;


WiFiUDP udp;
unsigned int localPort = 4210;
char packetBuffer[255];
float a0, a2, a4, a5;

void setup() {
  Serial.begin(115200);
  
  // TB6612 Standby Pin
  pinMode(STBY, OUTPUT); 
  digitalWrite(STBY, HIGH); 
  
  int pins[] = {RL_PWM, RL_IN1, RL_IN2, RR_PWM, RR_IN1, RR_IN2, FL_PWM, FL_IN1, FL_IN2, FR_PWM, FR_IN1, FR_IN2};
  for(int p : pins) pinMode(p, OUTPUT);

  // Set Static IP before connecting
  if (!WiFi.config(local_IP, gateway, subnet)) {
    Serial.println("Static IP Failed to Configure");
  }

  WiFi.begin("LAGARIMEDYA", "lagari5253");
  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("\nWiFi Connected!");
  Serial.print("ESP32 IP: "); Serial.println(WiFi.localIP());
  
  udp.begin(localPort);
}

void loop() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(packetBuffer, 255);
    packetBuffer[len] = 0;
    
    // Parses: "a0,a2,a4,a5"
    sscanf(packetBuffer, "%f,%f,%f,%f", &a0, &a2, &a4, &a5);

    // Motor Logic
    if (abs(a2) > 0.15) { // Slide Left
      sol(map(abs(a2) * 100, 0, 100, 0, 255));
    } else if (abs(a0) > 0.15) { // Rotation
      donus360(map(abs(a0) * 100, 0, 100, 0, 255), a0 > 0);
    } else if (a5 > -0.9) { // Forward
      ileri(map((a5 + 1) * 100, 0, 200, 0, 255));
    } else if (a4 > -0.9) { // Backward
      geri(map((a4 + 1) * 100, 0, 200, 0, 255));
    } else {
      dur();
    }
  }
}

// --- Direction Logic Functions ---

void setMotor(int p, int i1, int i2, int s, bool f) {
  digitalWrite(i1, f ? HIGH : LOW);
  digitalWrite(i2, f ? LOW : HIGH);
  analogWrite(p, s);
}

void ileri(int s) {
  setMotor(RL_PWM, RL_IN1, RL_IN2, s, true); setMotor(RR_PWM, RR_IN1, RR_IN2, s, true);
  setMotor(FL_PWM, FL_IN1, FL_IN2, s, true); setMotor(FR_PWM, FR_IN1, FR_IN2, s, true);
}

void geri(int s) {
  setMotor(RL_PWM, RL_IN1, RL_IN2, s, false); setMotor(RR_PWM, RR_IN1, RR_IN2, s, false);
  setMotor(FL_PWM, FL_IN1, FL_IN2, s, false); setMotor(FR_PWM, FR_IN1, FR_IN2, s, false);
}

void sol(int s) {
  // Mecanum Left: FL Bwd, FR Fwd, RL Fwd, RR Bwd
  setMotor(FL_PWM, FL_IN1, FL_IN2, s, false); setMotor(FR_PWM, FR_IN1, FR_IN2, s, true);
  setMotor(RL_PWM, RL_IN1, RL_IN2, s, true);  setMotor(RR_PWM, RR_IN1, RR_IN2, s, false);
}

void donus360(int s, bool sag) {
  setMotor(RL_PWM, RL_IN1, RL_IN2, s, sag);  setMotor(FL_PWM, FL_IN1, FL_IN2, s, sag);
  setMotor(RR_PWM, RR_IN1, RR_IN2, s, !sag); setMotor(FR_PWM, FR_IN1, FR_IN2, s, !sag);
}

void dur() {
  analogWrite(RL_PWM, 0); analogWrite(RR_PWM, 0);
  analogWrite(FL_PWM, 0); analogWrite(FR_PWM, 0);
}