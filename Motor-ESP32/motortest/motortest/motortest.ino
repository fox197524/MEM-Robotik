#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// --- PINS (Using the test pins that worked for you) ---
const int RL_PWM = 4;  const int RL_IN1 = 5;  const int RL_IN2 = 6;
const int RR_PWM = 7;  const int RR_IN1 = 15; const int RR_IN2 = 16;
const int FL_PWM = 17; const int FL_IN1 = 18; const int FL_IN2 = 8;
const int FR_PWM = 3;  const int FR_IN1 = 9;  const int FR_IN2 = 10;

// --- WIFI ---
const char* ssid = "LAGARIMEDYA";
const char* password = "lagari5253";
WiFiUDP udp;
unsigned int localPort = 4210;
char packetBuffer[255];

// --- VARIABLES & TRACKING ---
float axis0 = 0, axis2 = 0, axis4 = -1, axis5 = -1;
float p0 = 0, p2 = 0, p4 = -1, p5 = -1; // Previous values for change detection

void stopVehicle();

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(RL_PWM, OUTPUT); pinMode(RL_IN1, OUTPUT); pinMode(RL_IN2, OUTPUT);
  pinMode(RR_PWM, OUTPUT); pinMode(RR_IN1, OUTPUT); pinMode(RR_IN2, OUTPUT);
  pinMode(FL_PWM, OUTPUT); pinMode(FL_IN1, OUTPUT); pinMode(FL_IN2, OUTPUT);
  pinMode(FR_PWM, OUTPUT); pinMode(FR_IN1, OUTPUT); pinMode(FR_IN2, OUTPUT);

  stopVehicle();

  WiFi.begin(ssid, password);
  Serial.print("Connecting WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected! IP: " + WiFi.localIP().toString());
  udp.begin(localPort);
}

void loop() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(packetBuffer, 255);
    if (len > 0) packetBuffer[len] = 0;
    String msg = String(packetBuffer);

    // Parsing logic
    int idx[6];
    idx[0] = msg.indexOf(',');
    for (int i = 1; i < 6; i++) idx[i] = msg.indexOf(',', idx[i - 1] + 1);

    if (idx[4] > 0) {
      axis0 = msg.substring(0, idx[0]).toFloat();
      axis2 = msg.substring(idx[1] + 1, idx[2]).toFloat();
      axis4 = msg.substring(idx[3] + 1, idx[4]).toFloat();
      axis5 = msg.substring(idx[4] + 1).toFloat();

      // Check for change > 0.01
      bool changed = false;
      if (abs(axis0 - p0) > 0.01) { Serial.print("Axis0: "); Serial.println(axis0); changed = true; }
      if (abs(axis2 - p2) > 0.01) { Serial.print("Axis2: "); Serial.println(axis2); changed = true; }
      if (abs(axis4 - p4) > 0.01) { Serial.print("Axis4: "); Serial.println(axis4); changed = true; }
      if (abs(axis5 - p5) > 0.01) { Serial.print("Axis5: "); Serial.println(axis5); changed = true; }

      if (changed) {
        // Update previous values
        p0 = axis0; p2 = axis2; p4 = axis4; p5 = axis5;
        
        // You can re-enable your movement function here once you see the prints are correct
        // moveVehicle(); 
      }
    }
  }
}

void stopVehicle() {
  analogWrite(RL_PWM, 0); digitalWrite(RL_IN1, LOW); digitalWrite(RL_IN2, LOW);
  analogWrite(RR_PWM, 0); digitalWrite(RR_IN1, LOW); digitalWrite(RR_IN2, LOW);
  analogWrite(FL_PWM, 0); digitalWrite(FL_IN1, LOW); digitalWrite(FL_IN2, LOW);
  analogWrite(FR_PWM, 0); digitalWrite(FR_IN1, LOW); digitalWrite(FR_IN2, LOW);
}