#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// --- MOTOR PIN DEFINITIONS ---
const int RL_PWM = 7;   const int RL_IN1 = 8;   const int RL_IN2 = 9;   
const int RR_PWM = 11;  const int RR_IN1 = 13;  const int RR_IN2 = 12;
const int FL_PWM = 15;  const int FL_IN1 = 16;  const int FL_IN2 = 17;
const int FR_PWM = 4;   const int FR_IN1 = 5;   const int FR_IN2 = 6;

WiFiUDP udp;
unsigned int localPort = 4210;
char packetBuffer[255];
float a0=0, a2=0, a4=0, a5=0;
String lastMsg = "";
float last_a0=999, last_a2=999, last_a4=999, last_a5=999;

// Fonksiyon Prototipleri (Derleyici hatasını önlemek için)
void sol(int PWM); void sag(int PWM); void ileri(int PWM); 
void geri(int PWM); void donus360(int PWM, bool sagaDonus); 
void ileri_mix(int PWM); void aniDur();

void setup() {
  Serial.begin(115200);
  delay(1000); 
  Serial.println("\nSistem Baslatiliyor...");

  int pins[] = {RL_PWM, RL_IN1, RL_IN2, RR_PWM, RR_IN1, RR_IN2, FL_PWM, FL_IN1, FL_IN2, FR_PWM, FR_IN1, FR_IN2};
  for(int p : pins) pinMode(p, OUTPUT);

  // WiFi Bilgilerini Buraya Gir
  WiFi.begin("Ben", "sitrayburgbiriki3"); 
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
    String msg = String(packetBuffer);
    
    if (msg == lastMsg) return;
    lastMsg = msg;
    
    Serial.print("RAW: "); Serial.println(msg);
    
    bool axisChanged = false;

    // --- AXIS PARSING ---
    if (msg.startsWith("AXIS 0 ")) {
      float val = msg.substring(7).toFloat();
      if (abs(val - a0) > 0.01) { a0 = val; axisChanged = true; }
    }
    else if (msg.startsWith("AXIS 2 ")) {
      float val = msg.substring(7).toFloat();
      if (abs(val - a2) > 0.01) { a2 = val; axisChanged = true; }
    }  
    else if (msg.startsWith("AXIS 4 ")) {
      float val = msg.substring(7).toFloat();
      if (abs(val - a4) > 0.01) { a4 = val; axisChanged = true; }
    }
    else if (msg.startsWith("AXIS 5 ")) {
      float val = msg.substring(7).toFloat();
      if (abs(val - a5) > 0.01) { a5 = val; axisChanged = true; }
    }
    
    if (axisChanged) {
      Serial.printf("GUNCEL -> A0:%.2f A2:%.2f A4:%.2f A5:%.2f\n", a0, a2, a4, a5);
    }
    
    // --- MOTOR MANTIĞI ---
    int pwm_val;
    if (a2 < -0.05) { 
      pwm_val = map(abs(a2) * 100, 5, 100, 50, 255);
      sol(pwm_val);
    }
    else if (a2 > 0.05) {  
      pwm_val = map(a2 * 100, 5, 100, 50, 255);
      sag(pwm_val);
    }
    else if (abs(a0) > 0.05) {
      pwm_val = map(abs(a0) * 100, 5, 100, 50, 255);
      donus360(pwm_val, a0 > 0);
    }
    else if (a5 > -0.9 || a4 > -0.9) {
      int pwm_ileri = map((a5 + 1) * 50, 0, 100, 0, 255);
      int pwm_geri = map((a4 + 1) * 50, 0, 100, 0, 255);
      
      if (a5 > -0.9 && a4 <= -0.9) ileri(pwm_ileri);
      else if (a4 > -0.9 && a5 <= -0.9) geri(pwm_geri);
      else if (a5 > -0.9 && a4 > -0.9) ileri_mix(pwm_ileri);
    }
    else {
      aniDur();
    }
  } // if (packetSize) sonu
} // loop() sonu

// ============= MOTOR FONKSİYONLARI (LOOP DIŞINDA) =============

void ileri_mix(int PWM) {
  digitalWrite(RR_IN1, HIGH); digitalWrite(RR_IN2, LOW);
  digitalWrite(FR_IN1, HIGH); digitalWrite(FR_IN2, LOW);
  digitalWrite(RL_IN1, HIGH); digitalWrite(RL_IN2, LOW);
  digitalWrite(FL_IN1, HIGH); digitalWrite(FL_IN2, LOW);
  analogWrite(RR_PWM, PWM); analogWrite(RL_PWM, PWM * 0.75);
  analogWrite(FL_PWM, PWM * 0.75); analogWrite(FR_PWM, PWM);
}

void aniDur() {
  digitalWrite(RR_IN1, HIGH); digitalWrite(RR_IN2, HIGH);
  digitalWrite(RL_IN1, HIGH); digitalWrite(RL_IN2, HIGH);
  digitalWrite(FL_IN1, HIGH); digitalWrite(FL_IN2, HIGH);
  digitalWrite(FR_IN1, HIGH); digitalWrite(FR_IN2, HIGH);
  analogWrite(RR_PWM, 255); analogWrite(RL_PWM, 255);
  analogWrite(FL_PWM, 255); analogWrite(FR_PWM, 255);
  delay(30);
  digitalWrite(RR_IN1, LOW); digitalWrite(RR_IN2, LOW);
  digitalWrite(RL_IN1, LOW); digitalWrite(RL_IN2, LOW);
  digitalWrite(FL_IN1, LOW); digitalWrite(FL_IN2, LOW);
  digitalWrite(FR_IN1, LOW); digitalWrite(FR_IN2, LOW);
  analogWrite(RR_PWM, 0); analogWrite(RL_PWM, 0);
  analogWrite(FL_PWM, 0); analogWrite(FR_PWM, 0);
}

void sol(int PWM) {
  digitalWrite(RR_IN1, LOW); digitalWrite(RR_IN2, HIGH);
  digitalWrite(FR_IN1, HIGH); digitalWrite(FR_IN2, LOW);
  digitalWrite(RL_IN1, HIGH); digitalWrite(RL_IN2, LOW);
  digitalWrite(FL_IN1, LOW); digitalWrite(FL_IN2, HIGH);
  analogWrite(RR_PWM, PWM); analogWrite(RL_PWM, PWM);
  analogWrite(FL_PWM, PWM); analogWrite(FR_PWM, PWM);
}

void sag(int PWM) {
  digitalWrite(RR_IN1, HIGH); digitalWrite(RR_IN2, LOW);
  digitalWrite(FR_IN1, LOW); digitalWrite(FR_IN2, HIGH);
  digitalWrite(RL_IN1, LOW); digitalWrite(RL_IN2, HIGH);
  digitalWrite(FL_IN1, HIGH); digitalWrite(FL_IN2, LOW);
  analogWrite(RR_PWM, PWM); analogWrite(RL_PWM, PWM);
  analogWrite(FL_PWM, PWM); analogWrite(FR_PWM, PWM);
}

void ileri(int PWM) {
  digitalWrite(RR_IN1, HIGH); digitalWrite(RR_IN2, LOW);
  digitalWrite(FR_IN1, HIGH); digitalWrite(FR_IN2, LOW);
  digitalWrite(RL_IN1, HIGH); digitalWrite(RL_IN2, LOW);
  digitalWrite(FL_IN1, HIGH); digitalWrite(FL_IN2, LOW);
  analogWrite(RR_PWM, PWM); analogWrite(RL_PWM, PWM);
  analogWrite(FL_PWM, PWM); analogWrite(FR_PWM, PWM);
}

void geri(int PWM) {
  digitalWrite(RR_IN1, LOW); digitalWrite(RR_IN2, HIGH);
  digitalWrite(FR_IN1, LOW); digitalWrite(FR_IN2, HIGH);
  digitalWrite(RL_IN1, LOW); digitalWrite(RL_IN2, HIGH);
  digitalWrite(FL_IN1, LOW); digitalWrite(FL_IN2, HIGH);
  analogWrite(RR_PWM, PWM); analogWrite(RL_PWM, PWM);
  analogWrite(FL_PWM, PWM); analogWrite(FR_PWM, PWM);
}

void donus360(int PWM, bool sagaDonus) {
  if (sagaDonus) {
    digitalWrite(RR_IN1, LOW); digitalWrite(RR_IN2, HIGH);
    digitalWrite(FR_IN1, LOW); digitalWrite(FR_IN2, HIGH);
    digitalWrite(RL_IN1, HIGH); digitalWrite(RL_IN2, LOW);
    digitalWrite(FL_IN1, HIGH); digitalWrite(FL_IN2, LOW);
  } else {
    digitalWrite(RR_IN1, HIGH); digitalWrite(RR_IN2, LOW);
    digitalWrite(FR_IN1, HIGH); digitalWrite(FR_IN2, LOW);
    digitalWrite(RL_IN1, LOW); digitalWrite(RL_IN2, HIGH);
    digitalWrite(FL_IN1, LOW); digitalWrite(FL_IN2, HIGH);
  }
  analogWrite(RR_PWM, PWM); analogWrite(RL_PWM, PWM);
  analogWrite(FL_PWM, PWM); analogWrite(FR_PWM, PWM);
}