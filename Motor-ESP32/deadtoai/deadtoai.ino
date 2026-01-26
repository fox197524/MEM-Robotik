#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// --- MOTOR 4 PIN DEFINITIONS 
const int RL_PIN_PWM = 7;   
const int RL_PIN_IN1 = 8;   
const int RL_PIN_IN2 = 9;   

const int RR_PIN_PWM = 11;
const int RR_IN1 = 13;
const int RR_IN2 = 12;

const int FL_PIN_PWM = 15;
const int FL_IN1 = 16;
const int FL_IN2 = 17;

const int FR_PIN_PWM = 4;
const int FR_IN1 = 5;
const int FR_IN2 = 6;

const int RP = 35;

WiFiUDP udp;
unsigned int localPort = 4210;
char packetBuffer[255];
float a0=0, a2=0, a4=0, a5=0;
String lastMsg = "";

void setup() {
  Serial.begin(115200);
  
  int pins[] = {RL_PIN_PWM, RL_PIN_IN1, RL_PIN_IN2, RR_PIN_PWM, RR_IN1, RR_IN2, FL_PIN_PWM, FL_IN1, FL_IN2, FR_PIN_PWM, FR_IN1, FR_IN2};
  for(int p : pins) pinMode(p, OUTPUT);

  WiFi.begin("LAGARIMEDYA", "lagari5253");
  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("\nWiFi Connected!");
  Serial.print("ESP32 IP: "); Serial.println(WiFi.localIP());

  pinMode(RP, OUTPUT);
  digitalWrite(RP, HIGH);
  udp.begin(localPort);
}

void loop() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(packetBuffer, 255);
    packetBuffer[len] = 0;
    String msg = String(packetBuffer);
    
    // Tekrarlanan mesajları filtrele
    if (msg == lastMsg) return;
    lastMsg = msg;
    
    // Axis parsing
    if (msg.startsWith("AXIS 0 ")) a0 = msg.substring(7).toFloat();
    if (msg.startsWith("AXIS 2 ")) a2 = msg.substring(7).toFloat();  
    if (msg.startsWith("AXIS 4 ")) a4 = msg.substring(7).toFloat();
    if (msg.startsWith("AXIS 5 ")) a5 = msg.substring(7).toFloat();
    
    // MOTOR LOGIC - DÜŞÜK DEADZONE + MINIMAL SERIAL
    int pwm_val;
    if (abs(a2) > 0.05) {  // Sol kayma
      pwm_val = map(abs(a2) * 100, 5, 100, 50, 255);
      Serial.printf("SOL KAYMA: a2=%.3f PWM=%d\n", a2, pwm_val);
      sol(pwm_val);
    } 
    else if (abs(a0) > 0.05) {  // 360 dönüş
      pwm_val = map(abs(a0) * 100, 5, 100, 50, 255);
      Serial.printf("DÖNÜS: a0=%.3f PWM=%d YON=%s\n", a0, pwm_val, a0>0 ? "SAG" : "SOL");
      donus360(pwm_val, a0 > 0);
    } 
    else if (a5 > -0.9) {  // İleri
      pwm_val = map((a5 + 1) * 50, 0, 100, 0, 255);
      Serial.printf("ILERI: a5=%.3f PWM=%d\n", a5, pwm_val);
      ileri(pwm_val);
    } 
    else if (a4 > -0.9) {  // Geri
      pwm_val = map((a4 + 1) * 50, 0, 100, 0, 255);
      Serial.printf("GERI: a4=%.3f PWM=%d\n", a4, pwm_val);
      geri(pwm_val);
    } 
    else {  // Dur
      Serial.println("DUR");
      dur();
    }
  }
}

// Motor fonksiyonları aynı kalacak
void ileri(int PWM) {
  digitalWrite(RR_IN1, HIGH); digitalWrite(RR_IN2, LOW);
  digitalWrite(FR_IN1, HIGH); digitalWrite(FR_IN2, LOW);
  digitalWrite(RL_PIN_IN1, HIGH); digitalWrite(RL_PIN_IN2, LOW);
  digitalWrite(FL_IN1, HIGH); digitalWrite(FL_IN2, LOW);
  
  analogWrite(RR_PIN_PWM, PWM);
  analogWrite(RL_PIN_PWM, PWM);
  analogWrite(FL_PIN_PWM, PWM);
  analogWrite(FR_PIN_PWM, PWM);
}

void geri(int PWM) {
  digitalWrite(RR_IN1, LOW); digitalWrite(RR_IN2, HIGH);
  digitalWrite(FR_IN1, LOW); digitalWrite(FR_IN2, HIGH);
  digitalWrite(RL_PIN_IN1, LOW); digitalWrite(RL_PIN_IN2, HIGH);
  digitalWrite(FL_IN1, LOW); digitalWrite(FL_IN2, HIGH);
  
  analogWrite(RR_PIN_PWM, PWM);
  analogWrite(RL_PIN_PWM, PWM);
  analogWrite(FL_PIN_PWM, PWM);
  analogWrite(FR_PIN_PWM, PWM);
}

void sol(int PWM) {
  digitalWrite(RR_IN1, LOW); digitalWrite(RR_IN2, HIGH);
  digitalWrite(FR_IN1, HIGH); digitalWrite(FR_IN2, LOW);
  digitalWrite(RL_PIN_IN1, HIGH); digitalWrite(RL_PIN_IN2, LOW);
  digitalWrite(FL_IN1, LOW); digitalWrite(FL_IN2, HIGH);
  
  analogWrite(RR_PIN_PWM, PWM);
  analogWrite(RL_PIN_PWM, PWM);
  analogWrite(FL_PIN_PWM, PWM);
  analogWrite(FR_PIN_PWM, PWM);
}

void dur() {
  digitalWrite(RR_IN1, LOW); digitalWrite(RR_IN2, LOW);
  digitalWrite(RL_PIN_IN1, LOW); digitalWrite(RL_PIN_IN2, LOW);
  digitalWrite(FL_IN1, LOW); digitalWrite(FL_IN2, LOW);
  digitalWrite(FR_IN1, LOW); digitalWrite(FR_IN2, LOW);
  
  analogWrite(RR_PIN_PWM, 0);
  analogWrite(RL_PIN_PWM, 0);
  analogWrite(FL_PIN_PWM, 0);
  analogWrite(FR_PIN_PWM, 0);
}

void donus360(int PWM, bool sagaDonus) {
  if (sagaDonus) { // Sağa 360
    digitalWrite(RR_IN1, LOW); digitalWrite(RR_IN2, HIGH);
    digitalWrite(FR_IN1, LOW); digitalWrite(FR_IN2, HIGH);
    digitalWrite(RL_PIN_IN1, HIGH); digitalWrite(RL_PIN_IN2, LOW);
    digitalWrite(FL_IN1, HIGH); digitalWrite(FL_IN2, LOW);
  } else { // Sola 360
    digitalWrite(RR_IN1, HIGH); digitalWrite(RR_IN2, LOW);
    digitalWrite(FR_IN1, HIGH); digitalWrite(FR_IN2, LOW);
    digitalWrite(RL_PIN_IN1, LOW); digitalWrite(RL_PIN_IN2, HIGH);
    digitalWrite(FL_IN1, LOW); digitalWrite(FL_IN2, HIGH);
  }
  
  analogWrite(RR_PIN_PWM, PWM);
  analogWrite(RL_PIN_PWM, PWM);
  analogWrite(FL_PIN_PWM, PWM);
  analogWrite(FR_PIN_PWM, PWM);
}
