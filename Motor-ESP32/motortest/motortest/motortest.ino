#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// --- IP CONFIGURATION ---

// --- MOTOR 4 PIN DEFINITIONS 
const int RL_PIN_PWM = 7;   //karışma doğru 
const int RL_PIN_IN1 = 8;   
const int RL_PIN_IN2 = 9;   

const int RR_PIN_PWM = 11;//karışma
const int RR_PIN_IN1 = 13;
const int RR_PIN_IN2 = 12;

const int FL_PIN_PWM = 15;//karışma
const int FL_PIN_IN1 = 16;
const int FL_PIN_IN2 = 17;

const int FR_PIN_PWM = 4;//karışma doğru
const int FR_PIN_IN1 = 5;
const int FR_PIN_IN2 = 6;



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
// ====================== İleri Fonksiyonu =======================
void ileri() {

  digitalWrite(RR_PIN_IN1, HIGH);
  digitalWrite(RR_PIN_IN2, LOW);

  digitalWrite(FR_PIN_IN1, HIGH);
  digitalWrite(FR_PIN_IN2, LOW);

  digitalWrite(RL_PIN_IN1, HIGH);
  digitalWrite(RL_PIN_IN2, LOW);

  digitalWrite(FL_PIN_IN1, HIGH);
  digitalWrite(FL_PIN_IN2, LOW);


  analogWrite(RR_PIN_PWM, PWM);
  analogWrite(RL_PIN_PWM, PWM);
  analogWrite(FL_PIN_PWM, PWM);
  analogWrite(FR_PIN_PWM, PWM);

}
//==============Geri Fonksiyonu=============
void geri(){

  digitalWrite(RR_PIN_IN1, LOW);
  digitalWrite(RR_PIN_IN2, HIGH);

  digitalWrite(FR_PIN_IN1, LOW);
  digitalWrite(FR_PIN_IN2, HIGH);

  digitalWrite(RL_PIN_IN1, LOW);
  digitalWrite(RL_PIN_IN2, HIGH);

  digitalWrite(FL_PIN_IN1, LOW);
  digitalWrite(FL_PIN_IN2, HIGH);


  analogWrite(RR_PIN_PWM, PWM);
  analogWrite(RL_PIN_PWM, PWM);
  analogWrite(FL_PIN_PWM, PWM);
  analogWrite(FR_PIN_PWM, PWM);

}
//==============Sağa Kayma Fonksiyonu=============
void sag(){
  
  digitalWrite(RR_PIN_IN1, HIGH);
  digitalWrite(RR_PIN_IN2, LOW);

  digitalWrite(FR_PIN_IN1, LOW);
  digitalWrite(FR_PIN_IN2, HIGH);

  digitalWrite(RL_PIN_IN1, LOW);
  digitalWrite(RL_PIN_IN2, HIGH);

  digitalWrite(FL_PIN_IN1, HIGH);
  digitalWrite(FL_PIN_IN2, LOW);


  analogWrite(RR_PIN_PWM, PWM);
  analogWrite(RL_PIN_PWM, PWM);
  analogWrite(FL_PIN_PWM, PWM);
  analogWrite(FR_PIN_PWM, PWM);
}
//==============Sola Kayma Fonksiyonu=============
void sol(){
  
  digitalWrite(RR_PIN_IN1, LOW);
  digitalWrite(RR_PIN_IN2, HIGH);

  digitalWrite(FR_PIN_IN1, HIGH);
  digitalWrite(FR_PIN_IN2, LOW);

  digitalWrite(RL_PIN_IN1, HIGH);
  digitalWrite(RL_PIN_IN2, LOW);

  digitalWrite(FL_PIN_IN1, LOW);
  digitalWrite(FL_PIN_IN2, HIGH);


  analogWrite(RR_PIN_PWM, PWM);
  analogWrite(RL_PIN_PWM, PWM);
  analogWrite(FL_PIN_PWM, PWM);
  analogWrite(FR_PIN_PWM, PWM);
}
//==============Dur Fonksiyonu=============
void dur(){
  
  digitalWrite(RR_PIN_IN1, LOW);
  digitalWrite(RR_PIN_IN2, LOW);

  digitalWrite(RL_PIN_IN1, LOW);
  digitalWrite(RL_PIN_IN2, LOW);

  digitalWrite(FL_PIN_IN1, LOW);
  digitalWrite(FL_PIN_IN2, LOW);

  digitalWrite(FR_PIN_IN1, LOW);
  digitalWrite(FR_PIN_IN2, LOW);


  analogWrite(RR_PIN_PWM, PWMS);
  analogWrite(RL_PIN_PWM, PWMS);
  analogWrite(FL_PIN_PWM, PWMS);
  analogWrite(FR_PIN_PWM, PWMS);

}
//==============sola 360 dönme fonksiyonu=============
void sol360(){

  digitalWrite(RR_PIN_IN1, HIGH);
  digitalWrite(RR_PIN_IN2, LOW);

  digitalWrite(FR_PIN_IN1, HIGH);
  digitalWrite(FR_PIN_IN2, LOW);

  digitalWrite(RL_PIN_IN1, LOW);
  digitalWrite(RL_PIN_IN2, HIGH);

  digitalWrite(FL_PIN_IN1, LOW);
  digitalWrite(FL_PIN_IN2, HIGH);


  analogWrite(RR_PIN_PWM, PWM);
  analogWrite(RL_PIN_PWM, PWM);
  analogWrite(FL_PIN_PWM, PWM);
  analogWrite(FR_PIN_PWM, PWM);
}
//==============sağa 360 dönme fonksiyonu=============
void sag360(){

  digitalWrite(RR_PIN_IN1, LOW);
  digitalWrite(RR_PIN_IN2, HIGH);

  digitalWrite(FR_PIN_IN1, LOW);
  digitalWrite(FR_PIN_IN2, HIGH);

  digitalWrite(RL_PIN_IN1, HIGH);
  digitalWrite(RL_PIN_IN2, LOW);

  digitalWrite(FL_PIN_IN1, HIGH);
  digitalWrite(FL_PIN_IN2, LOW);


  analogWrite(RR_PIN_PWM, PWM);
  analogWrite(RL_PIN_PWM, PWM);
  analogWrite(FL_PIN_PWM, PWM);
  analogWrite(FR_PIN_PWM, PWM);
}