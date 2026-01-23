#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// --- MOTOR 4 PIN DEFINITIONS 
const int RL_PIN_PWM = 7;   // Speed Control (PWM) yeşil
const int RL_PIN_IN2 = 8;   // Direction 1.        turuncu
const int RL_PIN_IN1 = 9;   // Direction 2.        sarı

const int RR_PIN_PWM = 11;
const int RR_PIN_IN2 = 12;
const int RR_PIN_IN1 = 13;

const int FL_PIN_PWM = 15;
const int FL_PIN_IN2 = 16;
const int FL_PIN_IN1 = 17;

const int FR_PIN_PWM = 4;
const int FR_PIN_IN2 = 5;
const int FR_PIN_IN1 = 6;

// Configuration
const int PWM = 200; // Full speed
const int PWMS = 0; // 


// --- WIFI ---
const char* ssid = "LAGARIMEDYA";
const char* password = "lagari5253";
WiFiUDP udp;
unsigned int localPort = 4210;
char packetBuffer[255];


float axis0 = 0, axis2 = 0, axis4 = -1, axis5 = -1;
float p0 = 0, p2 = 0, p4 = -1, p5 = -1; // Previous values for change detection

void setup() {
  Serial.begin(115200);
  Serial.print("code by ""Dead To AI"" Community");

  dur();

  pinMode(RL_PIN_PWM, OUTPUT);
  pinMode(RL_PIN_IN1, OUTPUT);
  pinMode(RL_PIN_IN2, OUTPUT);
  pinMode(RR_PIN_PWM, OUTPUT);
  pinMode(RR_PIN_IN1, OUTPUT);
  pinMode(RR_PIN_IN2, OUTPUT);
  pinMode(FL_PIN_PWM, OUTPUT);
  pinMode(FL_PIN_IN1, OUTPUT);
  pinMode(FL_PIN_IN2, OUTPUT);
  pinMode(FR_PIN_PWM, OUTPUT);
  pinMode(FR_PIN_IN1, OUTPUT);
  pinMode(FR_PIN_IN2, OUTPUT);


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
 
if(axis5 >= -0.5){
  ileri();
}


}
// ============ Controller Trigger Convert Fonksiyonu ============
void trigger() {

}

// ============ Controller Joystick Convert Fonksiyonu ===========
void joystick() {

}

// ============= Controller Input Convert Fonksiyonu =============
void cic() {

}

// ====================== İleri Fonksiyonu =======================
void ileri() {

  digitalWrite(RR_PIN_IN1, HIGH);
  digitalWrite(RR_PIN_IN2, LOW);

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
//==============Geri Fonksiyonu=============
void geri(){

  digitalWrite(RR_PIN_IN1, LOW);
  digitalWrite(RR_PIN_IN2, HIGH);

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