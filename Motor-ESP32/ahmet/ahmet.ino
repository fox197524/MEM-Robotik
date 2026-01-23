#include <Arduino.h>

#define M1_HIGH HIGH
#define M1_LOW LOW

#define M2_HIGH LOW
#define M2_LOW HIGH

#define M3_HIGH HIGH
#define M3_LOW LOW

#define M4_HIGH HIGH
#define M4_LOW LOW

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

void setup() {
  Serial.begin(115200);
  Serial.print("code by ""Dead To AI"" Community");

  pinMode(RL_PIN_PWM, OUTPUT); // Motor 1 Rear left
  pinMode(RL_PIN_IN1, OUTPUT);
  pinMode(RL_PIN_IN2, OUTPUT);

  pinMode(RR_PIN_PWM, OUTPUT); // Motor 2 Rear Right
  pinMode(RR_PIN_IN1, OUTPUT); 
  pinMode(RR_PIN_IN2, OUTPUT); 

  pinMode(FL_PIN_PWM, OUTPUT); // Motor 3 Front Left
  pinMode(FL_PIN_IN1, OUTPUT); 
  pinMode(FL_PIN_IN2, OUTPUT); 

  pinMode(FR_PIN_PWM, OUTPUT); // Motor 4 Front Right
  pinMode(FR_PIN_IN1, OUTPUT);
  pinMode(FR_PIN_IN2, OUTPUT); 
}

void loop() {
  dur();
  delay(1000);
  ileri();
  delay(1000);
  dur();
  delay(1000);
  geri();
  delay(1000);
  dur();
  delay(1000);
  sag360();
  delay(1000);
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

  // RR = M1
  digitalWrite(RR_PIN_IN1, M1_HIGH);
  digitalWrite(RR_PIN_IN2, M1_LOW);

  // FR = M2
  digitalWrite(FR_PIN_IN1, M2_HIGH);
  digitalWrite(FR_PIN_IN2, M2_LOW);

  // RL = M3
  digitalWrite(RL_PIN_IN1, M3_HIGH);
  digitalWrite(RL_PIN_IN2, M3_LOW);

  // FL = M4
  digitalWrite(FL_PIN_IN1, M4_LOW);
  digitalWrite(FL_PIN_IN2, M4_HIGH);

  analogWrite(RR_PIN_PWM, PWM);
  analogWrite(RL_PIN_PWM, PWM);
  analogWrite(FL_PIN_PWM, PWM);
  analogWrite(FR_PIN_PWM, PWM);
}

//==============Geri Fonksiyonu=============
void geri(){

  // RR = M1
  digitalWrite(RR_PIN_IN1, M1_LOW);
  digitalWrite(RR_PIN_IN2, M1_HIGH);

  // FR = M2
  digitalWrite(FR_PIN_IN1, M2_LOW);
  digitalWrite(FR_PIN_IN2, M2_HIGH);

  // RL = M3
  digitalWrite(RL_PIN_IN1, M3_LOW);
  digitalWrite(RL_PIN_IN2, M3_HIGH);

  // FL = M4
  digitalWrite(FL_PIN_IN1, M4_HIGH);
  digitalWrite(FL_PIN_IN2, M4_LOW);

  analogWrite(RR_PIN_PWM, PWM);
  analogWrite(RL_PIN_PWM, PWM);
  analogWrite(FL_PIN_PWM, PWM);
  analogWrite(FR_PIN_PWM, PWM);
}

//==============Sağa Kayma Fonksiyonu=============
void sag(){
  
  // RR = M1
  digitalWrite(RR_PIN_IN1, M1_HIGH);
  digitalWrite(RR_PIN_IN2, M1_LOW);

  // FR = M2
  digitalWrite(FR_PIN_IN1, M2_LOW);
  digitalWrite(FR_PIN_IN2, M2_HIGH);

  // RL = M3
  digitalWrite(RL_PIN_IN1, M3_LOW);
  digitalWrite(RL_PIN_IN2, M3_HIGH);

  // FL = M4
  digitalWrite(FL_PIN_IN1, M4_HIGH);
  digitalWrite(FL_PIN_IN2, M4_LOW);

  analogWrite(RR_PIN_PWM, PWM);
  analogWrite(RL_PIN_PWM, PWM);
  analogWrite(FL_PIN_PWM, PWM);
  analogWrite(FR_PIN_PWM, PWM);
}

//==============Sola Kayma Fonksiyonu=============
void sol(){
  
  // RR = M1
  digitalWrite(RR_PIN_IN1, M1_LOW);
  digitalWrite(RR_PIN_IN2, M1_HIGH);

  // FR = M2
  digitalWrite(FR_PIN_IN1, M2_HIGH);
  digitalWrite(FR_PIN_IN2, M2_LOW);

  // RL = M3
  digitalWrite(RL_PIN_IN1, M3_HIGH);
  digitalWrite(RL_PIN_IN2, M3_LOW);

  // FL = M4
  digitalWrite(FL_PIN_IN1, M4_LOW);
  digitalWrite(FL_PIN_IN2, M4_HIGH);

  analogWrite(RR_PIN_PWM, PWM);
  analogWrite(RL_PIN_PWM, PWM);
  analogWrite(FL_PIN_PWM, PWM);
  analogWrite(FR_PIN_PWM, PWM);
}

//==============Dur Fonksiyonu=============
void dur(){
  
  digitalWrite(RR_PIN_IN1, M1_LOW);
  digitalWrite(RR_PIN_IN2, M1_LOW);

  digitalWrite(RL_PIN_IN1, M3_LOW);
  digitalWrite(RL_PIN_IN2, M3_LOW);

  digitalWrite(FL_PIN_IN1, M4_LOW);
  digitalWrite(FL_PIN_IN2, M4_LOW);

  digitalWrite(FR_PIN_IN1, M2_LOW);
  digitalWrite(FR_PIN_IN2, M2_LOW);

  analogWrite(RR_PIN_PWM, PWMS);
  analogWrite(RL_PIN_PWM, PWMS);
  analogWrite(FL_PIN_PWM, PWMS);
  analogWrite(FR_PIN_PWM, PWMS);
}

//==============sola 360 dönme fonksiyonu=============
void sol360(){

  // RR = M1
  digitalWrite(RR_PIN_IN1, M1_HIGH);
  digitalWrite(RR_PIN_IN2, M1_LOW);

  // FR = M2
  digitalWrite(FR_PIN_IN1, M2_HIGH);
  digitalWrite(FR_PIN_IN2, M2_LOW);

  // RL = M3
  digitalWrite(RL_PIN_IN1, M3_LOW);
  digitalWrite(RL_PIN_IN2, M3_HIGH);

  // FL = M4
  digitalWrite(FL_PIN_IN1, M4_LOW);
  digitalWrite(FL_PIN_IN2, M4_HIGH);

  analogWrite(RR_PIN_PWM, PWM);
  analogWrite(RL_PIN_PWM, PWM);
  analogWrite(FL_PIN_PWM, PWM);
  analogWrite(FR_PIN_PWM, PWM);
}

//==============sağa 360 dönme fonksiyonu=============
void sag360(){

  // RR = M1
  digitalWrite(RR_PIN_IN1, M1_LOW);
  digitalWrite(RR_PIN_IN2, M1_HIGH);

  // FR = M2
  digitalWrite(FR_PIN_IN1, M2_LOW);
  digitalWrite(FR_PIN_IN2, M2_HIGH);

  // RL = M3
  digitalWrite(RL_PIN_IN1, M3_HIGH);
  digitalWrite(RL_PIN_IN2, M3_LOW);

  // FL = M4
  digitalWrite(FL_PIN_IN1, M4_HIGH);
  digitalWrite(FL_PIN_IN2, M4_LOW);

  analogWrite(RR_PIN_PWM, PWM);
  analogWrite(RL_PIN_PWM, PWM);
  analogWrite(FL_PIN_PWM, PWM);
  analogWrite(FR_PIN_PWM, PWM);
}