#include <Arduino.h>
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

// Configuration
const int PWM = 255; // Full speed
const int PWMS = 0; // 



void setup() {
  Serial.begin(115200);
  Serial.print("code by ""Dead To AI"" Community");

  

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


dur ();
delay(500);
ileri();
delay(1000);
dur ();
delay(500);
geri();
delay(1000);
dur ();
delay(500);
sag();
delay(1000);
dur ();
delay(500);
sol();
delay(1000);
dur ();
delay(500);
sol360();
delay(1000);
dur ();
delay(500);
sag360();
delay(1000);
dur();
}

void loop() {
 







}

void sagarkaileri() {

  digitalWrite(RR_PIN_IN1, HIGH);
  digitalWrite(RR_PIN_IN2, LOW);

  analogWrite(RR_PIN_PWM, PWM);

}
void solarkaileri() {

  digitalWrite(RL_PIN_IN1, HIGH);
  digitalWrite(RL_PIN_IN2, LOW);


  analogWrite(RL_PIN_PWM, PWM);

}

void sagonileri() {


  digitalWrite(FR_PIN_IN1, HIGH);
  digitalWrite(FR_PIN_IN2, LOW);

  analogWrite(FR_PIN_PWM, PWM);

}

void solonileri() {

  digitalWrite(FL_PIN_IN1, HIGH);
  digitalWrite(FL_PIN_IN2, LOW);


  analogWrite(FL_PIN_PWM, PWM);

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