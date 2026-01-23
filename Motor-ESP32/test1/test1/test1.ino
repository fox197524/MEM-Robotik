#include <Arduino.h>

// --- MOTOR 4 PIN DEFINITIONS 
const int RL_PIN_PWM = 7;   // Speed Control (PWM) yeşil
const int RL_PIN_IN1 = 8;   // Direction 1.        turuncu
const int RL_PIN_IN2 = 9;   // Direction 2.        sarı

const int RR_PIN_PWM = 11;
const int RR_PIN_IN1 = 12;
const int RR_PIN_IN2 = 13;

const int FL_PIN_PWM = 15;
const int FL_PIN_IN1 = 16;
const int FL_PIN_IN2 = 17;

const int FR_PIN_PWM = 4;
const int FR_PIN_IN1 = 5;
const int FR_PIN_IN2 = 6;

// Configuration
const int PWM = 200; // Full speed
const int PWMS = 0; // Full speed

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
}

void loop() {

ileri ();
delay(1000);
dur ();
delay(1000);
geri ();
delay(1000);
dur ();
delay(1000);
sag ();
delay(1000);
dur ();
delay(1000);
sol ();
delay(1000);
dur ();
delay(1000);
sag360 ();
delay(1000);
dur ();
delay(1000);
sol360 ();
delay(1000);
dur ();
delay(1000);

}

void ileri(){

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