// 192.168.4.1
// MECHATAK - 
// MECHATAK - 


#define PROBOT_WIFI_AP_PASSWORD "kayra123"
#define PROBOT_WIFI_AP_SSID "MECHATAK"
#define PROBOT_WIFI_AP_CHANNEL 3

#include <probot.h>
#include <probot/devices/servo/servo.hpp>


const int FL_PWM = 15;
const int FL_IN1 = 16;
const int FL_IN2 = 17;

const int FR_PWM = 4;
const int FR_IN1 = 5;
const int FR_IN2 = 6;

const int RL_PWM = 7;
const int RL_IN1 = 8;
const int RL_IN2 = 9;

const int RR_PWM = 12;
const int RR_IN1 = 11;
const int RR_IN2 = 13;

const int E_PWM = 1;
const int E_IN1 = 2;
const int E_IN2 = 42;


void setup() {
  // put your setup code here, to run once:
pinMode(FL_PWM, OUTPUT);
pinMode(FL_IN1, OUTPUT);
pinMode(FL_IN2, OUTPUT);

pinMode(FR_PWM, OUTPUT);
pinMode(FR_IN1, OUTPUT);
pinMode(FR_IN2, OUTPUT);

pinMode(RL_PWM, OUTPUT);
pinMode(RL_IN1, OUTPUT);
pinMode(RL_IN2, OUTPUT);

pinMode(RR_PWM, OUTPUT);
pinMode(RR_IN1, OUTPUT);
pinMode(RR_IN2, OUTPUT);

pinMode(E_PWM, OUTPUT);
pinMode(E_IN1, OUTPUT);
pinMode(E_IN2, OUTPUT);



}

void loop() {
  // put your main code here, to run repeatedly:
if(){


}
else if(){

}



}



void ileri(int pwm){

digitalWrite(FL_IN1, HIGH);
digitalWrite(FL_IN2, LOW);

digitalWrite(FR_IN1, HIGH);
digitalWrite(FR_IN2, LOW);

digitalWrite(RL_IN1, HIGH);
digitalWrite(RL_IN2, LOW);

digitalWrite(RR_IN1, HIGH);
digitalWrite(RR_IN2, LOW);

analogWrite(FL_PWM, pwm);
analogWrite(FR_PWM, pwm);
analogWrite(RL_PWM, pwm);
analogWrite(RR_PWM, pwm);

}

void geri(int pwm){

digitalWrite(FL_IN1, LOW);
digitalWrite(FL_IN2, HIGH);

digitalWrite(FR_IN1, LOW);
digitalWrite(FR_IN2, HIGH);

digitalWrite(RL_IN1, LOW);
digitalWrite(RL_IN2, HIGH);

digitalWrite(RR_IN1, LOW);
digitalWrite(RR_IN2, HIGH);

analogWrite(FL_PWM, pwm);
analogWrite(FR_PWM, pwm);
analogWrite(RL_PWM, pwm);
analogWrite(RR_PWM, pwm);

}

void sagslide(int pwm){

digitalWrite(FL_IN1, HIGH);
digitalWrite(FL_IN2, LOW);

digitalWrite(FR_IN1, LOW);
digitalWrite(FR_IN2, HIGH);

digitalWrite(RL_IN1, LOW);
digitalWrite(RL_IN2, HIGH);

digitalWrite(RR_IN1, HIGH);
digitalWrite(RR_IN2, LOW);

analogWrite(FL_PWM, pwm);
analogWrite(FR_PWM, pwm);
analogWrite(RL_PWM, pwm);
analogWrite(RR_PWM, pwm);

}

void solslide(int pwm){

digitalWrite(FL_IN1, LOW);
digitalWrite(FL_IN2, HIGH);

digitalWrite(FR_IN1, HIGH);
digitalWrite(FR_IN2, LOW);

digitalWrite(RL_IN1, LOW);
digitalWrite(RL_IN2, HIGH);

digitalWrite(RR_IN1, HIGH);
digitalWrite(RR_IN2, LOW);

analogWrite(FL_PWM, pwm);
analogWrite(FR_PWM, pwm);
analogWrite(RL_PWM, pwm);
analogWrite(RR_PWM, pwm);

}

void sol360(int pwm){

digitalWrite(FL_IN1, HIGH);
digitalWrite(FL_IN2, LOW);

digitalWrite(FR_IN1, HIGH);
digitalWrite(FR_IN2, LOW);

digitalWrite(RL_IN1, HIGH);
digitalWrite(RL_IN2, LOW);

digitalWrite(RR_IN1, HIGH);
digitalWrite(RR_IN2, LOW);

analogWrite(FL_PWM, pwm);
analogWrite(FR_PWM, pwm);
analogWrite(RL_PWM, pwm);
analogWrite(RR_PWM, pwm);

}

void sag360(int pwm){

digitalWrite(FL_IN1, HIGH);
digitalWrite(FL_IN2, LOW);

digitalWrite(FR_IN1, HIGH);
digitalWrite(FR_IN2, LOW);

digitalWrite(RL_IN1, HIGH);
digitalWrite(RL_IN2, LOW);

digitalWrite(RR_IN1, HIGH);
digitalWrite(RR_IN2, LOW);

analogWrite(FL_PWM, pwm);
analogWrite(FR_PWM, pwm);
analogWrite(RL_PWM, pwm);
analogWrite(RR_PWM, pwm);

}

void dur(){

digitalWrite(FL_IN1, LOW);
digitalWrite(FL_IN2, LOW);

digitalWrite(FR_IN1, LOW);
digitalWrite(FR_IN2, LOW);

digitalWrite(RL_IN1, LOW);
digitalWrite(RL_IN2, LOW);

digitalWrite(RR_IN1, LOW);
digitalWrite(RR_IN2, LOW);

analogWrite(FL_PWM, 0);
analogWrite(FR_PWM, 0);
analogWrite(RL_PWM, 0);
analogWrite(RR_PWM, 0);

}

void anidur(){

digitalWrite(FL_IN1, HIGH);
digitalWrite(FL_IN2, HIGH);

digitalWrite(FR_IN1, HIGH);
digitalWrite(FR_IN2, HIGH);

digitalWrite(RL_IN1, HIGH);
digitalWrite(RL_IN2, HIGH);

digitalWrite(RR_IN1, HIGH);
digitalWrite(RR_IN2, HIGH);

analogWrite(FL_PWM, 255);
analogWrite(FR_PWM, 255);
analogWrite(RL_PWM, 255);
analogWrite(RR_PWM, 255);

delay(30);

digitalWrite(FL_IN1, LOW);
digitalWrite(FL_IN2, LOW);

digitalWrite(FR_IN1, LOW);
digitalWrite(FR_IN2, LOW);

digitalWrite(RL_IN1, LOW);
digitalWrite(RL_IN2, LOW);

digitalWrite(RR_IN1, LOW);
digitalWrite(RR_IN2, LOW);

analogWrite(FL_PWM, 0);
analogWrite(FR_PWM, 0);
analogWrite(RL_PWM, 0);
analogWrite(RR_PWM, 0);

}

void sagon(){
digitalWrite(FR_IN1, HIGH);
digitalWrite(FR_IN2, LOW);

analogWrite(FR_PWM, 255);

}

void solon(){
digitalWrite(FL_IN1, HIGH);
digitalWrite(FL_IN2, LOW);

analogWrite(FL_PWM, 255);

}

void sagarka(){
digitalWrite(RR_IN1, HIGH);
digitalWrite(RR_IN2, LOW);

analogWrite(RR_PWM, 255);

}

void solarka(){
digitalWrite(RL_IN1, HIGH);
digitalWrite(RL_IN2, LOW);

analogWrite(RL_PWM, 255);

}

void eup(int pwma){


}

void edown(int pwma){


}

