//===========THE BIG PID SYSTEM==============
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

//======PID PINOUT=====

// ENCODERS
const int EN_RL1 = 1; // Encoder on Right Left Motor First pin
const int EN_RL2 = 2; // Encoder on Right Left Motor Second pin

const int EN_RR1 = 4;
const int EN_RR2 = 5;

const int EN_FL1 = 6;
const int EN_FL2 = 7;

const int EN_FR1 = 8;
const int EN_FR2 = 9;

const int EN_EL1 = 10;
const int EN_EL2 = 11;

const int EN_ER1 = 12;
const int EN_ER2 = 13;

// DISTANCE SENSORS HC-SR04

const int D_TRIG = 14; // Common Trig Pin 14 for all distance sensors

const int D_FRONT = 15; // Echo pin for front distance sensor
const int D_RIGHT = 16;
const int D_LEFT = 17;
const int D_REAR = 18;

// ACCELERATOR MPU6050 SENSOR
const int MPU_SDA = 41
const int MPU_SCL = 42


void setup() {
// PIN MODE  
  pinMode(EN_RL1, OUTPUT);  
  pinMode(EN_RL2, OUTPUT);
  pinMode(EN_RR1, OUTPUT);
  pinMode(EN_RR2, OUTPUT);
  pinMode(EN_FL1, OUTPUT);
  pinMode(EN_FL2, OUTPUT);
  pinMode(EN_FR1, OUTPUT);
  pinMode(EN_FR2, OUTPUT);
  pinMode(EN_EL1, OUTPUT);
  pinMode(EN_EL2, OUTPUT);
  pinMode(EN_ER1, OUTPUT);
  pinMode(EN_ER2, OUTPUT);
  pinMode(D_FRONT, OUTPUT);
  pinMode(D_RIGHT, OUTPUT);
  pinMode(D_LEFT, OUTPUT);
  pinMode(D_REAR, OUTPUT);
  pinMode(MPU_SDA, OUTPUT);
  pinMode(MPU_SCL, OUTPUT);




}

void loop() {
  // put your main code here, to run repeatedly:




}
