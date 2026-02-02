//===========THE BIG PID SYSTEM==============
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>

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

const int EN_FR1 = 10;
const int EN_FR2 = 11;

const int EN_FR1 = 12;
const int EN_FR2 = 13;

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
  // put your setup code here, to run once:




}

void loop() {
  // put your main code here, to run repeatedly:




}
