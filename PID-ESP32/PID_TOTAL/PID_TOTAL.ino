#include <Arduino.h>
#include <Wire.h>
#include "MPU6050.h"
#include <BasicLinearAlgebra.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
using namespace BLA;

// --- State vector X = [x, y, theta, v, omega] ---
BLA::Matrix<5,1> X = {0,0,0,0,0};
BLA::Matrix<5,5> P = BLA::Identity<5,5>() * 0.1;

// Noise matrices
BLA::Matrix<5,5> Q = BLA::Identity<5,5>() * 0.01;
BLA::Matrix<5,5> Rultra= BLA::Identity<5,5>() * 0.5;



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

const int D_FRNT = 15; // Echo pin for front distance sensor
const int D_RGHT = 16;
const int D_LEFT = 17;
const int D_REAR = 18;

// ACCELERATOR MPU6050 SENSOR

const int MPU_SDA = 41;
const int MPU_SCL = 42;

//RX-TX to N8R2 esp32
const int RX_PIN = 47;
const int TX_PIN = 20;

Wire.begin(MPU_SDA, MPU_SCL);


// --- Calibration constants ---
const float wheelCircumference = 31.4; // cm
const int ticksPerRev = 48;
const float tickToCm = wheelCircumference / ticksPerRev;
const float wheelbase = 31.75; // cm
const float gyroScale = 131.0;   // LSB/°/s for ±250 range
const float gyroOffset = -135.0; // measured offset

// --- Hall counters ---
volatile long RL_count = 0, RR_count = 0, FL_count = 0, FR_count = 0;

// --- ISRs ---
void IRAM_ATTR RL_ISR() { if (digitalRead(RL_D2)==HIGH) RL_count++; else RL_count--; }
void IRAM_ATTR RR_ISR() { if (digitalRead(RR_D2)==HIGH) RR_count++; else RR_count--; }
void IRAM_ATTR FL_ISR() { if (digitalRead(FL_D2)==HIGH) FL_count++; else FL_count--; }
void IRAM_ATTR FR_ISR() { if (digitalRead(FR_D2)==HIGH) FR_count++; else FR_count--; }

// --- Ultrasonic ---
float readUltrasonic(int echoPin) {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000);
  return duration * 0.034 / 2; // cm
}

// Median filter wrapper
float medianFilter(int echoPin) {
  float vals[5];
  for (int i=0; i<5; i++) {
    vals[i] = readUltrasonic(echoPin);
    delay(5);
  }
  std::sort(vals, vals+5);
  return vals[2];
}

// --- Kalman Prediction ---
void kalmanPredict(float dt, int16_t gz, long RL, long RR, long FL, long FR) {
  float dL = ((RL + FL) / 2.0) * tickToCm;
  float dR = ((RR + FR) / 2.0) * tickToCm;
  float d = (dL + dR) / 2.0;
  float dTheta = (dR - dL) / wheelbase;

  // Drift compensation: if slip detected, trust gyro more
  if (abs(dL - dR) > 10.0) { // slip threshold in cm
    dTheta = ((gz - gyroOffset) / gyroScale) * (PI/180.0) * dt;
  }

  // Gyro corrected
  float gyroZ = (gz - gyroOffset) / gyroScale; // °/s
  float gyroZrad = gyroZ * (PI/180.0);

  // Predict state
  X(0) += d * cos(X(2));
  X(1) += d * sin(X(2));
  X(2) += dTheta + gyroZrad * dt;
  X(3) = d / dt;
  X(4) = gyroZrad;

  // Predict covariance
  BLA::Matrix<5,5> F = BLA::Identity<5,5>();
  F(0,2) = -X(3)*dt*sin(X(2));
  F(0,3) = dt*cos(X(2));
  F(1,2) =  X(3)*dt*cos(X(2));
  F(1,3) = dt*sin(X(2));
  F(2,4) = dt;
  P = F * P * ~F + Q;
}

// --- Kalman Update ---
void kalmanUpdate(BLA::Matrix<5,1> Z, BLA::Matrix<5,5> H, BLA::Matrix<5,5> R) {
  BLA::Matrix<5,1> y = Z - H*X;
  BLA::Matrix<5,5> S = H*P*~H + R;
  BLA::Matrix<5,5> K = P*~H*Inverse(S);
  X = X + K*y;
  P = (BLA::Identity<5,5>() - K*H)*P;
}

// --- PID Controller for Position ---
float targetX = 50, targetY = 80; // example waypoint
float Kp = 0.5, Ki = 0.0, Kd = 0.1;
float integralX=0, integralY=0;
float prevErrorX=0, prevErrorY=0;

void positionPID() {
  float errorX = targetX - X(0);
  float errorY = targetY - X(1);

  integralX += errorX;
  integralY += errorY;

  float dErrorX = errorX - prevErrorX;
  float dErrorY = errorY - prevErrorY;

  float controlX = Kp*errorX + Ki*integralX + Kd*dErrorX;
  float controlY = Kp*errorY + Ki*integralY + Kd*dErrorY;

  prevErrorX = errorX;
  prevErrorY = errorY;

  // Translate controlX/controlY into motor commands here
  Serial.print("ControlX: "); Serial.print(controlX);
  Serial.print(" ControlY: "); Serial.println(controlY);
}

// --- Setup ---
MPU6050 mpu;

void setup() {
  // PIN MODE  
  pinMode(EN_RL1, INPUT);  
  pinMode(EN_RL2, INPUT);
  pinMode(EN_RR1, INPUT);
  pinMode(EN_RR2, INPUT);
  pinMode(EN_FL1, INPUT);
  pinMode(EN_FL2, INPUT);
  pinMode(EN_FR1, INPUT);
  pinMode(EN_FR2, INPUT);
  pinMode(EN_EL1, INPUT);
  pinMode(EN_EL2, INPUT);
  pinMode(EN_ER1, INPUT);
  pinMode(EN_ER2, INPUT);
    pinMode(D_TRIG, OUTPUT); // Trig pin mode OUTPUT
    pinMode(D_FRNT, INPUT); // Echo pin mode INPUT
    pinMode(D_RGHT, INPUT);
    pinMode(D_LEFT, INPUT);
    pinMode(D_REAR, INPUT);

  Serial.begin(115200);
  delay(1000);
  Serial.print("ESP32-N16R8 Başlatıldı.");
    // MPU6050 Initialization
    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 bağlantı hatası!");
        while (1);
    } else {
        Serial.println("MPU6050 bağlantısı başarılı.");
    }
}

// --- Loop ---
void loop() {