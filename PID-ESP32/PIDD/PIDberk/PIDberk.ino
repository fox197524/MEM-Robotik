#include <Arduino.h>
#include <Wire.h>
#include "MPU6050.h"
#include <BasicLinearAlgebra.h>
using namespace BLA;

// --- State vector X = [x, y, theta, v, omega] ---
BLA::Matrix<5,1> X = {0,0,0,0,0};
BLA::Matrix<5,5> P = BLA::Identity<5,5>() * 0.1;

// Noise matrices
BLA::Matrix<5,5> Q = BLA::Identity<5,5>() * 0.01;
BLA::Matrix<5,5> Rultra= BLA::Identity<5,5>() * 0.5;

// --- Pinouts ---
// Hall sensors
#define RL_D1 1
#define RL_D2 2
#define RR_D1 4
#define RR_D2 5
#define FL_D1 6
#define FL_D2 7
#define FR_D1 8
#define FR_D2 9

// Ultrasonics
#define TRIG_PIN 14
#define ECHO_FRONT 15
#define ECHO_RIGHT 16
#define ECHO_LEFT 17
#define ECHO_REAR 18

// MPU6050 I2C
#define SDA_PIN 41
#define SCL_PIN 42

// --- Calibration constants ---
float wheelCircumference = 31.4; // cm
int ticksPerRev = 48;
float tickToCm = wheelCircumference / ticksPerRev;
float wheelbase = 31.75; // cm
float gyroScale = 131.0;   // LSB/°/s for ±250 range
float gyroOffset = -135.0; // measured offset

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
  Serial.begin(115200);

  // Encoders
  pinMode(RL_D1, INPUT); pinMode(RL_D2, INPUT);
  pinMode(RR_D1, INPUT); pinMode(RR_D2, INPUT);
  pinMode(FL_D1, INPUT); pinMode(FL_D2, INPUT);
  pinMode(FR_D1, INPUT); pinMode(FR_D2, INPUT);

  attachInterrupt(digitalPinToInterrupt(RL_D1), RL_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RR_D1), RR_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(FL_D1), FL_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(FR_D1), FR_ISR, RISING);

  // Ultrasonics
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  pinMode(ECHO_RIGHT, INPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(ECHO_REAR, INPUT);

  // MPU6050
  Wire.begin(SDA_PIN, SCL_PIN);
  mpu.initialize();
}

// --- Loop ---
void loop() {
  static unsigned long lastTime = millis();
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  // Gyro
  int16_t gx, gy, gz;
  mpu.getRotation(&gx, &gy, &gz);

  // Prediction step
  kalmanPredict(dt, gz, RL_count, RR_count, FL_count, FR_count);

  // Median-filtered ultrasonic readings
  float distFront = medianFilter(ECHO_FRONT);
  float distRear  = medianFilter(ECHO_REAR);
  float distLeft  = medianFilter(ECHO_LEFT);
  float distRight = medianFilter(ECHO_RIGHT);

  // --- Auto-localization ---
  // Arena length/width can be set once (e.g. 100 cm), but robot estimates its position
  float arenaLength = 100.0; // cm
  float arenaWidth  = 100.0; // cm

  // Estimate X and Y from pairs of sensors
  float estX = (arenaLength - distFront + distRear) / 2.0;
  float estY = (arenaWidth  - distLeft  + distRight) / 2.0;

  // Build measurement vector
  BLA::Matrix<5,1> Zpos = {estX, estY, 0, 0, 0};
  BLA::Matrix<5,5> Hpos = BLA::Identity<5,5>();

  // Update Kalman with inferred position
  kalmanUpdate(Zpos, Hpos, Rultra);

  // PID control toward target waypoint
  positionPID();

  // Log fused state
  Serial.print("X: "); Serial.print(X(0));
  Serial.print(" Y: "); Serial.print(X(1));
  Serial.print(" Theta: "); Serial.print(X(2));
  Serial.print(" V: "); Serial.print(X(3));
  Serial.print(" Omega: "); Serial.println(X(4));

  delay(50);
}