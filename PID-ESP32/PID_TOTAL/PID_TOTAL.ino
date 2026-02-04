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

// --- Pinouts ---
// Encoders
const int RL_D1 = 1;
const int RL_D2 = 2;
const int RR_D1 = 4;
const int RR_D2 = 5;
const int FL_D1 = 6;
const int FL_D2 = 7;
const int FR_D1 = 8;
const int FR_D2 = 9;
const int EL_D1 = 10;
const int EL_D2 = 11;
const int ER_D1 = 12;
const int ER_D2 = 13;

// Ultrasonics
const int TRIG_PIN = 14;
const int ECHO_FRONT = 15;
const int ECHO_RIGHT = 16;
const int ECHO_LEFT = 17;
const int ECHO_REAR = 18;

// MPU6050 I2C
const int SDA_PIN = 41;
const int SCL_PIN = 42;

//RX-TX to N8R2 esp32
const int RX_PIN = 47;
const int TX_PIN = 20;

// --- Calibration constants ---
const float wheelCircumference = 31.4; // cm
const int ticksPerRev = 48;
const float tickToCm = wheelCircumference / ticksPerRev;
const float wheelbase = 31.75; // cm
const float gyroScale = 131.0;   // LSB/°/s for ±250 range
const float gyroOffset = -135.0; // measured offset

// --- Hall counters ---
volatile long RL_count = 0, RR_count = 0, FL_count = 0, FR_count = 0;

// --- NETWORKING ---
WiFiUDP udp;
unsigned int localPort = 4210;  // UDP port for receiving commands
char packetBuffer[255];         // Buffer for incoming UDP packets
 
// --- PID ---
float targetX = 50, targetY = 80; // example waypoint
float Kp_turn = 0.5;
float Kp_forward = 0.02;

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

// --- Send axis via UART ---
void sendAxis(int id, float val) {
  Serial2.print("AXIS ");
  Serial2.print(id);
  Serial2.print(" ");
  Serial2.println(val, 3);
}

// --- Setup ---
MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

  // Encoders
  pinMode(RL_D1, INPUT); pinMode(RL_D2, INPUT);
  pinMode(RR_D1, INPUT); pinMode(RR_D2, INPUT);
  pinMode(FL_D1, INPUT); pinMode(FL_D2, INPUT);
  pinMode(FR_D1, INPUT); pinMode(FR_D2, INPUT);
  pinMode(EL_D1, INPUT); pinMode(EL_D2, INPUT);
  pinMode(ER_D1, INPUT); pinMode(ER_D2, INPUT);

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
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 bağlantı hatası!");
    while (1);
  } else {
    Serial.println("MPU6050 bağlantısı başarılı.");
  }

  // WiFi
  WiFi.begin("Fox-2", "Kyra2bin9");
  Serial.print("WiFi Baglaniyor");
  int timeout = 0;
  while (WiFi.status() != WL_CONNECTED && timeout < 20) {
    delay(500);
    Serial.print(".");
    timeout++;
  }
  Serial.println("\nWiFi OK!");
  Serial.print("IP: "); Serial.println(WiFi.localIP());
  udp.begin(localPort);
  Serial.println("UDP Basladi");
}


// --- Calibration constants ---
const float wheelCircumference = 31.4; // cm
const int ticksPerRev = 48;
const float tickToCm = wheelCircumference / ticksPerRev;
const float wheelbase = 31.75; // cm
const float gyroScale = 131.0;   // LSB/°/s for ±250 range
const float gyroOffset = -135.0; // measured offset

// --- Hall counters ---
volatile long RL_count = 0, RR_count = 0, FL_count = 0, FR_count = 0;

// --- NETWORKING ---
WiFiUDP udp;
unsigned int localPort = 4210;  // UDP port for receiving commands
char packetBuffer[255];         // Buffer for incoming UDP packets

// --- PID ---
float targetX = 50, targetY = 80; // example waypoint
float Kp_turn = 0.5;
float Kp_forward = 0.02;

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

// --- Send axis via UART ---
void sendAxis(int id, float val) {
  Serial2.print("AXIS ");
  Serial2.print(id);
  Serial2.print(" ");
  Serial2.println(val, 3);
}

// --- Setup ---
MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

  // Encoders
  pinMode(RL_D1, INPUT); pinMode(RL_D2, INPUT);
  pinMode(RR_D1, INPUT); pinMode(RR_D2, INPUT);
  pinMode(FL_D1, INPUT); pinMode(FL_D2, INPUT);
  pinMode(FR_D1, INPUT); pinMode(FR_D2, INPUT);
  pinMode(EL_D1, INPUT); pinMode(EL_D2, INPUT);
  pinMode(ER_D1, INPUT); pinMode(ER_D2, INPUT);

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
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 bağlantı hatası!");
    while (1);
  } else {
    Serial.println("MPU6050 bağlantısı başarılı.");
  }

  // WiFi
  WiFi.begin("Fox-2", "Kyra2bin9");
  Serial.print("WiFi Baglaniyor");
  int timeout = 0;
  while (WiFi.status() != WL_CONNECTED && timeout < 20) {
    delay(500);
    Serial.print(".");
    timeout++;
  }
  Serial.println("\nWiFi OK!");
  Serial.print("IP: "); Serial.println(WiFi.localIP());
  udp.begin(localPort);
  Serial.println("UDP Basladi");
}

// --- Loop ---
void loop() {
  static unsigned long startTime = millis();
  static unsigned long lastTime = millis();

  if (millis() - startTime < 60000) { // PID mode for first 60 seconds
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    lastTime = now;

    // Gyro
    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);

    // Store current counts and reset
    long rl = RL_count; RL_count = 0;
    long rr = RR_count; RR_count = 0;
    long fl = FL_count; FL_count = 0;
    long fr = FR_count; FR_count = 0;

    // Prediction step
    kalmanPredict(dt, gz, rl, rr, fl, fr);

    // Median-filtered ultrasonic readings
    float distFront = medianFilter(ECHO_FRONT);
    float distRear  = medianFilter(ECHO_REAR);
    float distLeft  = medianFilter(ECHO_LEFT);
    float distRight = medianFilter(ECHO_RIGHT);

    // --- Auto-localization ---
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

    // Compute movement towards target
    float dx = targetX - X(0);
    float dy = targetY - X(1);
    float distance = sqrt(dx*dx + dy*dy);

    if (distance > 1.0) { // threshold
      float angleToTarget = atan2(dy, dx);
      float angleError = angleToTarget - X(2);

      // Normalize angleError to -PI to PI
      while (angleError > PI) angleError -= 2*PI;
      while (angleError < -PI) angleError += 2*PI;

      // PID for turn
      float turnControl = Kp_turn * angleError;
      turnControl = constrain(turnControl, -1.0, 1.0);

      // Forward speed proportional to distance
      float forwardSpeed = Kp_forward * distance;
      forwardSpeed = constrain(forwardSpeed, -1.0, 1.0);

      // Send axis commands
      sendAxis(5, forwardSpeed); // forward
      sendAxis(0, turnControl);  // turn
      sendAxis(2, 0);            // strafe
      sendAxis(4, 0);            // backward
    } else {
      // Stop
      sendAxis(5, 0);
      sendAxis(0, 0);
      sendAxis(2, 0);
      sendAxis(4, 0);
    }

    // Log fused state
    Serial.print("X: "); Serial.print(X(0));
    Serial.print(" Y: "); Serial.print(X(1));
    Serial.print(" Theta: "); Serial.print(X(2));
    Serial.print(" V: "); Serial.print(X(3));
    Serial.print(" Omega: "); Serial.println(X(4));

    delay(50);
  } else { // UDP mode after 60 seconds
    // Check WiFi connection
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi Koptu, Restart...");
      ESP.restart();
    }

    // Check for incoming UDP packets
    int packetSize = udp.parsePacket();
    if (packetSize) {
      int len = udp.read(packetBuffer, 255);
      if (len > 0 && len < 255) {
        packetBuffer[len] = 0;
        String msg = String(packetBuffer);
        Serial2.println(msg); // Forward to N8R2
      }
    }

    delay(10); // Small delay in UDP mode
  }
}