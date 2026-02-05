/*
 * UNIFIED MASTER CONTROLLER - ESP32-S3-N16R8
 * Combines strengths of PID_N16R8_MASTER, dungecemaster, and PIDberk
 * 
 * Features:
 * - Dual-core tasks: Core0 WiFi, Core1 Sensors + Navigation
 * - Robust Kalman filter with slip detection
 * - PID waypoint navigation (first 60s), then teleop via WiFi
 * - Elevator safety limits (pulse thresholds)
 * - UART communication with slave board
 * - Debug toggles for RPM and ultrasonic sensors
 */

#include <Arduino.h>
#include <Wire.h>
#include "MPU6050.h"
#include <BasicLinearAlgebra.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <algorithm>

using namespace BLA;

// ================= USER CONFIG =================
const char* ssid = "Fox-2";
const char* password = "Kyra2bin9";
const int localPort = 4210;

// Autonomous mode duration
const unsigned long AUTONOMOUS_DURATION = 60000; // 60s

// Waypoints
struct Waypoint { float x; float y; };
Waypoint waypoints[] = {
  {0,0}, {100,0}, {100,100}, {0,0}
};
int waypointCount = sizeof(waypoints)/sizeof(Waypoint);
int currentWaypoint = 0;

// ================= PIN DEFINITIONS =================
// Encoders
#define RL_A 1
#define RL_B 2
#define RR_A 4
#define RR_B 5
#define FL_A 6
#define FL_B 7
#define FR_A 8
#define FR_B 9

// Elevator
#define EL_A 10
#define EL_B 11
#define ER_A 12
#define ER_B 13

// Ultrasonic
#define TRIG_PIN 14
#define ECHO_FRONT 15
#define ECHO_RIGHT 16
#define ECHO_LEFT 17
#define ECHO_REAR 18

// I2C & UART
#define SDA_PIN 41
#define SCL_PIN 42
#define RX_PIN 47
#define TX_PIN 20

// ================= CONSTANTS =================
const float WHEEL_CIRCUMFERENCE = 31.4;
const int TICKS_PER_REV = 48;
const float TICK_TO_CM = WHEEL_CIRCUMFERENCE / TICKS_PER_REV;
const float WHEELBASE = 31.75;
const float GYRO_SCALE = 131.0;
const float GYRO_OFFSET = -135.0;
bool debugMode = false;
bool showRPM = false;
bool showUltrasonic = false;
const int ELEVATOR_MAX_PULSES = 133;
const int ELEVATOR_MIN_PULSES = 0;

// ================= STATE VARIABLES =================
volatile long cntRL=0, cntRR=0, cntFL=0, cntFR=0;
volatile long cntEL=0, cntER=0;

unsigned long startTime;
bool isAutonomous = true;

// WiFi
WiFiUDP udp;
char packetBuffer[255];

// Teleop inputs
float axis0=0, axis2=0, axis4=0, axis5=0;
int btn6=0, btn12=0, btn13=0;

// Kalman state
BLA::Matrix<5,1> X = {0,0,0,0,0};
BLA::Matrix<5,5> P = BLA::Identity<5,5>()*0.1;
BLA::Matrix<5,5> Q = BLA::Identity<5,5>()*0.01;

// Debug toggles
bool showRPM=false, showUltrasonic=false;

// ================= ISRs =================
void IRAM_ATTR isrRL(){ if(digitalRead(RL_A)==digitalRead(RL_B)) cntRL++; else cntRL--; }
void IRAM_ATTR isrRR(){ if(digitalRead(RR_A)==digitalRead(RR_B)) cntRR++; else cntRR--; }
void IRAM_ATTR isrFL(){ if(digitalRead(FL_A)==digitalRead(FL_B)) cntFL++; else cntFL--; }
void IRAM_ATTR isrFR(){ if(digitalRead(FR_A)==digitalRead(FR_B)) cntFR++; else cntFR--; }
void IRAM_ATTR isrEL(){ if(digitalRead(EL_A)==digitalRead(EL_B)) cntEL++; else cntEL--; }
void IRAM_ATTR isrER(){ if(digitalRead(ER_A)==digitalRead(ER_B)) cntER++; else cntER--; }

// ================= FUNCTIONS =================
float calcRPM(long pulses){ return (pulses/(float)TICKS_PER_REV)*60.0; }

long readHCSR(int echoPin){
  digitalWrite(TRIG_PIN,LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN,HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN,LOW);
  long duration=pulseIn(echoPin,HIGH,30000);
  if(duration==0) return -1;
  return duration*0.034/2;
}

float medianFilterUltrasonic(int echoPin){
  float vals[5];
  for(int i=0;i<5;i++){ vals[i]=readHCSR(echoPin); delay(5); }
  std::sort(vals,vals+5);
  return vals[2];
}

void sendToSlave(float f,float s,float t,int servo,int el_up,int el_down){
  Serial2.printf("MOV %.2f,%.2f,%.2f,%d,%d,%d\n",f,s,t,servo,el_up,el_down);
}

// Kalman prediction with slip detection
void kalmanPredict(float dt,int16_t gz,long RL,long RR,long FL,long FR){
  float dL=((RL+FL)/2.0)*TICK_TO_CM;
  float dR=((RR+FR)/2.0)*TICK_TO_CM;
  float d=(dL+dR)/2.0;
  float dTheta=(dR-dL)/WHEELBASE;

  if(abs(dL-dR)>10.0){
    dTheta=((gz-GYRO_OFFSET)/GYRO_SCALE)*(PI/180.0)*dt;
  }

  float gyroZ=(gz-GYRO_OFFSET)/GYRO_SCALE;
  float gyroZ_rad=gyroZ*(PI/180.0);

  X(0)+=d*cos(X(2));
  X(1)+=d*sin(X(2));
  X(2)+=dTheta+gyroZ_rad*dt;
  X(3)=d/dt;
  X(4)=gyroZ_rad;
}

// Navigation PID
void navigationToPID(float tx,float ty){
  float ex=tx-X(0), ey=ty-X(1);
  float dist=sqrt(ex*ex+ey*ey);
  float desired=atan2(ey,ex);
  float angleErr=desired-X(2);
  while(angleErr>PI) angleErr-=2*PI;
  while(angleErr<-PI) angleErr+=2*PI;

  float f=0.02*dist;
  float t=0.5*angleErr;
  f=constrain(f,-1.0,1.0);
  t=constrain(t,-1.0,1.0);

  sendToSlave(f,0,t,btn6,btn13,btn12);
}

// ================= TASKS =================
void TaskWiFi(void*){
  WiFi.begin(ssid,password);
  while(WiFi.status()!=WL_CONNECTED){ delay(500); }
  udp.begin(localPort);

  while(true){
    int packetSize=udp.parsePacket();
    if(packetSize){
      int len=udp.read(packetBuffer,255);
      if(len>0) packetBuffer[len]=0;
      String msg(packetBuffer);

      if(msg.startsWith("AXIS")){
        int id=msg.substring(5,6).toInt();
        float val=msg.substring(7).toFloat();
        if(id==0) axis0=val;
        if(id==2) axis2=val;
        if(id==4) axis4=val;
        if(id==5) axis5=val;
      }
      if(msg.startsWith("BUTTON")){
        int id=msg.substring(7,9).toInt();
        int val=msg.substring(10).toInt();
        if(id==6) btn6=val;
        if(id==12) btn12=val;
        if(id==13) btn13=val;
      }
    }
    vTaskDelay(10/portTICK_PERIOD_MS);
  }
}

// ================= SETUP =================
MPU6050 mpu;
void setup(){
  Serial.begin(115200);
  Serial2.begin(115200,SERIAL_8N1,RX_PIN,TX_PIN);

  pinMode(RL_A,INPUT); pinMode(RL_B,INPUT);
  pinMode(RR_A,INPUT); pinMode(RR_B,INPUT);
  pinMode(FL_A,INPUT); pinMode(FL_B,INPUT);
  pinMode(FR_A,INPUT); pin

    // Encoder interrupts
  attachInterrupt(digitalPinToInterrupt(RL_A), isrRL, RISING);
  attachInterrupt(digitalPinToInterrupt(RR_A), isrRR, RISING);
  attachInterrupt(digitalPinToInterrupt(FL_A), isrFL, RISING);
  attachInterrupt(digitalPinToInterrupt(FR_A), isrFR, RISING);
  attachInterrupt(digitalPinToInterrupt(EL_A), isrEL, RISING);
  attachInterrupt(digitalPinToInterrupt(ER_A), isrER, RISING);

  // Ultrasonic pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  pinMode(ECHO_RIGHT, INPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(ECHO_REAR, INPUT);

  // MPU init
  Wire.begin(SDA_PIN, SCL_PIN);
  mpu.initialize();
  if(!mpu.testConnection()){
    Serial.println("MPU6050 not found!");
    while(1);
  }

  // WiFi task on Core0
  xTaskCreatePinnedToCore(TaskWiFi, "WiFiTask", 8192, NULL, 1, NULL, 0);

  Serial.println("MASTER CONTROLLER READY");
  startTime = millis();
}

void loop() {
  static unsigned long lastLoop = 0;
  unsigned long now = millis();
  float dt = (now - lastLoop) / 1000.0;
  if (dt < 0.01) { delay(1); return; }
  lastLoop = now;

  // === SERIAL COMMAND PARSING (already dropped in by you) ===
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.equalsIgnoreCase("DEBUGON")) {
      debugMode = true;
      Serial.println("Debugging mode enabled");
    } else if (cmd.equalsIgnoreCase("DEBUGOFF")) {
      debugMode = false;
      Serial.println("Normal robot loop enabled");
    }

    // RPM toggles
    else if (cmd.equalsIgnoreCase("RPMON")) showRPM = true;
    else if (cmd.equalsIgnoreCase("RPMOFF")) showRPM = false;

    // Ultrasonic toggles
    else if (cmd.equalsIgnoreCase("USON")) showUltrasonic = true;
    else if (cmd.equalsIgnoreCase("USOFF")) showUltrasonic = false;
  }

  // === MODE SWITCH ===
  if (debugMode) {
    // --- DEBUGGING MODE ---
    if (showRPM) {
      Serial.printf("RPM RL=%.1f RR=%.1f FL=%.1f FR=%.1f\n",
        calcRPM(cntRL), calcRPM(cntRR), calcRPM(cntFL), calcRPM(cntFR));
    }
    if (showUltrasonic) {
      Serial.printf("US Front=%.1f Right=%.1f Left=%.1f Rear=%.1f\n",
        medianFilterUltrasonic(ECHO_FRONT),
        medianFilterUltrasonic(ECHO_RIGHT),
        medianFilterUltrasonic(ECHO_LEFT),
        medianFilterUltrasonic(ECHO_REAR));
    }
    delay(500); // slower loop for readability
  } else {
    // --- NORMAL ROBOT LOOP ---
    // Gyro read
    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);

    // Kalman prediction
    kalmanPredict(dt, gz, cntRL, cntRR, cntFL, cntFR);
    cntRL = cntRR = cntFL = cntFR = 0; // reset after use

    // Mode switch timing
    if (now - startTime > AUTONOMOUS_DURATION) isAutonomous = false;

    if (isAutonomous) {
      // Autonomous PID navigation
      if (currentWaypoint < waypointCount) {
        navigationToPID(waypoints[currentWaypoint].x, waypoints[currentWaypoint].y);
        float dx = waypoints[currentWaypoint].x - X(0);
        float dy = waypoints[currentWaypoint].y - X(1);
        if (sqrt(dx*dx + dy*dy) < 5.0) currentWaypoint++;
      } else {
        sendToSlave(0, 0, 0, btn6, 0, 0); // stop
      }
    } else {
      // Teleop mode (priority chain: forward/back > strafe > turn)
      float f = 0, s = 0, t = 0;
      if (axis4 > 0.1) { f = 1.0; }
      else if (axis5 > 0.1) { f = -1.0; }
      else if (abs(axis0) > 0.1) { s = axis0; }
      else if (abs(axis2) > 0.1) { t = axis2; }
      else { f = s = t = 0; }

      // Elevator safety
      int el_up = 0, el_down = 0;
      if (btn13 == 1 && cntEL < ELEVATOR_MAX_PULSES) el_up = 1;
      if (btn12 == 1 && cntEL > ELEVATOR_MIN_PULSES) el_down = 1;

      sendToSlave(f, s, t, btn6, el_up, el_down);
    }

    delay(20); // ~50Hz loop
  }
}