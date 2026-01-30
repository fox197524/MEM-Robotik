/*
 * ROBOT AUTONOMOUS NAVIGATION (AUTO-DETECT START + HARDCODED MISSION)
 * 1. Detects Start Position using Back-Left or Back-Right Sonar.
 * 2. Navigates 5 Hardcoded Waypoints (Source -> Sarnic).
 */

#include <Arduino.h>
#include <MPU6050.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <vector>

// ======================= CONFIGURATION =======================
// --- PHYSICAL ROBOT MEASUREMENTS (MEASURE THESE!) ---
#define WHEEL_DIAMETER_M 0.08    
#define ENC_TICKS_PER_REV 600.0  
#define DIST_PER_TICK (WHEEL_DIAMETER_M * PI / ENC_TICKS_PER_REV)
#define LIFT_METERS_PER_TICK 0.0001 

// --- NAVIGATION SETTINGS ---
#define MAP_WIDTH_M 3.0          // Width of the field in meters
#define WAYPOINT_TOLERANCE 0.10  // 10cm tolerance
#define MAX_SPEED 150            
#define TURN_SPEED 120           
#define LIFT_SPEED 150           

// ======================= PIN DEFINITIONS =======================
// HC-SR04 Sensors
#define HC_TRIG_F  3   // Front
#define HC_ECHO_F  46
#define HC_TRIG_BL 7   // Back-Left (Used for Start Detection)
#define HC_ECHO_BL 21
#define HC_TRIG_BR 15  // Back-Right (Used for Start Detection)
#define HC_ECHO_BR 14

// Encoders
#define ENC_FL_CLK 4
#define ENC_FL_DT 5
#define ENC_FR_CLK 12
#define ENC_FR_DT 13
#define ENC_LU_CLK 10
#define ENC_LU_DT 11
#define ENC_LD_CLK 40
#define ENC_LD_DT 39

// I2C & UART
#define I2C_SDA 8
#define I2C_SCL 9
#define UART1_TX 17
#define UART1_RX 18

// ======================= DATA STRUCTURES =======================
struct Pose {
  float x;       
  float y;       
  float theta;   
  float height;  
};

struct Waypoint {
  float x;
  float y;
  float height;
  String name; // Just for debugging
};

// ======================= GLOBALS =======================
MPU6050 mpu;
WiFiUDP udp;

volatile Pose currentPose = {0,0,0,0}; 
std::vector<Waypoint> missionPlan;
int currentWaypointIndex = 0;
bool missionComplete = false;

// Counters
volatile long enc_fl_cnt = 0, enc_fr_cnt = 0;
volatile long enc_lu_cnt = 0, enc_ld_cnt = 0;
volatile float gyro_z_offset = 0;

SemaphoreHandle_t mutexPose;

// WiFi
const char* ssid = "Ben";
const char* password = "Gisherman2010";

// ======================= FUNCTION PROTOTYPES =======================
void initHardware();
void detectStartAndLoadMission();
void odometryTask(void *pvParam);
void navigationTask(void *pvParam);
void networkingTask(void *pvParam);
float readSonar(int trig, int echo);
void sendMotorCommand(int left, int right);
void sendLiftCommand(int speed);

// ISRs
void IRAM_ATTR isr_enc_fl() { enc_fl_cnt++; }
void IRAM_ATTR isr_enc_fr() { enc_fr_cnt++; }
void IRAM_ATTR isr_enc_lu() { enc_lu_cnt++; }
void IRAM_ATTR isr_enc_ld() { enc_ld_cnt++; }

// ======================= SETUP =======================
void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, UART1_RX, UART1_TX);
  delay(1000);

  Serial.println("--- ROBOT INITIALIZING ---");

  mutexPose = xSemaphoreCreateMutex();
  initHardware();

  // === CRITICAL STEP: DETECT START POSITION ===
  detectStartAndLoadMission();

  // Start Tasks
  xTaskCreatePinnedToCore(networkingTask, "Net", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(navigationTask, "Nav", 8192, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(odometryTask, "Odom", 4096, NULL, 5, NULL, 1);

  Serial.println("--- MISSION STARTED ---");
  vTaskDelete(NULL);
}

// ======================= STARTUP LOGIC =======================
void detectStartAndLoadMission() {
  Serial.println("Detecting Starting Position...");
  
  // Read Back-Left and Back-Right sensors
  // We take 3 readings and average to be sure
  float d_bl = 0, d_br = 0;
  for(int i=0; i<3; i++) {
    d_bl += readSonar(HC_TRIG_BL, HC_ECHO_BL);
    d_br += readSonar(HC_TRIG_BR, HC_ECHO_BR);
    delay(50);
  }
  d_bl /= 3.0;
  d_br /= 3.0;

  Serial.printf("Sonar Readings: BL=%.2fm, BR=%.2fm\n", d_bl, d_br);

  // Logic: Check which side is close to a wall (< 30cm)
  if (d_bl < 0.30) {
    // === SCENARIO 1: BOTTOM-LEFT START ===
    Serial.println("DETECTED: BOTTOM-LEFT START");
    
    xSemaphoreTake(mutexPose, portMAX_DELAY);
    currentPose.x = 0.20; // 20cm padding from wall
    currentPose.y = 0.20;
    currentPose.theta = PI / 2.0; // Facing UP (North)
    xSemaphoreGive(mutexPose);

  } else if (d_br < 0.30) {
    // === SCENARIO 2: BOTTOM-RIGHT START ===
    Serial.println("DETECTED: BOTTOM-RIGHT START");
    
    xSemaphoreTake(mutexPose, portMAX_DELAY);
    currentPose.x = MAP_WIDTH_M - 0.20; // e.g., 2.80m
    currentPose.y = 0.20;
    currentPose.theta = PI / 2.0; // Facing UP (North)
    xSemaphoreGive(mutexPose);
    
  } else {
    // Default Fallback
    Serial.println("WARNING: No wall detected. Defaulting to (0,0)");
    currentPose.x = 0; currentPose.y = 0;
  }

  // === LOAD HARDCODED MISSION (5 POINTS) ===
  // Coordinates: X (meters), Y (meters), Height (meters)
  
  // 1. Clear zone (move away from wall)
  missionPlan.push_back({currentPose.x, 0.5, 0.0, "Clear Wall"});
  
  // 2. The "SOURCE" (Made up: Center Top)
  missionPlan.push_back({1.5, 2.5, 0.0, "Source"});
  
  // 3. Approach "SARNIC"
  missionPlan.push_back({0.5, 1.0, 0.0, "Approach Sarnic"});
  
  // 4. At "SARNIC" (Made up: Left Side) - LIFT UP
  missionPlan.push_back({0.5, 1.5, 0.3, "Sarnic (Lift Up)"});
  
  // 5. Return Home (Based on start)
  missionPlan.push_back({currentPose.x, 0.2, 0.0, "Return Home"});

  Serial.println("Mission Plan Loaded:");
  for(auto &wp : missionPlan) {
    Serial.printf(" -> %s (%.1f, %.1f, H:%.2f)\n", wp.name.c_str(), wp.x, wp.y, wp.height);
  }
}

// ======================= CORE 1: ODOMETRY =======================
void odometryTask(void *pvParam) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 10 / portTICK_PERIOD_MS;
  
  long old_fl=0, old_fr=0;
  unsigned long lastTime = micros();

  while(1) {
    unsigned long now = micros();
    float dt = (now - lastTime) / 1000000.0;
    lastTime = now;
    if(dt <= 0) dt = 0.001;

    // Gyro
    int16_t raw_gz = mpu.getRotationZ();
    float gyro_vel = (raw_gz - gyro_z_offset) / 131.0; 
    gyro_vel = gyro_vel * DEG_TO_RAD; 

    // Encoders
    long curr_fl = enc_fl_cnt;
    long curr_fr = enc_fr_cnt; 
    float d_left = (curr_fl - old_fl) * DIST_PER_TICK;
    float d_right = (curr_fr - old_fr) * DIST_PER_TICK;
    float d_center = (d_left + d_right) / 2.0;
    old_fl = curr_fl; old_fr = curr_fr;

    // Lift
    float h = (enc_lu_cnt - enc_ld_cnt) * LIFT_METERS_PER_TICK;

    xSemaphoreTake(mutexPose, portMAX_DELAY);
    currentPose.theta += gyro_vel * dt;
    
    // Normalize Theta
    if(currentPose.theta > PI) currentPose.theta -= TWO_PI;
    else if(currentPose.theta < -PI) currentPose.theta += TWO_PI;

    currentPose.x += d_center * cos(currentPose.theta);
    currentPose.y += d_center * sin(currentPose.theta);
    currentPose.height = h;
    xSemaphoreGive(mutexPose);

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// ======================= CORE 0: NAVIGATION =======================
void navigationTask(void *pvParam) {
  while(1) {
    if (missionComplete || missionPlan.empty()) {
      sendMotorCommand(0, 0); sendLiftCommand(0);
      vTaskDelay(1000); continue;
    }

    // 1. Collision Avoidance
    float dist = readSonar(HC_TRIG_F, HC_ECHO_F);
    if (dist < 0.20) { // Stop if 20cm from obstacle
      sendMotorCommand(0, 0);
      continue;
    }

    // 2. Get Target
    Waypoint target = missionPlan[currentWaypointIndex];
    
    // 3. Get Pose
    Pose myPose;
    xSemaphoreTake(mutexPose, portMAX_DELAY);
    myPose = currentPose;
    xSemaphoreGive(mutexPose);

    // 4. Errors
    float dx = target.x - myPose.x;
    float dy = target.y - myPose.y;
    float distToTarget = sqrt(dx*dx + dy*dy);
    float headingToTarget = atan2(dy, dx);
    float headingError = headingToTarget - myPose.theta;
    
    while (headingError > PI) headingError -= TWO_PI;
    while (headingError < -PI) headingError += TWO_PI;
    
    float liftError = target.height - myPose.height;

    // 5. Logic
    int leftPWM = 0, rightPWM = 0, liftPWM = 0;

    // Has it reached the XY coordinate?
    if (distToTarget < WAYPOINT_TOLERANCE) {
      
      // Has it reached the Height?
      if (abs(liftError) < 0.02) { // 2cm tolerance
        Serial.printf("Reached: %s\n", target.name.c_str());
        currentWaypointIndex++;
        if (currentWaypointIndex >= missionPlan.size()) {
          missionComplete = true;
          Serial.println("--- FINISHED ---");
        }
      } else {
        // Stop motors, adjust lift
        leftPWM = 0; rightPWM = 0;
        liftPWM = (liftError > 0) ? LIFT_SPEED : -LIFT_SPEED;
      }
      
    } else {
      // Move towards target
      // If heading error is large (>20 deg), turn in place
      if (abs(headingError) > 0.35) {
         int dir = (headingError > 0) ? 1 : -1; // 1 = Left, -1 = Right
         leftPWM = -dir * TURN_SPEED;
         rightPWM = dir * TURN_SPEED;
      } else {
         // Drive forward with correction
         leftPWM = MAX_SPEED - (headingError * 150);
         rightPWM = MAX_SPEED + (headingError * 150);
      }
      
      // Adjust lift while moving? (Optional, safer to do it when stopped)
      // For now, lift stops while driving to save power/stability
      liftPWM = 0;
    }

    sendMotorCommand(leftPWM, rightPWM);
    sendLiftCommand(liftPWM);

    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

// ======================= HARDWARE INIT & HELPERS =======================
void initHardware() {
  Wire.begin(I2C_SDA, I2C_SCL);
  mpu.initialize();
  
  // Calibrate Gyro
  long sum = 0;
  for(int i=0; i<200; i++) { sum += mpu.getRotationZ(); delay(2); }
  gyro_z_offset = sum / 200.0;

  // Pins
  pinMode(HC_TRIG_F, OUTPUT); pinMode(HC_ECHO_F, INPUT);
  pinMode(HC_TRIG_BL, OUTPUT); pinMode(HC_ECHO_BL, INPUT);
  pinMode(HC_TRIG_BR, OUTPUT); pinMode(HC_ECHO_BR, INPUT);

  pinMode(ENC_FL_CLK, INPUT_PULLUP); attachInterrupt(ENC_FL_CLK, isr_enc_fl, RISING);
  pinMode(ENC_FR_CLK, INPUT_PULLUP); attachInterrupt(ENC_FR_CLK, isr_enc_fr, RISING);
  pinMode(ENC_LU_CLK, INPUT_PULLUP); attachInterrupt(ENC_LU_CLK, isr_enc_lu, RISING);
  pinMode(ENC_LD_CLK, INPUT_PULLUP); attachInterrupt(ENC_LD_CLK, isr_enc_ld, RISING);
}

void sendMotorCommand(int left, int right) {
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);
  Serial2.printf("FL %d\nFR %d\nBL %d\nBR %d\n", left, right, left, right);
}

void sendLiftCommand(int speed) {
  speed = constrain(speed, -255, 255);
  Serial2.printf("LU %d\n", speed);
}

float readSonar(int trig, int echo) {
  digitalWrite(trig, LOW); delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long dur = pulseIn(echo, HIGH, 25000); 
  if(dur == 0) return 5.0; 
  return (dur * 0.034 / 2.0) / 100.0; 
}

void networkingTask(void *pvParam) {
  WiFi.begin(ssid, password);
  while(WiFi.status() != WL_CONNECTED) vTaskDelay(500);
  udp.begin(4210);
  while(1) {
    // Telemetry: X, Y, Height, Current Waypoint Index
    xSemaphoreTake(mutexPose, portMAX_DELAY);
    String t = "POS:" + String(currentPose.x) + "," + String(currentPose.y) + 
               " H:" + String(currentPose.height) + " WP:" + String(currentWaypointIndex);
    xSemaphoreGive(mutexPose);
    // udp.beginPacket(..., ...); udp.print(t); udp.endPacket();
    vTaskDelay(200);
  }
}