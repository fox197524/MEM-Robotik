/*
 * ROBOT LOCALIZATION SYSTEM WITH ELEVATOR (ESP32-S3)
 * Features: MPU6050, 4x HC-SR04, 4x Drive Encoders, 2x Lift Encoders, SD Card Map, Particle Filter
 */

#include <Arduino.h>
#include <MPU6050.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <SPI.h>
#include <SD.h>
#include <FS.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// ======================= CONFIGURATION =======================
#define SD_CS_PIN 5  
#define MAP_FILENAME "/map.bin"

// Robot Physical Constants (Drive)
#define WHEEL_DIAMETER_M 0.1   
#define ENC_TICKS_PER_REV 48.0 
#define DIST_PER_TICK (WHEEL_DIAMETER_M * PI / ENC_TICKS_PER_REV)

// Elevator Physical Constants
// ADJUST THIS: How many meters does the lift move per encoder tick?
// Example: 1 rotation = 5cm rise, 600 ticks per rotation -> 0.05 / 600
#define LIFT_METERS_PER_TICK 0.0001 // koyulacak

// Particle Filter Constants
#define NUM_PARTICLES 50
#define SENSOR_NOISE 0.15       
#define MOTION_NOISE_POS 0.02   
#define MOTION_NOISE_ANG 0.02   

// ======================= PIN DEFINITIONS =======================
// HC-SR04
#define HC_TRIG_1 3
#define HC_ECHO_1 46
#define HC_TRIG_2 6
#define HC_ECHO_2 47
#define HC_TRIG_3 7
#define HC_ECHO_3 21
#define HC_TRIG_4 15
#define HC_ECHO_4 14

// Drive Encoders
#define ENC_FL_CLK 4
#define ENC_FL_DT 5
#define ENC_FR_CLK 12
#define ENC_FR_DT 13
#define ENC_BL_CLK 37
#define ENC_BL_DT 38
#define ENC_BR_CLK 41
#define ENC_BR_DT 42

// Lift Encoders
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
  float x;       // Meters
  float y;       // Meters
  float theta;   // Radians
  float height;  // Meters (Elevator Height)
};

struct GlobalMap {
  uint16_t width = 3744;
  uint16_t height = 3600;
  float resolution = 1; 
  uint8_t* data;    
};

struct Particle {
  float x;
  float y;
  float theta;
  float weight;
};

// ======================= GLOBALS =======================
MPU6050 mpu;
WiFiUDP udp;
GlobalMap activeMap;

// State Variables
volatile Pose estimatedPose = {0,0,0,0};
volatile long enc_fl_cnt = 0, enc_fr_cnt = 0, enc_bl_cnt = 0, enc_br_cnt = 0;
volatile long enc_lu_cnt = 0, enc_ld_cnt = 0; // Lift counters
volatile float gyro_z_offset = 0;
Particle particles[NUM_PARTICLES];

// Synchronization
SemaphoreHandle_t mutexState;
SemaphoreHandle_t mutexMap;

// WiFi
const char* ssid = "Ben";
const char* password = "Gisherman2010";
unsigned int localPort = 4210;

// ======================= FUNCTION PROTOTYPES =======================
void initHardware();
void loadMapFromSD();
void initParticles();
void networkingTask(void *pvParam);
void controlTask(void *pvParam);
void localizationTask(void *pvParam);
float readSonar(int trig, int echo);
float rayCast(float x, float y, float theta);
float gaussian(float mu, float sigma, float x);

// Drive ISRs
void IRAM_ATTR isr_enc_fl() { enc_fl_cnt++; }
void IRAM_ATTR isr_enc_fr() { enc_fr_cnt++; }
void IRAM_ATTR isr_enc_bl() { enc_bl_cnt++; }
void IRAM_ATTR isr_enc_br() { enc_br_cnt++; }

// Lift ISRs
void IRAM_ATTR isr_enc_lu() { enc_lu_cnt++; }
void IRAM_ATTR isr_enc_ld() { enc_ld_cnt++; }

// ======================= SETUP =======================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("--- SYSTEM START ---");

  mutexState = xSemaphoreCreateMutex();
  mutexMap = xSemaphoreCreateMutex();

  initHardware();
  loadMapFromSD();
  initParticles();

  // Task Creation
  xTaskCreatePinnedToCore(networkingTask, "NetTask", 4096, NULL, 2, NULL, 0); // Core 0
  xTaskCreatePinnedToCore(controlTask, "CtrlTask", 4096, NULL, 5, NULL, 1);   // Core 1 (High Prio)
  xTaskCreatePinnedToCore(localizationTask, "LocTask", 8192, NULL, 3, NULL, 1); // Core 1 (Low Prio)

  Serial.println("--- TASKS RUNNING ---");
  vTaskDelete(NULL);
}

// ======================= HARDWARE INIT =======================
void initHardware() {
  // 1. I2C & MPU
  Wire.begin(I2C_SDA, I2C_SCL);
  mpu.initialize();
  if(!mpu.testConnection()) Serial.println("MPU connection failed");
  
  // Calibrate Gyro
  Serial.println("Calibrating Gyro...");
  long sum = 0;
  for(int i=0; i<100; i++) {
    sum += mpu.getRotationZ();
    delay(5);
  }
  gyro_z_offset = sum / 100.0;
  Serial.println("Gyro Calibrated.");

  // 2. Pins (Sonar)
  pinMode(HC_TRIG_1, OUTPUT); pinMode(HC_ECHO_1, INPUT);
  pinMode(HC_TRIG_2, OUTPUT); pinMode(HC_ECHO_2, INPUT);
  pinMode(HC_TRIG_3, OUTPUT); pinMode(HC_ECHO_3, INPUT);
  pinMode(HC_TRIG_4, OUTPUT); pinMode(HC_ECHO_4, INPUT);

  // 3. Pins (Drive Encoders)
  pinMode(ENC_FL_CLK, INPUT_PULLUP); attachInterrupt(ENC_FL_CLK, isr_enc_fl, RISING);
  pinMode(ENC_FR_CLK, INPUT_PULLUP); attachInterrupt(ENC_FR_CLK, isr_enc_fr, RISING);
  pinMode(ENC_BL_CLK, INPUT_PULLUP); attachInterrupt(ENC_BL_CLK, isr_enc_bl, RISING);
  pinMode(ENC_BR_CLK, INPUT_PULLUP); attachInterrupt(ENC_BR_CLK, isr_enc_br, RISING);

  // 4. Pins (Lift Encoders)
  pinMode(ENC_LU_CLK, INPUT_PULLUP); attachInterrupt(ENC_LU_CLK, isr_enc_lu, RISING);
  pinMode(ENC_LD_CLK, INPUT_PULLUP); attachInterrupt(ENC_LD_CLK, isr_enc_ld, RISING);

  // 5. SD Card
  if(!SD.begin(SD_CS_PIN)) {
    Serial.println("SD Card Mount Failed!");
  } else {
    Serial.println("SD Card Mounted.");
  }
}

// ======================= CORE 1: FAST CONTROL / ODOMETRY =======================
void controlTask(void *pvParam) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 20 / portTICK_PERIOD_MS; // 50Hz
  
  long old_fl=0, old_fr=0;
  unsigned long lastTime = micros();

  while(1) {
    // --- 1. Timing & Gyro ---
    unsigned long now = micros();
    float dt = (now - lastTime) / 1000000.0;
    lastTime = now;
    if(dt <= 0) dt = 0.001;

    int16_t raw_gz = mpu.getRotationZ();
    float gyro_vel = (raw_gz - gyro_z_offset) / 131.0; 
    gyro_vel = gyro_vel * DEG_TO_RAD; 

    // --- 2. Drive Odometry ---
    long curr_fl = enc_fl_cnt;
    long curr_fr = enc_fr_cnt; 
    
    float d_left = (curr_fl - old_fl) * DIST_PER_TICK;
    float d_right = (curr_fr - old_fr) * DIST_PER_TICK;
    float d_center = (d_left + d_right) / 2.0;

    old_fl = curr_fl;
    old_fr = curr_fr;

    // --- 3. Lift Height Calculation ---
    // We assume LU counts UP and LD counts DOWN
    long curr_lu = enc_lu_cnt;
    long curr_ld = enc_ld_cnt;
    float currentHeight = (curr_lu - curr_ld) * LIFT_METERS_PER_TICK;

    // --- 4. Update State ---
    xSemaphoreTake(mutexState, portMAX_DELAY);
    
    // Update Pose
    estimatedPose.x += d_center * cos(estimatedPose.theta);
    estimatedPose.y += d_center * sin(estimatedPose.theta);
    estimatedPose.theta += gyro_vel * dt;
    estimatedPose.height = currentHeight; // Update Height
    
    if(estimatedPose.theta > PI) estimatedPose.theta -= TWO_PI;
    if(estimatedPose.theta < -PI) estimatedPose.theta += TWO_PI;
    
    // Update Particles (Prediction)
    for(int i=0; i<NUM_PARTICLES; i++) {
        float noise_d = d_center + ((random(100)-50)/1000.0 * MOTION_NOISE_POS);
        float noise_t = (gyro_vel * dt) + ((random(100)-50)/1000.0 * MOTION_NOISE_ANG);
        
        particles[i].x += noise_d * cos(particles[i].theta);
        particles[i].y += noise_d * sin(particles[i].theta);
        particles[i].theta += noise_t;
        
        if(particles[i].theta > PI) particles[i].theta -= TWO_PI;
        else if(particles[i].theta < -PI) particles[i].theta += TWO_PI;
    }
    xSemaphoreGive(mutexState);

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// ======================= CORE 1: LOCALIZATION (SLOW) =======================
// No changes needed here, Particle Filter works on 2D plane (X, Y)
void localizationTask(void *pvParam) {
  while(1) {
    float s_front = readSonar(HC_TRIG_1, HC_ECHO_1);
    float s_right = readSonar(HC_TRIG_2, HC_ECHO_2);

    xSemaphoreTake(mutexMap, portMAX_DELAY);
    xSemaphoreTake(mutexState, portMAX_DELAY);

    float totalWeight = 0;
    
    for(int i=0; i<NUM_PARTICLES; i++) {
        if(particles[i].x < 0 || particles[i].x >= activeMap.width*activeMap.resolution ||
           particles[i].y < 0 || particles[i].y >= activeMap.height*activeMap.resolution) {
            particles[i].weight = 0;
            continue;
        }

        float exp_front = rayCast(particles[i].x, particles[i].y, particles[i].theta);
        float exp_right = rayCast(particles[i].x, particles[i].y, particles[i].theta - PI/2);

        float p_front = gaussian(exp_front, SENSOR_NOISE, s_front);
        float p_right = gaussian(exp_right, SENSOR_NOISE, s_right);

        particles[i].weight = p_front * p_right;
        totalWeight += particles[i].weight;
    }

    if(totalWeight > 0) {
        Particle newParticles[NUM_PARTICLES];
        float avgX=0, avgY=0;
        
        for(int i=0; i<NUM_PARTICLES; i++) particles[i].weight /= totalWeight;

        float beta = 0;
        int index = random(NUM_PARTICLES);
        float maxW = 0;
        for(int i=0; i<NUM_PARTICLES; i++) if(particles[i].weight > maxW) maxW = particles[i].weight;

        for(int i=0; i<NUM_PARTICLES; i++) {
            beta += ((float)random(1000)/1000.0) * 2.0 * maxW;
            while(beta > particles[index].weight) {
                beta -= particles[index].weight;
                index = (index + 1) % NUM_PARTICLES;
            }
            newParticles[i] = particles[index];
            avgX += newParticles[i].x;
            avgY += newParticles[i].y;
        }
        memcpy(particles, newParticles, sizeof(particles));
        
        estimatedPose.x = avgX / NUM_PARTICLES;
        estimatedPose.y = avgY / NUM_PARTICLES;
    }

    xSemaphoreGive(mutexState);
    xSemaphoreGive(mutexMap);

    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}

// ======================= CORE 0: NETWORKING =======================
void networkingTask(void *pvParam) {
  WiFi.begin(ssid, password);
  while(WiFi.status() != WL_CONNECTED) vTaskDelay(500);
  udp.begin(localPort);
  
  while(1) {
    int packetSize = udp.parsePacket();
    if(packetSize) {
       char packet[255];
       udp.read(packet, 255);
       // Handle Commands
    }

    // Broadcast Position AND Height
    xSemaphoreTake(mutexState, portMAX_DELAY);
    String tel = "POS " + String(estimatedPose.x, 2) + " " 
               + String(estimatedPose.y, 2) + " " 
               + String(estimatedPose.theta, 2) + " "
               + String(estimatedPose.height, 3); // Added Height
    xSemaphoreGive(mutexState);
    
    // Broadcast usage (uncomment to send)
    // udp.beginPacket(IPAddress(255,255,255,255), 4210);
    // udp.print(tel);
    // udp.endPacket();

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// ======================= HELPERS =======================
void loadMapFromSD() {
  xSemaphoreTake(mutexMap, portMAX_DELAY);
  File file = SD.open(MAP_FILENAME, FILE_READ);
  if(!file) {
    activeMap.width = 100; activeMap.height = 100; activeMap.resolution = 0.1;
    activeMap.data = (uint8_t*)malloc(10000);
    memset(activeMap.data, 0, 10000);
  } else {
    file.read((uint8_t*)&activeMap.width, 2);
    file.read((uint8_t*)&activeMap.height, 2);
    file.read((uint8_t*)&activeMap.resolution, 4);
    int dataSize = activeMap.width * activeMap.height;
    activeMap.data = (uint8_t*)malloc(dataSize);
    if(activeMap.data) file.read(activeMap.data, dataSize);
    file.close();
  }
  xSemaphoreGive(mutexMap);
}

void initParticles() {
  xSemaphoreTake(mutexState, portMAX_DELAY);
  for(int i=0; i<NUM_PARTICLES; i++) {
    particles[i].x = (activeMap.width * activeMap.resolution) / 2.0;
    particles[i].y = (activeMap.height * activeMap.resolution) / 2.0;
    particles[i].theta = 0;
    particles[i].weight = 1.0 / NUM_PARTICLES;
  }
  estimatedPose = {particles[0].x, particles[0].y, 0, 0};
  xSemaphoreGive(mutexState);
}

float readSonar(int trig, int echo) {
  digitalWrite(trig, LOW); delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long dur = pulseIn(echo, HIGH, 25000); 
  if(dur == 0) return 4.0; 
  return (dur * 0.034 / 2.0) / 100.0; 
}

float rayCast(float x, float y, float theta) {
  float stepSize = activeMap.resolution / 2.0;
  float maxDist = 3.0; 
  float curDist = 0;
  float curX = x, curY = y;
  float dx = cos(theta) * stepSize;
  float dy = sin(theta) * stepSize;

  while(curDist < maxDist) {
    curX += dx; curY += dy; curDist += stepSize;
    int mapX = (int)(curX / activeMap.resolution);
    int mapY = (int)(curY / activeMap.resolution);
    if(mapX < 0 || mapX >= activeMap.width || mapY < 0 || mapY >= activeMap.height) return maxDist;
    if(activeMap.data[mapY * activeMap.width + mapX] == 1) return curDist;
  }
  return maxDist;
}

float gaussian(float mu, float sigma, float x) {
  return (1.0 / (sigma * sqrt(2.0 * PI))) * exp(-0.5 * pow((x - mu) / sigma, 2.0));
}