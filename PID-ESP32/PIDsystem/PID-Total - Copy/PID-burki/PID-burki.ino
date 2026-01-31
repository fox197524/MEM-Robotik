/*
 * ROBOT PID NAVIGATION SYSTEM v2.0 (MM + ENCODER TURNS)
 * Autonomous Mission: BASLANGIC → KAYNAK → SARNICSOL → KAYNAK
 * Commands: "START" (auto), "GO X" (manual), "STOP" via UDP
 * Controls: ILERI/GERI/SOL/SAG/DONUS + encoder turns
 */

#include <Arduino.h>
#include <MPU6050.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// ======================= CONFIGURATION =======================
#define DIST_PER_TICK 0.0006545f  // 0.1m wheel, 48 ticks/rev = 0.6545mm/tick
#define LIFT_MM_PER_TICK 0.1f     // Elevator: adjust this!
#define STOP_DIST 30.0f           // 30mm safety
#define NUM_PARTICLES 50

// Locations (1px = 1mm)
struct Location {
  const char* name;
  float x, y;  // mm
};

Location locations[] = {
  {"KAYNAK", 2, 700},
  {"SARNICSOL", 1382, 1105},
  {"SARNICSAG", 1879, 1105},
  {"BASLANGIC", 200, 200}
};
#define NUM_LOCATIONS 4

// Autonomous mission sequence
int mission[] = {3, 0, 1, 0};  // BASLANGIC→KAYNAK→SARNICSOL→KAYNAK
int missionStep = 0;
bool autoMission = false;

#define HC_TRIG_F 3  #define HC_ECHO_F 46
#define HC_TRIG_R 6  #define HC_ECHO_R 47
#define HC_TRIG_L 7  #define HC_ECHO_L 21
#define HC_TRIG_B 15 #define HC_ECHO_B 14

#define ENC_FL_CLK 4  #define ENC_FL_DT 5
#define ENC_FR_CLK 12 #define ENC_FR_DT 13
#define ENC_BL_CLK 37 #define ENC_BL_DT 38
#define ENC_BR_CLK 41 #define ENC_BR_DT 42
#define ENC_LU_CLK 10 #define ENC_LU_DT 11
#define ENC_LD_CLK 40 #define ENC_LD_DT 39

#define I2C_SDA 8 #define I2C_SCL 9
#define UART1_TX 17 #define UART1_RX 18

// ======================= GLOBALS =======================
MPU6050 mpu;
WiFiUDP udp;
HardwareSerial pidUart(1);

struct Pose { float x,y,theta,height; };  // All mm
volatile Pose estimatedPose = {200,200,0,0};

volatile long enc_fl=0, enc_fr=0, enc_bl=0, enc_br=0;
volatile long enc_lu=0, enc_ld=0;
volatile float gyro_offset = 0;

int targetIndex = -1;
bool navigating = false;
float sonar_f,r,l,b;

// PID state
float pid_dist_i=0, pid_dist_prev=0;
float pid_theta_i=0, pid_theta_prev=0;
unsigned long pid_time=0;

SemaphoreHandle_t mutexPose;
const char* ssid = "Ben";
const char* password = "Gisherman2010";
unsigned int localPort = 4210;

// ======================= ISRs =======================
void IRAM_ATTR enc_fl_isr() { enc_fl++; }
void IRAM_ATTR enc_fr_isr() { enc_fr++; }
void IRAM_ATTR enc_bl_isr() { enc_bl++; }
void IRAM_ATTR enc_br_isr() { enc_br++; }
void IRAM_ATTR enc_lu_isr() { enc_lu++; }
void IRAM_ATTR enc_ld_isr() { enc_ld++; }

void setup() {
  Serial.begin(115200);
  mutexPose = xSemaphoreCreateMutex();
  
  // Hardware init
  Wire.begin(I2C_SDA, I2C_SCL);
  mpu.initialize();
  
  // Gyro calibration
  Serial.println("Calibrating gyro...");
  long sum=0; for(int i=0; i<200; i++) { sum+=mpu.getRotationZ(); delay(5); }
  gyro_offset = sum/200.0;
  
  // Sonar pins
  pinMode(HC_TRIG_F,OUTPUT); pinMode(HC_ECHO_F,INPUT);
  pinMode(HC_TRIG_R,OUTPUT); pinMode(HC_ECHO_R,INPUT);
  pinMode(HC_TRIG_L,OUTPUT); pinMode(HC_ECHO_L,INPUT);
  pinMode(HC_TRIG_B,OUTPUT); pinMode(HC_ECHO_B,INPUT);
  
  // Encoder pins (only CLK for directionless count)
  pinMode(ENC_FL_CLK,INPUT_PULLUP); attachInterrupt(ENC_FL_CLK,enc_fl_isr, RISING);
  pinMode(ENC_FR_CLK,INPUT_PULLUP); attachInterrupt(ENC_FR_CLK,enc_fr_isr, RISING);
  pinMode(ENC_BL_CLK,INPUT_PULLUP); attachInterrupt(ENC_BL_CLK,enc_bl_isr, RISING);
  pinMode(ENC_BR_CLK,INPUT_PULLUP); attachInterrupt(ENC_BR_CLK,enc_br_isr, RISING);
  pinMode(ENC_LU_CLK,INPUT_PULLUP); attachInterrupt(ENC_LU_CLK,enc_lu_isr, RISING);
  pinMode(ENC_LD_CLK,INPUT_PULLUP); attachInterrupt(ENC_LD_CLK,enc_ld_isr, RISING);
  
  // UART to motor driver
  pidUart.begin(115200, SERIAL_8N1, UART1_RX, UART1_TX);
  
  // WiFi
  WiFi.begin(ssid,password);
  while(WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  udp.begin(localPort);
  
  Serial.println("\n PID NAV READY (MM + ENCODER TURNS)");
  Serial.printf("Mission: ");
  for(int i=0; i<4; i++) Serial.printf("%s→", locations[mission[i]].name);
  Serial.println("COMPLETE");
  
  xTaskCreatePinnedToCore(controlTask, "PID", 16384, NULL, 6, NULL, 1);
  xTaskCreatePinnedToCore(networkTask, "NET", 4096, NULL, 2, NULL, 0);
  vTaskDelete(NULL);
}

void controlTask(void *param) {
  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t freq = 10 / portTICK_PERIOD_MS; // 100Hz
  
  long old_fl=0, old_fr=0;
  unsigned long lastTime = micros();
  
  while(1) {
    unsigned long now = micros();
    float dt = (now-lastTime)/1e6; 
    if(dt<0.001f) dt=0.001f;
    lastTime = now;
    
    // === SENSORS ===
    sonar_f = readSonar(HC_TRIG_F,HC_ECHO_F);
    sonar_r = readSonar(HC_TRIG_R,HC_ECHO_R);
    sonar_l = readSonar(HC_TRIG_L,HC_ECHO_L);
    sonar_b = readSonar(HC_TRIG_B,HC_ECHO_B);
    
    // === ODOMETRY ===
    long fl=enc_fl, fr=enc_fr;
    float dleft = (fl-old_fl)*DIST_PER_TICK*1000;  // mm
    float dright = (fr-old_fr)*DIST_PER_TICK*1000;
    float dcenter = (dleft+dright)/2;
    old_fl=fl; old_fr=fr;
    
    float height = (enc_lu-enc_ld)*LIFT_MM_PER_TICK;
    
    // === POSE UPDATE ===
    xSemaphoreTake(mutexPose, portMAX_DELAY);
    estimatedPose.x += dcenter*cos(estimatedPose.theta);
    estimatedPose.y += dcenter*sin(estimatedPose.theta);
    estimatedPose.height = height;
    
    int16_t gz = mpu.getRotationZ();
    float gyro = ((gz-gyro_offset)/131.0f) * DEG_TO_RAD * dt;
    estimatedPose.theta += gyro;
    estimatedPose.theta = fmod(estimatedPose.theta+3*PI, TWO_PI) - PI;
    xSemaphoreGive(mutexPose);
    
    // === NAVIGATION ===
    if(targetIndex>=0 && !navigating) {
      navigating = true;
      Serial.printf(" TARGET %s (%.0f,%.0f)\n", 
        locations[targetIndex].name, locations[targetIndex].x, locations[targetIndex].y);
    }
    
    if(navigating && targetIndex>=0) {
      // SAFETY STOP
      if(sonar_f<STOP_DIST || sonar_r<STOP_DIST || sonar_l<STOP_DIST || sonar_b<STOP_DIST) {
        pidUart.println("STOP");
        navigating = false;
        Serial.printf(" COLLISION! sonar_f=%.0fmm\n", sonar_f*1000);
        vTaskDelay(500/portTICK_PERIOD_MS);
        continue;
      }
      
      // Errors
      float tx=locations[targetIndex].x, ty=locations[targetIndex].y;
      float dx=tx-estimatedPose.x, dy=ty-estimatedPose.y;
      float dist=sqrtf(dx*dx+dy*dy);
      float tgt_theta=atan2f(dy,dx);
      float theta_err = tgt_theta-estimatedPose.theta;
      theta_err = fmodf(theta_err+3*PI, TWO_PI)-PI;
      
      // PID
      pid_dist_i += dist*dt; pid_dist_i=constrain(pid_dist_i,-500,500);
      float dist_d = (dist-pid_dist_prev)/dt;
      float v_cmd = constrain(1.8f*dist + 0.3f*pid_dist_i + 0.4f*dist_d, 50, 800);
      
      pid_theta_i += theta_err*dt; pid_theta_i=constrain(pid_theta_i,-2,2);
      float theta_d = (theta_err-pid_theta_prev)/dt;
      float w_cmd = constrain(2.2f*theta_err + 0.4f*pid_theta_i + 0.3f*theta_d, -3,3);
      
      // EXECUTE with encoder turns
      executeMove(v_cmd, w_cmd);
      
      pid_dist_prev=dist; pid_theta_prev=theta_err;
      
      // ARRIVED?
      if(dist<120 && fabsf(theta_err)<0.15f) {  // 12cm tolerance
        pidUart.println("STOP");
        Serial.printf(" ARRIVED %s! (err=%.0fmm)\n", locations[targetIndex].name, dist);
        
        // Mission complete check
        if(autoMission) {
          missionStep++;
          if(missionStep >= 4) {
            Serial.println(" MISSION COMPLETE!");
            autoMission = false;
          } else {
            targetIndex = mission[missionStep];
            navigating = false;  // Reset for next step
            Serial.printf(" Next: %s\n", locations[targetIndex].name);
            vTaskDelay(2000/portTICK_PERIOD_MS);
          }
        } else {
          targetIndex = -1;
        }
        navigating = false;
      }
    }
    
    vTaskDelayUntil(&lastWake, freq);
  }
}

void networkTask(void *param) {
  while(1) {
    int pkt = udp.parsePacket();
    if(pkt) {
      char buf[64]; 
      int len = udp.read(buf,64);
      buf[len]=0;
      String cmd = String(buf);
      cmd.trim();
      
      if(cmd=="START") {
        autoMission = true;
        missionStep = 0;
        targetIndex = mission[0];
        Serial.println(" AUTO MISSION START");
      }
      else if(cmd.startsWith("GO ")) {
        targetIndex = cmd.substring(3).toInt();
        if(targetIndex>=0 && targetIndex<NUM_LOCATIONS) {
          autoMission = false;
          Serial.printf(" MANUAL GO %d=%s\n", targetIndex, locations[targetIndex].name);
        }
      }
      else if(cmd=="STOP") {
        targetIndex=-1; navigating=false; autoMission=false;
        pidUart.println("STOP");
        Serial.println(" EMERGENCY STOP");
      }
    }
    
    // Broadcast pose
    xSemaphoreTake(mutexPose, portMAX_DELAY);
    Serial.printf("POS: %.0f %.0f %.1f h=%.0f\n", 
      estimatedPose.x, estimatedPose.y, estimatedPose.theta*180/PI, estimatedPose.height);
    xSemaphoreGive(mutexPose);
    
    vTaskDelay(100/portTICK_PERIOD_MS);
  }
}

void executeMove(float v, float w) {
  // Convert mm/s, rad/s → encoder turns (1 turn = ~314mm circumference)
  const float MM_PER_TURN = 314.0f;  // PI*100mm wheel
  const float TURN_TIME_MS = 500;    // Base execution time
  
  if(fabsf(w) > 0.8f) {  // ROTATE dominant (>80°/s equivalent)
    int dir = w>0 ? 1 : 0;
    int turns = constrain(fabsf(w)*0.8f, 0.5f, 3.0f);  // 0.5-3 turns
    pidUart.printf("DONUS %d %d\n", 200, (int)(turns*10));  // 1 decimal
  }
  else if(v > 30) {  // FORWARD
    int turns = constrain(v*0.6f/MM_PER_TURN, 0.5f, 8.0f);
    pidUart.printf("ILERI %d %d\n", 220, (int)(turns*10));
  }
  else if(v < -30) {  // BACKWARD
    int turns = constrain((-v)*0.6f/MM_PER_TURN, 0.5f, 8.0f);
    pidUart.printf("GERI %d %d\n", 220, (int)(turns*10));
  }
  else if(w > 0.2f) {  // RIGHT STRAFE
    int turns = constrain(w*0.4f, 0.3f, 2.0f);
    pidUart.printf("SAG %d %d\n", 200, (int)(turns*10));
  }
  else if(w < -0.2f) {  // LEFT STRAFE
    int turns = constrain((-w)*0.4f, 0.3f, 2.0f);
    pidUart.printf("SOL %d %d\n", 200, (int)(turns*10));
  }
}

float readSonar(int trig, int echo) {
  digitalWrite(trig,LOW); delayMicroseconds(2);
  digitalWrite(trig,HIGH); delayMicroseconds(10);
  digitalWrite(trig,LOW);
  long duration = pulseIn(echo,HIGH,25000);
  return duration==0 ? 4.0f : (duration*0.034f/2.0f);
}
