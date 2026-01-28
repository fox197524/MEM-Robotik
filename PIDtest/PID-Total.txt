#include <MPU6050.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// ======================= PIN DEFINITIONS =======================
// HC-SR04 Ultrasonic Sensors (Front, Front-Right, Back-Left, Back-Right)
#define HC_TRIG_1 3
#define HC_ECHO_1 46
#define HC_TRIG_2 6
#define HC_ECHO_2 47
#define HC_TRIG_3 7
#define HC_ECHO_3 21
#define HC_TRIG_4 15
#define HC_ECHO_4 14

// Encoder Pins (CLK, DT) - Front Left, Front Right, Back Left, Back Right, LiftUp, LiftDown
#define ENC_FL_CLK 4
#define ENC_FL_DT 5
#define ENC_FR_CLK 12
#define ENC_FR_DT 13
#define ENC_BL_CLK 37
#define ENC_BL_DT 38
#define ENC_BR_CLK 41
#define ENC_BR_DT 42
#define ENC_LU_CLK 10
#define ENC_LU_DT 11
#define ENC_LD_CLK 40
#define ENC_LD_DT 39

// I2C Pins (MPU6050)
#define I2C_SDA 8
#define I2C_SCL 9

// UART1 Pins (to second ESP32)
#define UART1_TX 17
#define UART1_RX 18

// ======================= WIFI & UDP =======================
const char* ssid = "Ben";
const char* password = "Gisherman2010";
WiFiUDP udp;
unsigned int localPort = 4210;

// ======================= SENSOR OBJECTS =======================
MPU6050 mpu;

// ======================= SENSOR DATA (SHARED) =======================
// MPU6050
volatile int16_t ax, ay, az, gx, gy, gz;

// HC-SR04 Distances (cm)
volatile float distance_front = 0, distance_fr = 0, distance_bl = 0, distance_br = 0;

// Encoder Counters (pulses)
volatile long encoder_fl = 0, encoder_fr = 0, encoder_bl = 0, encoder_br = 0;
volatile long encoder_lu = 0, encoder_ld = 0;

// ======================= FREERTOS OBJECTS =======================
SemaphoreHandle_t motorMutex;       // Protects motor command variables
SemaphoreHandle_t encoderMutex;     // Protects encoder counters

// ======================= MOTOR COMMAND VARIABLES (SHARED) =======================
volatile int target_fl = 0, target_fr = 0, target_bl = 0, target_br = 0;
volatile int target_lu = 0, target_ld = 0;

// ======================= TASK HANDLES =======================
TaskHandle_t networkTaskHandle = NULL;
TaskHandle_t controlTaskHandle = NULL;

// ======================= FUNCTION PROTOTYPES =======================
void networkingTask(void *pvParameters);
void controlTask(void *pvParameters);
void initWiFi();
void initUART1();
void readMPU6050();
void readHCSR04();
void handleWiFiCommand(String msg);
void sendUART1(String msg);
void printSensorData();

// ======================= ENCODER ISR =======================
void IRAM_ATTR encFL_ISR() { encoder_fl = encoder_fl + 1; }
void IRAM_ATTR encFR_ISR() { encoder_fr = encoder_fr + 1; }
void IRAM_ATTR encBL_ISR() { encoder_bl = encoder_bl + 1; }
void IRAM_ATTR encBR_ISR() { encoder_br = encoder_br + 1; }
void IRAM_ATTR encLU_ISR() { encoder_lu = encoder_lu + 1; }
void IRAM_ATTR encLD_ISR() { encoder_ld = encoder_ld + 1; }

// ======================= SETUP =======================
void setup() {
  Serial.begin(115200);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  
  Serial.println("\n\n=== ESP32-S3 Dual-Core Robotics Controller ===");
  Serial.println("[CORE] Main setup() running on Core " + String(xPortGetCoreID()));
  
  // Create Mutexes
  motorMutex = xSemaphoreCreateMutex();
  encoderMutex = xSemaphoreCreateMutex();
  
  if (motorMutex == NULL || encoderMutex == NULL) {
    Serial.println("[ERROR] Failed to create mutexes!");
    while(1);
  }
  Serial.println("[MUTEX] Semaphores created");
  
  // Initialize I2C for MPU6050
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.println("[I2C] Initialized");
  
  // Initialize MPU6050
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("[ERROR] MPU6050 not detected!");
    while(1);
  }
  Serial.println("[MPU6050] Connected");
  
  // Initialize HC-SR04 pins
  pinMode(HC_TRIG_1, OUTPUT);
  pinMode(HC_ECHO_1, INPUT);
  pinMode(HC_TRIG_2, OUTPUT);
  pinMode(HC_ECHO_2, INPUT);
  pinMode(HC_TRIG_3, OUTPUT);
  pinMode(HC_ECHO_3, INPUT);
  pinMode(HC_TRIG_4, OUTPUT);
  pinMode(HC_ECHO_4, INPUT);
  Serial.println("[HC-SR04] Pins configured");
  
  // Initialize Encoder ISRs
  pinMode(ENC_FL_CLK, INPUT_PULLUP);
  attachInterrupt(ENC_FL_CLK, encFL_ISR, RISING);
  pinMode(ENC_FR_CLK, INPUT_PULLUP);
  attachInterrupt(ENC_FR_CLK, encFR_ISR, RISING);
  pinMode(ENC_BL_CLK, INPUT_PULLUP);
  attachInterrupt(ENC_BL_CLK, encBL_ISR, RISING);
  pinMode(ENC_BR_CLK, INPUT_PULLUP);
  attachInterrupt(ENC_BR_CLK, encBR_ISR, RISING);
  pinMode(ENC_LU_CLK, INPUT_PULLUP);
  attachInterrupt(ENC_LU_CLK, encLU_ISR, RISING);
  pinMode(ENC_LD_CLK, INPUT_PULLUP);
  attachInterrupt(ENC_LD_CLK, encLD_ISR, RISING);
  Serial.println("[Encoders] ISRs attached");
  
  // Initialize UART1
  initUART1();
  
  // Create FreeRTOS Tasks
  // CORE 0: Networking Task (WiFi + UART)
  xTaskCreatePinnedToCore(
    networkingTask,        // Task function
    "NetworkingTask",      // Task name
    4096,                  // Stack size (bytes)
    NULL,                  // Parameters
    2,                     // Priority (higher number = higher priority)
    &networkTaskHandle,    // Task handle
    0                      // Core 0
  );
  Serial.println("[TASK] Networking Task created on Core 0");
  
  // CORE 1: Control Task (Sensors + PID)
  xTaskCreatePinnedToCore(
    controlTask,           // Task function
    "ControlTask",         // Task name
    4096,                  // Stack size (bytes)
    NULL,                  // Parameters
    3,                     // Priority (higher = more important)
    &controlTaskHandle,    // Task handle
    1                      // Core 1
  );
  Serial.println("[TASK] Control Task created on Core 1");
  
  Serial.println("=== Setup Complete - Tasks Running ===\n");
  
  // Delete setup task (no longer needed)
  vTaskDelete(NULL);
}

// ======================= CORE 0: NETWORKING TASK =======================
void networkingTask(void *pvParameters) {
  // Initialize WiFi in this task
  initWiFi();
  
  Serial.println("[NETWORKING] Task started on Core " + String(xPortGetCoreID()));
  
  const TickType_t xDelay = 10 / portTICK_PERIOD_MS;  // 10ms loop
  
  while(1) {
    // Parse incoming WiFi UDP commands
    char packet[255];
    int len = udp.parsePacket();
    if (len) {
      udp.read(packet, len);
      packet[len] = '\0';
      handleWiFiCommand(String(packet));
    }
    
    vTaskDelay(xDelay);
  }
}

// ======================= CORE 1: CONTROL TASK =======================
void controlTask(void *pvParameters) {
  Serial.println("[CONTROL] Task started on Core " + String(xPortGetCoreID()));
  
  const TickType_t xDelay = 10 / portTICK_PERIOD_MS;  // 100Hz control loop (10ms)
  unsigned long printCounter = 0;
  
  while(1) {
    // Read sensors (I2C + GPIO)
    readMPU6050();
    readHCSR04();
    
    // Print sensor data every 100ms (every 10 iterations)
    if (++printCounter >= 10) {
      printCounter = 0;
      printSensorData();
    }
    
    // Task timing
    vTaskDelay(xDelay);
  }
}

// ======================= WiFi INITIALIZATION =======================
void initWiFi() {
  Serial.print("[WiFi] Connecting to ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  int timeout = 20;
  while (WiFi.status() != WL_CONNECTED && timeout > 0) {
    vTaskDelay(500 / portTICK_PERIOD_MS);
    Serial.print(".");
    timeout--;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[WiFi] Connected!");
    Serial.print("[IP] ");
    Serial.println(WiFi.localIP());
    udp.begin(localPort);
    Serial.println("[UDP] Listening on port 4210");
  } else {
    Serial.println("\n[ERROR] WiFi connection failed!");
  }
}

// ======================= UART1 INITIALIZATION =======================
void initUART1() {
  Serial2.begin(115200, SERIAL_8N1, UART1_RX, UART1_TX);
  Serial.println("[UART1] Initialized at 115200 baud");
}

// ======================= SEND UART1 =======================
void sendUART1(String msg) {
  Serial2.println(msg);
}

// ======================= READ MPU6050 =======================
void readMPU6050() {
  mpu.getMotion6((int16_t*)&ax, (int16_t*)&ay, (int16_t*)&az, 
                 (int16_t*)&gx, (int16_t*)&gy, (int16_t*)&gz);
}

// ======================= READ HC-SR04 =======================
void readHCSR04() {
  // HC-SR04 #1 (Front)
  digitalWrite(HC_TRIG_1, LOW);
  delayMicroseconds(2);
  digitalWrite(HC_TRIG_1, HIGH);
  delayMicroseconds(10);
  digitalWrite(HC_TRIG_1, LOW);
  long duration_1 = pulseIn(HC_ECHO_1, HIGH, 30000);
  distance_front = duration_1 / 58.2;
  
  // HC-SR04 #2 (Front-Right)
  digitalWrite(HC_TRIG_2, LOW);
  delayMicroseconds(2);
  digitalWrite(HC_TRIG_2, HIGH);
  delayMicroseconds(10);
  digitalWrite(HC_TRIG_2, LOW);
  long duration_2 = pulseIn(HC_ECHO_2, HIGH, 30000);
  distance_fr = duration_2 / 58.2;
  
  // HC-SR04 #3 (Back-Left)
  digitalWrite(HC_TRIG_3, LOW);
  delayMicroseconds(2);
  digitalWrite(HC_TRIG_3, HIGH);
  delayMicroseconds(10);
  digitalWrite(HC_TRIG_3, LOW);
  long duration_3 = pulseIn(HC_ECHO_3, HIGH, 30000);
  distance_bl = duration_3 / 58.2;
  
  // HC-SR04 #4 (Back-Right)
  digitalWrite(HC_TRIG_4, LOW);
  delayMicroseconds(2);
  digitalWrite(HC_TRIG_4, HIGH);
  delayMicroseconds(10);
  digitalWrite(HC_TRIG_4, LOW);
  long duration_4 = pulseIn(HC_ECHO_4, HIGH, 30000);
  distance_br = duration_4 / 58.2;
}

// ======================= HANDLE WiFi COMMANDS =======================
void handleWiFiCommand(String msg) {
  if (msg.startsWith("AXIS")) {
    int firstSpace = msg.indexOf(' ');
    int secondSpace = msg.indexOf(' ', firstSpace + 1);
    int axisId = msg.substring(firstSpace + 1, secondSpace).toInt();
    float val = msg.substring(secondSpace + 1).toFloat();
    
    Serial.print("[WiFi] AXIS ");
    Serial.print(axisId);
    Serial.print(" = ");
    Serial.println(val);
    
    // Map axis values to motor PWM (-255 to +255)
    // Assuming dual-stick controller: Axis 0,1 = Left stick, Axis 2,3 = Right stick
    // Left stick Y (axis 1) controls front motors (FL, FR)
    // Right stick Y (axis 3) controls back motors (BL, BR)
    
    if (axisId == 0) {
      // Left stick X - Rotation/Steering
      int pwm = map(val, -1.0, 1.0, -255, 255);
      xSemaphoreTake(motorMutex, portMAX_DELAY);
      target_fl = pwm;
      target_br = pwm;
      target_fr = -pwm;
      target_bl = -pwm;
      xSemaphoreGive(motorMutex);
    }
    else if (axisId == 1) {
      // Left stick Y - Forward/Backward
      int pwm = map(val, -1.0, 1.0, -255, 255);
      xSemaphoreTake(motorMutex, portMAX_DELAY);
      target_fl = pwm;
      target_fr = pwm;
      target_bl = pwm;
      target_br = pwm;
      xSemaphoreGive(motorMutex);
      
      sendUART1("FL " + String(pwm));
      sendUART1("FR " + String(pwm));
      sendUART1("BL " + String(pwm));
      sendUART1("BR " + String(pwm));
      Serial.print("[UART] Motors set to ");
      Serial.println(pwm);
    }
  }
  
  else if (msg.startsWith("BUTTON")) {
    int firstSpace = msg.indexOf(' ');
    int secondSpace = msg.indexOf(' ', firstSpace + 1);
    int btnId = msg.substring(firstSpace + 1, secondSpace).toInt();
    int state = msg.substring(secondSpace + 1).toInt();
    
    Serial.print("[WiFi] BUTTON ");
    Serial.print(btnId);
    Serial.print(" = ");
    Serial.println(state ? "PRESSED" : "RELEASED");
    
    // Handle button press (lift up/down via UART)
    if (state == 1) {  // Button pressed
      if (btnId == 5) {  // Example: Button 5 = Lift Up
        xSemaphoreTake(motorMutex, portMAX_DELAY);
        target_lu = 150;
        target_ld = 150;
        xSemaphoreGive(motorMutex);
        sendUART1("LU 150");
        Serial.println("[UART] Lift UP: 150");
      }
      else if (btnId == 6) {  // Button 6 = Lift Down
        xSemaphoreTake(motorMutex, portMAX_DELAY);
        target_lu = -150;
        target_ld = -150;
        xSemaphoreGive(motorMutex);
        sendUART1("LU -150");
        Serial.println("[UART] Lift DOWN: -150");
      }
    }
    else {  // Button released
      xSemaphoreTake(motorMutex, portMAX_DELAY);
      target_lu = 0;
      target_ld = 0;
      xSemaphoreGive(motorMutex);
      sendUART1("LU 0");
      Serial.println("[UART] Lift STOP");
    }
  }
  
  else if (msg.startsWith("HAT")) {
    int firstSpace = msg.indexOf(' ');
    int secondSpace = msg.indexOf(' ', firstSpace + 1);
    int thirdSpace = msg.indexOf(' ', secondSpace + 1);
    
    int hatId = msg.substring(firstSpace + 1, secondSpace).toInt();
    int x = msg.substring(secondSpace + 1, thirdSpace).toInt();
    int y = msg.substring(thirdSpace + 1).toInt();
    
    Serial.print("[WiFi] D-PAD: X=");
    Serial.print(x);
    Serial.print(" Y=");
    Serial.println(y);
    
    // D-pad navigation (e.g., select map waypoints)
  }
}

// ======================= PRINT SENSOR DATA (CORE 1 ONLY) =======================
void printSensorData() {
  Serial.print("MPU6050: AX=");
  Serial.print(ax);
  Serial.print(" AY=");
  Serial.print(ay);
  Serial.print(" AZ=");
  Serial.print(az);
  Serial.print(" | GX=");
  Serial.print(gx);
  Serial.print(" GY=");
  Serial.print(gy);
  Serial.print(" GZ=");
  Serial.println(gz);
  
  Serial.print("HC-SR04: F=");
  Serial.print(distance_front, 1);
  Serial.print("cm FR=");
  Serial.print(distance_fr, 1);
  Serial.print("cm BL=");
  Serial.print(distance_bl, 1);
  Serial.print("cm BR=");
  Serial.print(distance_br, 1);
  Serial.println("cm");
  
  Serial.print("ENC: FL=");
  Serial.print(encoder_fl);
  Serial.print("p FR=");
  Serial.print(encoder_fr);
  Serial.print("p BL=");
  Serial.print(encoder_bl);
  Serial.print("p BR=");
  Serial.print(encoder_br);
  Serial.print("p LU=");
  Serial.print(encoder_lu);
  Serial.print("p LD=");
  Serial.print(encoder_ld);
  Serial.println("p");
  
  Serial.println("---");
}

// ======================= EMPTY LOOP (FREERTOS TAKES OVER) =======================
void loop() {
  // All work is done by FreeRTOS tasks
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}