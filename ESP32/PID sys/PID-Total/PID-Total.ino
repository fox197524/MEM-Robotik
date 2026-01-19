#include <MPU6050.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>

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

// Encoder Pins (CLK, DT) - FL, FR, BL, BR, LiftUp, LiftDown
#define ENC_FL_CLK 4
#define ENC_FL_DT 5
#define ENC_FR_CLK 12
#define ENC_FR_DT 13
#define ENC_BL_CLK 16
#define ENC_BL_DT 2
#define ENC_BR_CLK 19
#define ENC_BR_DT 20
#define ENC_LU_CLK 10
#define ENC_LU_DT 11
#define ENC_LD_CLK 41
#define ENC_LD_DT 40

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

// ======================= SENSOR DATA =======================
// MPU6050
int16_t ax, ay, az, gx, gy, gz;

// HC-SR04 Distances (cm)
float distance_front = 0, distance_fr = 0, distance_bl = 0, distance_br = 0;

// Encoder Counters (pulses)
volatile long encoder_fl = 0, encoder_fr = 0, encoder_bl = 0, encoder_br = 0;
volatile long encoder_lu = 0, encoder_ld = 0;

// Timing
unsigned long lastSensorRead = 0;
const long sensorInterval = 100; // 100ms = 10Hz sensor readout

// ======================= ENCODER ISR =======================
void IRAM_ATTR encFL_ISR() { encoder_fl++; }
void IRAM_ATTR encFR_ISR() { encoder_fr++; }
void IRAM_ATTR encBL_ISR() { encoder_bl++; }
void IRAM_ATTR encBR_ISR() { encoder_br++; }
void IRAM_ATTR encLU_ISR() { encoder_lu++; }
void IRAM_ATTR encLD_ISR() { encoder_ld++; }

// ======================= FUNCTION PROTOTYPES =======================
void initWiFi();
void initUART1();
void readMPU6050();
void readHCSR04();
void handleWiFiCommand(String msg);
void sendUART1(String msg);
void printSensorData();

// ======================= SETUP =======================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n\n=== ESP32-S3 Sensor Controller Starting ===");
  
  // Initialize I2C for MPU6050
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.println("[I2C] Initialized");
  
  // Initialize MPU6050
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("[ERROR] MPU6050 not detected!");
    while(1); // Halt
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
  
  // Initialize UART1 (to second ESP32)
  initUART1();
  
  // Initialize WiFi
  initWiFi();
  
  Serial.println("=== System Ready ===\n");
}

// ======================= MAIN LOOP =======================
void loop() {
  unsigned long currentMillis = millis();
  
  // Handle WiFi commands
  char packet[255];
  int len = udp.parsePacket();
  if (len) {
    udp.read(packet, len);
    packet[len] = '\0';
    handleWiFiCommand(String(packet));
  }
  
  // Read sensors at fixed interval
  if (currentMillis - lastSensorRead >= sensorInterval) {
    lastSensorRead = currentMillis;
    
    readMPU6050();
    readHCSR04();
    printSensorData();
  }
}

// ======================= WiFi INITIALIZATION =======================
void initWiFi() {
  Serial.print("[WiFi] Connecting to ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  int timeout = 20; // 10 seconds max
  while (WiFi.status() != WL_CONNECTED && timeout > 0) {
    delay(500);
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
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
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
    
    // Send to second ESP32 based on axis
    // Example: AXIS 0 = Front-Left/Right (Y), AXIS 1 = Front-Right (X)
    // Adjust mapping based on your controller layout
  }
  
  else if (msg.startsWith("BUTTON")) {
    int firstSpace = msg.indexOf(' ');
    int secondSpace = msg.indexOf(' ', firstSpace + 1);
    int btnId = msg.substring(firstSpace + 1, secondSpace).toInt();
    int state = msg.substring(secondSpace + 1).toInt();
    
    // Handle button press (lift up/down, etc.)
  }
  
  else if (msg.startsWith("HAT")) {
    int firstSpace = msg.indexOf(' ');
    int secondSpace = msg.indexOf(' ', firstSpace + 1);
    int thirdSpace = msg.indexOf(' ', secondSpace + 1);
    
    int hatId = msg.substring(firstSpace + 1, secondSpace).toInt();
    int x = msg.substring(secondSpace + 1, thirdSpace).toInt();
    int y = msg.substring(thirdSpace + 1).toInt();
  }
}

// ======================= PRINT SENSOR DATA =======================
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

// ======================= ENCODER CALIBRATION =======================
/*
  CALIBRATION PROCEDURE FOR LIFT SYSTEM:
  
  1. Manually move the lifter up to a known starting height (e.g., bottom position)
  2. Reset encoder counters: encoder_lu = 0; encoder_ld = 0;
  3. Run the lifter up to a measured height (e.g., lift 30cm)
  4. Note the encoder_lu pulses
  5. Calculate: pulses_per_cm = encoder_lu / height_in_cm
  
  Example: If lifting 30cm = 150 pulses, then 5 pulses/cm
  
  Once calibrated, use: height_cm = encoder_lu / pulses_per_cm
  
  To reset encoders during runtime:
  - Add serial command "RESET_ENC" to Serial.readString()
  - Then: encoder_lu = 0; encoder_ld = 0;
*/