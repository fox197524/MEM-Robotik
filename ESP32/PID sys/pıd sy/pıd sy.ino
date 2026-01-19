#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <MPU6050.h>

/* * ENCODER CALIBRATION GUIDE:
 * 1. Measure the current height (x cm) of your lift system.
 * 2. Rotate the lift motors until the height changes by exactly 1 cm.
 * 3. Note the change in encoderCounts[4] or [5].
 * 4. Use that value as your "Ticks Per CM" variable later.
 * 5. Current logic: Y turns = x cm (To be mapped in the merged code).
 */

// --- UART & I2C PINS ---
#define UART1_TX 17  // Connect to Slave RX
#define UART1_RX 18  // Connect to Slave TX
#define SDA_PIN 1
#define SCL_PIN 2

// --- SENSOR PINS ---
#define TRIG_PIN 42  // Shared Trigger for all 4 HC-SR04
const int ECHO_PINS[4] = {41, 40, 39, 38}; 

// Encoders (FL, FR, BL, BR, Lift1, Lift2) - N16R8 Safe GPIOs
const int ENC_A[6] = {4, 6, 15, 7, 16, 12};
const int ENC_B[6] = {5, 9, 14, 8, 21, 13}; 

// --- GLOBALS ---
MPU6050 mpu;
WiFiUDP udp;
const char* ssid = "Ben";
const char* password = "Gisherman2010";

volatile long encoderCounts[6] = {0,0,0,0,0,0};
float distances[4] = {0,0,0,0};
int16_t ax, ay, az, gx, gy, gz;

unsigned long prevMillis = 0;
const int loopInterval = 100; // 10Hz update for Serial/UART

// --- ENCODER INTERRUPTS ---
void IRAM_ATTR isr0() { (digitalRead(ENC_B[0])) ? encoderCounts[0]++ : encoderCounts[0]--; }
void IRAM_ATTR isr1() { (digitalRead(ENC_B[1])) ? encoderCounts[1]++ : encoderCounts[1]--; }
void IRAM_ATTR isr2() { (digitalRead(ENC_B[2])) ? encoderCounts[2]++ : encoderCounts[2]--; }
void IRAM_ATTR isr3() { (digitalRead(ENC_B[3])) ? encoderCounts[3]++ : encoderCounts[3]--; }
void IRAM_ATTR isr4() { (digitalRead(ENC_B[4])) ? encoderCounts[4]++ : encoderCounts[4]--; }
void IRAM_ATTR isr5() { (digitalRead(ENC_B[5])) ? encoderCounts[5]++ : encoderCounts[5]--; }

void setup() {
  Serial.begin(115200); // PC Serial Monitor
  Serial1.begin(115200, SERIAL_8N1, UART1_RX, UART1_TX); // Slave UART
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(500);
  udp.begin(4210);

  Wire.begin(SDA_PIN, SCL_PIN);
  mpu.initialize();

  pinMode(TRIG_PIN, OUTPUT);
  for(int i=0; i<4; i++) pinMode(ECHO_PINS[i], INPUT);

  // Attach Interrupts for High-Speed Tracking
  void (*isrFuncs[])() = {isr0, isr1, isr2, isr3, isr4, isr5};
  for(int i=0; i<6; i++) {
    pinMode(ENC_A[i], INPUT_PULLUP);
    pinMode(ENC_B[i], INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENC_A[i]), isrFuncs[i], RISING);
  }
}

void loop() {
  handleUDP();

  unsigned long now = millis();
  if (now - prevMillis >= loopInterval) {
    prevMillis = now;
    readSensors();
    outputToMonitor();
    sendToSlave();
  }
}

void readSensors() {
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  for(int i=0; i<4; i++) {
    digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    long duration = pulseIn(ECHO_PINS[i], HIGH, 20000); // 20ms timeout
    distances[i] = (duration == 0) ? 400 : duration / 58.2;
  }
}

void handleUDP() {
  char packet[255];
  int len = udp.parsePacket();
  if (len) {
    udp.read(packet, 255);
    packet[len] = '\0';
    
    // Pass raw control to Slave for Motor Action
    // Slave will handle logic like "FR-pwm255"
    Serial1.println(packet); 
  }
}

void outputToMonitor() {
  // --- SERIAL MONITOR OUTPUT (EXACT FORMAT REQUESTED) ---
  Serial.print("MPU6050 accelerometer(");
  Serial.print(ax); Serial.print(",");
  Serial.print(ay); Serial.print(",");
  Serial.print(az); Serial.println(")");

  Serial.print("MPU6050 gyroscope(");
  Serial.print(gx); Serial.print(",");
  Serial.print(gy); Serial.print(",");
  Serial.print(gz); Serial.println(")");

  Serial.print("HC-SR04 Distances: ");
  for(int i=0; i<4; i++) { Serial.print(distances[i], 1); Serial.print(i==3?" cm":" | "); }
  Serial.println();

  Serial.print("Encoder Rotations (FL|FR|BL|BR|L1|L2): ");
  for(int i=0; i<6; i++) { Serial.print(encoderCounts[i]); Serial.print(i==5?"":" | "); }
  Serial.println("\n-------------------------------------------");
}

void sendToSlave() {
  // Sending Sensor State to Slave in a clean DATA format for its internal logic
  Serial1.printf("STATE|%ld,%ld,%ld,%ld,%ld,%ld|%.1f,%.1f,%.1f,%.1f\n", 
    encoderCounts[0], encoderCounts[1], encoderCounts[2], encoderCounts[3], encoderCounts[4], encoderCounts[5],
    distances[0], distances[1], distances[2], distances[3]);
}