#include <Wire.h>
#include <MPU6050.h>

#define HC_TRIG_1 3
#define HC_ECHO_1 46
#define HC_TRIG_2 6
#define HC_ECHO_2 47
#define HC_TRIG_3 7
#define HC_ECHO_3 21
#define HC_TRIG_4 15
#define HC_ECHO_4 14

#define ENC_LU_CLK 10
#define ENC_LU_DT 11
#define ENC_LD_CLK 40
#define ENC_LD_DT 39

#define I2C_SDA 8
#define I2C_SCL 9

MPU6050 mpu;

volatile long encoder_lu = 0;
volatile long encoder_ld = 0;

void IRAM_ATTR encLU_ISR() { encoder_lu++; }
void IRAM_ATTR encLD_ISR() { encoder_ld++; }

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("=== ESP32-S3 Sensor Test ===");

  Wire.begin(I2C_SDA, I2C_SCL);
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("[ERROR] MPU6050 not detected!");
    while (1);
  }
  Serial.println("[MPU6050] Connected");
  pinMode(HC_TRIG_1, OUTPUT); pinMode(HC_ECHO_1, INPUT);
  pinMode(HC_TRIG_2, OUTPUT); pinMode(HC_ECHO_2, INPUT);
  pinMode(HC_TRIG_3, OUTPUT); pinMode(HC_ECHO_3, INPUT);
  pinMode(HC_TRIG_4, OUTPUT); pinMode(HC_ECHO_4, INPUT);
  Serial.println("[HC-SR04] Pins configured");
  pinMode(ENC_LU_CLK, INPUT_PULLUP);
  attachInterrupt(ENC_LU_CLK, encLU_ISR, RISING);
  pinMode(ENC_LD_CLK, INPUT_PULLUP);
  attachInterrupt(ENC_LD_CLK, encLD_ISR, RISING);
  Serial.println("[Encoders] ISRs attached");

  Serial.println("=== Setup Complete ===\n");
}

float readHCSR04(int trig, int echo) {
  digitalWrite(trig, LOW); delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long duration = pulseIn(echo, HIGH, 30000); // timeout 30ms
  return duration / 58.2; // cm
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  float d1 = readHCSR04(HC_TRIG_1, HC_ECHO_1);
  float d2 = readHCSR04(HC_TRIG_2, HC_ECHO_2);
  float d3 = readHCSR04(HC_TRIG_3, HC_ECHO_3);
  float d4 = readHCSR04(HC_TRIG_4, HC_ECHO_4);
  Serial.print("Front:"); Serial.print(d1); Serial.print(" ");
  Serial.print("FrontRight:"); Serial.print(d2); Serial.print(" ");
  Serial.print("BackLeft:"); Serial.print(d3); Serial.print(" ");
  Serial.print("BackRight:"); Serial.print(d4); Serial.print(" ");
  Serial.print("Lift1:"); Serial.print(encoder_lu); Serial.print(" ");
  Serial.print("Lift2:"); Serial.print(encoder_ld); Serial.print(" ");
  Serial.print("AX:"); Serial.print(ax); Serial.print(" ");
  Serial.print("AY:"); Serial.print(ay); Serial.print(" ");
  Serial.print("AZ:"); Serial.println(az); 
  delay(100); 
}