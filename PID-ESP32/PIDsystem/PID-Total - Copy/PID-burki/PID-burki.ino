#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>

/////////////////////////////////////////////////////
// ARENA AYARLARI (MILIMETRE)
/////////////////////////////////////////////////////
#define ARENA_SIZE_MM 3600.0f   // Arena: 3.6m x 3.6m

/////////////////////////////////////////////////////
// TEKERLEK & ENCODER AYARLARI
/////////////////////////////////////////////////////
#define WHEEL_DIAMETER_MM 100.0f
#define ENC_TICKS_PER_REV 48.0f
#define DIST_PER_TICK_MM (WHEEL_DIAMETER_MM * PI / ENC_TICKS_PER_REV)

/////////////////////////////////////////////////////
// LOOP ZAMANLAMASI
/////////////////////////////////////////////////////
#define LOOP_DT 0.02f // 50 Hz kontrol döngüsü

/////////////////////////////////////////////////////
// SONAR PINLERI
/////////////////////////////////////////////////////
#define HC_TRIG_FRONT 3
#define HC_ECHO_FRONT 46
#define HC_TRIG_RIGHT 6
#define HC_ECHO_RIGHT 47
#define HC_TRIG_LEFT 7
#define HC_ECHO_LEFT 21
#define HC_TRIG_BACK 15
#define HC_ECHO_BACK 14

/////////////////////////////////////////////////////
// ENCODER PINLERI
/////////////////////////////////////////////////////
#define ENC_FL_CLK 4
#define ENC_FR_CLK 12

/////////////////////////////////////////////////////
// I2C & UART
/////////////////////////////////////////////////////
#define I2C_SDA 8
#define I2C_SCL 9
#define UART1_TX 17
#define UART1_RX 18

/////////////////////////////////////////////////////
// ROBOT POZISYON YAPISI
/////////////////////////////////////////////////////
struct Pose {
  float x;     // mm cinsinden X konumu
  float y;     // mm cinsinden Y konumu
  float theta; // Radyan cinsinden yön açısı
};

/////////////////////////////////////////////////////
// HEDEF NOKTA TABLOSU (ARENA HARITASI)
/////////////////////////////////////////////////////
struct Location {
  const char* id;
  int x;
  int y;
};

Location locations[] = {
  {"MERKEZ", 1800, 1800},
  {"SARNICSOL", 1382, 1105},
  {"SARNICSAG", 1879, 1105},
  {"BASLANGIC", 200, 200}
};

/////////////////////////////////////////////////////
// DONANIM NESNELERI
/////////////////////////////////////////////////////
MPU6050 mpu;
HardwareSerial uart1(1);

/////////////////////////////////////////////////////
// GLOBAL DURUM DEGISKENLERI
/////////////////////////////////////////////////////
volatile long enc_fl_cnt = 0;
volatile long enc_fr_cnt = 0;
volatile float gyro_z_offset = 0;

Pose estimatedPose = {0,0,0};

/////////////////////////////////////////////////////
// ENCODER INTERRUPT SERVIS FONKSIYONLARI
/////////////////////////////////////////////////////
void IRAM_ATTR isr_enc_fl() { enc_fl_cnt++; }
void IRAM_ATTR isr_enc_fr() { enc_fr_cnt++; }

/////////////////////////////////////////////////////
// SONAR OKUMA FONKSIYONU (mm DONER)
/////////////////////////////////////////////////////
float readSonarMM(int trig, int echo) {
  digitalWrite(trig, LOW); delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);

  long dur = pulseIn(echo, HIGH, 25000);
  if (dur == 0) return 4000; // Okuma yoksa max mesafe varsay

  float cm = (dur * 0.034 / 2.0);
  return cm * 10.0f; // mm'ye çevir
}

/////////////////////////////////////////////////////
// ACILARI -PI ILE +PI ARASINDA TUT
/////////////////////////////////////////////////////
float normAngle(float a) {
  while (a > PI) a -= 2 * PI;
  while (a < -PI) a += 2 * PI;
  return a;
}

/////////////////////////////////////////////////////
// BASLANGICTA OTOMATIK KONUM BULMA
// Robot yavas doner ve duvarlari kullanarak
// X, Y ve YON acisini otomatik hesaplar
/////////////////////////////////////////////////////
void autoLocalize() {
  Serial.println(">> Baslangic konumu bulunuyor...");

  float bestLeft = 9999;
  float bestBack = 9999;
  float bestAngle = 0;

  // Robot yavasca donerek duvar mesafelerini tarar
  for (int angle = 0; angle < 360; angle += 3) {

    // Sadece sinyal bekleniyor — motor sürme diğer ESP32’de olacak
    uart1.println("CMD V=0 W=0.8"); // yavas donus
    delay(20);

    float left = readSonarMM(HC_TRIG_LEFT, HC_ECHO_LEFT);
    float back = readSonarMM(HC_TRIG_BACK, HC_ECHO_BACK);

    // En stabil ve en yakin duvar kombinasyonu secilir
    if (left < bestLeft && back < bestBack) {
      bestLeft = left;
      bestBack = back;
      bestAngle = angle * DEG_TO_RAD;
    }
  }

  // Donusu durdur
  uart1.println("CMD V=0 W=0");

  // Baslangic pozisyonunu sabitle
  estimatedPose.x = bestLeft;
  estimatedPose.y = bestBack;
  estimatedPose.theta = bestAngle;

  Serial.println(">> Baslangic pozu belirlendi!");
}

/////////////////////////////////////////////////////
void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);
  mpu.initialize();

  // GYRO OFFSET KALIBRASYONU
  long sum = 0;
  for(int i=0; i<200; i++){
    sum += mpu.getRotationZ();
    delay(3);
  }
  gyro_z_offset = sum / 200.0;

  // SONAR PIN AYARLARI
  pinMode(HC_TRIG_FRONT, OUTPUT);
  pinMode(HC_ECHO_FRONT, INPUT);
  pinMode(HC_TRIG_RIGHT, OUTPUT);
  pinMode(HC_ECHO_RIGHT, INPUT);
  pinMode(HC_TRIG_LEFT, OUTPUT);
  pinMode(HC_ECHO_LEFT, INPUT);
  pinMode(HC_TRIG_BACK, OUTPUT);
  pinMode(HC_ECHO_BACK, INPUT);

  // ENCODER PIN AYARI
  pinMode(ENC_FL_CLK, INPUT_PULLUP);
  pinMode(ENC_FR_CLK, INPUT_PULLUP);
  attachInterrupt(ENC_FL_CLK, isr_enc_fl, RISING);
  attachInterrupt(ENC_FR_CLK, isr_enc_fr, RISING);

  // UART BASLAT (Motor kontrol ESP32’ye bagli)
  uart1.begin(115200, SERIAL_8N1, UART1_RX, UART1_TX);

  // BASLANGICTA POZ BUL
  autoLocalize();
}

/////////////////////////////////////////////////////
// HEDEF SECIMI
/////////////////////////////////////////////////////
const char* targetID = "SARNICSOL";

/////////////////////////////////////////////////////
void loop() {

  /////////////////////////////////////////////////////
  // 1) GYRO OKU → ACI HIZI
  /////////////////////////////////////////////////////
  float gz = (mpu.getRotationZ() - gyro_z_offset) / 131.0;
  float gyro_rad_s = gz * DEG_TO_RAD;

  /////////////////////////////////////////////////////
  // 2) ENCODER OKU → ILERLEME MESAFESI
  /////////////////////////////////////////////////////
  static long old_fl = 0, old_fr = 0;
  long fl = enc_fl_cnt;
  long fr = enc_fr_cnt;

  long dfl = fl - old_fl;
  long dfr = fr - old_fr;
  old_fl = fl;
  old_fr = fr;

  float dLeft = dfl * DIST_PER_TICK_MM;
  float dRight = dfr * DIST_PER_TICK_MM;
  float dCenter = (dLeft + dRight) / 2.0;

  /////////////////////////////////////////////////////
  // 3) ODOMETRI → TAHMINI POZ GUNCELLE
  /////////////////////////////////////////////////////
  estimatedPose.theta = normAngle(estimatedPose.theta + gyro_rad_s * LOOP_DT);
  estimatedPose.x += dCenter * cos(estimatedPose.theta);
  estimatedPose.y += dCenter * sin(estimatedPose.theta);

  /////////////////////////////////////////////////////
  // 4) SONAR ILE KONUM DUZELTME
  /////////////////////////////////////////////////////
  float sonarX = readSonarMM(HC_TRIG_LEFT, HC_ECHO_LEFT);
  float sonarY = readSonarMM(HC_TRIG_BACK, HC_ECHO_BACK);

  float alpha = 0.85; // Encoder agirligi (0.0–1.0)
  estimatedPose.x = alpha * estimatedPose.x + (1 - alpha) * sonarX;
  estimatedPose.y = alpha * estimatedPose.y + (1 - alpha) * sonarY;

  /////////////////////////////////////////////////////
  // 5) HEDEF KOORDINATINI BUL
  /////////////////////////////////////////////////////
  int X_target = 0, Y_target = 0;
  for (auto &loc : locations) {
    if (strcmp(loc.id, targetID) == 0) {
      X_target = loc.x;
      Y_target = loc.y;
    }
  }

  /////////////////////////////////////////////////////
  // 6) HEDEFE GORE ACI VE MESAFE HESAPLA
  /////////////////////////////////////////////////////
  float dx = X_target - estimatedPose.x;
  float dy = Y_target - estimatedPose.y;

  float heading = atan2(dy, dx);
  float distance = sqrt(dx*dx + dy*dy);
  float angErr = normAngle(heading - estimatedPose.theta);

  /////////////////////////////////////////////////////
  // 7) HIZ KONTROLU (NORMAL SURUS)
  /////////////////////////////////////////////////////
  float v = constrain(distance * 0.6, 0, 300); // ileri hiz
  float w = constrain(angErr * 2.0, -2.5, 2.5); // donus hizi

  /////////////////////////////////////////////////////
  // 8) DOCKING MODU (HASSAS YAKLASMA)
  /////////////////////////////////////////////////////
  if (distance < 200) {
    v = constrain(distance * 0.25, 0, 80);
    w = constrain(angErr * 3.0, -1.2, 1.2);
  }

  // HEDEFE ULASINCA DUR
  if (distance < 10) {
    v = 0;
    w = 0;
  }

  /////////////////////////////////////////////////////
  // 9) MOTOR KOMUTUNU DIGER ESP32'YE GONDER
  /////////////////////////////////////////////////////
  uart1.print("CMD V=");
  uart1.print(v, 2);
  uart1.print(" W=");
  uart1.println(w, 3);

  /////////////////////////////////////////////////////
  // 10) DEBUG TELEMETRI (USB SERIAL)
  /////////////////////////////////////////////////////
  Serial.print("POS: ");
  Serial.print(estimatedPose.x); Serial.print(", ");
  Serial.print(estimatedPose.y);
  Serial.print("  TH: ");
  Serial.print(estimatedPose.theta * 57.3);
  Serial.print("  DIST: ");
  Serial.println(distance);

  delay(20);
}
