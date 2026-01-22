#include <WiFi.h>
#include <WiFiUdp.h>

// ======================= PIN TANIMLARI (DEĞİŞTİRME) =======================
// SAĞ GRUP (Fiziksel Sağ)
#define FL_AIN1 5
#define FL_AIN2 4
#define FL_PWM  6

#define RL_AIN1 11
#define RL_AIN2 12
#define RL_PWM  13

// SOL GRUP (Fiziksel Sol)
#define FR_AIN1 15
#define FR_AIN2 16
#define FR_PWM  17

#define RR_AIN1 2
#define RR_AIN2 42
#define RR_PWM  41

// ======================= WIFI AYARLARI =======================
const char* ssid = "LAGARIMEDYA";
const char* password = "lagari5253";
WiFiUDP udp;
unsigned int localPort = 4210;
char packetBuffer[255];

// ======================= DEĞİŞKENLER =======================
// Varsayılan -1.0 (Basılı değil)
float axis_R_Fwd = -1.0;  // Axis 5 (Sağ İleri)
float axis_R_Back = -1.0; // Axis 11 (Sağ Geri)
float axis_L_Fwd = -1.0;  // Axis 4 (Sol İleri)
float axis_L_Back = -1.0; // Axis 10 (Sol Geri)

unsigned long lastPacketTime = 0;
const int MAX_SPEED = 255; // Hız limiti

void setup() {
  Serial.begin(115200);

  // Pin Modları
  pinMode(RR_AIN1, OUTPUT); pinMode(RR_AIN2, OUTPUT); pinMode(RR_PWM, OUTPUT);
  pinMode(RL_AIN1, OUTPUT); pinMode(RL_AIN2, OUTPUT); pinMode(RL_PWM, OUTPUT);
  pinMode(FR_AIN1, OUTPUT); pinMode(FR_AIN2, OUTPUT); pinMode(FR_PWM, OUTPUT);
  pinMode(FL_AIN1, OUTPUT); pinMode(FL_AIN2, OUTPUT); pinMode(FL_PWM, OUTPUT);

  // WiFi Bağlantısı
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(500);
  udp.begin(localPort);
}

void loop() {
  int packetSize = udp.parsePacket();

  if (packetSize) {
    udp.read(packetBuffer, 255);
    String msg = String(packetBuffer);
    lastPacketTime = millis();

    parseCommand(msg);
    processMovement();
  }

  // 2 saniye veri gelmezse dur
  if (millis() - lastPacketTime > 2000) {
    stopMotors();
  }
}

// Gelen veriyi (AXIS 5 -1.000 vb) ayıkla
void parseCommand(String msg) {
  if (msg.startsWith("AXIS")) {
    int s1 = msg.indexOf(' ');
    int s2 = msg.indexOf(' ', s1 + 1);
    int id = msg.substring(s1 + 1, s2).toInt();
    float val = msg.substring(s2 + 1).toFloat();

    if (id == 5)       axis_R_Fwd = val;
    else if (id == 11) axis_R_Back = val;
    else if (id == 4)  axis_L_Fwd = val;
    else if (id == 10) axis_L_Back = val;
  }
}

// 0-1 arasına çekme fonksiyonu
float normalize(float input) {
  float n = (input + 1.0) / 2.0;
  if (n < 0.1) return 0.0; // Deadzone
  if (n > 1.0) return 1.0;
  return n;
}

void processMovement() {
  // --- SOL TARAFI KONTROL ET ---
  int leftPWM = 0;
  float val_L_Fwd = normalize(axis_L_Fwd);
  float val_L_Back = normalize(axis_L_Back);

  if (val_L_Fwd > 0.1 && val_L_Back < 0.1) {
    // Sadece Axis 4 (İleri) basılı -> POZİTİF GÜÇ
    leftPWM = (int)(val_L_Fwd * MAX_SPEED);
  } 
  else if (val_L_Back > 0.1 && val_L_Fwd < 0.1) {
    // Sadece Axis 10 (Geri) basılı -> NEGATİF GÜÇ
    leftPWM = -(int)(val_L_Back * MAX_SPEED);
  } 
  else {
    // İkisi de basılıysa veya hiçbiri basılı değilse DUR
    leftPWM = 0;
  }

  // --- SAĞ TARAFI KONTROL ET ---
  int rightPWM = 0;
  float val_R_Fwd = normalize(axis_R_Fwd);
  float val_R_Back = normalize(axis_R_Back);

  if (val_R_Fwd > 0.1 && val_R_Back < 0.1) {
    // Sadece Axis 5 (İleri) basılı.
    // DİKKAT: Sağ taraf ileri gitmek için NEGATİF sinyal istiyor (Orijinal koda göre)
    rightPWM = -(int)(val_R_Fwd * MAX_SPEED);
  } 
  else if (val_R_Back > 0.1 && val_R_Fwd < 0.1) {
    // Sadece Axis 11 (Geri) basılı.
    // DİKKAT: Sağ taraf geri gitmek için POZİTİF sinyal istiyor
    rightPWM = (int)(val_R_Back * MAX_SPEED);
  } 
  else {
    rightPWM = 0;
  }

  // --- MOTORLARA GÖNDER ---
  // SOL MOTORLAR (Axis 4/10)
  setOneMotor(FR_AIN1, FR_AIN2, FR_PWM, leftPWM);
  setOneMotor(RR_AIN1, RR_AIN2, RR_PWM, leftPWM);

  // SAĞ MOTORLAR (Axis 5/11)
  setOneMotor(FL_AIN1, FL_AIN2, FL_PWM, rightPWM);
  setOneMotor(RL_AIN1, RL_AIN2, RL_PWM, rightPWM);
}

void stopMotors() {
  setOneMotor(FR_AIN1, FR_AIN2, FR_PWM, 0);
  setOneMotor(RR_AIN1, RR_AIN2, RR_PWM, 0);
  setOneMotor(FL_AIN1, FL_AIN2, FL_PWM, 0);
  setOneMotor(RL_AIN1, RL_AIN2, RL_PWM, 0);
}

// Tekil motor kontrolcüsü
void setOneMotor(int p1, int p2, int pwm, int val) {
  if (val > 255) val = 255;
  if (val < -255) val = -255;

  if (val > 0) {
    // Pozitif Değer: IN1 HIGH, IN2 LOW
    digitalWrite(p1, HIGH);
    digitalWrite(p2, LOW);
    analogWrite(pwm, val);
  } else if (val < 0) {
    // Negatif Değer: IN1 LOW, IN2 HIGH
    digitalWrite(p1, LOW);
    digitalWrite(p2, HIGH);
    analogWrite(pwm, -val); // Mutlak değer PWM'e
  } else {
    // Dur
    digitalWrite(p1, LOW);
    digitalWrite(p2, LOW);
    analogWrite(pwm, 0);
  }
}