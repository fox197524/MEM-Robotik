#include <WiFi.h>
#include <WiFiUdp.h>

// ======================= PIN AYARLARI =======================

// --- SAĞ GRUP (Fiziksel Sağ) ---
// FL Etiketi kodda Front Left ama senin araçta SAĞ ÖN motor.
// NOT: Ters dönme sorununu çözmek için 4 ve 5'i ters tutmaya devam ediyoruz.
#define FL_AIN1 5   
#define FL_AIN2 4   
#define FL_PWM  6   

// RL Etiketi kodda Rear Left ama senin araçta SAĞ ARKA motor.
#define RL_AIN1 11  
#define RL_AIN2 12  
#define RL_PWM  13  

// --- SOL GRUP (Fiziksel Sol) ---
// FR Etiketi kodda Front Right ama senin araçta SOL ÖN motor.
#define FR_AIN1 15  
#define FR_AIN2 16  
#define FR_PWM  17  

// RR Etiketi kodda Rear Right ama senin araçta SOL ARKA motor.
#define RR_AIN1 2   
#define RR_AIN2 42  
#define RR_PWM  41  

// ======================= WIFI AYARLARI =======================
const char* ssid = "LAGARIMEDYA";
const char* password = "lagari5253";
WiFiUDP udp;
unsigned int localPort = 4210;
char packetBuffer[255];

// ======================= KONTROL DEĞİŞKENLERİ =======================
// Yeni istediğin eksenler:
float val_strafe = 0.0;     // Axis 0 (Sağa/Sola kayma)
float val_rotate = 0.0;     // Axis 1 (Kendi etrafında dönme)
float val_throttle_fwd = -1.0; // Axis 5 (İleri Gaz - Tetik)
float val_throttle_back = -1.0;// Axis 4 (Geri Gaz - Tetik)

unsigned long lastPacketTime = 0;
const int MAX_SPEED = 255; 

void setup() {
  Serial.begin(115200);
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  udp.begin(localPort);

  // Pin Modları
  pinMode(RR_AIN1, OUTPUT); pinMode(RR_AIN2, OUTPUT); pinMode(RR_PWM, OUTPUT);
  pinMode(RL_AIN1, OUTPUT); pinMode(RL_AIN2, OUTPUT); pinMode(RL_PWM, OUTPUT);
  pinMode(FR_AIN1, OUTPUT); pinMode(FR_AIN2, OUTPUT); pinMode(FR_PWM, OUTPUT);
  pinMode(FL_AIN1, OUTPUT); pinMode(FL_AIN2, OUTPUT); pinMode(FL_PWM, OUTPUT);
}

void loop() {
  int packetSize = udp.parsePacket();
  
  if (packetSize) {
    int len = udp.read(packetBuffer, 255);
    if (len > 0) packetBuffer[len] = 0;
    
    String msg = String(packetBuffer);
    lastPacketTime = millis();
    
    parseCommand(msg);
    moveVehicle();
  }

  // Güvenlik: 2 saniye sinyal yoksa dur
  if (millis() - lastPacketTime > 2000) {
    stopMotors();
  }
}

void parseCommand(String msg) {
  if (msg.startsWith("AXIS")) {
    int firstSpace = msg.indexOf(' ');
    int secondSpace = msg.indexOf(' ', firstSpace + 1);
    
    int axisId = msg.substring(firstSpace + 1, secondSpace).toInt();
    float val = msg.substring(secondSpace + 1).toFloat();
    
    // === YENİ KONTROL ŞEMASI ===
    
    // Axis 0: Yengeç Yürüyüşü (Strafe) - Sağa/Sola kayma
    if (axisId == 0) {
      val_strafe = val; // -1 (Sol) ... +1 (Sağ)
    }
    // Axis 1: Dönüş (Rotation) - Olduğu yerde dönme
    else if (axisId == 1) {
      val_rotate = val; // -1 (Sol Dönüş) ... +1 (Sağ Dönüş)
    }
    // Axis 5: İleri Gaz (Throttle Forward)
    else if (axisId == 5) {
      val_throttle_fwd = val;
    }
    // Axis 4: Geri Gaz (Throttle Backward)
    else if (axisId == 4) {
      val_throttle_back = val;
    }
  }
}

// -1 ile 1 arasındaki joystick verisini -1.0 ile 1.0 arasında temizle
float normalizeStick(float input) {
  if (abs(input) < 0.05) return 0.0; // Deadzone
  return input;
}

// Tetiği (Trigger) 0.0 ile 1.0 arasına çevir
float normalizeTrigger(float input) {
  // Tetikler bırakılınca -1, basılınca +1 verir. Bunu 0 ile 1 yapalım.
  float norm = (input + 1.0) / 2.0;
  if (norm < 0.05) return 0.0;
  if (norm > 1.0) return 1.0;
  return norm;
}

void moveVehicle() {
  // 1. GİRDİLERİ HAZIRLA
  
  // Gaz (İleri - Geri)
  float fwd = normalizeTrigger(val_throttle_fwd);  // Axis 5
  float back = normalizeTrigger(val_throttle_back);// Axis 4
  float throttle = fwd - back; // Pozitifse ileri, negatifse geri
  
  // Yönlendirme
  float strafe = normalizeStick(val_strafe); // Axis 0 (Sağ/Sol Kayma)
  float turn = normalizeStick(val_rotate);   // Axis 1 (Dönüş)

  // 2. MECANUM TEKERLEK MATEMATİĞİ (Vektörel Karışım)
  // İstediğin "Sadece Sağa Gitme" mantığı burada çalışır:
  // Strafe +1 (Sağ) olduğunda:
  // FL (Sol Ön) = (+1) -> İleri döner
  // FR (Sağ Ön) = (-1) -> Geri döner
  // RL (Sol Arka)= (-1) -> Geri döner
  // RR (Sağ Arka)= (+1) -> İleri döner
  
  // Kodda FR/RR Sol taraf, FL/RL Sağ taraf olduğu için haritalama şöyledir:
  
  // SOL ÖN (FR Pinleri)
  float motorFR_Power = throttle + strafe + turn;
  
  // SOL ARKA (RR Pinleri)
  float motorRR_Power = throttle - strafe + turn;
  
  // SAĞ ÖN (FL Pinleri - Fiziksel Sağ)
  float motorFL_Power = throttle - strafe - turn;
  
  // SAĞ ARKA (RL Pinleri - Fiziksel Sağ)
  float motorRL_Power = throttle + strafe - turn;

  // 3. HIZLARI MOTORLARA GÖNDER
  // Değerleri -255 ile 255 arasına ölçekle ve sınırla
  setMotor(FR_AIN1, FR_AIN2, FR_PWM, constrainPWM(motorFR_Power));
  setMotor(RR_AIN1, RR_AIN2, RR_PWM, constrainPWM(motorRR_Power));
  setMotor(FL_AIN1, FL_AIN2, FL_PWM, constrainPWM(motorFL_Power));
  setMotor(RL_AIN1, RL_AIN2, RL_PWM, constrainPWM(motorRL_Power));
}

// PWM Değerini -255 ile 255 arasına sıkıştıran yardımcı fonksiyon
int constrainPWM(float power) {
  // Power değişkeni genellikle -1 ile 1 arasında ama toplamda (1+1+1) 3'e çıkabilir.
  // Bu yüzden önce MAX_SPEED ile çarpıyoruz, sonra limitliyoruz.
  int pwm = (int)(power * MAX_SPEED);
  
  if (pwm > 255) return 255;
  if (pwm < -255) return -255;
  return pwm;
}

void setMotor(int pinIN1, int pinIN2, int pinPWM, int speedVal) {
  if (speedVal > 0) {
    // İLERİ
    digitalWrite(pinIN1, HIGH);
    digitalWrite(pinIN2, LOW);
    analogWrite(pinPWM, abs(speedVal));
  } else if (speedVal < 0) {
    // GERİ
    digitalWrite(pinIN1, LOW);
    digitalWrite(pinIN2, HIGH);
    analogWrite(pinPWM, abs(speedVal));
  } else {
    // DUR
    digitalWrite(pinIN1, LOW);
    digitalWrite(pinIN2, LOW);
    analogWrite(pinPWM, 0);
  }
}

void stopMotors() {
  analogWrite(FL_PWM, 0); analogWrite(RL_PWM, 0);
  analogWrite(FR_PWM, 0); analogWrite(RR_PWM, 0);
  digitalWrite(FL_AIN1, 0); digitalWrite(FL_AIN2, 0);
  digitalWrite(RL_AIN1, 0); digitalWrite(RL_AIN2, 0);
  digitalWrite(FR_AIN1, 0); digitalWrite(FR_AIN2, 0);
  digitalWrite(RR_AIN1, 0); digitalWrite(RR_AIN2, 0);
}