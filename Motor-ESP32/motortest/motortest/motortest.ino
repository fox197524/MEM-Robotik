#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// --- MOTOR PIN DEFINITIONS ---
// Rear Left
const int RL_PIN_PWM = 7;
const int RL_PIN_IN2 = 8;
const int RL_PIN_IN1 = 9;

// Rear Right
const int RR_PIN_PWM = 11;
const int RR_PIN_IN2 = 13;
const int RR_PIN_IN1 = 12;

// Front Left
const int FL_PIN_PWM = 15;
const int FL_PIN_IN2 = 16;
const int FL_PIN_IN1 = 17;

// Front Right
const int FR_PIN_PWM = 4;
const int FR_PIN_IN2 = 5;
const int FR_PIN_IN1 = 6;

// ======================= WIFI AYARLARI =======================
const char* ssid = "LAGARIMEDYA";
const char* password = "lagari5253";
WiFiUDP udp;
unsigned int localPort = 4210;
char packetBuffer[255];
unsigned long lastPacketTime = 0;

// ======================= DEĞİŞKENLER =======================
// Gelen verileri tutacak eksen değişkenleri (Global)
float axis0 = 0; // Sol/Sağ Dönüş (Joystick)
float axis2 = 0; // Sola Kayma (Trigger?)
float axis4 = -1; // Geri (Trigger/Button)
float axis5 = -1; // İleri (Trigger/Button)

// Fonksiyon Prototipleri
void ileri(int speed);
void geri(int speed);
void sol(int speed);
void sag(int speed);
void sag360(int speed);
void sol360(int speed);
void dur();
void parseCommand(String msg);
void moveVehicle();

void setup() {
  Serial.begin(115200);
  Serial.println("Code by 'Dead To AI' Community");

  // WiFi Başlatma
  WiFi.begin(ssid, password);
  Serial.print("WiFi'ye baglaniliyor");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

  }
  Serial.println("\nBaglandi!");
  udp.begin(localPort);

  // Pin Modları
  pinMode(RL_PIN_PWM, OUTPUT); pinMode(RL_PIN_IN1, OUTPUT); pinMode(RL_PIN_IN2, OUTPUT);
  pinMode(RR_PIN_PWM, OUTPUT); pinMode(RR_PIN_IN1, OUTPUT); pinMode(RR_PIN_IN2, OUTPUT);
  pinMode(FL_PIN_PWM, OUTPUT); pinMode(FL_PIN_IN1, OUTPUT); pinMode(FL_PIN_IN2, OUTPUT);
  pinMode(FR_PIN_PWM, OUTPUT); pinMode(FR_PIN_IN1, OUTPUT); pinMode(FR_PIN_IN2, OUTPUT);
}

void loop() {
  int packetSize = udp.parsePacket();
  
  if (packetSize) {
    int len = udp.read(packetBuffer, 255);
    if (len > 0) packetBuffer[len] = 0;
    
    String msg = String(packetBuffer);
    lastPacketTime = millis(); // Son veri zamanını güncelle
    
    parseCommand(msg); // Mesajı parçala ve axis değişkenlerini güncelle
    moveVehicle();     // Motorları yeni değerlere göre hareket ettir
  }

  // Güvenlik: 2 saniye sinyal yoksa dur
  if (millis() - lastPacketTime > 2000) {
    dur();
  }
}

// ======================= KOMUT PARÇALAMA =======================
// Bu fonksiyon gelen mesajın "0.5,0.1,1.0..." gibi virgülle ayrıldığını varsayar.
// Controller uygulamanızın gönderdiği formata göre burayı güncellemek gerekebilir.
void parseCommand(String msg) {
  // Basit CSV parser örneği (Axis0, Axis1, Axis2, Axis3, Axis4, Axis5 varsayımı)
  // Eğer formatınız farklıysa lütfen belirtin!
  
  int firstComma = msg.indexOf(',');
  int secondComma = msg.indexOf(',', firstComma + 1);
  int thirdComma = msg.indexOf(',', secondComma + 1);
  int fourthComma = msg.indexOf(',', thirdComma + 1);
  int fifthComma = msg.indexOf(',', fourthComma + 1);

  // String'den float'a çeviriyoruz
  // Not: Gelen verinin sırası önemli. Burayı kendi uygulamanıza göre kontrol edin.
  // Varsayım: Axis0, Axis1, Axis2, Axis3, Axis4, Axis5
  if (firstComma > 0) axis0 = msg.substring(0, firstComma).toFloat();
  if (secondComma > 0) axis2 = msg.substring(secondComma + 1, thirdComma).toFloat(); // Aradaki axis1'i atladım örnek olarak
  if (fourthComma > 0) axis4 = msg.substring(fourthComma + 1, fifthComma).toFloat(); 
  if (fifthComma > 0) axis5 = msg.substring(fifthComma + 1).toFloat();
  
  // Hata ayıklama için (Serial monitörden değerleri kontrol edebilirsiniz)
  // Serial.print("A0: "); Serial.print(axis0);
  // Serial.print(" A2: "); Serial.print(axis2);
  // Serial.print(" A4: "); Serial.print(axis4);
  // Serial.print(" A5: "); Serial.println(axis5);
}

// ======================= HAREKET MANTIĞI =======================
void moveVehicle() {
  int pwmVal = 0;
  float deadzone = 0.1; // Joystick hassasiyeti (titremeyi önlemek için)

  // 1. ÖNCELİK: DÖNÜŞ (Axis 0)
  if (abs(axis0) > deadzone) {
    pwmVal = map(abs(axis0) * 100, 0, 100, 60, 255); // 0-1 arası değeri 60-255 PWM'e çevir
    
    if (axis0 > 0) {
      // Axis 0 Pozitif -> SAĞA 360
      sag360(pwmVal);
    } else {
      // Axis 0 Negatif -> SOLA 360
      sol360(pwmVal);
    }
  }
  // 2. ÖNCELİK: SOLA KAYMA (Axis 2)
  else if (axis2 > deadzone) {
    // Axis 2 değerini (0.0 - 1.0) PWM (0 - 255) değerine çeviriyoruz
    pwmVal = map(axis2 * 100, 0, 100, 60, 255);
    sol(pwmVal); // Dinamik PWM değeri ile fonksiyonu çağır
  }
  // 3. ÖNCELİK: İLERİ (Axis 5)
  else if (axis5 > deadzone) {
    pwmVal = map(axis5 * 100, 0, 100, 60, 255);
    ileri(pwmVal);
  }
  // 4. ÖNCELİK: GERİ (Axis 4)
  else if (axis4 > deadzone) {
    pwmVal = map(axis4 * 100, 0, 100, 60, 255);
    geri(pwmVal);
  }
  // HİÇBİR GİRDİ YOKSA -> DUR
  else {
    dur();
  }
}


// ====================== MOTOR FONKSİYONLARI =======================

// İleri: Tüm tekerler ileri
void ileri(int speed) {
  digitalWrite(RR_PIN_IN1, HIGH); digitalWrite(RR_PIN_IN2, LOW);
  digitalWrite(FR_PIN_IN1, HIGH); digitalWrite(FR_PIN_IN2, LOW);
  digitalWrite(RL_PIN_IN1, HIGH); digitalWrite(RL_PIN_IN2, LOW);
  digitalWrite(FL_PIN_IN1, HIGH); digitalWrite(FL_PIN_IN2, LOW);

  analogWrite(RR_PIN_PWM, speed);
  analogWrite(RL_PIN_PWM, speed);
  analogWrite(FL_PIN_PWM, speed);
  analogWrite(FR_PIN_PWM, speed);
}

// Geri: Tüm tekerler geri
void geri(int speed) {
  digitalWrite(RR_PIN_IN1, LOW); digitalWrite(RR_PIN_IN2, HIGH);
  digitalWrite(FR_PIN_IN1, LOW); digitalWrite(FR_PIN_IN2, HIGH);
  digitalWrite(RL_PIN_IN1, LOW); digitalWrite(RL_PIN_IN2, HIGH);
  digitalWrite(FL_PIN_IN1, LOW); digitalWrite(FL_PIN_IN2, HIGH);

  analogWrite(RR_PIN_PWM, speed);
  analogWrite(RL_PIN_PWM, speed);
  analogWrite(FL_PIN_PWM, speed);
  analogWrite(FR_PIN_PWM, speed);
}

// Sağa Kayma (Mecanum/Omni mantığı)
void sag(int speed) {
  digitalWrite(RR_PIN_IN1, HIGH); digitalWrite(RR_PIN_IN2, LOW); // Sol Arka İleri mi? (Mecanum mantığına göre değişir)
  // Standart Mecanum Sağa: FL İleri, RR İleri, FR Geri, RL Geri
  // Senin kodundaki mantığı korudum:
  digitalWrite(FR_PIN_IN1, LOW);  digitalWrite(FR_PIN_IN2, HIGH);
  digitalWrite(RL_PIN_IN1, LOW);  digitalWrite(RL_PIN_IN2, HIGH);
  digitalWrite(FL_PIN_IN1, HIGH); digitalWrite(FL_PIN_IN2, LOW);
  
  // Eksik olan RR mantığını ekledim (Senin kodunda vardı):
  digitalWrite(RR_PIN_IN1, HIGH); digitalWrite(RR_PIN_IN2, LOW);

  analogWrite(RR_PIN_PWM, speed);
  analogWrite(RL_PIN_PWM, speed);
  analogWrite(FL_PIN_PWM, speed);
  analogWrite(FR_PIN_PWM, speed);
}

// Sola Kayma (Axis 2 ile kontrol edilecek)
void sol(int speed) {
  // Senin kodundaki mantığı koruyarak speed değişkenini ekledim
  digitalWrite(RR_PIN_IN1, LOW);  digitalWrite(RR_PIN_IN2, HIGH);
  digitalWrite(FR_PIN_IN1, HIGH); digitalWrite(FR_PIN_IN2, LOW);
  digitalWrite(RL_PIN_IN1, HIGH); digitalWrite(RL_PIN_IN2, LOW);
  digitalWrite(FL_PIN_IN1, LOW);  digitalWrite(FL_PIN_IN2, HIGH);

  analogWrite(RR_PIN_PWM, speed);
  analogWrite(RL_PIN_PWM, speed);
  analogWrite(FL_PIN_PWM, speed);
  analogWrite(FR_PIN_PWM, speed);
}

// Sola 360 Dönüş (Kendi ekseni etrafında)
// Sol taraf geri, Sağ taraf ileri
void sol360(int speed) {
  digitalWrite(RR_PIN_IN1, HIGH); digitalWrite(RR_PIN_IN2, LOW);
  digitalWrite(FR_PIN_IN1, HIGH); digitalWrite(FR_PIN_IN2, LOW);
  digitalWrite(RL_PIN_IN1, LOW);  digitalWrite(RL_PIN_IN2, HIGH);
  digitalWrite(FL_PIN_IN1, LOW);  digitalWrite(FL_PIN_IN2, HIGH);

  analogWrite(RR_PIN_PWM, speed);
  analogWrite(RL_PIN_PWM, speed);
  analogWrite(FL_PIN_PWM, speed);
  analogWrite(FR_PIN_PWM, speed);
}

// Sağa 360 Dönüş (Kendi ekseni etrafında)
// Sol taraf ileri, Sağ taraf geri
void sag360(int speed) {
  digitalWrite(RR_PIN_IN1, LOW);  digitalWrite(RR_PIN_IN2, HIGH);
  digitalWrite(FR_PIN_IN1, LOW);  digitalWrite(FR_PIN_IN2, HIGH);
  digitalWrite(RL_PIN_IN1, HIGH); digitalWrite(RL_PIN_IN2, LOW);
  digitalWrite(FL_PIN_IN1, HIGH); digitalWrite(FL_PIN_IN2, LOW);

  analogWrite(RR_PIN_PWM, speed);
  analogWrite(RL_PIN_PWM, speed);
  analogWrite(FL_PIN_PWM, speed);
  analogWrite(FR_PIN_PWM, speed);
}

// Dur
void dur() {
  digitalWrite(RR_PIN_IN1, LOW); digitalWrite(RR_PIN_IN2, LOW);
  digitalWrite(RL_PIN_IN1, LOW); digitalWrite(RL_PIN_IN2, LOW);
  digitalWrite(FL_PIN_IN1, LOW); digitalWrite(FL_PIN_IN2, LOW);
  digitalWrite(FR_PIN_IN1, LOW); digitalWrite(FR_PIN_IN2, LOW);

  analogWrite(RR_PIN_PWM, 0);
  analogWrite(RL_PIN_PWM, 0);
  analogWrite(FL_PIN_PWM, 0);
  analogWrite(FR_PIN_PWM, 0);
}