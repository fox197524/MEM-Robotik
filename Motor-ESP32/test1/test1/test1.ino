#include <WiFi.h>       // ESP32'nin WiFi özelliklerini kullanabilmesi için gerekli kütüphane.
#include <WiFiUdp.h>    // Hızlı veri transferi sağlayan UDP protokolü kütüphanesi.

// ======================= PIN AYARLARI =======================
// Motor sürücü (TB6612FNG) pin tanımlamaları.
// AIN1/AIN2 yönü belirler, PWM hızı belirler.

// --- 1. GRUP (Axis 4 ve 10 Kontrol Ediyor - Fiziksel SAĞ Taraf) ---
// NOT: Bu grup senin aracında fiziksel olarak SAĞ taraftaki motorlara bağlı.
// FL (Front Left) ismi kodda kaldı ama işlevi SAĞ ÖN olarak çalışıyor.
#define FL_AIN1 5   // Yön Pini 1 (Motorun ters dönmesini düzeltmek için 4 yerine 5 yaptık)
#define FL_AIN2 4   // Yön Pini 2 (Motorun ters dönmesini düzeltmek için 5 yerine 4 yaptık)
#define FL_PWM  6   // Hız Kontrol Pini (0-255 arası değer alır)

// RL (Rear Left) - Kodda Sol Arka ama fiziksel olarak SAĞ ARKA motor.
// Bu motorun yönü doğru olduğu için pinlerine dokunmadık.
#define RL_AIN1 11  // Yön Pini 1
#define RL_AIN2 12  // Yön Pini 2
#define RL_PWM  13  // Hız Kontrol Pini


// --- 2. GRUP (Axis 5 ve 11 Kontrol Ediyor - Fiziksel SOL Taraf) ---
// FR (Front Right) - Kodda Sağ Ön ama fiziksel olarak SOL ÖN motor.
#define FR_AIN1 15  // Yön Pini 1
#define FR_AIN2 16  // Yön Pini 2
#define FR_PWM  17  // Hız Kontrol Pini

// RR (Rear Right) - Kodda Sağ Arka ama fiziksel olarak SOL ARKA motor.
#define RR_AIN1 2   // Yön Pini 1
#define RR_AIN2 42  // Yön Pini 2
#define RR_PWM  41  // Hız Kontrol Pini

// ======================= WIFI AYARLARI =======================
const char* ssid = "LAGARIMEDYA";      // Bağlanılacak WiFi ağının adı.
const char* password = "lagari5253";   // WiFi ağının şifresi.
WiFiUDP udp;                           // UDP nesnesi oluşturuluyor (haberleşme için).
unsigned int localPort = 4210;         // ESP32'nin dinleyeceği port numarası.
char packetBuffer[255];                // Gelen veriyi geçici olarak tutacak hafıza alanı (Buffer).

// ======================= KONTROL DEĞİŞKENLERİ =======================
// Joystick'ten gelen verileri saklamak için değişkenler.
// Başlangıç değeri -1.0 (PS5 kolunda tetik bırakıldığında değer -1 olur).
float val_left_fwd_cmd = -1.0;   // Sol tarafı ileri götürme komutu (Axis 5)
float val_left_back_cmd = -1.0;  // Sol tarafı geri götürme komutu (Axis 11)
float val_right_fwd_cmd = -1.0;  // Sağ tarafı ileri götürme komutu (Axis 4)
float val_right_back_cmd = -1.0; // Sağ tarafı geri götürme komutu (Axis 10)

unsigned long lastPacketTime = 0; // Son veri paketinin ne zaman geldiğini tutar (Zaman aşımı kontrolü için).
const int MAX_SPEED = 255;        // Motorlara verilecek maksimum PWM hızı (0-255 arası).

void setup() {
  Serial.begin(115200); // Seri haberleşmeyi başlat (Bilgisayara veri yazdırmak için).
  
  // WiFi Bağlantısını Başlat
  WiFi.begin(ssid, password);
  
  // Bağlantı kurulana kadar bekle
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);         // Yarım saniye bekle.
    // (Buraya ekrana nokta yazdırma vs eklenebilir ama şu an sade bıraktık)
  }
  
  udp.begin(localPort); // UDP dinlemesini başlat (Veri kabul etmeye başla).

  // Pin Modlarını Ayarla
  // Tüm motor pinlerini OUTPUT (Çıkış) olarak ayarlıyoruz çünkü onlara elektrik vereceğiz.
  pinMode(RR_AIN1, OUTPUT); pinMode(RR_AIN2, OUTPUT); pinMode(RR_PWM, OUTPUT);
  pinMode(RL_AIN1, OUTPUT); pinMode(RL_AIN2, OUTPUT); pinMode(RL_PWM, OUTPUT);
  pinMode(FR_AIN1, OUTPUT); pinMode(FR_AIN2, OUTPUT); pinMode(FR_PWM, OUTPUT);
  pinMode(FL_AIN1, OUTPUT); pinMode(FL_AIN2, OUTPUT); pinMode(FL_PWM, OUTPUT);
}

void loop() {
  // Gelen bir UDP paketi var mı kontrol et.
  int packetSize = udp.parsePacket();
  
  if (packetSize) { // Eğer paket boyutu 0'dan büyükse (yani veri geldiyse):
    
    // Veriyi oku ve 'packetBuffer' içine yaz.
    int len = udp.read(packetBuffer, 255);
    
    if (len > 0) packetBuffer[len] = 0; // Gelen verinin sonuna 'bitiş karakteri' (null terminator) koy.
    
    String msg = String(packetBuffer); // Karakter dizisini String objesine çevir (işlemesi kolay olsun diye).
    lastPacketTime = millis();         // Son veri alma zamanını şu anki zaman olarak güncelle.
    
    parseCommand(msg); // Gelen mesajı parçala ve anlamlandır.
    moveVehicle();     // Motorları yeni verilere göre sür.
  }

  // Güvenlik Kontrolü: Eğer 2000 ms (2 saniye) boyunca hiç yeni veri gelmezse...
  // Bu, WiFi koptuğunda veya Python programı kapandığında aracın kontrolden çıkmasını engeller.
  if (millis() - lastPacketTime > 2000) {
    stopMotors(); // Tüm motorları durdur.
  }
}

// Gelen metin mesajını (örn: "AXIS 4 0.53") sayısal verilere çeviren fonksiyon.
void parseCommand(String msg) {
  if (msg.startsWith("AXIS")) { // Eğer mesaj "AXIS" kelimesi ile başlıyorsa:
    
    // Boşluk karakterlerinin yerini bul
    int firstSpace = msg.indexOf(' ');
    int secondSpace = msg.indexOf(' ', firstSpace + 1);
    
    // Boşlukların arasındaki sayı Axis ID'sidir (4, 5, 10, 11 gibi).
    int axisId = msg.substring(firstSpace + 1, secondSpace).toInt();
    
    // İkinci boşluktan sonraki kısım Joystick değeridir (-1.0 ile 1.0 arası).
    float val = msg.substring(secondSpace + 1).toFloat();
    
    // === HARİTALAMA (Hangi Axis Hangi Motor Grubunu Kontrol Edecek?) ===
    
    // Axis 4 -> SAĞ Grubu İLERİ sürer
    if (axisId == 4) {
      val_right_fwd_cmd = val;
    }
    // Axis 10 -> SAĞ Grubu GERİ sürer
    else if (axisId == 10) {
      val_right_back_cmd = val;
    }
    // Axis 5 -> SOL Grubu İLERİ sürer
    else if (axisId == 5) {
      val_left_fwd_cmd = val;
    }
    // Axis 11 -> SOL Grubu GERİ sürer
    else if (axisId == 11) {
      val_left_back_cmd = val;
    }
  }
}

// Joystick'ten gelen -1.0 (Bırakılmış) ile 1.0 (Basılmış) değerini
// 0.0 (Dur) ile 1.0 (Tam Gaz) arasına çeviren matematiksel fonksiyon.
float normalizeInput(float input) {
  // Formül: (-1 + 1) / 2 = 0  ||  (1 + 1) / 2 = 1
  float norm = (input + 1.0) / 2.0;
  
  // Ölü Bölge (Deadzone): Eğer değer 0.05'ten küçükse parmak titremesidir, 0 kabul et.
  if (norm < 0.05) return 0.0;
  
  // Eğer matematiksel hata ile 1.0'ı geçerse 1.0'a sabitle.
  if (norm > 1.0) return 1.0;
  
  return norm; // Temizlenmiş değeri döndür.
}

// Motor hızlarını hesaplayıp sürücüye gönderen ana sürüş fonksiyonu.
void moveVehicle() {
  // Önce tüm komutları 0.0 ile 1.0 arasına normalize et.
  float p_L_fwd = normalizeInput(val_left_fwd_cmd); 
  float p_L_back = normalizeInput(val_left_back_cmd);
  float p_R_fwd = normalizeInput(val_right_fwd_cmd);
  float p_R_back = normalizeInput(val_right_back_cmd);

  // Net Gücü Hesapla: (İleri gitme isteği) - (Geri gitme isteği)
  // Sonuç Pozitifse -> İleri gitmeli.
  // Sonuç Negatifse -> Geri gitmeli.
  float left_net = p_L_fwd - p_L_back;
  float right_net = p_R_fwd - p_R_back;

  // Hesaplanan 0-1 arası değeri PWM (0-255) değerine çevir.
  int leftPWM = (int)(left_net * MAX_SPEED);
  int rightPWM = (int)(right_net * MAX_SPEED);

  // SOL GRUP 
  // Sol tarafın PWM değerini gönder.
  setMotor(FL_AIN1, FL_AIN2, FL_PWM, leftPWM);
  setMotor(RL_AIN1, RL_AIN2, RL_PWM, leftPWM);

  // SAĞ GRUP
  // Sağ tarafın PWM değerini gönder.
  setMotor(FR_AIN1, FR_AIN2, FR_PWM, rightPWM);
  setMotor(RR_AIN1, RR_AIN2, RR_PWM, rightPWM);
}

// Tek bir motoru sürmek için yardımcı fonksiyon.
// pinIN1/IN2: Yön pinleri, pinPWM: Hız pini, speedVal: İstenen hız (-255 ile +255 arası)
void setMotor(int pinIN1, int pinIN2, int pinPWM, int speedVal) {
  if (speedVal > 0) {
    // === İLERİ YÖN ===
    digitalWrite(pinIN1, HIGH);         // Bir ucu Artı
    digitalWrite(pinIN2, LOW);          // Diğer ucu Eksi
    analogWrite(pinPWM, abs(speedVal)); // Hızı ayarla (Mutlak değer kullan, çünkü hız negatif olamaz)
  } else if (speedVal < 0) {
    // === GERİ YÖN ===
    digitalWrite(pinIN1, LOW);          // Bir ucu Eksi
    digitalWrite(pinIN2, HIGH);         // Diğer ucu Artı (Kutuplar değişti)
    analogWrite(pinPWM, abs(speedVal)); // Hızı ayarla
  } else {
    // === DUR (Fren veya Boşta) ===
    digitalWrite(pinIN1, LOW);          // İki uç da Eksi
    digitalWrite(pinIN2, LOW);          // Motor akım çekmez
    analogWrite(pinPWM, 0);             // Hız sıfır
  }
}

// Tüm motorları acil durduran fonksiyon (Güvenlik için).
void stopMotors() {
  // Tüm PWM (Hız) pinlerini 0 yap.
  analogWrite(FL_PWM, 0); analogWrite(RL_PWM, 0);
  analogWrite(FR_PWM, 0); analogWrite(RR_PWM, 0);
  
  // Tüm Yön pinlerini LOW (0) yap.
  digitalWrite(FL_AIN1, 0); digitalWrite(FL_AIN2, 0);
  digitalWrite(RL_AIN1, 0); digitalWrite(RL_AIN2, 0);
  digitalWrite(FR_AIN1, 0); digitalWrite(FR_AIN2, 0);
  digitalWrite(RR_AIN1, 0); digitalWrite(RR_AIN2, 0);
}