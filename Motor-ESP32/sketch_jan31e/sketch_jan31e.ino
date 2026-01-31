
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// --- MOTOR 4 PIN DEFINITIONS 
const int RL_PIN_PWM = 7;   
const int RL_PIN_IN1 = 8;   
const int RL_PIN_IN2 = 9;   

const int RR_PIN_PWM = 12;
const int RR_IN1 = 11;
const int RR_IN2 = 13;

const int FL_PIN_PWM = 15;
const int FL_IN1 = 16;
const int FL_IN2 = 17;

const int FR_PIN_PWM = 4;
const int FR_IN1 = 5;
const int FR_IN2 = 6;



WiFiUDP udp;
unsigned int localPort = 4210;
char packetBuffer[255];
float a0=0, a2=0, a4=0, a5=0;
String lastMsg = "";
float last_a0=999, last_a2=999, last_a4=999, last_a5=999;  // DEĞİŞİM TAKİP

void setup() {
  Serial.begin(115200);
  Serial.print("calis");
  int pins[] = {RL_PIN_PWM, RL_PIN_IN1, RL_PIN_IN2, RR_PIN_PWM, RR_IN1, RR_IN2, FL_PIN_PWM, FL_IN1, FL_IN2, FR_PIN_PWM, FR_IN1, FR_IN2};
  for(int p : pins) pinMode(p, OUTPUT);

  WiFi.begin("Ben", "sitrayburgbiriki3");
  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("\nWiFi Connected!");
  Serial.print("ESP32 IP: "); Serial.println(WiFi.localIP());


  udp.begin(localPort);
}

void loop() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(packetBuffer, 255);
    packetBuffer[len] = 0;
    String msg = String(packetBuffer);
    
    Serial.print("RAW: "); Serial.println(msg);  // SADECE RAW DEBUG
    
    // Tekrarlanan mesajları filtrele
    if (msg == lastMsg) return;
    lastMsg = msg;
    
    // Axis parsing - SADECE DEĞİŞİM VARSA PRINT
    bool axisChanged = false;
    if (msg.startsWith("AXIS 0 ")) {
      float new_a0 = msg.substring(7).toFloat();
      if (abs(new_a0 - a0) > 0.01) {  // 0.01 fark varsa güncelle
        a0 = new_a0;
        if (abs(a0 - last_a0) > 0.01) {
          Serial.printf("A0:%.3f ", a0);
          last_a0 = a0;
          axisChanged = true;
        }
      }
    }
    if (msg.startsWith("AXIS 2 ")) {
      float new_a2 = msg.substring(7).toFloat();
      if (abs(new_a2 - a2) > 0.01) {
        a2 = new_a2;
        if (abs(a2 - last_a2) > 0.01) {
          Serial.printf("A2:%.3f ", a2);
          last_a2 = a2;
          axisChanged = true;
        }
      }
    }  
    if (msg.startsWith("AXIS 4 ")) {
      float new_a4 = msg.substring(7).toFloat();
      if (abs(new_a4 - a4) > 0.01) {
        a4 = new_a4;
        if (abs(a4 - last_a4) > 0.01) {
          Serial.printf("A4:%.3f ", a4);
          last_a4 = a4;
          axisChanged = true;
        }
      }
    }
    if (msg.startsWith("AXIS 5 ")) {
      float new_a5 = msg.substring(7).toFloat();
      if (abs(new_a5 - a5) > 0.01) {
        a5 = new_a5;
        if (abs(a5 - last_a5) > 0.01) {
          Serial.printf("A5:%.3f\n", a5);
          last_a5 = a5;
          axisChanged = true;
        }
      }
    }
    
    if (axisChanged) Serial.println();  // Axis satırı kapat
    
        // MOTOR LOGIC - ÖNCELİK SIRASI
    int pwm_val;
    if (a2 < -0.05) {  // SOLA KAYMA
      pwm_val = map(abs(a2) * 100, 5, 100, 50, 255);
      Serial.printf("SOL: a2=%.3f PWM=%d\n", a2, pwm_val);
      sol(pwm_val);
    }
    else if (a2 > 0.05) {  // SAĞA KAYMA  
      pwm_val = map(a2 * 100, 5, 100, 50, 255);
      Serial.printf("SAĞ: a2=%.3f PWM=%d\n", a2, pwm_val);
      sag(pwm_val);
    }
    else if (abs(a0) > 0.05) {  // 360 DÖNÜŞ
      pwm_val = map(abs(a0) * 100, 5, 100, 50, 255);
      Serial.printf("DÖN: a0=%.3f PWM=%d\n", a0, pwm_val);
      donus360(pwm_val, a0 > 0);
    }
    else if (a5 > -0.9 || a4 > -0.9) {  // İleri/Geri + MIX
      int pwm_ileri = map((a5 + 1) * 50, 0, 100, 0, 255);
      int pwm_geri = map((a4 + 1) * 50, 0, 100, 0, 255);
      
      if (a5 > -0.9 && a4 <= -0.9) {  // SADECE İLERİ
        Serial.printf("ILERI: a5=%.3f PWM=%d\n", a5, pwm_ileri);
        ileri(pwm_ileri);
      }
      else if (a4 > -0.9 && a5 <= -0.9) {  // SADECE GERİ
        Serial.printf("GERI: a4=%.3f PWM=%d\n", a4, pwm_geri);
        geri(pwm_geri);
      }
      else if (a5 > -0.9 && a4 > -0.9) {  // MIX İLERİ+DÖNME
        int mix_pwm = (pwm_ileri * 0.4 + 80 * 0.6);
        Serial.printf("MIX: a5=%.3f PWM=%d\n", a5, mix_pwm);
        ileri_mix(mix_pwm);
      }
    }
    else {  // ANİ DUR
      Serial.println("DUR!");
      aniDur();
    }
  }
}

// İLERİ + HAFİF DÖNME MIX
void ileri_mix(int PWM) {
  digitalWrite(RR_IN1, HIGH); digitalWrite(RR_IN2, LOW);
  digitalWrite(FR_IN1, HIGH); digitalWrite(FR_IN2, LOW);
  digitalWrite(RL_PIN_IN1, HIGH); digitalWrite(RL_PIN_IN2, LOW);
  digitalWrite(FL_IN1, HIGH); digitalWrite(FL_IN2, LOW);
  
  analogWrite(RR_PIN_PWM, PWM);      // Sağ hızlı
  analogWrite(RL_PIN_PWM, PWM * 0.75);  // Sol yavaş (sağ dönüş)
  analogWrite(FL_PIN_PWM, PWM * 0.75);  // Ö sol yavaş
  analogWrite(FR_PIN_PWM, PWM);         // Ö sağ hızlı
}

// ANİ FRENLİ DUR
void aniDur() {
  // BRAKE: Tüm motorlar HIGH-HIGH
  digitalWrite(RR_IN1, HIGH); digitalWrite(RR_IN2, HIGH);
  digitalWrite(RL_PIN_IN1, HIGH); digitalWrite(RL_PIN_IN2, HIGH);
  digitalWrite(FL_IN1, HIGH); digitalWrite(FL_IN2, HIGH);
  digitalWrite(FR_IN1, HIGH); digitalWrite(FR_IN2, HIGH);
  
  analogWrite(RR_PIN_PWM, 255);
  analogWrite(RL_PIN_PWM, 255);
  analogWrite(FL_PIN_PWM, 255);
  analogWrite(FR_PIN_PWM, 255);
  
  delay(30);  // 30ms fren
  
  // OFF
  digitalWrite(RR_IN1, LOW); digitalWrite(RR_IN2, LOW);
  digitalWrite(RL_PIN_IN1, LOW); digitalWrite(RL_PIN_IN2, LOW);
  digitalWrite(FL_IN1, LOW); digitalWrite(FL_IN2, LOW);
  digitalWrite(FR_IN1, LOW); digitalWrite(FR_IN2, LOW);
  
  analogWrite(RR_PIN_PWM, 0);
  analogWrite(RL_PIN_PWM, 0);
  analogWrite(FL_PIN_PWM, 0);
  analogWrite(FR_PIN_PWM, 0);
}

void sol(int PWM) {
  digitalWrite(RR_IN1, LOW); digitalWrite(RR_IN2, HIGH);
  digitalWrite(FR_IN1, HIGH); digitalWrite(FR_IN2, LOW);
  digitalWrite(RL_PIN_IN1, HIGH); digitalWrite(RL_PIN_IN2, LOW);
  digitalWrite(FL_IN1, LOW); digitalWrite(FL_IN2, HIGH);
  
  analogWrite(RR_PIN_PWM, PWM);
  analogWrite(RL_PIN_PWM, PWM);
  analogWrite(FL_PIN_PWM, PWM);
  analogWrite(FR_PIN_PWM, PWM);
}

void ileri(int PWM) {
  digitalWrite(RR_IN1, HIGH); digitalWrite(RR_IN2, LOW);
  digitalWrite(FR_IN1, HIGH); digitalWrite(FR_IN2, LOW);
  digitalWrite(RL_PIN_IN1, HIGH); digitalWrite(RL_PIN_IN2, LOW);
  digitalWrite(FL_IN1, HIGH); digitalWrite(FL_IN2, LOW);
  
  analogWrite(RR_PIN_PWM, PWM);
  analogWrite(RL_PIN_PWM, PWM);
  analogWrite(FL_PIN_PWM, PWM);
  analogWrite(FR_PIN_PWM, PWM);
}

void geri(int PWM) {
  digitalWrite(RR_IN1, LOW); digitalWrite(RR_IN2, HIGH);
  digitalWrite(FR_IN1, LOW); digitalWrite(FR_IN2, HIGH);
  digitalWrite(RL_PIN_IN1, LOW); digitalWrite(RL_PIN_IN2, HIGH);
  digitalWrite(FL_IN1, LOW); digitalWrite(FL_IN2, HIGH);
  
  analogWrite(RR_PIN_PWM, PWM);
  analogWrite(RL_PIN_PWM, PWM);
  analogWrite(FL_PIN_PWM, PWM);
  analogWrite(FR_PIN_PWM, PWM);
}

void donus360(int PWM, bool sagaDonus) {
  if (sagaDonus) { // Sağa 360
    digitalWrite(RR_IN1, LOW); digitalWrite(RR_IN2, HIGH);
    digitalWrite(FR_IN1, LOW); digitalWrite(FR_IN2, HIGH);
    digitalWrite(RL_PIN_IN1, HIGH); digitalWrite(RL_PIN_IN2, LOW);
    digitalWrite(FL_IN1, HIGH); digitalWrite(FL_IN2, LOW);
  } else { // Sola 360
    digitalWrite(RR_IN1, HIGH); digitalWrite(RR_IN2, LOW);
    digitalWrite(FR_IN1, HIGH); digitalWrite(FR_IN2, LOW);
    digitalWrite(RL_PIN_IN1, LOW); digitalWrite(RL_PIN_IN2, HIGH);
    digitalWrite(FL_IN1, LOW); digitalWrite(FL_IN2, HIGH);
  }
  
  analogWrite(RR_PIN_PWM, PWM);
  analogWrite(RL_PIN_PWM, PWM);
  analogWrite(FL_PIN_PWM, PWM);
  analogWrite(FR_PIN_PWM, PWM);
}
void sag(int PWM) {
  digitalWrite(RR_IN1, HIGH); digitalWrite(RR_IN2, LOW);
  digitalWrite(FR_IN1, LOW); digitalWrite(FR_IN2, HIGH);
  digitalWrite(RL_PIN_IN1, LOW); digitalWrite(RL_PIN_IN2, HIGH);
  digitalWrite(FL_IN1, HIGH); digitalWrite(FL_IN2, LOW);
  
  analogWrite(RR_PIN_PWM, PWM);
  analogWrite(RL_PIN_PWM, PWM);
  analogWrite(FL_PIN_PWM, PWM);
  analogWrite(FR_PIN_PWM, PWM);
}

