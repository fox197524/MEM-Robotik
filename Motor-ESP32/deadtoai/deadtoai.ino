#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// --- MOTOR 4 PIN DEFINITIONS 
const int RL_PIN_PWM = 7;   // Speed Control (PWM) yeşil
const int RL_PIN_IN2 = 8;   // Direction 1.        turuncu
const int RL_PIN_IN1 = 9;   // Direction 2.        sarı

const int RR_PIN_PWM = 11;
const int RR_PIN_IN2 = 12;
const int RR_PIN_IN1 = 13;

const int FL_PIN_PWM = 15;
const int FL_PIN_IN2 = 16;
const int FL_PIN_IN1 = 17;

const int FR_PIN_PWM = 4;
const int FR_PIN_IN2 = 5;
const int FR_PIN_IN1 = 6;

// Configuration
const int PWM = 200; // Full speed
const int PWMS = 0; // 


// --- WIFI ---
const char* ssid = "LAGARIMEDYA";
const char* password = "lagari5253";
WiFiUDP udp;
unsigned int localPort = 4210;
char packetBuffer[255];


float axis0 = 0, axis2 = 0, axis4 = -1, axis5 = -1;
float p0 = 0, p2 = 0, p4 = -1, p5 = -1; // Previous values for change detection

void setup() {
  Serial.begin(115200);
  Serial.print("code by ""Dead To AI"" Community");

  dur();

  pinMode(RL_PIN_PWM, OUTPUT);
  pinMode(RL_PIN_IN1, OUTPUT);
  pinMode(RL_PIN_IN2, OUTPUT);
  pinMode(RR_PIN_PWM, OUTPUT);
  pinMode(RR_PIN_IN1, OUTPUT);
  pinMode(RR_PIN_IN2, OUTPUT);
  pinMode(FL_PIN_PWM, OUTPUT);
  pinMode(FL_PIN_IN1, OUTPUT);
  pinMode(FL_PIN_IN2, OUTPUT);
  pinMode(FR_PIN_PWM, OUTPUT);
  pinMode(FR_PIN_IN1, OUTPUT);
  pinMode(FR_PIN_IN2, OUTPUT);


    WiFi.begin(ssid, password);
  Serial.print("Connecting WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected! IP: " + WiFi.localIP().toString());
  udp.begin(localPort);
 
  

}

void loop() {
 
int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(packetBuffer, 255);
    if (len > 0) packetBuffer[len] = 0;
    
    String msg = String(packetBuffer);
    // Örnek format: "AXIS 5 0.750"
    if (msg.startsWith("AXIS 0")) axis0 = msg.substring(7).toFloat();
    if (msg.startsWith("AXIS 2")) axis2 = msg.substring(7).toFloat();
    if (msg.startsWith("AXIS 4")) axis4 = msg.substring(7).toFloat();
    if (msg.startsWith("AXIS 5")) axis5 = msg.substring(7).toFloat();
  }


// Karar Mekanizması
  if (axis5 > -0.9) { // Tetik basılıysa (PS5 tetikleri boştayken -1'dir)
    int pwmVal = map(axis5 * 100, -100, 100, 0, 255);
    ileri(pwmVal);
  } else if (axis4 > -0.9) {
    int pwmVal = map(axis4 * 100, -100, 100, 0, 255);
    geri(pwmVal);
  } else if (abs(axis0) > 0.1) { // Sağ-Sol Dönüş (%10 deadzone)
    int pwmVal = map(abs(axis0) * 100, 0, 100, 0, 255);
    if (axis0 > 0) sag360(pwmVal); else sol360(pwmVal);
  } else if (axis2 > 0.1) { // Sola kayma axis 2 demiştin
    int pwmVal = map(axis2 * 100, 0, 100, 0, 255);
    sol(pwmVal);
  } else {
    dur();
  }
}




// ============ Controller Trigger Convert Fonksiyonu ============
void trigger() {

}

// ============ Controller Joystick Convert Fonksiyonu ===========
void joystick() {

}

// ============= Controller Input Convert Fonksiyonu =============
void cic() {

}

// ====================== İleri Fonksiyonu =======================
void ileri(int hiz) {

  digitalWrite(RR_PIN_IN1, HIGH);
  digitalWrite(RR_PIN_IN2, LOW);

  digitalWrite(FR_PIN_IN1, HIGH);
  digitalWrite(FR_PIN_IN2, LOW);

  digitalWrite(RL_PIN_IN1, HIGH);
  digitalWrite(RL_PIN_IN2, LOW);

  digitalWrite(FL_PIN_IN1, LOW);
  digitalWrite(FL_PIN_IN2, HIGH);


  analogWrite(RR_PIN_PWM, hiz);
  analogWrite(RL_PIN_PWM, hiz);
  analogWrite(FL_PIN_PWM, hiz);
  analogWrite(FR_PIN_PWM, hiz);

}
//==============Geri Fonksiyonu=============
void geri(int hiz){

  digitalWrite(RR_PIN_IN1, LOW);
  digitalWrite(RR_PIN_IN2, HIGH);

  digitalWrite(FR_PIN_IN1, LOW);
  digitalWrite(FR_PIN_IN2, HIGH);

  digitalWrite(RL_PIN_IN1, LOW);
  digitalWrite(RL_PIN_IN2, HIGH);

  digitalWrite(FL_PIN_IN1, HIGH);
  digitalWrite(FL_PIN_IN2, LOW);


  analogWrite(RR_PIN_PWM, hiz);
  analogWrite(RL_PIN_PWM, hiz);
  analogWrite(FL_PIN_PWM, hiz);
  analogWrite(FR_PIN_PWM, hiz);

}
//==============Sağa Kayma Fonksiyonu=============
void sag(int hiz){
  
  digitalWrite(RR_PIN_IN1, HIGH);
  digitalWrite(RR_PIN_IN2, LOW);

  digitalWrite(FR_PIN_IN1, LOW);
  digitalWrite(FR_PIN_IN2, HIGH);

  digitalWrite(RL_PIN_IN1, LOW);
  digitalWrite(RL_PIN_IN2, HIGH);

  digitalWrite(FL_PIN_IN1, HIGH);
  digitalWrite(FL_PIN_IN2, LOW);


  analogWrite(RR_PIN_PWM, hiz);
  analogWrite(RL_PIN_PWM, hiz);
  analogWrite(FL_PIN_PWM, hiz);
  analogWrite(FR_PIN_PWM, hiz);
}
//==============Sola Kayma Fonksiyonu=============
void sol(int hiz){
  
  digitalWrite(RR_PIN_IN1, LOW);
  digitalWrite(RR_PIN_IN2, HIGH);

  digitalWrite(FR_PIN_IN1, HIGH);
  digitalWrite(FR_PIN_IN2, LOW);

  digitalWrite(RL_PIN_IN1, HIGH);
  digitalWrite(RL_PIN_IN2, LOW);

  digitalWrite(FL_PIN_IN1, LOW);
  digitalWrite(FL_PIN_IN2, HIGH);


  analogWrite(RR_PIN_PWM, hiz);
  analogWrite(RL_PIN_PWM, hiz);
  analogWrite(FL_PIN_PWM, hiz);
  analogWrite(FR_PIN_PWM, hiz);
}
//==============Dur Fonksiyonu=============
void dur(){
  
  digitalWrite(RR_PIN_IN1, LOW);
  digitalWrite(RR_PIN_IN2, LOW);

  digitalWrite(RL_PIN_IN1, LOW);
  digitalWrite(RL_PIN_IN2, LOW);

  digitalWrite(FL_PIN_IN1, LOW);
  digitalWrite(FL_PIN_IN2, LOW);

  digitalWrite(FR_PIN_IN1, LOW);
  digitalWrite(FR_PIN_IN2, LOW);


  analogWrite(RR_PIN_PWM, PWMS);
  analogWrite(RL_PIN_PWM, PWMS);
  analogWrite(FL_PIN_PWM, PWM);
  analogWrite(FR_PIN_PWM, PWMS);

}
//==============sola 360 dönme fonksiyonu=============
void sol360(int hiz){

  digitalWrite(RR_PIN_IN1, HIGH);
  digitalWrite(RR_PIN_IN2, LOW);

  digitalWrite(FR_PIN_IN1, HIGH);
  digitalWrite(FR_PIN_IN2, LOW);

  digitalWrite(RL_PIN_IN1, LOW);
  digitalWrite(RL_PIN_IN2, HIGH);

  digitalWrite(FL_PIN_IN1, LOW);
  digitalWrite(FL_PIN_IN2, HIGH);


  analogWrite(RR_PIN_PWM, hiz);
  analogWrite(RL_PIN_PWM, hiz);
  analogWrite(FL_PIN_PWM, hiz);
  analogWrite(FR_PIN_PWM, hiz);
}
//==============sağa 360 dönme fonksiyonu=============
void sag360(int hiz){

  digitalWrite(RR_PIN_IN1, LOW);
  digitalWrite(RR_PIN_IN2, HIGH);

  digitalWrite(FR_PIN_IN1, HIGH);
  digitalWrite(FR_PIN_IN2, LOW);

  digitalWrite(RL_PIN_IN1, HIGH);
  digitalWrite(RL_PIN_IN2, LOW);

  digitalWrite(FL_PIN_IN1, HIGH);
  digitalWrite(FL_PIN_IN2, LOW);


  analogWrite(RR_PIN_PWM, hiz);
  analogWrite(RL_PIN_PWM, hiz);
  analogWrite(FL_PIN_PWM, hiz);
  analogWrite(FR_PIN_PWM, hiz);
}