#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>

// --- MOTOR PIN DEFINITIONS ---
//Rear Left Pins

const int RL_PWM = 7;   
const int RL_IN1 = 8;   
const int RL_IN2 = 9;  

//Rear Right Pins

const int RR_PWM = 12;
const int RR_IN1 = 11;
const int RR_IN2 = 13;

//Front Left Pins

const int FL_PWM = 15;
const int FL_IN1 = 16;
const int FL_IN2 = 17;

//Front Right Pins

const int FR_PWM = 4;
const int FR_IN1 = 5;
const int FR_IN2 = 6;

// --- ELEVATOR LEFT MOTOR ---

const int EL_PWM = 18;
const int EL_IN1 = 38;
const int EL_IN2 = 39;

// --- ELEVATOR RIGHT MOTOR ---

const int ER_PWM = 1;
const int ER_IN1 = 2;
const int ER_IN2 = 42;

// --- SERVO LID ---

const int E_LID = 41;
const int B_LID = 14;
const int H_LID = 40;

WiFiUDP udp;
unsigned int localPort = 4210;
char packetBuffer[255];

unsigned long lastUpdate = 0;

void setup() {
//========PIN MODE=========
pinMode(RR_IN1, OUTPUT);
pinMode(RR_IN2, OUTPUT);
pinMode(RR_PWM, OUTPUT);

pinMode(RL_IN1, OUTPUT);
pinMode(RL_IN2, OUTPUT);
pinMode(RL_PWM, OUTPUT);

pinMode(FR_IN1, OUTPUT);
pinMode(FR_IN2, OUTPUT);
pinMode(FR_PWM, OUTPUT);

pinMode(FL_IN1, OUTPUT);
pinMode(FL_IN2, OUTPUT);
pinMode(FL_PWM, OUTPUT);

pinMode(EL_IN1, OUTPUT);
pinMode(EL_IN2, OUTPUT);
pinMode(EL_PWM, OUTPUT);

pinMode(ER_IN1, OUTPUT);
pinMode(ER_IN2, OUTPUT);
pinMode(ER_PWM, OUTPUT);

pinMode(E_LID, OUTPUT);
pinMode(H_LID, OUTPUT);
pinMode(B_LID, OUTPUT);

Serial.begin(115200);
delay(1000);

Serial.print("ESP-32s3 N8R2 Başlatildi");

//Wifi Begin

  WiFi.begin("Fox-2", "Kyra2bin9");
  Serial.print("WiFi Baglaniyor");
  int timeout = 0;
  while (WiFi.status() != WL_CONNECTED && timeout < 20) {
    delay(500);
    Serial.print(".");
    timeout++;
  }
  
  Serial.println("\nWiFi OK!");
  Serial.print("IP: "); Serial.println(WiFi.localIP());
  udp.begin(localPort);
  Serial.println("UDP Basladi");

}

void loop() {
  // put your main code here, to run repeatedly:
  // 
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi Koptu, Restart...");
    ESP.restart();
  }

  // Parse UDP packets
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(packetBuffer, 255);
    if (len > 0 && len < 255) {
      packetBuffer[len] = 0;
      String msg = String(packetBuffer);
      
      parseInput(msg);
      
      // Send to serial for debugging
      Serial.printf("A0:%.2f A2:%.2f A4:%.2f A5:%.2f B0:%d B11:%d B12:%d\n", a0, a2, a4, a5, b0, b11, b12);
      
      // Process movement
      processMovement();
    }  } else {
    // No packet received - ensure all motors are OFF
    stopAll();
    controlElevator();  // Still handle elevator timers
    controlServo();     // Still handle servo
  }








//LOOP PARANTEZİ!!!
}



void sagon(){

digitalWrite(FR_IN1, HIGH);
digitalWrite(FR_IN2, LOW);

analogWrite(FR_PWM, 255);

}

void solon(){

digitalWrite(FL_IN1, HIGH);
digitalWrite(FL_IN2, LOW);

analogWrite(FL_PWM, 255);

}

void solarka(){

digitalWrite(RL_IN1, HIGH);
digitalWrite(RL_IN2, LOW);

analogWrite(RL_PWM, 255);

}

void sagarka(){

digitalWrite(RR_IN1, HIGH);
digitalWrite(RR_IN2, LOW);

analogWrite(RR_PWM, 255);

}
void ileri(int pwm) {

digitalWrite(RR_IN1, HIGH);
digitalWrite(RR_IN2, LOW);

digitalWrite(FR_IN1, HIGH);
digitalWrite(FR_IN2, LOW);

digitalWrite(RL_IN1, HIGH);
digitalWrite(RL_IN2, LOW);

digitalWrite(FL_IN1, HIGH);
digitalWrite(FL_IN2, LOW);


analogWrite(RR_PWM, pwm);
analogWrite(FR_PWM, pwm);
analogWrite(RL_PWM, pwm);
analogWrite(FL_PWM, pwm);

}

void geri(int pwm){

digitalWrite(RR_IN1, LOW);
digitalWrite(RR_IN2, HIGH);

digitalWrite(FR_IN1, LOW);
digitalWrite(FR_IN2, HIGH);

digitalWrite(RL_IN1, LOW);
digitalWrite(RL_IN2, HIGH);

digitalWrite(FL_IN1, LOW);
digitalWrite(FL_IN2, HIGH);


analogWrite(RR_PWM, 255);
analogWrite(FR_PWM, 255);
analogWrite(RL_PWM, 255);
analogWrite(FL_PWM, 255);

}

void anidur(){


}

void dur(){

digitalWrite(RR_IN1, LOW);
digitalWrite(RR_IN2, LOW);

digitalWrite(FR_IN1, LOW);
digitalWrite(FR_IN2, LOW);

digitalWrite(RL_IN1, LOW);
digitalWrite(RL_IN2, LOW);

digitalWrite(FL_IN1, LOW);
digitalWrite(FL_IN2, LOW);


analogWrite(RR_PWM, 255);
analogWrite(FR_PWM, 255);
analogWrite(RL_PWM, 255);
analogWrite(FL_PWM, 255);

}

void eyukari(){


}

void easagi() {

  
}

void solakay(int pwm){

digitalWrite(RR_IN1, LOW);
digitalWrite(RR_IN2, HIGH);

digitalWrite(FR_IN1, HIGH);
digitalWrite(FR_IN2, LOW);

digitalWrite(RL_IN1, HIGH);
digitalWrite(RL_IN2, LOW);

digitalWrite(FL_IN1, HIGH);
digitalWrite(FL_IN2, LOW);


analogWrite(RR_PWM, 255);
analogWrite(FR_PWM, 255);
analogWrite(RL_PWM, 255);
analogWrite(FL_PWM, 255);

}
 
void sagakay(int pwm){

digitalWrite(RR_IN1, HIGH);
digitalWrite(RR_IN2, LOW);

digitalWrite(FR_IN1, LOW);
digitalWrite(FR_IN2, HIGH);

digitalWrite(RL_IN1, HIGH);
digitalWrite(RL_IN2, LOW);

digitalWrite(FL_IN1, LOW);
digitalWrite(FL_IN2, HIGH);


analogWrite(RR_PWM, 255);
analogWrite(FR_PWM, 255);
analogWrite(RL_PWM, 255);
analogWrite(FL_PWM, 255);

} 

void d360(int pwm){

if(){

digitalWrite(RR_IN1, LOW);
digitalWrite(RR_IN2, HIGH);

digitalWrite(FR_IN1, LOW);
digitalWrite(FR_IN2, HIGH);

digitalWrite(RL_IN1, HIGH);
digitalWrite(RL_IN2, LOW);

digitalWrite(FL_IN1, HIGH);
digitalWrite(FL_IN2, LOW);


analogWrite(RR_PWM, pwm);
analogWrite(FR_PWM, pwm);
analogWrite(RL_PWM, pwm);
analogWrite(FL_PWM, pwm);

}


else if(){


digitalWrite(RR_IN1, HIGH);
digitalWrite(RR_IN2, LOW);

digitalWrite(FR_IN1, HIGH);
digitalWrite(FR_IN2, LOW);

digitalWrite(RL_IN1, LOW);
digitalWrite(RL_IN2, HIGH);

digitalWrite(FL_IN1, LOW);
digitalWrite(FL_IN2, HIGH);


analogWrite(RR_PWM, pwm);
analogWrite(FR_PWM, pwm);
analogWrite(RL_PWM, pwm);
analogWrite(FL_PWM, pwm);


}
