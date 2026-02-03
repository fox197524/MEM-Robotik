#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

WiFiUDP udp;
unsigned int localPort = 4210;
char packetBuffer[255];
unsigned long lastUpdate = 0;

#define WIFI_SSID "Fox-2"
#define WIFI_PSWD "Kyra2bin9"

// --- MOTOR PIN DEFINITIONS ---
const int RL_PWM = 7;
const int RL_IN1 = 8;
const int RL_IN2 = 9;

const int RR_PWM = 12;
const int RR_IN1 = 11;
const int RR_IN2 = 13;

const int FL_PWM = 15;
const int FL_IN1 = 16;
const int FL_IN2 = 17;

const int FR_PWM = 4;
const int FR_IN1 = 5;
const int FR_IN2 = 6;

// Elevator
const int EL_PWM = 18;
const int EL_IN1 = 38;
const int EL_IN2 = 39;

const int ER_PWM = 3;
const int ER_IN1 = 10;
const int ER_IN2 = 42;
    

const int E_LID = 41;

// Inputs
float a0=0, a2=0, a4=0, a5=0;
int b0=0, b6=0, b11=0, b12=0, b13=0;


void setup() {
  Serial.begin(115200);
  
  // Motor pins
  pinMode(RL_IN1, OUTPUT);
  pinMode(RL_IN2, OUTPUT);
  pinMode(RL_PWM, OUTPUT);

  pinMode(RR_IN1, OUTPUT);
  pinMode(RR_IN2, OUTPUT);
  pinMode(RR_PWM, OUTPUT);

  pinMode(FL_IN1, OUTPUT);
  pinMode(FL_IN2, OUTPUT);
  pinMode(FL_PWM, OUTPUT);

  pinMode(FR_IN1, OUTPUT);
  pinMode(FR_IN2, OUTPUT);
  pinMode(FR_PWM, OUTPUT);
  
  pinMode(EL_IN1, OUTPUT);
  pinMode(EL_IN2, OUTPUT);
  pinMode(EL_PWM, OUTPUT);

  pinMode(ER_IN1, OUTPUT);
  pinMode(ER_IN2, OUTPUT);
  pinMode(ER_PWM, OUTPUT);
  
  
  // WiFi
  WiFi.begin(WIFI_SSID, WIFI_PSWD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi OK! IP: " + WiFi.localIP().toString());
  udp.begin(localPort);
}


void loop(){
  digitalWrite(RL_IN1, HIGH);
  digitalWrite(RL_IN2, LOW);
  analogWrite(RL_PWM, 255);

  digitalWrite(RR_IN1, LOW);
  digitalWrite(RR_IN2, HIGH);
  analogWrite(RR_PWM, 255);

  digitalWrite(FL_IN1, HIGH);
  digitalWrite(FL_IN2, LOW);
  analogWrite(FL_PWM, 255);

  digitalWrite(FR_IN1, HIGH);
  digitalWrite(FR_IN2, LOW);
  analogWrite(FR_PWM, 255);


}
