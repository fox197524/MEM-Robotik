//===========THE BIG PID SYSTEM==============
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <HCSR04.h>
#include <Arduino_FreeRTOS.h>

WiFiUDP udp;
unsigned int localPort = 4210;
char packetBuffer[255];
unsigned long lastUpdate = 0;

//======PID PINOUT=====

// ENCODERS

const int EN_RL1 = 1; // Encoder on Right Left Motor First pin
const int EN_RL2 = 2; // Encoder on Right Left Motor Second pin

const int EN_RR1 = 4;
const int EN_RR2 = 5;

const int EN_FL1 = 6;
const int EN_FL2 = 7;

const int EN_FR1 = 8;
const int EN_FR2 = 9;

const int EN_EL1 = 10;
const int EN_EL2 = 11;

const int EN_ER1 = 12;
const int EN_ER2 = 13;

// DISTANCE SENSORS HC-SR04

const int D_TRIG = 14; // Common Trig Pin 14 for all distance sensors

const int D_FRNT = 15; // Echo pin for front distance sensor
const int D_RGHT = 16;
const int D_LEFT = 17;
const int D_REAR = 18;

// ACCELERATOR MPU6050 SENSOR

const int MPU_SDA = 41
const int MPU_SCL = 42

void setup() {
    // PIN MODE  
    pinMode(EN_RL1, OUTPUT);  
    pinMode(EN_RL2, OUTPUT);
    pinMode(EN_RR1, OUTPUT);
    pinMode(EN_RR2, OUTPUT);
    pinMode(EN_FL1, OUTPUT);
    pinMode(EN_FL2, OUTPUT);
    pinMode(EN_FR1, OUTPUT);
    pinMode(EN_FR2, OUTPUT);
    pinMode(EN_EL1, OUTPUT);
    pinMode(EN_EL2, OUTPUT);
    pinMode(EN_ER1, OUTPUT);
    pinMode(EN_ER2, OUTPUT);
    pinMode(D_TRIG, OUTPUT);
    pinMode(D_FRNT, INPUT);
    pinMode(D_RGHT, INPUT);
    pinMode(D_LEFT, INPUT);
    pinMode(D_REAR, INPUT);
    pinMode(MPU_SDA, OUTPUT);
    pinMode(MPU_SCL, OUTPUT);

    Serial.begin(115200);
    delay(1000);
    Serial.print("ESP32-N16R8 Başlatıldı");

    //Wifi Begin
    
    WiFi.begin("Fox-2", "Kyra2bin9");
    Serial.print("WiFi Bağlanıyor");
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




}
