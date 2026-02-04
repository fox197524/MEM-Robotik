#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

WiFiUDP udp;
unsigned int localPort = 4210;
char packetBuffer[255];

#define WIFI_SSID "Fox-17"
#define WIFI_PSWD "Kyra2bin9"

const int RL_PWM = 7, RL_IN1 = 8, RL_IN2 = 9;
const int RR_PWM = 12, RR_IN1 = 11, RR_IN2 = 13;
const int FL_PWM = 15, FL_IN1 = 16, FL_IN2 = 17;
const int FR_PWM = 4, FR_IN1 = 5, FR_IN2 = 6;

const int EL_PWM = 18, EL_IN1 = 38, EL_IN2 = 39;
const int ER_PWM = 3, ER_IN1 = 10, ER_IN2 = 42;
const int E_LID = 41;

void setMotor(int in1, int in2, int pwm, int dir, int speed) {
  if (dir > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (dir < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
  analogWrite(pwm, abs(speed));
}

void setup() {
  Serial.begin(115200);
  
  pinMode(RL_IN1, OUTPUT); pinMode(RL_IN2, OUTPUT); pinMode(RL_PWM, OUTPUT);
  pinMode(RR_IN1, OUTPUT); pinMode(RR_IN2, OUTPUT); pinMode(RR_PWM, OUTPUT);
  pinMode(FL_IN1, OUTPUT); pinMode(FL_IN2, OUTPUT); pinMode(FL_PWM, OUTPUT);
  pinMode(FR_IN1, OUTPUT); pinMode(FR_IN2, OUTPUT); pinMode(FR_PWM, OUTPUT);
  
  pinMode(EL_IN1, OUTPUT); pinMode(EL_IN2, OUTPUT); pinMode(EL_PWM, OUTPUT);
  pinMode(ER_IN1, OUTPUT); pinMode(ER_IN2, OUTPUT); pinMode(ER_PWM, OUTPUT);
  
  WiFi.begin(WIFI_SSID, WIFI_PSWD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi OK! IP: " + WiFi.localIP().toString());
  udp.begin(localPort);
}

void loop() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(packetBuffer, 255);
    if (len > 0) packetBuffer[len] = 0;
    
    float fwd = atof(strtok(packetBuffer, ","));
    float strafe = atof(strtok(NULL, ","));
    float turn = atof(strtok(NULL, ","));
    float rot = atof(strtok(NULL, ","));
    
    if (abs(fwd) < 0.1) fwd = 0;
    if (abs(strafe) < 0.1) strafe = 0;
    if (abs(turn) < 0.1) turn = 0;
    if (abs(rot) < 0.1) rot = 0;
    
    int rl_speed = constrain((fwd - strafe - turn + rot) * 255, -255, 255);
    int rr_speed = constrain((fwd + strafe + turn - rot) * 255, -255, 255);
    int fl_speed = constrain((fwd + strafe - turn - rot) * 255, -255, 255);
    int fr_speed = constrain((fwd - strafe + turn + rot) * 255, -255, 255);
    
    setMotor(RL_IN1, RL_IN2, RL_PWM, rl_speed >= 0 ? 1 : -1, abs(rl_speed));
    setMotor(RR_IN1, RR_IN2, RR_PWM, rr_speed >= 0 ? 1 : -1, abs(rr_speed));
    setMotor(FL_IN1, FL_IN2, FL_PWM, fl_speed >= 0 ? 1 : -1, abs(fl_speed));
    setMotor(FR_IN1, FR_IN2, FR_PWM, fr_speed >= 0 ? 1 : -1, abs(fr_speed));
    
    Serial.printf("F:%.2f S:%.2f T:%.2f R:%.2f | RL:%d RR:%d FL:%d FR:%d\n", 
                  fwd, strafe, turn, rot, rl_speed, rr_speed, fl_speed, fr_speed);
  }
}
