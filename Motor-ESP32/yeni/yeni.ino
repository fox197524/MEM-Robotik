#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>

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

const int ER_PWM = 1;
const int ER_IN1 = 2;
const int ER_IN2 = 42;

const int EL_PWM = 18;
const int EL_IN1 = 19;
const int EL_IN2 = 21;

const int SERVO_PIN = 41;
Servo lidServo;

WiFiUDP udp;
unsigned int localPort = 4210;
char packetBuffer[255];

float a0=0, a2=0, a4=0, a5=0;
bool b0=false, b11=false, b12=false;
String lastMsg = "";

float last_a0=999, last_a2=999, last_a4=999, last_a5=999;
bool last_b0=false, last_b11=false, last_b12=false;

// ELEVATOR TIMERS
unsigned long elevUpStart=0, elevDownStart=0;
unsigned long elevUpTotal=0, elevDownTotal=0;
const unsigned long ELEV_LIMIT = 3000;
bool elevUpActive=false, elevDownActive=false;
bool elevUpLocked=false, elevDownLocked=false;
bool lidOpen = false;
unsigned long lastControl = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("BASLATILDI");
  
  int pins[] = {RL_PWM, RL_IN1, RL_IN2, RR_PWM, RR_IN1, RR_IN2, FL_PWM, FL_IN1, FL_IN2, 
                FR_PWM, FR_IN1, FR_IN2, ER_IN1, ER_IN2, ER_PWM, EL_IN1, EL_IN2, EL_PWM};
  for(int p : pins) pinMode(p, OUTPUT);
  
  lidServo.attach(SERVO_PIN);
  lidServo.write(0);
  lidOpen = false;

  WiFi.begin("LAGARIMEDYA", "lagari5253");
  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("\nWiFi OK!");
  Serial.print("IP: "); Serial.println(WiFi.localIP());
  udp.begin(localPort);
}

void loop() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(packetBuffer, 255);
    packetBuffer[len] = 0;
    String msg = String(packetBuffer);
    
    if (millis() - lastControl < 50) return;
    lastControl = millis();
    
    if (msg == lastMsg) return;
    lastMsg = msg;
    
    parseControls(msg);
    controlElevator();
    controlLid();
    controlChassis();
  }
}

void parseControls(String msg) {
  if (msg.startsWith("AXIS 0 ")) a0 = msg.substring(7).toFloat();
  if (msg.startsWith("AXIS 2 ")) a2 = msg.substring(7).toFloat();
  if (msg.startsWith("AXIS 4 ")) a4 = msg.substring(7).toFloat();
  if (msg.startsWith("AXIS 5 ")) a5 = msg.substring(7).toFloat();
  if (msg.startsWith("BUTTON 0 ")) b0 = (msg.substring(9).toFloat() > 0.5);
  if (msg.startsWith("BUTTON 11 ")) b11 = (msg.substring(10).toFloat() > 0.5);
  if (msg.startsWith("BUTTON 12 ")) b12 = (msg.substring(10).toFloat() > 0.5);
}

void controlElevator() {
  // YUKARIYA ÇIKMA KONTROLÜ
  if (b11 && !elevUpLocked) {
    if (!elevUpActive) {
      elevUpStart = millis();
      elevUpTotal = 0;
      elevUpActive = true;
    }
    elevUpTotal = millis() - elevUpStart;
    
    if (elevUpTotal < ELEV_LIMIT) {
      yukari(200);
    } else {
      edur();
      elevUpLocked = true;
      elevUpActive = false;
    }
  } else if (!b11 && elevUpActive) {
    edur();
    elevUpActive = false;
  }
  
  // AŞAĞIYA İNME KONTROLÜ
  if (b12 && !elevDownLocked) {
    if (!elevDownActive) {
      elevDownStart = millis();
      elevDownTotal = 0;
      elevDownActive = true;
    }
    elevDownTotal = millis() - elevDownStart;
    
    if (elevDownTotal < ELEV_LIMIT) {
      asagi(200);
    } else {
      edur();
      elevDownLocked = true;
      elevDownActive = false;
    }
  } else if (!b12 && elevDownActive) {
    edur();
    elevDownActive = false;
  }
  
  // HER İKİ BUTON BIRAKILINCA LOCK'U KALDIR
  if (!b11 && !b12) {
    elevUpLocked = false;
    elevDownLocked = false;
  }
}

void controlLid() {
  if (b0 && !last_b0) {
    lidOpen = !lidOpen;
    if (lidOpen) {
      lidServo.write(90);
      Serial.println("LID ACILDI");
    } else {
      lidServo.write(0);
      Serial.println("LID KAPANDI");
    }
  }
  last_b0 = b0;
}

void controlChassis() {
  bool deadzone = (abs(a2)<=0.05 && abs(a0)<=0.05 && a5<=-0.9 && a4<=-0.9);
  if (deadzone) {
    aniDur();
    return;
  }
  
  if (abs(a2) > 0.05) {
    int slide_pwm = constrain(map(abs(a2)*100, 5, 100, 80, 255), 80, 255);
    if (a2 < 0) {
      sol(slide_pwm);
    } else {
      sag(slide_pwm);
    }
  }
  else if (abs(a0) > 0.05) {
    int pwm = constrain(map(abs(a0)*100, 5, 100, 80, 255), 80, 255);
    donus360(pwm, a0 > 0);
  }
  else if (a5 > -0.9) {
    int pwm = constrain(map((a5+1)*50, 0, 100, 0, 255), 0, 255);
    ileri(pwm);
  }
  else if (a4 > -0.9) {
    int pwm = constrain(map((a4+1)*50, 0, 100, 0, 255), 0, 255);
    geri(pwm);
  }
}

void sol_ileri_mix(int pwm) {
  digitalWrite(RR_IN1, LOW); digitalWrite(RR_IN2, HIGH);
  digitalWrite(FR_IN1, HIGH); digitalWrite(FR_IN2, LOW);
  digitalWrite(RL_IN1, HIGH); digitalWrite(RL_IN2, LOW);
  digitalWrite(FL_IN1, LOW); digitalWrite(FL_IN2, HIGH);
  analogWrite(RR_PWM, pwm); analogWrite(RL_PWM, pwm);
  analogWrite(FL_PWM, pwm); analogWrite(FR_PWM, pwm);
}

void sag_ileri_mix(int pwm) {
  digitalWrite(RR_IN1, HIGH); digitalWrite(RR_IN2, LOW);
  digitalWrite(FR_IN1, LOW); digitalWrite(FR_IN2, HIGH);
  digitalWrite(RL_IN1, LOW); digitalWrite(RL_IN2, HIGH);
  digitalWrite(FL_IN1, HIGH); digitalWrite(FL_IN2, LOW);
  analogWrite(RR_PWM, pwm); analogWrite(RL_PWM, pwm);
  analogWrite(FL_PWM, pwm); analogWrite(FR_PWM, pwm);
}

void sol_geri_mix(int pwm) {
  digitalWrite(RR_IN1, HIGH); digitalWrite(RR_IN2, LOW);
  digitalWrite(FR_IN1, LOW); digitalWrite(FR_IN2, HIGH);
  digitalWrite(RL_IN1, LOW); digitalWrite(RL_IN2, HIGH);
  digitalWrite(FL_IN1, HIGH); digitalWrite(FL_IN2, LOW);
  analogWrite(RR_PWM, pwm); analogWrite(RL_PWM, pwm);
  analogWrite(FL_PWM, pwm); analogWrite(FR_PWM, pwm);
}

void sag_geri_mix(int pwm) {
  digitalWrite(RR_IN1, LOW); digitalWrite(RR_IN2, HIGH);
  digitalWrite(FR_IN1, HIGH); digitalWrite(FR_IN2, LOW);
  digitalWrite(RL_IN1, HIGH); digitalWrite(RL_IN2, LOW);
  digitalWrite(FL_IN1, LOW); digitalWrite(FL_IN2, HIGH);
  analogWrite(RR_PWM, pwm); analogWrite(RL_PWM, pwm);
  analogWrite(FL_PWM, pwm); analogWrite(FR_PWM, pwm);
}

void ileri_mix(int PWM) {
  digitalWrite(RR_IN1, HIGH); digitalWrite(RR_IN2, LOW);
  digitalWrite(FR_IN1, HIGH); digitalWrite(FR_IN2, LOW);
  digitalWrite(RL_IN1, HIGH); digitalWrite(RL_IN2, LOW);
  digitalWrite(FL_IN1, HIGH); digitalWrite(FL_IN2, LOW);
  analogWrite(RR_PWM, PWM); analogWrite(RL_PWM, PWM * 0.75);
  analogWrite(FL_PWM, PWM * 0.75); analogWrite(FR_PWM, PWM);
}

void aniDur() {
  digitalWrite(RR_IN1, HIGH); digitalWrite(RR_IN2, HIGH);
  digitalWrite(RL_IN1, HIGH); digitalWrite(RL_IN2, HIGH);
  digitalWrite(FL_IN1, HIGH); digitalWrite(FL_IN2, HIGH);
  digitalWrite(FR_IN1, HIGH); digitalWrite(FR_IN2, HIGH);
  analogWrite(RR_PWM, 255); analogWrite(RL_PWM, 255);
  analogWrite(FL_PWM, 255); analogWrite(FR_PWM, 255);
  delay(30);
  digitalWrite(RR_IN1, LOW); digitalWrite(RR_IN2, LOW);
  digitalWrite(RL_IN1, LOW); digitalWrite(RL_IN2, LOW);
  digitalWrite(FL_IN1, LOW); digitalWrite(FL_IN2, LOW);
  digitalWrite(FR_IN1, LOW); digitalWrite(FR_IN2, LOW);
  analogWrite(RR_PWM, 0); analogWrite(RL_PWM, 0);
  analogWrite(FL_PWM, 0); analogWrite(FR_PWM, 0);
}

void sol(int PWM) {
  digitalWrite(RR_IN1, LOW); digitalWrite(RR_IN2, HIGH);
  digitalWrite(FR_IN1, HIGH); digitalWrite(FR_IN2, LOW);
  digitalWrite(RL_IN1, HIGH); digitalWrite(RL_IN2, LOW);
  digitalWrite(FL_IN1, LOW); digitalWrite(FL_IN2, HIGH);
  analogWrite(RR_PWM, PWM); analogWrite(RL_PWM, PWM);
  analogWrite(FL_PWM, PWM); analogWrite(FR_PWM, PWM);
}

void sag(int PWM) {
  digitalWrite(RR_IN1, HIGH); digitalWrite(RR_IN2, LOW);
  digitalWrite(FR_IN1, LOW); digitalWrite(FR_IN2, HIGH);
  digitalWrite(RL_IN1, LOW); digitalWrite(RL_IN2, HIGH);
  digitalWrite(FL_IN1, HIGH); digitalWrite(FL_IN2, LOW);
  analogWrite(RR_PWM, PWM); analogWrite(RL_PWM, PWM);
  analogWrite(FL_PWM, PWM); analogWrite(FR_PWM, PWM);
}

void ileri(int PWM) {
  digitalWrite(RR_IN1, HIGH); digitalWrite(RR_IN2, LOW);
  digitalWrite(FR_IN1, HIGH); digitalWrite(FR_IN2, LOW);
  digitalWrite(RL_IN1, HIGH); digitalWrite(RL_IN2, LOW);
  digitalWrite(FL_IN1, HIGH); digitalWrite(FL_IN2, LOW);
  analogWrite(RR_PWM, PWM); analogWrite(RL_PWM, PWM);
  analogWrite(FL_PWM, PWM); analogWrite(FR_PWM, PWM);
}

void geri(int PWM) {
  digitalWrite(RR_IN1, LOW); digitalWrite(RR_IN2, HIGH);
  digitalWrite(FR_IN1, LOW); digitalWrite(FR_IN2, HIGH);
  digitalWrite(RL_IN1, LOW); digitalWrite(RL_IN2, HIGH);
  digitalWrite(FL_IN1, LOW); digitalWrite(FL_IN2, HIGH);
  analogWrite(RR_PWM, PWM); analogWrite(RL_PWM, PWM);
  analogWrite(FL_PWM, PWM); analogWrite(FR_PWM, PWM);
}

void donus360(int PWM, bool sagaDonus) {
  if (sagaDonus) {
    digitalWrite(RR_IN1, LOW); digitalWrite(RR_IN2, HIGH);
    digitalWrite(FR_IN1, LOW); digitalWrite(FR_IN2, HIGH);
    digitalWrite(RL_IN1, HIGH); digitalWrite(RL_IN2, LOW);
    digitalWrite(FL_IN1, HIGH); digitalWrite(FL_IN2, LOW);
  } else {
    digitalWrite(RR_IN1, HIGH); digitalWrite(RR_IN2, LOW);
    digitalWrite(FR_IN1, HIGH); digitalWrite(FR_IN2, LOW);
    digitalWrite(RL_IN1, LOW); digitalWrite(RL_IN2, HIGH);
    digitalWrite(FL_IN1, LOW); digitalWrite(FL_IN2, HIGH);
  }
  analogWrite(RR_PWM, PWM); analogWrite(RL_PWM, PWM);
  analogWrite(FL_PWM, PWM); analogWrite(FR_PWM, PWM);
}

void yukari(int PWM) {
  digitalWrite(EL_IN1, HIGH); digitalWrite(EL_IN2, LOW);
  digitalWrite(ER_IN1, LOW); digitalWrite(ER_IN2, HIGH);
  analogWrite(EL_PWM, PWM);
  analogWrite(ER_PWM, PWM);
}

void asagi(int PWM) {
  digitalWrite(EL_IN1, LOW); 
  digitalWrite(EL_IN2, HIGH);
  digitalWrite(ER_IN1, HIGH); 
  digitalWrite(ER_IN2, LOW);
  
  analogWrite(EL_PWM, PWM);
  analogWrite(ER_PWM, PWM);
}

void edur() {
  digitalWrite(ER_IN1, HIGH); 
  digitalWrite(ER_IN2, HIGH);
  digitalWrite(EL_IN1, HIGH); 
  digitalWrite(EL_IN2, HIGH);
  
  analogWrite(EL_PWM, 255);
  analogWrite(ER_PWM, 255);
  
  delay(30);
  
  digitalWrite(ER_IN1, LOW); 
  digitalWrite(ER_IN2, LOW);
  digitalWrite(EL_IN1, LOW); 
  digitalWrite(EL_IN2, LOW);
  
  analogWrite(EL_PWM, 0);
  analogWrite(ER_PWM, 0);
}
