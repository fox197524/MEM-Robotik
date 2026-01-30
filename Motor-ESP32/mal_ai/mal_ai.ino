

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

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

const int ER_PWM = 35;
const int ER_IN1 = 36;
const int ER_IN2 = 37;

const int EL_PWM = 18;
const int EL_IN1 = 19;
const int EL_IN2 = 21;

const int E_LID = 48; // MG90S Signal Pin
const int RP = 35; 

// --- UDP COMMUNICATION ---

WiFiUDP udp;
unsigned int localPort = 4210;
char packetBuffer[255];

float a0 = 0, a2 = 0, a4 = 0, a5 = 0, a12 = 0, a13 = 0, a6 = 0; 
String lastMsg = "";

void setup() {
  Serial.begin(115200);
  
  int pins[] = {
    RL_PWM, RL_IN1, RL_IN2, 
    RR_PWM, RR_IN1, RR_IN2, 
    FL_PWM, FL_IN1, FL_IN2, 
    FR_PWM, FR_IN1, FR_IN2, 
    ER_IN1, ER_IN2, ER_PWM, 
    EL_IN1, EL_IN2, EL_PWM, 
    E_LID, RP
  };

  for(int p : pins) {
    pinMode(p, OUTPUT);
  }

  // Set MG90S Frequency
  analogWriteFrequency(E_LID, 50); 

  WiFi.begin("LAGARIMEDYA", "lagari5253");
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

    if (msg == lastMsg) return;
    lastMsg = msg;

    // --- PARSING ---
    if (msg.startsWith("AXIS 0 ")) a0 = msg.substring(7).toFloat();
    if (msg.startsWith("AXIS 2 ")) a2 = msg.substring(7).toFloat();
    if (msg.startsWith("AXIS 4 ")) a4 = msg.substring(7).toFloat();
    if (msg.startsWith("AXIS 5 ")) a5 = msg.substring(7).toFloat();
    if (msg.startsWith("AXIS 12 ")) a12 = msg.substring(8).toFloat();
    if (msg.startsWith("AXIS 13 ")) a13 = msg.substring(8).toFloat();
    if (msg.startsWith("AXIS 6 ")) a6 = msg.substring(7).toFloat();

    // --- DRIVE CONTROL ---
    if (a2 < -0.05) {
      sol(map(abs(a2) * 100, 5, 100, 50, 255));
    }
    else if (a2 > 0.05) {
      sag(map(a2 * 100, 5, 100, 50, 255));
    }
    else if (abs(a0) > 0.05) {
      donus360(map(abs(a0) * 100, 5, 100, 50, 255), a0 > 0);
    }
    else if (a5 > -0.9 || a4 > -0.9) {
      int p_i = map((a5 + 1) * 50, 0, 100, 0, 255);
      int p_g = map((a4 + 1) * 50, 0, 100, 0, 255);

      if (a5 > -0.9 && a4 <= -0.9) {
        ileri(p_i);
      }
      else if (a4 > -0.9 && a5 <= -0.9) {
        geri(p_g);
      }
      else if (a5 > -0.9 && a4 > -0.9) {
        ileri_mix(p_i);
      }
    } 
    else {
      aniDur();
    }

    // --- ELEVATOR & LID CONTROL ---
    if (a13 > 0.1) yukari(255); 
    else if (a12 > 0.1) asagi(255);
    else edur();

    if (a6 > 0.5) lid_open();
    else if (a6 < -0.5) lid_close();
  }
}

// --- MOVEMENT FUNCTIONS ---

void ileri(int PWM) {
  digitalWrite(RR_IN1, HIGH);
  digitalWrite(RR_IN2, LOW);
  digitalWrite(FR_IN1, HIGH);
  digitalWrite(FR_IN2, LOW);
  digitalWrite(RL_IN1, HIGH);
  digitalWrite(RL_IN2, LOW);
  digitalWrite(FL_IN1, HIGH);
  digitalWrite(FL_IN2, LOW);

  analogWrite(RR_PWM, PWM);
  analogWrite(RL_PWM, PWM);
  analogWrite(FL_PWM, PWM);
  analogWrite(FR_PWM, PWM);
}

void geri(int PWM) {
  digitalWrite(RR_IN1, LOW);
  digitalWrite(RR_IN2, HIGH);
  digitalWrite(FR_IN1, LOW);
  digitalWrite(FR_IN2, HIGH);
  digitalWrite(RL_IN1, LOW);
  digitalWrite(RL_IN2, HIGH);
  digitalWrite(FL_IN1, LOW);
  digitalWrite(FL_IN2, HIGH);

  analogWrite(RR_PWM, PWM);
  analogWrite(RL_PWM, PWM);
  analogWrite(FL_PWM, PWM);
  analogWrite(FR_PWM, PWM);
}

void sol(int PWM) {
  digitalWrite(RR_IN1, LOW);
  digitalWrite(RR_IN2, HIGH);
  digitalWrite(FR_IN1, HIGH);
  digitalWrite(FR_IN2, LOW);
  digitalWrite(RL_IN1, HIGH);
  digitalWrite(RL_IN2, LOW);
  digitalWrite(FL_IN1, LOW);
  digitalWrite(FL_IN2, HIGH);

  analogWrite(RR_PWM, PWM);
  analogWrite(RL_PWM, PWM);
  analogWrite(FL_PWM, PWM);
  analogWrite(FR_PWM, PWM);
}

void sag(int PWM) {
  digitalWrite(RR_IN1, HIGH);
  digitalWrite(RR_IN2, LOW);
  digitalWrite(FR_IN1, LOW);
  digitalWrite(FR_IN2, HIGH);
  digitalWrite(RL_IN1, LOW);
  digitalWrite(RL_IN2, HIGH);
  digitalWrite(FL_IN1, HIGH);
  digitalWrite(FL_IN2, LOW);

  analogWrite(RR_PWM, PWM);
  analogWrite(RL_PWM, PWM);
  analogWrite(FL_PWM, PWM);
  analogWrite(FR_PWM, PWM);
}

void donus360(int PWM, bool saga) {
  if (saga) {
    digitalWrite(RR_IN1, LOW);
    digitalWrite(RR_IN2, HIGH);
    digitalWrite(FR_IN1, LOW);
    digitalWrite(FR_IN2, HIGH);
    digitalWrite(RL_IN1, HIGH);
    digitalWrite(RL_IN2, LOW);
    digitalWrite(FL_IN1, HIGH);
    digitalWrite(FL_IN2, LOW);
  } else {
    digitalWrite(RR_IN1, HIGH);
    digitalWrite(RR_IN2, LOW);
    digitalWrite(FR_IN1, HIGH);
    digitalWrite(FR_IN2, LOW);
    digitalWrite(RL_IN1, LOW);
    digitalWrite(RL_IN2, HIGH);
    digitalWrite(FL_IN1, LOW);
    digitalWrite(FL_IN2, HIGH);
  }

  analogWrite(RR_PWM, PWM);
  analogWrite(RL_PWM, PWM);
  analogWrite(FL_PWM, PWM);
  analogWrite(FR_PWM, PWM);
}

void ileri_mix(int PWM) {
  digitalWrite(RR_IN1, HIGH);
  digitalWrite(RR_IN2, LOW);
  digitalWrite(FR_IN1, HIGH);
  digitalWrite(FR_IN2, LOW);
  digitalWrite(RL_IN1, HIGH);
  digitalWrite(RL_IN2, LOW);
  digitalWrite(FL_IN1, HIGH);
  digitalWrite(FL_IN2, LOW);

  analogWrite(RR_PWM, PWM);
  analogWrite(RL_PWM, PWM * 0.75);
  analogWrite(FL_PWM, PWM * 0.75);
  analogWrite(FR_PWM, PWM);
}

void aniDur() {
  digitalWrite(RR_IN1, HIGH);
  digitalWrite(RR_IN2, HIGH);
  digitalWrite(RL_IN1, HIGH);
  digitalWrite(RL_IN2, HIGH);
  digitalWrite(FL_IN1, HIGH);
  digitalWrite(FL_IN2, HIGH);
  digitalWrite(FR_IN1, HIGH);
  digitalWrite(FR_IN2, HIGH);

  delay(30);

  digitalWrite(RR_IN1, LOW);
  digitalWrite(RR_IN2, LOW);
  digitalWrite(RL_IN1, LOW);
  digitalWrite(RL_IN2, LOW);
  digitalWrite(FL_IN1, LOW);
  digitalWrite(FL_IN2, LOW);
  digitalWrite(FR_IN1, LOW);
  digitalWrite(FR_IN2, LOW);

  analogWrite(RR_PWM, 0);
  analogWrite(RL_PWM, 0);
  analogWrite(FL_PWM, 0);
  analogWrite(FR_PWM, 0);
}

// --- ELEVATOR FUNCTIONS ---

void yukari(int PWM) {
  digitalWrite(EL_IN1, HIGH);
  digitalWrite(EL_IN2, LOW);
  digitalWrite(ER_IN1, LOW);
  digitalWrite(ER_IN2, HIGH);

  analogWrite(ER_PWM, PWM);
  analogWrite(EL_PWM, PWM);
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
  digitalWrite(ER_IN1, LOW);
  digitalWrite(ER_IN2, LOW);
  digitalWrite(EL_IN1, LOW);
  digitalWrite(EL_IN2, LOW);

  analogWrite(EL_PWM, 0);
  analogWrite(ER_PWM, 0);
}

// --- SERVO FUNCTIONS ---

void moveServo(int angle) {
  int pwm = map(angle, 0, 180, 6, 32); 
  analogWrite(E_LID, pwm);
}

void lid_open() {
  moveServo(90);
}

void lid_close() {
  moveServo(0);
}