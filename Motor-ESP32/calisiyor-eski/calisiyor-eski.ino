#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// --- MOTOR PIN TANIMLARI ---
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
const int EL_IN1 = 38;
const int EL_IN2 = 39;

const int E_LID = 41;
const int B_LID = 14;
const int H_LID = 40;

// --- UDP AYARLARI ---
WiFiUDP udp;
unsigned int localPort = 4210;
char packetBuffer[255];

// --- DEĞİŞKENLER ---
float a0 = 0, a2 = 0, a4 = 0, a5 = 0;
bool button11_up = false, button12_down = false, button0_lid = false;
String lastMsg = "";

float last_a0 = 999, last_a2 = 999, last_a4 = 999, last_a5 = 999;
bool last_button11 = false, last_button12 = false, last_button0 = false;

// Asansör zamanlayıcıları
unsigned long elevator_up_start = 0;
unsigned long elevator_down_start = 0;
unsigned long elevator_up_total = 0;
unsigned long elevator_down_total = 0;
const unsigned long ELEVATOR_LIMIT_MS = 3000;

// LID durumu
bool lid_open = false;
bool lid_moving = false;
unsigned long lid_move_start = 0;
const unsigned long LID_MOVE_DURATION = 500;

HardwareSerial masterUart(1);
volatile long cmd_enc_target = 0;
volatile long cmd_enc_start = 0;
String cmd_motion = "";

void setup() {
  Serial.begin(115200);
  masterUart.begin(115200, SERIAL_8N1, 16, 17);
  Serial.println("BASLATILIYOR...");
  
  // Pinleri ayarla - YENİ PİNLER DAHİL
  pinMode(RL_PWM, OUTPUT); 
  pinMode(RL_IN1, OUTPUT); 
  pinMode(RL_IN2, OUTPUT);

  pinMode(RR_PWM, OUTPUT); 
  pinMode(RR_IN1, OUTPUT); 
  pinMode(RR_IN2, OUTPUT);

  pinMode(FL_PWM, OUTPUT); 
  pinMode(FL_IN1, OUTPUT);
  pinMode(FL_IN2, OUTPUT);

  pinMode(FR_PWM, OUTPUT);
  pinMode(FR_IN1, OUTPUT); 
  pinMode(FR_IN2, OUTPUT);

  pinMode(ER_PWM, OUTPUT); 
  pinMode(ER_IN1, OUTPUT); 
  pinMode(ER_IN2, OUTPUT);

  pinMode(EL_PWM, OUTPUT); 
  pinMode(EL_IN1, OUTPUT); 
  pinMode(EL_IN2, OUTPUT);

  pinMode(E_LID, OUTPUT); 
  pinMode(B_LID, OUTPUT); 
  pinMode(H_LID, OUTPUT);
  
  // WiFi bağlantısı
  WiFi.begin("LAGARIMEDYA", "lagari5253");
  Serial.print("WIFI BAĞLANIYOR");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWIFI BAĞLANDI!");
  Serial.print("IP: "); 
  Serial.println(WiFi.localIP());
  
  udp.begin(localPort);
}

void loop() {
  // PID commands override joystick
  if(masterUart.available()) {
    String cmd = masterUart.readStringUntil('\n');
    cmd.trim();
    
    if(cmd=="STOP") {
      aniDurChassis();
      cmd_motion = ""; cmd_enc_target = 0;
      Serial.println("PID STOP");
      return;
    }
    
    // Parse: ILERI 220 35  (= 3.5 encoder turns)
    int pwm, turns;
    sscanf(cmd.c_str(), "%s %d %d", (char*)cmd.c_str(), &pwm, &turns);
    cmd_enc_target = turns / 10;  // Convert to whole turns
    cmd_enc_start = enc_fl;       // Use FL encoder as reference
    cmd_motion = String(cmd.c_str());
    
    Serial.printf("PID CMD: %s PWM=%d TURNS=%.1f\n", cmd_motion.c_str(), pwm, turns/10.0);
  }
  
  // Execute encoder-controlled motion
  if(cmd_enc_target > 0 && cmd_motion != "") {
    long enc_now = enc_fl;
    long enc_traveled = enc_now - cmd_enc_start;
    
    if(enc_traveled >= cmd_enc_target) {
      aniDurChassis();
      Serial.printf(" %s %.1f turns COMPLETE\n", cmd_motion.c_str(), cmd_enc_target);
      cmd_motion = ""; cmd_enc_target = 0;
      return;
    }
    
    // Execute during travel
    if(cmd_motion=="ILERI") ileri(220);
    else if(cmd_motion=="GERI") geri(220);
    else if(cmd_motion=="SOL") sol(200);
    else if(cmd_motion=="SAG") sag(200);
    else if(cmd_motion=="DONUS") donus360(200, turns>0);
  }
  else {
    // Joystick fallback (your existing UDP code)
    // ... existing joystick parsing ...
  }
}

void parseAxis(String msg) {
  bool changed = false;
  
  if (msg.startsWith("AXIS 0 ")) {
    float new_val = msg.substring(7).toFloat();
    if (abs(new_val - a0) > 0.01) {
      a0 = new_val;
      if (abs(a0 - last_a0) > 0.01) {
        Serial.printf("A0:%.3f ", a0);
        last_a0 = a0;
        changed = true;
      }
    }
  }
  
  if (msg.startsWith("AXIS 2 ")) {
    float new_val = msg.substring(7).toFloat();
    if (abs(new_val - a2) > 0.01) {
      a2 = new_val;
      if (abs(a2 - last_a2) > 0.01) {
        Serial.printf("A2:%.3f ", a2);
        last_a2 = a2;
        changed = true;
      }
    }
  }
  
  if (msg.startsWith("AXIS 4 ")) {
    float new_val = msg.substring(7).toFloat();
    if (abs(new_val - a4) > 0.01) {
      a4 = new_val;
      if (abs(a4 - last_a4) > 0.01) {
        Serial.printf("A4:%.3f ", a4);
        last_a4 = a4;
        changed = true;
      }
    }
  }
  
  if (msg.startsWith("AXIS 5 ")) {
    float new_val = msg.substring(7).toFloat();
    if (abs(new_val - a5) > 0.01) {
      a5 = new_val;
      if (abs(a5 - last_a5) > 0.01) {
        Serial.printf("A5:%.3f\n", a5);
        last_a5 = a5;
        changed = true;
      }
    }
  }
  
  if (changed) Serial.println();
}

void parseButtons(String msg) {
  if (msg.startsWith("BUTTON 11 ")) {
    float val = msg.substring(9).toFloat();
    bool state = (val > 0.5);
    if (state != last_button11) {
      button11_up = state;
      last_button11 = state;
      Serial.printf("ASANSOR YUKARI: %s\n", state ? "ON" : "OFF");
    }
  }
  
  if (msg.startsWith("BUTTON 12 ")) {
    float val = msg.substring(9).toFloat();
    bool state = (val > 0.5);
    if (state != last_button12) {
      button12_down = state;
      last_button12 = state;
      Serial.printf("ASANSOR ASAGI: %s\n", state ? "ON" : "OFF");
    }
  }
  
  if (msg.startsWith("BUTTON 0 ")) {
    float val = msg.substring(8).toFloat();
    bool state = (val > 0.5);
    if (state && !last_button0) {
      lid_open = !lid_open;
      lid_moving = true;
      lid_move_start = millis();
      Serial.printf("KAPAK: %s\n", lid_open ? "ACILIYOR" : "KAPANIYOR");
    }
    last_button0 = state;
  }
}

void handleElevator() {
  unsigned long now = millis();
  
  // YUKARI
  static bool up_running = false;
  if (button11_up && elevator_up_total < ELEVATOR_LIMIT_MS) {
    if (!up_running) {
      elevator_up_start = now;
      up_running = true;
    }
    unsigned long elapsed = now - elevator_up_start;
    if (elapsed > 100) {
      yukari(200);
      elevator_up_total = now - elevator_up_start;
      Serial.println("ASANSOR YUKARI 200PWM");
    }
  } else if (button11_up) {
    edur();
    Serial.println("ASANSOR YUKARI LIMIT");
  } else if (up_running) {
    elevator_up_total += (now - elevator_up_start);
    up_running = false;
  }
  
  // ASAGI
  static bool down_running = false;
  if (button12_down && elevator_down_total < ELEVATOR_LIMIT_MS) {
    if (!down_running) {
      elevator_down_start = now;
      down_running = true;
    }
    unsigned long elapsed = now - elevator_down_start;
    if (elapsed > 100) {
      asagi(200);
      elevator_down_total = now - elevator_down_start;
      Serial.println("ASANSOR ASAGI 200PWM");
    }
  } else if (button12_down) {
    edur();
    Serial.println("ASANSOR ASAGI LIMIT");
  } else if (down_running) {
    elevator_down_total += (now - elevator_down_start);
    down_running = false;
  }
}

void handleLid() {
  unsigned long now = millis();
  if (!lid_moving) return;
  
  unsigned long elapsed = now - lid_move_start;
  if (elapsed >= LID_MOVE_DURATION) {
    analogWrite(E_LID, 0);  // E_LID PWM KAPAT
    lid_moving = false;
    Serial.printf("E_LID %s TAMAMLANDI\n", lid_open ? "ACIK (90°)" : "KAPALI (0°)");  
    return;
  }
  analogWrite(E_LID, 200);  // SADECE E_LID ÇALIŞIR
}


void chassisMovement() {
  bool has_fwdback = (abs(a5 + 1) > 0.1 || abs(a4 + 1) > 0.1);
  bool has_siderot = (abs(a2) > 0.05 || abs(a0) > 0.05);
  
  if (has_fwdback && has_siderot) {
    Serial.println("MIX 40/60 MOD");
    mixMovement();
  }
  else if (a2 < -0.05) {
    int pwm = map(constrain((int)(abs(a2) * 100), 5, 100), 5, 100, 50, 255);
    Serial.printf("SOL KAYMA PWM=%d\n", pwm);
    sol(pwm);
  }
  else if (a2 > 0.05) {
    int pwm = map(constrain((int)(a2 * 100), 5, 100), 5, 100, 50, 255);
    Serial.printf("SAG KAYMA PWM=%d\n", pwm);
    sag(pwm);
  }
  else if (abs(a0) > 0.05) {
    int pwm = map(constrain((int)(abs(a0) * 100), 5, 100), 5, 100, 50, 255);
    Serial.printf("DONUS PWM=%d\n", pwm);
    donus360(pwm, a0 > 0);
  }
  else if (a5 > -0.9) {
    int pwm = map(constrain((int)((a5 + 1) * 50), 0, 100), 0, 100, 0, 255);
    Serial.printf("ILERI PWM=%d\n", pwm);
    ileri(pwm);
  }
  else if (a4 > -0.9) {
    int pwm = map(constrain((int)((a4 + 1) * 50), 0, 100), 0, 100, 0, 255);
    Serial.printf("GERI PWM=%d\n", pwm);
    geri(pwm);
  }
  else {
    Serial.println("DUR! ANI DURMA");
    aniDurChassis();
  }
}

void mixMovement() {
  // **HATA DÜZELTİLDİ: max() → constrain()**
  float fwd = constrain((a5 + 1) * 0.4, 0, 1);
  float back = constrain((a4 + 1) * 0.4, 0, 1);
  float side = constrain(a2 * 0.6, -1, 1);
  float rot = constrain(a0 * 0.6, -1, 1);
  
  int FL = constrain((int)((fwd - side + rot) * 255), -255, 255);
  int FR = constrain((int)((fwd + side - rot) * 255), -255, 255);
  int RL = constrain((int)((back + side + rot) * 255), -255, 255);
  int RR = constrain((int)((back - side - rot) * 255), -255, 255);
  
  setMotor(FL_PWM, FL_IN1, FL_IN2, abs(FL), FL >= 0);
  setMotor(FR_PWM, FR_IN1, FR_IN2, abs(FR), FR >= 0);
  setMotor(RL_PWM, RL_IN1, RL_IN2, abs(RL), RL >= 0);
  setMotor(RR_PWM, RR_IN1, RR_IN2, abs(RR), RR >= 0);
}

void setMotor(int pwm_pin, int in1, int in2, int pwm, bool forward) {
  digitalWrite(in1, forward ? HIGH : LOW);
  digitalWrite(in2, forward ? LOW : HIGH);
  analogWrite(pwm_pin, pwm);
}

// MOTOR FONKSİYONLARI (aynı kaldı)
void sol(int PWM) {
  digitalWrite(RR_IN1, LOW);  
  digitalWrite(RR_IN2, HIGH);
  digitalWrite(FR_IN1, HIGH); 
  digitalWrite(FR_IN2, LOW);
  digitalWrite(RL_IN1, HIGH);
  digitalWrite(RL_IN2, LOW);
  digitalWrite(FL_IN1, LOW);  
  digitalWrite(FL_IN2, HIGH);
  analogWrite(RR_PWM, PWM); analogWrite(FR_PWM, PWM);
  analogWrite(RL_PWM, PWM);  analogWrite(FL_PWM, PWM);
}

void sag(int PWM) {
  digitalWrite(RR_IN1, HIGH); digitalWrite(RR_IN2, LOW);
  digitalWrite(FR_IN1, LOW);  digitalWrite(FR_IN2, HIGH);
  digitalWrite(RL_IN1, LOW);  digitalWrite(RL_IN2, HIGH);
  digitalWrite(FL_IN1, HIGH); digitalWrite(FL_IN2, LOW);
  analogWrite(RR_PWM, PWM); analogWrite(FR_PWM, PWM);
  analogWrite(RL_PWM, PWM);  analogWrite(FL_PWM, PWM);
}

void ileri(int PWM) {
  digitalWrite(RR_IN1, HIGH); digitalWrite(RR_IN2, LOW);
  digitalWrite(FR_IN1, HIGH); digitalWrite(FR_IN2, LOW);
  digitalWrite(RL_IN1, HIGH); digitalWrite(RL_IN2, LOW);
  digitalWrite(FL_IN1, HIGH); digitalWrite(FL_IN2, LOW);
  analogWrite(RR_PWM, PWM); analogWrite(FR_PWM, PWM);
  analogWrite(RL_PWM, PWM);  analogWrite(FL_PWM, PWM);
}

void geri(int PWM) {
  digitalWrite(RR_IN1, LOW);  digitalWrite(RR_IN2, HIGH);
  digitalWrite(FR_IN1, LOW);  digitalWrite(FR_IN2, HIGH);
  digitalWrite(RL_IN1, LOW);  digitalWrite(RL_IN2, HIGH);
  digitalWrite(FL_IN1, LOW);  digitalWrite(FL_IN2, HIGH);
  analogWrite(RR_PWM, PWM); analogWrite(FR_PWM, PWM);
  analogWrite(RL_PWM, PWM);  analogWrite(FL_PWM, PWM);
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
  analogWrite(FR_PWM, PWM);
  analogWrite(RL_PWM, PWM);  
  analogWrite(FL_PWM, PWM);
}

void yukari(int PWM) {
  digitalWrite(EL_IN1, HIGH); 
  digitalWrite(EL_IN2, LOW);
  digitalWrite(ER_IN1, LOW);  
  digitalWrite(ER_IN2, HIGH);
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

void aniDurChassis() {
  digitalWrite(RR_IN1, HIGH); 
  digitalWrite(RR_IN2, HIGH);
  digitalWrite(RL_IN1, HIGH); 
  digitalWrite(RL_IN2, HIGH);
  digitalWrite(FL_IN1, HIGH); 
  digitalWrite(FL_IN2, HIGH);
  digitalWrite(FR_IN1, HIGH); 
  digitalWrite(FR_IN2, HIGH);
  analogWrite(RR_PWM, 255); 
  analogWrite(RL_PWM, 255);
  analogWrite(FL_PWM, 255); 
  analogWrite(FR_PWM, 255);
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

// YENİ LID FONKSİYONLARI
void BLID_open() {
}

void BLID_close() {

}

void HLID_open() {

}

void HLID_close() {

}
