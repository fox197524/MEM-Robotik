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
float last_a0=999, last_a2=999, last_a4=999, last_a5=999;

void setup() {
  Serial.begin(115200);
  Serial.print("calis");
  
  int pins[] = {RL_PIN_PWM, RL_PIN_IN1, RL_PIN_IN2, 
                RR_PIN_PWM, RR_IN1, RR_IN2, 
                FL_PIN_PWM, FL_IN1, FL_IN2, 
                FR_PIN_PWM, FR_IN1, FR_IN2};
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
  // WiFi watchdog
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi lost, restarting...");
    ESP.restart();
  }
  
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(packetBuffer, 255);
    if (len > 0 && len < 255) {
      packetBuffer[len] = 0;
      String msg = String(packetBuffer);
      
      Serial.print("RAW: "); Serial.println(msg);
      
      if (msg == lastMsg) return;
      lastMsg = msg;
      
      bool axisChanged = false;
      
      if (msg.startsWith("AXIS 0 ")) {
        float new_a0 = msg.substring(7).toFloat();
        if (abs(new_a0 - a0) > 0.01) {
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
      
      if (axisChanged) Serial.println();
      
      // Movement priority logic with proper PWM calculation
      if (abs(a2) > 0.05) {  // Steering priority (left/right drift)
        int pwm_val = constrain(map(abs(a2) * 100, 5, 100, 50, 255), 0, 255);
        Serial.printf("STEER: a2=%.3f PWM=%d\n", a2, pwm_val);
        if (a2 < 0) sol(pwm_val);
        else sag(pwm_val);
      } 
      else if (abs(a0) > 0.05) {  // Spin priority (360 turn)
        int pwm_val = constrain(map(abs(a0) * 100, 5, 100, 50, 255), 0, 255);
        Serial.printf("SPIN: a0=%.3f PWM=%d\n", a0, pwm_val);
        donus360(pwm_val, a0 > 0);
      }
      else if (abs(a5 + 0.5) > 0.1) {  // Forward/Back on AXIS 5
        int pwm_val = constrain(map(abs(a5 + 0.5) * 500, 100, 500, 50, 255), 0, 255);
        Serial.printf("MOVE: a5=%.3f PWM=%d\n", a5, pwm_val);
        if (a5 > -0.5) ileri(pwm_val);
        else geri(pwm_val);
      }
      else if (abs(a4 + 0.5) > 0.1) {  // Alternative Forward/Back on AXIS 4
        int pwm_val = constrain(map(abs(a4 + 0.5) * 500, 100, 500, 50, 255), 0, 255);
        Serial.printf("MOVE: a4=%.3f PWM=%d\n", a4, pwm_val);
        if (a4 > -0.5) ileri(pwm_val);
        else geri(pwm_val);
      }
      else {
        Serial.println("STOP");
        aniDur();
      }
    }
  }
}

// FIXED: Proper emergency braking with longer brake time
void aniDur() {
  // Apply brakes (short circuit motors)
  digitalWrite(RR_IN1, HIGH); digitalWrite(RR_IN2, HIGH);
  digitalWrite(RL_PIN_IN1, HIGH); digitalWrite(RL_PIN_IN2, HIGH);
  digitalWrite(FL_IN1, HIGH); digitalWrite(FL_IN2, HIGH);
  digitalWrite(FR_IN1, HIGH); digitalWrite(FR_IN2, HIGH);
  
  analogWrite(RR_PIN_PWM, 255);
  analogWrite(RL_PIN_PWM, 255);
  analogWrite(FL_PIN_PWM, 255);
  analogWrite(FR_PIN_PWM, 255);
  
  delay(100);  // Increased brake time for better stopping power
  
  // Coast stop (all off)
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
  if (sagaDonus) {
    digitalWrite(RR_IN1, LOW); digitalWrite(RR_IN2, HIGH);
    digitalWrite(FR_IN1, LOW); digitalWrite(FR_IN2, HIGH);
    digitalWrite(RL_PIN_IN1, HIGH); digitalWrite(RL_PIN_IN2, LOW);
    digitalWrite(FL_IN1, HIGH); digitalWrite(FL_IN2, LOW);
  } else {
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
