#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// --- MOTOR PIN DEFINITIONS ---
// Rear Left Motor Pins
const int RL_PWM = 7;   
const int RL_IN1 = 8;   
const int RL_IN2 = 9;   

// Rear Right Motor Pins
const int RR_PWM = 12;
const int RR_IN1 = 11;
const int RR_IN2 = 13;

// Front Left Motor Pins
const int FL_PWM = 15;
const int FL_IN1 = 16;
const int FL_IN2 = 17;

// Front Right Motor Pins
const int FR_PWM = 4;
const int FR_IN1 = 5;
const int FR_IN2 = 6;

// Elevator Right Motor Pins
const int ER_PWM = 1;
const int ER_IN1 = 2;
const int ER_IN2 = 42;

// Elevator Left Motor Pins
const int EL_PWM = 18;
const int EL_IN1 = 19;
const int EL_IN2 = 21;

const int E_LID = 41;


// --- UDP COMMUNICATION SETTINGS ---
WiFiUDP udp;
unsigned int localPort = 4210;
char packetBuffer[255];

// Joystick Axis Values
float a0=0, a2=0, a4=0, a5=0, a12=0, a13=0, a6=0; 
String lastMsg = "";

// Variables to track changes to prevent Serial flooding
float last_a0=999, last_a2=999, last_a4=999, last_a5=999, last_a12=999, last_a13=999, last_a6=999,;  

void setup() {
  Serial.begin(115200);
  Serial.print("calis");
  
  // Initialize motor pins as OUTPUT
  int pins[] = {RL_PWM, RL_IN1, RL_IN2, RR__PWM, RR_IN1, RR_IN2, FL_PWM, FL_IN1, FL_IN2, FR_PWM, FR_IN1, FR_IN2, ER_IN1, ER_IN2, ER_PWM, EL_IN1, EL_IN2, EL_PWM, E_LID };
  for(int p : pins) pinMode(p, OUTPUT);

  // Connect to WiFi network
  WiFi.begin("LAGARIMEDYA", "lagari5253");
  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("\nWiFi Connected!");
  Serial.print("ESP32 IP: "); Serial.println(WiFi.localIP());

  // Set power/relay pin state
  
  // Start listening for UDP packets
  udp.begin(localPort);
}

void loop() {
  // Check for incoming UDP packets
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(packetBuffer, 255);
    packetBuffer[len] = 0;
    String msg = String(packetBuffer);
    
    Serial.print("RAW: "); Serial.println(msg);  // Debug raw message
    
    // Ignore if the message is the same as the previous one
    if (msg == lastMsg) return;
    lastMsg = msg;
    
    // Axis parsing - Update values and print only if a significant change occurs
    bool axisChanged = false;
    
    // Parse Axis 0 (Typically used for Rotation)
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
    
    // Parse Axis 2 (Typically used for Sideways/Strafe)
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
    
    // Parse Axis 4 (Typically used for Backward movement)
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
    
    // Parse Axis 5 (Typically used for Forward movement)
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
    // Parse Axis 5 (Typically used for Forward movement)
    if (msg.startsWith("AXIS 12 ")) {
      float new_a12 = msg.substring(7).toFloat();
      if (abs(new_a12 - a12) > 0.01) {
        a12 = new_a12;
        if (abs(a12 - last_a12) > 0.01) {
          Serial.printf("A12:%.3f\n", a12);
          last_a12 = a12;
          axisChanged = true;
        }
      }
    }
    // Parse Axis 5 (Typically used for Forward movement)
    if (msg.startsWith("AXIS 13 ")) {
      float new_a13 = msg.substring(7).toFloat();
      if (abs(new_a13 - a13) > 0.01) {
        a13 = new_a13;
        if (abs(a13 - last_a13) > 0.01) {
          Serial.printf("A13:%.3f\n", a13);
          last_a13 = a13;
          axisChanged = true;
        }
      }
    }
    // Parse Axis 5 (Typically used for Forward movement)
    if (msg.startsWith("AXIS 6 ")) {
      float new_a6 = msg.substring(7).toFloat();
      if (abs(new_a6 - a6) > 0.01) {
        a6 = new_a6;
        if (abs(a6 - last_a6) > 0.01) {
          Serial.printf("A6:%.3f\n", a6);
          last_a6 = a6;
          axisChanged = true;
        }
      }
    }
    
    if (axisChanged) Serial.println();
    
    // MOTOR LOGIC - Priority Based Movement
    int pwm_val;
    
    // STRAFE LEFT
    if (a2 < -0.05) {  
      pwm_val = map(abs(a2) * 100, 5, 100, 50, 255);
      Serial.printf("SOL: a2=%.3f PWM=%d\n", a2, pwm_val);
      sol(pwm_val);
    }
    // STRAFE RIGHT
    else if (a2 > 0.05) {  
      pwm_val = map(a2 * 100, 5, 100, 50, 255);
      Serial.printf("SAĞ: a2=%.3f PWM=%d\n", a2, pwm_val);
      sag(pwm_val);
    }
    // ROTATE 360 (Z-axis)
    else if (abs(a0) > 0.05) {  
      pwm_val = map(abs(a0) * 100, 5, 100, 50, 255);
      Serial.printf("DÖN: a0=%.3f PWM=%d\n", a0, pwm_val);
      donus360(pwm_val, a0 > 0);
    }
    // FORWARD / BACKWARD / MIXED MOVEMENT
    else if (a5 > -0.9 || a4 > -0.9) {  
      int pwm_ileri = map((a5 + 1) * 50, 0, 100, 0, 255);
      int pwm_geri = map((a4 + 1) * 50, 0, 100, 0, 255);
      
      // MOVE FORWARD ONLY
      if (a5 > -0.9 && a4 <= -0.9) {  
        Serial.printf("ILERI: a5=%.3f PWM=%d\n", a5, pwm_ileri);
        ileri(pwm_ileri);
      }
      // MOVE BACKWARD ONLY
      else if (a4 > -0.9 && a5 <= -0.9) {  
        Serial.printf("GERI: a4=%.3f PWM=%d\n", a4, pwm_geri);
        geri(pwm_geri);
      }
      // MIXED FORWARD + TURNING
      else if (a5 > -0.9 && a4 > -0.9) {  
        int mix_pwm = (pwm_ileri * 0.4 + 80 * 0.6);
        Serial.printf("MIX: a5=%.3f PWM=%d\n", a5, mix_pwm);
        ileri_mix(mix_pwm);
      }
    }
    //asansör 
    else if (a12 > -0.9 && a13 <= -0.9){
      int
      

    }
    // STOP IF NO INPUT
    else {  
      Serial.println("DUR!");
      aniDur();
    }
  }
}

// FORWARD + LIGHT TURN MIX (Differential speed)
void ileri_mix(int PWM) {
  digitalWrite(RR_IN1, HIGH); digitalWrite(RR_IN2, LOW);
  digitalWrite(FR_IN1, HIGH); digitalWrite(FR_IN2, LOW);
  digitalWrite(RL_IN1, HIGH); digitalWrite(RL_IN2, LOW);
  digitalWrite(FL_IN1, HIGH); digitalWrite(FL_IN2, LOW);
  
  analogWrite(RR_PWM, PWM);         // Right side full speed
  analogWrite(RL_PWM, PWM * 0.75);  // Left side 75% speed
  analogWrite(FL_PWM, PWM * 0.75);  // Front Left 75% speed
  analogWrite(FR_PWM, PWM);         // Front Right full speed
}

// EMERGENCY BRAKE AND STOP
void aniDur() {
  // BRAKE PHASE: All motor pins set to HIGH to lock motors
  digitalWrite(RR_IN1, HIGH); digitalWrite(RR_IN2, HIGH);
  digitalWrite(RL_IN1, HIGH); digitalWrite(RL_IN2, HIGH);
  digitalWrite(FL_IN1, HIGH); digitalWrite(FL_IN2, HIGH);
  digitalWrite(FR_IN1, HIGH); digitalWrite(FR_IN2, HIGH);
  
  analogWrite(RR_PWM, 255);
  analogWrite(RL_PWM, 255);
  analogWrite(FL_PWM, 255);
  analogWrite(FR_PWM, 255);
  
  delay(30);  // Short brake duration
  
  // OFF PHASE: All pins set to LOW to release power
  digitalWrite(RR_IN1, LOW); digitalWrite(RR_IN2, LOW);
  digitalWrite(RL_IN1, LOW); digitalWrite(RL_IN2, LOW);
  digitalWrite(FL_IN1, LOW); digitalWrite(FL_IN2, LOW);
  digitalWrite(FR_IN1, LOW); digitalWrite(FR_IN2, LOW);
  
  analogWrite(RR_PWM, 0);
  analogWrite(RL_PWM, 0);
  analogWrite(FL_PWM, 0);
  analogWrite(FR_PWM, 0);
}

// STRAFE LEFT (Mecanum Logic)
void sol(int PWM) {
  digitalWrite(RR_IN1, LOW); digitalWrite(RR_IN2, HIGH);
  digitalWrite(FR_IN1, HIGH); digitalWrite(FR_IN2, LOW);
  digitalWrite(RL_IN1, HIGH); digitalWrite(RL_IN2, LOW);
  digitalWrite(FL_IN1, LOW); digitalWrite(FL_IN2, HIGH);
  
  analogWrite(RR_PWM, PWM);
  analogWrite(RL_PWM, PWM);
  analogWrite(FL_PWM, PWM);
  analogWrite(FR_PWM, PWM);
}

// MOVE STRAIGHT FORWARD
void ileri(int PWM) {
  digitalWrite(RR_IN1, HIGH); digitalWrite(RR_IN2, LOW);
  digitalWrite(FR_IN1, HIGH); digitalWrite(FR_IN2, LOW);
  digitalWrite(RL_IN1, HIGH); digitalWrite(RL_IN2, LOW);
  digitalWrite(FL_IN1, HIGH); digitalWrite(FL_IN2, LOW);
  
  analogWrite(RR_PWM, PWM);
  analogWrite(RL_PWM, PWM);
  analogWrite(FL_PWM, PWM);
  analogWrite(FR_PWM, PWM);
}

// MOVE STRAIGHT BACKWARD
void geri(int PWM) {
  digitalWrite(RR_IN1, LOW); digitalWrite(RR_IN2, HIGH);
  digitalWrite(FR_IN1, LOW); digitalWrite(FR_IN2, HIGH);
  digitalWrite(RL_IN1, LOW); digitalWrite(RL_IN2, HIGH);
  digitalWrite(FL_IN1, LOW); digitalWrite(FL_IN2, HIGH);
  
  analogWrite(RR_PWM, PWM);
  analogWrite(RL_PWM, PWM);
  analogWrite(FL_PWM, PWM);
  analogWrite(FR_PWM, PWM);
}

// ROTATE ON AXIS (360 DEGREES)
void donus360(int PWM, bool sagaDonus) {
  if (sagaDonus) { // Rotate Right
    digitalWrite(RR_IN1, LOW); digitalWrite(RR_IN2, HIGH);
    digitalWrite(FR_IN1, LOW); digitalWrite(FR_IN2, HIGH);
    digitalWrite(RL_IN1, HIGH); digitalWrite(RL_IN2, LOW);
    digitalWrite(FL_IN1, HIGH); digitalWrite(FL_IN2, LOW);
  } else { // Rotate Left
    digitalWrite(RR_IN1, HIGH); digitalWrite(RR_IN2, LOW);
    digitalWrite(FR_IN1, HIGH); digitalWrite(FR_IN2, LOW);
    digitalWrite(RL_IN1, LOW); digitalWrite(RL_IN2, HIGH);
    digitalWrite(FL_IN1, LOW); digitalWrite(FL_IN2, HIGH);
  }
  
  analogWrite(RR_PWM, PWM);
  analogWrite(RL_PWM, PWM);
  analogWrite(FL_PWM, PWM);
  analogWrite(FR_PWM, PWM);
}

// STRAFE RIGHT (Mecanum Logic)
void sag(int PWM) {
  digitalWrite(RR_IN1, HIGH); digitalWrite(RR_IN2, LOW);
  digitalWrite(FR_IN1, LOW); digitalWrite(FR_IN2, HIGH);
  digitalWrite(RL_IN1, LOW); digitalWrite(RL_IN2, HIGH);
  digitalWrite(FL_IN1, HIGH); digitalWrite(FL_IN2, LOW);
  
  analogWrite(RR_PWM, PWM);
  analogWrite(RL_PWM, PWM);
  analogWrite(FL_PWM, PWM);
  analogWrite(FR_PWM, PWM);
}

// Future implementation for upward movement
void yukari (int PWM) {

  digitalWrite(EL_IN1, HIGH); digitalWrite(EL_IN2, LOW);
  digitalWrite(ER_IN1, LOW); digitalWrite(ER_IN2, HIGH);

  analogWrite(ER_PWM, PWM);
  analogWrite(EL_PWM, PWM);

}

// Future implementation for downward movement
void asagi(int PWM) {

  digitalWrite(EL_IN1, HIGH); digitalWrite(EL_IN2, LOW);
  digitalWrite(ER_IN1, LOW); digitalWrite(ER_IN2, HIGH);

  analogWrite(EL_PWM, PWM);
  analogWrite(ER_PWM, PWM);

}

void edur() {

  // BRAKE PHASE: All motor pins set to HIGH to lock motors
  digitalWrite(ER_IN1, HIGH); digitalWrite(ER_IN2, HIGH);
  digitalWrite(EL_IN1, HIGH); digitalWrite(EL_IN2, HIGH);

  analogWrite(EL_PWM, 255);
  analogWrite(ER_PWM, 255);
  
  delay(30);  // Short brake duration
  
  // OFF PHASE: All pins set to LOW to release power
  digitalWrite(ER_IN1, LOW); digitalWrite(ER_IN2, LOW);
  digitalWrite(EL_IN1, LOW); digitalWrite(EL_IN2, LOW);

  analogWrite(EL_PWM, 0);
  analogWrite(ER_PWM, 0);

}

void open(int PWM) {
  
  analogWrite(E_LID, PWM);

}

void close(){



}







