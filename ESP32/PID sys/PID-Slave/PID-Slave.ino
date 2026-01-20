// ======================= SLAVE MOTOR CONTROLLER =======================
// ESP32-S3-N8R2 - Drives 6 TB6612FNG Motors via UART1 commands
// 4 Movement Motors (FL, FR, BL, BR) - Forward/Backward
// 2 Lift Motors (LU, LD) - Synchronized, Negative=Down, Positive=Up

// ======================= PIN DEFINITIONS =======================
// UART1 Pins (receiving from Master ESP32-S3-N16R8)
// Master TX (pin 17) connects to Slave RX (pin 17)
// Master RX (pin 18) connects to Slave TX (pin 18)
#define UART1_RX 17
#define UART1_TX 18

// Motor Control Pins (TB6612FNG - IN1, IN2, PWM for each motor)
// Front-Left Motor
#define FL_IN1 1
#define FL_IN2 2
#define FL_PWM 3

// Front-Right Motor
#define FR_IN1 4
#define FR_IN2 5
#define FR_PWM 6

// Back-Left Motor
#define BL_IN1 7
#define BL_IN2 8
#define BL_PWM 9

// Back-Right Motor
#define BR_IN1 10
#define BR_IN2 11
#define BR_PWM 12

// Lift-Up Motor
#define LU_IN1 13
#define LU_IN2 14
#define LU_PWM 15

// Lift-Down Motor
#define LD_IN1 19
#define LD_IN2 20
#define LD_PWM 21

// ======================= PWM CONFIGURATION =======================
const int pwmFrequency = 5000;      // 5kHz PWM frequency
const int pwmResolution = 8;        // 8-bit resolution (0-255)
const int pwmChannel_FL = 0;
const int pwmChannel_FR = 1;
const int pwmChannel_BL = 2;
const int pwmChannel_BR = 3;
const int pwmChannel_LU = 4;
const int pwmChannel_LD = 5;

// ======================= MOTOR STATE =======================
int motor_fl = 0;  // -255 to +255
int motor_fr = 0;
int motor_bl = 0;
int motor_br = 0;
int motor_lu = 0;  // -255 to +255 (negative = down, positive = up)
int motor_ld = 0;  // Synchronized with LU

// ======================= TIMING =======================
unsigned long lastMotorUpdate = 0;
const long motorUpdateInterval = 50; // 50ms update rate

// ======================= FUNCTION PROTOTYPES =======================
void initUART1();
void initMotors();
void parseUART1Command(String cmd);
void setMotor(int motorPin_IN1, int motorPin_IN2, int pwmChannel, int pwmValue);
void updateMotors();
void printMotorStatus();

// ======================= SETUP =======================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n\n=== ESP32-S3-N8R2 Motor Controller Starting ===");
  
  initMotors();
  initUART1();
  
  Serial.println("=== System Ready ===\n");
}

// ======================= MAIN LOOP =======================
void loop() {
  // Check for UART1 commands
  if (Serial2.available()) {
    String command = Serial2.readStringUntil('\n');
    command.trim();
    if (command.length() > 0) {
      parseUART1Command(command);
    }
  }
  
  // Update motors at fixed interval
  unsigned long currentMillis = millis();
  if (currentMillis - lastMotorUpdate >= motorUpdateInterval) {
    lastMotorUpdate = currentMillis;
    updateMotors();
    printMotorStatus();
  }
}

// ======================= UART1 INITIALIZATION =======================
void initUART1() {
  Serial2.begin(115200, SERIAL_8N1, UART1_RX, UART1_TX);
  Serial.println("[UART1] Initialized at 115200 baud");
  Serial.println("[UART1] Waiting for motor commands...\n");
}

// ======================= MOTOR INITIALIZATION =======================
void initMotors() {
  // Setup PWM channels for all motors
  ledcSetup(pwmChannel_FL, pwmFrequency, pwmResolution);
  ledcSetup(pwmChannel_FR, pwmFrequency, pwmResolution);
  ledcSetup(pwmChannel_BL, pwmFrequency, pwmResolution);
  ledcSetup(pwmChannel_BR, pwmFrequency, pwmResolution);
  ledcSetup(pwmChannel_LU, pwmFrequency, pwmResolution);
  ledcSetup(pwmChannel_LD, pwmFrequency, pwmResolution);
  
  // Attach PWM pins
  ledcAttachPin(FL_PWM, pwmChannel_FL);
  ledcAttachPin(FR_PWM, pwmChannel_FR);
  ledcAttachPin(BL_PWM, pwmChannel_BL);
  ledcAttachPin(BR_PWM, pwmChannel_BR);
  ledcAttachPin(LU_PWM, pwmChannel_LU);
  ledcAttachPin(LD_PWM, pwmChannel_LD);
  
  // Setup direction pins
  pinMode(FL_IN1, OUTPUT);
  pinMode(FL_IN2, OUTPUT);
  pinMode(FR_IN1, OUTPUT);
  pinMode(FR_IN2, OUTPUT);
  pinMode(BL_IN1, OUTPUT);
  pinMode(BL_IN2, OUTPUT);
  pinMode(BR_IN1, OUTPUT);
  pinMode(BR_IN2, OUTPUT);
  pinMode(LU_IN1, OUTPUT);
  pinMode(LU_IN2, OUTPUT);
  pinMode(LD_IN1, OUTPUT);
  pinMode(LD_IN2, OUTPUT);
  
  // All motors off initially
  setMotor(FL_IN1, FL_IN2, pwmChannel_FL, 0);
  setMotor(FR_IN1, FR_IN2, pwmChannel_FR, 0);
  setMotor(BL_IN1, BL_IN2, pwmChannel_BL, 0);
  setMotor(BR_IN1, BR_IN2, pwmChannel_BR, 0);
  setMotor(LU_IN1, LU_IN2, pwmChannel_LU, 0);
  setMotor(LD_IN1, LD_IN2, pwmChannel_LD, 0);
  
  Serial.println("[Motors] All motors initialized and stopped");
}

// ======================= PARSE UART COMMANDS =======================
void parseUART1Command(String cmd) {
  // Expected format: "MOT_ID PWM_VALUE"
  // Example: "FL 200", "BR -150", "LU 100" (lift up), "LU -100" (lift down)
  
  int spaceIndex = cmd.indexOf(' ');
  if (spaceIndex == -1) return;
  
  String motorId = cmd.substring(0, spaceIndex);
  int pwmValue = cmd.substring(spaceIndex + 1).toInt();
  
  // Constrain to valid range
  pwmValue = constrain(pwmValue, -255, 255);
  
  // Update motor based on ID
  if (motorId == "FL") {
    motor_fl = pwmValue;
    Serial.println("[CMD] FL Motor: " + String(pwmValue));
  }
  else if (motorId == "FR") {
    motor_fr = pwmValue;
    Serial.println("[CMD] FR Motor: " + String(pwmValue));
  }
  else if (motorId == "BL") {
    motor_bl = pwmValue;
    Serial.println("[CMD] BL Motor: " + String(pwmValue));
  }
  else if (motorId == "BR") {
    motor_br = pwmValue;
    Serial.println("[CMD] BR Motor: " + String(pwmValue));
  }
  else if (motorId == "LU") {
    // Lift motors synchronized
    motor_lu = pwmValue;
    motor_ld = pwmValue;
    if (pwmValue > 0) {
      Serial.println("[CMD] Lift UP: " + String(pwmValue));
    } else if (pwmValue < 0) {
      Serial.println("[CMD] Lift DOWN: " + String(abs(pwmValue)));
    } else {
      Serial.println("[CMD] Lift STOP");
    }
  }
  else if (motorId == "LD") {
    // If separate lift down command received, sync both
    motor_lu = pwmValue;
    motor_ld = pwmValue;
    if (pwmValue > 0) {
      Serial.println("[CMD] Lift UP: " + String(pwmValue));
    } else if (pwmValue < 0) {
      Serial.println("[CMD] Lift DOWN: " + String(abs(pwmValue)));
    } else {
      Serial.println("[CMD] Lift STOP");
    }
  }
}

// ======================= SET MOTOR FUNCTION =======================
// pwmValue: -255 to +255 (negative = backward, positive = forward)
void setMotor(int motorPin_IN1, int motorPin_IN2, int pwmChannel, int pwmValue) {
  // Constrain value
  pwmValue = constrain(pwmValue, -255, 255);
  
  if (pwmValue > 0) {
    // Forward
    digitalWrite(motorPin_IN1, HIGH);
    digitalWrite(motorPin_IN2, LOW);
    ledcWrite(pwmChannel, pwmValue);
  }
  else if (pwmValue < 0) {
    // Backward
    digitalWrite(motorPin_IN1, LOW);
    digitalWrite(motorPin_IN2, HIGH);
    ledcWrite(pwmChannel, abs(pwmValue));
  }
  else {
    // Stop (coast)
    digitalWrite(motorPin_IN1, LOW);
    digitalWrite(motorPin_IN2, LOW);
    ledcWrite(pwmChannel, 0);
  }
}

// ======================= UPDATE ALL MOTORS =======================
void updateMotors() {
  setMotor(FL_IN1, FL_IN2, pwmChannel_FL, motor_fl);
  setMotor(FR_IN1, FR_IN2, pwmChannel_FR, motor_fr);
  setMotor(BL_IN1, BL_IN2, pwmChannel_BL, motor_bl);
  setMotor(BR_IN1, BR_IN2, pwmChannel_BR, motor_br);
  setMotor(LU_IN1, LU_IN2, pwmChannel_LU, motor_lu);
  setMotor(LD_IN1, LD_IN2, pwmChannel_LD, motor_ld);
}

// ======================= PRINT MOTOR STATUS =======================
void printMotorStatus() {
  Serial.print("MOTORS: FL=");
  Serial.print(motor_fl);
  Serial.print(" FR=");
  Serial.print(motor_fr);
  Serial.print(" BL=");
  Serial.print(motor_bl);
  Serial.print(" BR=");
  Serial.print(motor_br);
  Serial.print(" | LIFT: ");
  if (motor_lu > 0) {
    Serial.print("UP(");
    Serial.print(motor_lu);
    Serial.print(")");
  } else if (motor_lu < 0) {
    Serial.print("DOWN(");
    Serial.print(abs(motor_lu));
    Serial.print(")");
  } else {
    Serial.print("STOP");
  }
  Serial.println();
}

// ======================= MOTOR CONTROL REFERENCE =======================
/*
  UART Message Format from Master:
  
  Movement Motors (Forward/Backward):
  - "FL 200"   -> Front-Left forward at 200/255 PWM
  - "FL -200"  -> Front-Left backward at 200/255 PWM
  - "FL 0"     -> Front-Left stop
  
  Same for: FR, BL, BR
  
  Lift Motors (Synchronized):
  - "LU 150"   -> Lift UP at 150/255 PWM (both motors)
  - "LU -150"  -> Lift DOWN at 150/255 PWM (both motors)
  - "LU 0"     -> Lift STOP (both motors)
  
  PWM Range: -255 to +255
  Negative = Backward/Down
  Positive = Forward/Up
  
  All lift motors move together regardless of LU or LD command.
*/
