// === Pin Definitions ===
// Encoders
#define RL_A 1
#define RL_B 2
#define RR_A 4
#define RR_B 5
#define FL_A 6
#define FL_B 7
#define FR_A 8
#define FR_B 9

// Ultrasonic
#define HCSR_TRIG 14
#define HCSR_FRONT 15
#define HCSR_RIGHT 16
#define HCSR_LEFT 17
#define HCSR_REAR 18

// === Encoder Counts ===
volatile long countRL = 0;
volatile long countRR = 0;
volatile long countFL = 0;
volatile long countFR = 0;

// === Timing ===
unsigned long lastTime = 0;

// === ISRs ===
void IRAM_ATTR isrRL() { if (digitalRead(RL_A) == digitalRead(RL_B)) countRL++; else countRL--; }
void IRAM_ATTR isrRR() { if (digitalRead(RR_A) == digitalRead(RR_B)) countRR++; else countRR--; }
void IRAM_ATTR isrFL() { if (digitalRead(FL_A) == digitalRead(FL_B)) countFL++; else countFL--; }
void IRAM_ATTR isrFR() { if (digitalRead(FR_A) == digitalRead(FR_B)) countFR++; else countFR--; }

// === RPM Calculation ===
float calcRPM(long &pulses) {
  const int PPR = 100; // replace with your encoder’s pulses per revolution
  long p = pulses;
  pulses = 0; // reset after reading
  return (p / (float)PPR) * 60.0;
}

// === HC-SR04 Distance Function ===
long readHCSR(int echoPin) {
  digitalWrite(HCSR_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(HCSR_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(HCSR_TRIG, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000); // timeout 30ms
  long distance = duration * 0.034 / 2;          // cm
  return distance;
}

// === Flags for Continuous Output ===
bool showFRRPM = false;
bool showFLRPM = false;
bool showRRRPM = false;
bool showRLRPM = false;

bool showFRHCSR = false;
bool showRTHCSR = false;
bool showLTHCSR = false;
bool showRRHCSR = false;

// === Setup ===
void setup() {
  Serial.begin(115200);

  // Encoder pins
  pinMode(RL_A, INPUT); pinMode(RL_B, INPUT);
  pinMode(RR_A, INPUT); pinMode(RR_B, INPUT);
  pinMode(FL_A, INPUT); pinMode(FL_B, INPUT);
  pinMode(FR_A, INPUT); pinMode(FR_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(RL_A), isrRL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RR_A), isrRR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(FL_A), isrFL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(FR_A), isrFR, CHANGE);

  // Ultrasonic pins
  pinMode(HCSR_TRIG, OUTPUT);
  pinMode(HCSR_FRONT, INPUT);
  pinMode(HCSR_RIGHT, INPUT);
  pinMode(HCSR_LEFT, INPUT);
  pinMode(HCSR_REAR, INPUT);

  Serial.println("Ready to start.");
  lastTime = millis();
}

// === Loop ===
void loop() {
  // Handle serial commands
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    // Enable continuous RPM
    if (cmd.equalsIgnoreCase("FRRPM")) showFRRPM = true;
    else if (cmd.equalsIgnoreCase("FLRPM")) showFLRPM = true;
    else if (cmd.equalsIgnoreCase("RRRPM")) showRRRPM = true;
    else if (cmd.equalsIgnoreCase("RLRPM")) showRLRPM = true;

    // Disable continuous RPM
    else if (cmd.equalsIgnoreCase("STOPFRRPM")) showFRRPM = false;
    else if (cmd.equalsIgnoreCase("STOPFLRPM")) showFLRPM = false;
    else if (cmd.equalsIgnoreCase("STOPRRRPM")) showRRRPM = false;
    else if (cmd.equalsIgnoreCase("STOPRLRPM")) showRLRPM = false;

    // Enable continuous HCSR
    else if (cmd.equalsIgnoreCase("FRHCSR")) showFRHCSR = true;
    else if (cmd.equalsIgnoreCase("RTHCSR")) showRTHCSR = true;
    else if (cmd.equalsIgnoreCase("LTHCSR")) showLTHCSR = true;
    else if (cmd.equalsIgnoreCase("RRHCSR")) showRRHCSR = true;

    // Disable continuous HCSR
    else if (cmd.equalsIgnoreCase("STOPFRHCSR")) showFRHCSR = false;
    else if (cmd.equalsIgnoreCase("STOPRTHCSR")) showRTHCSR = false;
    else if (cmd.equalsIgnoreCase("STOPLTHCSR")) showLTHCSR = false;
    else if (cmd.equalsIgnoreCase("STOPRRHCSR")) showRRHCSR = false;

    else Serial.println("Unknown command.");
  }

  // Continuous outputs every 1 second
  unsigned long now = millis();
  if (now - lastTime >= 1000) {
    if (showFRRPM) { Serial.print("FR RPM: "); Serial.println(calcRPM(countFR)); }
    if (showFLRPM) { Serial.print("FL RPM: "); Serial.println(calcRPM(countFL)); }
    if (showRRRPM) { Serial.print("RR RPM: "); Serial.println(calcRPM(countRR)); }
    if (showRLRPM) { Serial.print("RL RPM: "); Serial.println(calcRPM(countRL)); }

    if (showFRHCSR) { Serial.print("Front Distance: "); Serial.print(readHCSR(HCSR_FRONT)); Serial.println(" cm"); }
    if (showRTHCSR) { Serial.print("Right Distance: "); Serial.print(readHCSR(HCSR_RIGHT)); Serial.println(" cm"); }
    if (showLTHCSR) { Serial.print("Left Distance: "); Serial.print(readHCSR(HCSR_LEFT)); Serial.println(" cm"); }
    if (showRRHCSR) { Serial.print("Rear Distance: "); Serial.print(readHCSR(HCSR_REAR)); Serial.println(" cm"); }

    lastTime = now;
  }
}