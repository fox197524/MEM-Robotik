/*
  First: rear right: AIN1: 4 AIN2: 5 A1: motor positive A2: motor negative STBY: 6
  Second: rear left: AIN1: 7 AIN2: 15 A1: motor positive A2: motor negative STBY: 16
  Third: front right: AIN1: 17 AIN2: 18 A1: motor positive A2: motor negative STBY: 8
  Fourth: front left: AIN1: 3 AIN2: 46 A1: motor positive A2: motor negative STBY: 9
*/

#define RR_AIN1 4
#define RR_AIN2 5
#define RR_STBY 6

#define RL_AIN1 7
#define RL_AIN2 15
#define RL_STBY 16

#define FR_AIN1 17
#define FR_AIN2 18
#define FR_STBY 8

#define FL_AIN1 3
#define FL_AIN2 46
#define FL_STBY 9

void setup() {
  pinMode(RR_AIN1, OUTPUT); pinMode(RR_AIN2, OUTPUT); pinMode(RR_STBY, OUTPUT);
  pinMode(RL_AIN1, OUTPUT); pinMode(RL_AIN2, OUTPUT); pinMode(RL_STBY, OUTPUT);
  pinMode(FR_AIN1, OUTPUT); pinMode(FR_AIN2, OUTPUT); pinMode(FR_STBY, OUTPUT);
  pinMode(FL_AIN1, OUTPUT); pinMode(FL_AIN2, OUTPUT); pinMode(FL_STBY, OUTPUT);
  
  // Enable all drivers
  digitalWrite(RR_STBY, HIGH);
  digitalWrite(RL_STBY, HIGH);
  digitalWrite(FR_STBY, HIGH);
  digitalWrite(FL_STBY, HIGH);
}

void loop() {
  // Test 1: All forward 3 seconds
  Serial.println("All FORWARD");
  digitalWrite(RR_AIN1, HIGH); digitalWrite(RR_AIN2, LOW);
  digitalWrite(RL_AIN1, HIGH); digitalWrite(RL_AIN2, LOW);
  digitalWrite(FR_AIN1, HIGH); digitalWrite(FR_AIN2, LOW);
  digitalWrite(FL_AIN1, HIGH); digitalWrite(FL_AIN2, LOW);
  delay(3000);
  
  // Test 2: All stop 1 second
  Serial.println("ALL STOP");
  digitalWrite(RR_AIN1, LOW); digitalWrite(RR_AIN2, LOW);
  digitalWrite(RL_AIN1, LOW); digitalWrite(RL_AIN2, LOW);
  digitalWrite(FR_AIN1, LOW); digitalWrite(FR_AIN2, LOW);
  digitalWrite(FL_AIN1, LOW); digitalWrite(FL_AIN2, LOW);
  delay(1000);
  
  // Test 3: All reverse 3 seconds
  Serial.println("All REVERSE");
  digitalWrite(RR_AIN1, LOW); digitalWrite(RR_AIN2, HIGH);
  digitalWrite(RL_AIN1, LOW); digitalWrite(RL_AIN2, HIGH);
  digitalWrite(FR_AIN1, LOW); digitalWrite(FR_AIN2, HIGH);
  digitalWrite(FL_AIN1, LOW); digitalWrite(FL_AIN2, HIGH);
  delay(3000);
  
  // Test 4: Individual tests
  Serial.println("Rear Right FORWARD");
  digitalWrite(RR_AIN1, HIGH); digitalWrite(RR_AIN2, LOW);
  delay(1000); digitalWrite(RR_AIN1, LOW); digitalWrite(RR_AIN2, LOW);
  
  Serial.println("Rear Left FORWARD");
  digitalWrite(RL_AIN1, HIGH); digitalWrite(RL_AIN2, LOW);
  delay(1000); digitalWrite(RL_AIN1, LOW); digitalWrite(RL_AIN2, LOW);
  
  Serial.println("Front Right FORWARD");
  digitalWrite(FR_AIN1, HIGH); digitalWrite(FR_AIN2, LOW);
  delay(1000); digitalWrite(FR_AIN1, LOW); digitalWrite(FR_AIN2, LOW);
  
  Serial.println("Front Left FORWARD");
  digitalWrite(FL_AIN1, HIGH); digitalWrite(FL_AIN2, LOW);
  delay(1000); digitalWrite(FL_AIN1, LOW); digitalWrite(FL_AIN2, LOW);
  
  Serial.println("--- Cycle Complete ---");
  delay(2000);
}
