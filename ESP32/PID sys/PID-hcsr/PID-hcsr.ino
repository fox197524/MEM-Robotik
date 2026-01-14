// ESP32-S3 N8R2 - HC-SR04 (Serial Fix)
#define trigPin 10
#define echoPin 11

void setup() {
  // Start Serial communication
  Serial.begin(115200);

  // WAIT for the Serial Monitor to open. 
  // Without this, the S3 might print before the USB connection is fully active.
  // We add a timeout (4000ms) so it doesn't hang forever if you power it via battery later.
  long start = millis();
  while (!Serial && (millis() - start < 4000));

  Serial.println("--- SERIAL CONNECTION ESTABLISHED ---");
  Serial.println("Setup starting...");

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  digitalWrite(trigPin, LOW);
  
  delay(500);
  Serial.println("Setup complete. Loop starting...");
}

void loop() {
  // 1. Trigger Pulse
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // 2. Read Pulse (Timeout 30ms to prevent hanging)
  long duration = pulseIn(echoPin, HIGH, 30000);
  
  // 3. Calc Distance
  int distance = duration * 0.034 / 2;

  // 4. Print
  if (duration == 0) {
    Serial.println("Sensor Timeout (Check wiring or Ground connection)");
  } else {
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
  }

  delay(1000);
}