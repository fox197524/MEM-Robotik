  
// ESP32-S3 N8R2 GPIO pins for HC-SR04
#define trigPin 10   // GPIO 10 - Trigger pin
#define echoPin 11   // GPIO 11 - Echo pin

int maximumRange = 200;
int minimumRange = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  digitalWrite(trigPin, LOW);
}

void loop() {
  int distance = mesafe(maximumRange, minimumRange);

  // Print only the distance in cm
  Serial.print(distance);
  Serial.println(" cm");

  delay(200);
}

// HC-SR04 distance measurement function for ESP32
int mesafe(int maxRange, int minRange) {
  // Send trigger pulse
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Measure echo (timeout 50ms)
  long duration = pulseIn(echoPin, HIGH, 50000);

  if (duration == 0) {
    // No echo received -> return max range
    return maxRange;
  }

  int distance = duration / 58; // cm

  if (distance > maxRange) distance = maxRange;
  if (distance < minRange) distance = minRange;

  return distance;
}

