#include <Arduino.h>

// --- MOTOR 4 PIN DEFINITIONS (Pins 7, 8, 9) ---
const int PIN_PWM = 7;   // Speed Control (PWM)
const int PIN_IN1 = 8;   // Direction 1
const int PIN_IN2 = 9;   // Direction 2

// Configuration
const int PWM_MIN_SPEED = 100; // Min speed
const int PWM_MAX_SPEED = 255; // Full speed

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("--- TB6612FNG Speed Test (Pins 7, 8, 9) ---");

  pinMode(PIN_PWM, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
}

void loop() {
  // 1. Set Direction (Forward)
  // Important: One must be HIGH, the other LOW
  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, LOW);

  // 2. Go FULL SPEED (255) for 1 second
  Serial.println("Speed: FULL (255)");
  analogWrite(PIN_PWM, PWM_MAX_SPEED);
  delay(1000);

  // 3. Go MIN SPEED (100) for 1 second
  Serial.println("Speed: MIN (100)");
  analogWrite(PIN_PWM, PWM_MIN_SPEED);
  delay(1000);
}