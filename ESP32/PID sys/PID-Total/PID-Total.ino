#define echoPin 11
#define trigPin 10

int maximumRange = 200;
int minimumRange = 0;

void setup() { 
  Serial.begin(115200);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

}

void loop() {
  int olcum = mesafe(maximumRange, minimumRange);
  
  // Mesafeyi ekrana yazdırıyoruz
  Serial.print("Mesafe: ");
  Serial.print(olcum);
  Serial.println(" cm");
  delay(1000);


}

int mesafe(int maxrange, int minrange) {
  long duration, distance;

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = duration / 58.2;
  delay(30);

  if (distance >= maxrange || distance <= minrange) {
    return 0;
  }
  
  return distance;
}
