  
#define echoPin 8
#define trigPin 3


int maximumRange = 200;
int minimumRange = 0;



void setup() { 

  Serial.begin(115200);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

}

void loop() {



  int olcum = mesafe(maximumRange, minimumRange);

if (distance =< 10)
Serial.print("Zenci");

}

int mesafe(int maxrange, int minrange)
{
  long duration, distance;

  digitalWrite(trigPin,LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = duration / 58.2;
  delay(30);

  if(distance >= maxrange || distance <= minrange)
  return 0;
  return distance;
  if(distance <= minrange){

 }
}

