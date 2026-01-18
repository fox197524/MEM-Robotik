#include<MPU6050.h>
#include<Wire.h>
#include<KY040.h>
#define echoPin 11
#define trigPin 10
#define enclk 4
#define endt 5

MPU6050 mpu;
KY040 encoder(enclk, endt);


unsigned long previousMillis = 0;
const long interval = 500;

int16_t ax, ay, az; //accelerometer x,y,z'si
int16_t gx, gy, gz; //gyroscope x,y,z'si

int maximumRange = 200; //hcsr04 maks menzil cm cinsinden
int minimumRange = 0; //hcsr04 min menzil cm cinsinden

void setup() { 

  Serial.begin(115200);
  while (!Serial) delay(10); 
  // I2C Başlatma 
  Wire.begin(8, 9);
  //Pinmode atamaları
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(enclk, INPUT);
  pinMode(end, INPUT);
//encoder ve mpu başlatma
  encoder.begin();
  if (!encoder.testConnection()){
Serial.println("Encoder nanay baba")
  }
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 sıkıntı kardeşim ya");
  }
  if (echoPin == NULL || trigPin == NULL) {
    Serial.println("HC-SR04 pin atamasında sıkıntı var.");
  }
  Serial.println("Sistem Hazır...");
}

void loop() {
  unsigned long currentMillis = millis(); // Zamanlayıcı için mevcut zamanı alıyoruz
  int olcum = mesafe(maximumRange, minimumRange); // HC-SR04 ile mesafe ölçümü bölümü
  
  // Mesafeyi ekrana yazdırıyoruz
  Serial.print("Mesafe: "); //serial monitörde mesafeyi yazdırıyoz
  Serial.print(olcum); //mesafenin değer değişkeni
  Serial.println(" cm");
  delay(1000);
if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // 1. MPU6050 Okuma
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); // İvmeölçer ve jiroskop verilerini aldık

    // 2. HC-SR04 Okuma
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    long duration = pulseIn(ECHO_PIN, HIGH, 30000); 
    float mesafe = duration / 58.2;

    // 3. Encoder Tur Hesabı (KY-040: 20 Tık = 1 Tur)
    int encoderPos = encoder.getCounter(); 
    float tur = (float)encoderPos / 20.0;

    // --- TEK SATIR FORMATLI ÇIKTI ---
    // Format: MPU / HCSR / Encoder
    
    Serial.print("MPU: [X:");
    Serial.print(ax);
    Serial.print(" Y:");
    Serial.print(ay);
    Serial.print(" Z:");
    Serial.print(az);
    Serial.print("] | ");

    Serial.print("HCSR: ");
    if (mesafe <= 0 || mesafe > 400) Serial.print("---");
    else Serial.print(mesafe, 1);
    Serial.print(" cm | ");

    Serial.print("Encoder: ");
    Serial.print(tur, 2);
    Serial.println(" Tur"); // Satır sonu println
  }

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
