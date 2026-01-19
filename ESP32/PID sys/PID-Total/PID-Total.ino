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
  mpu.initialize();

  if (!encoder.testConnection() && !mpu.testConnection() && echoPin == NULL && trigPin == NULL) {
Serial.println(""" xXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX$XXXXXXXXXX$XX$XX$XX$XX$X$$X$$X$$X$$XXXXXXX$$$$$$$$$$$XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXxxXxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX$XX$XX$XX$XX$$XX$$X$$X$$X$$XX$XXX$XX$$X$XX$$$$$$$$$$$$$$X$XX$XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXxXXXXXXXXXXXXXXxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX$XXXXXXXXXXX$$$$X$XX$$XX$$XX$$XX$$X$$X$$X$$X$$X$XX$XX$XX$X$X$$$$$$$$$$$XXXXXX$XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXxxXxxXxxXxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
xXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX$XX$XX$XX$$XX$$X$$XX$XX$XX$XX$$X$$X$$$$$$$$$$$$$$$$$$XXXX$$X$$X$XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXxXXXXXXxXXXxxXxxXxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX$XX$$X$$X$$X$$$$X$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$X$$$$$X$$$$X$XX$XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXxXXXXXXXXxXXxXXxXxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
xXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX$$$$$$XX$XX$$X$$X$$$X$$$X$$X$$X$$X$$X$$$$$$$$$$$$$$$$$$XXXXX$$$$$$$X$$XXXXXXXXXXXX$XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXxXXxXXXXXXxxXxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX$$$$$$$$$$$$$$$XXXXX$$$$$$$$$$$$$$XXXXX$$$$$$$$$$$$$$$XXXX$$$$$$$$$$$$$$$$$$$$$$$$$$$X$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$X$$$$X$$$$$$$$$$$$$$$$$$$XXXXXXXXXXXXXX$$$$$$$$$$$$$$$$$$$X$$$$$$$$$$$$$XXXXxxX$$$$$$$$$$$$$XXxXXxxxxxXXxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
xXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX$$$x;;;;;;;;+X$$XXX$$x;;;;;;;;x$$$XXXX$$$x;;;;;;;;;+$$$X$$XXX$$$x;;;;;;;;;;;;;;;;;xX$$$$$$X+;;;;;;;;;;+X$$X$$$x;;;;;;;;;;xX$$X$$$$X$$X;;;;;;;;;;x$$$$$$$$$$$$XXXXXXXX$$x;;;;;;;;+X$$$X$$$x;;;;;;;;x$$$XXXXX$Xx;;;;;;;;;+X$XXxxxxXxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
xXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX$$x;;;;;;;;+$$$X$$x;;;;;;;;x$$XXX$XX$$x;;;;;;;;;;;x$$$XXXXX$$$+;;;;;;;;;;;;;;;;;;;+X$$$$x+;;;;;;;;;;;x$$$$$x;;;;;;;;;;;+X$$$$XXX$$X+;;;;;;;;;;;x$$$$$$$$$$$$$$X$XXXX$$X;;;;;;;;+X$$$$$x;;;;;;;;x$$XXXXXXX$x;;;;;;;;;;;x$$XXXXxxxXxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX$$x;;;;;;;;+X$$$x;;;;;;;;x$$$XXXXX$$$;;;;;;;;;;;;;x$$XX$$X$$$+;;;;;;;;;;;;;;;;;;;;;X$$$x+;;;;;;;;;;;;$$$$$x;;;;;;;;;;;+X$$$$XX$$$x;;;;;;;;;;;;;$$$$X$$$$$$$$$$$$$$XX$$x;;;;;;;;+X$$$x;;;;;;;;xX$XXXxXXX$X+;;;;;;;;;;;;x$XxXXXXxxXXxXXxXxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX$$X+;;;;;;;+x$x;;;;;;;;x$$XXXXXXX$Xx;;;;;;;;;;;;;+X$$X$$$$$$x;;;;;;;+XXXxx;;;;;;;;xX$$x+;;:;;;;;;;;;x$$$X;;;;;;;;;;;;+X$$$$$$$$x;;;;;;;;;;;;;;x$$$$$$$$$$$$$$$$$$$$$$$X+;;;;;;;;X$x;;;;;;;;x$$XXXXXXX$$x;;;;;;;;;;;;;;X$XxXXXxxxXXxxxXxXXxXXxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
xXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX$Xx;;;;;;;;+;;;;;;;;X$$XXXXXXX$$x;;;;;;+x;;;;;;;+$$$$$$$$$+;;;;;;;x$$$$$+;;;;;;;+X$$x+;;;;;;;;;:;;;X$$x;;;;;;;;;:;;+X$$$$$$$$+;;;;;;++;:;;;;;X$$$$X$$$$$XXXXXXXX$$X$$$+;;;;;;;;;;;;;;;;+x$$XXXXXXX$$x;;;;;;;x;;;;;;;x$$XXxxxxXXxXXxxxxxxxxXXxXxxXxxxxxxxxxxxxxxxxxxxxxxxx
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX$$Xx;;;;;;;;;;;;;;+$$$XXXXXXXX$X+;;;;;;xXx;;;;;;;X$$$$$$$$+;;;;;;;x$$$$X;;;;;;;;+X$$x+;;;;;;;;;;;;;x$$;;;;;;+;;;;;;+X$$$$$$$x;;;;;;+$x+;;;;;;xX$$$$$$$$$$$$$$$X$$$XX$$$+;;;;;;;;;;;;;;+X$$XXXXXXXX$X;;;;;;;x$x;;;;;;;x$XXXXXxxXxxXXxXXXXXXXXXXXXXXXXXXXXXX$XXXX$$$$$$$$$$
xXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX$$$x;;;;;;;;;;;;+$$$XXXXXXX$$$+;;;;;;;X$$;;;;;;;;$$$$$$$$+;;;;;;;;+++;;;;;;;;;;x$$$x+;;;;;;++;;:;;+xx;;;;;++;;;;;;+X$$$$$$X;;;;;;;x$$x;;;;;;;x$$$$$$$$$$$$$$XX$$$$$X$$$x;;;;;;;;;;;;+X$$XXXXXXXX$$x;;;;:;+X$X+;;;;;;+X$XXXXXXXXXXXXXXXXX$X$$$$$$$$$$$$$$$$$$$X$$$X$$$$$$
xXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX$$x;;;;;:;;;;x$$XXXXXXXXX$$x;;;;;;;x$$$+;;:;;;;x$$$$$$$+;:;;;;;;;;;;;;;;;;:;x$$$$x+;;;:;;xx;;;;;;;;;;;;x+;;;;;::+X$$$$$Xx;;;;;;+X$$X;;;:;;;;X$$$$$$$$$XX$$XX$$XX$XX$$$X;;;;;;;;;;x$$$XXXXXXXXX$x;;;:;;;x$$$x;;:;;;;+$$XXXXXX$$$$$$$$$$$$$$$$$$X$$X$$$$$$$$$$$X$XX$XXXX
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX$$x;;;;;;:;x$$XXXXXXXXX$$$;;;;;;;;X$$$x;;;;;;;;x$$$$$$+;;;;;;;;;;;;;;;;;;+X$$$$$x+;;;:;;xX;;;;:;;;;;;;Xx;;;;;;;+X$$$$$x;;;;;;;xX$$Xx;;;;;;;+$$$$$$$X$$$XX$XX$$XX$$X$$$x;;;;;;;;x$$$XXXXXXXX$$$+;;;;;;;xX$$x;;;;;;;;x$$$$$$$$$$$$$$$$$$$$$$$$$$$$XXX$$X$$XX$XXX$XXX$$X
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX$X;;;;;;;;X$$XXXXXXXX$$Xx;;;;;;;;;;;;;;;;;;;;;+X$$$$$+;;;;;;;+xxxxxxxxXX$$$$$$$x+;;;;;;xXx;;;;;;;;;;+$x;;;;;;;+X$$$$X;;;;;;;;;;;;;;;;;;;;;;x$$$$$$$$X$$X$$XXX$XX$XX$$$+;;;;;;;x$$XXXXXXXXX$$+;;;;;;;;;;;;;;;;;;;;;;$$$$$$$$$$$$$$$$$$$$$$$$$$$$$XXXXXXXX$$$$X$XXXXXX
xXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX$$X;;;;;;;;X$$XXXXXXXX$$x;;;;;;;;;;;;;;;;;;;;;;;x$$$$$+;;;:;;;x$$$$$$$$$$$$$$$$$x+;;;;;;x$$+;;;;;;;:;x$x;;;;;;;+X$$$$+;;;;;;;;;;;;;;;;;;;;;;;X$$$$X$$XX$XXX$$X$XX$XX$$X;;;::;;;x$$XXXXXXXX$$X;;;;;;;;;;;;;;;;;;;;;;;x$$X$$XX$XX$XX$X$XXXXXXXXXXX$$$$$$$Xxxxx++++;;;;;
xXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX$X;;;;;;;;X$$XXXXXXX$$X+;;;;;;;;;;;;;;;;;;;;;;;;X$$$$+;;;;;;;x$$$$$$$$$$$$$$$$$x+::;;;;x$$x;;;;;;;;+$$x;;;;;:;+X$$$x;;;;;;;;;;;;;;;;;;;;;;;;x$$$XXX$$XX$$X$$XX$XXXX$$$+;;;;;;;x$$XXXXXXXX$X;;;;;;;;;;;;;;;;;;;;;;;;;x$$$$$$XX$$$$$$$$$$XXXXxxxx++;;;+;;;;;;;;;;;;;;:
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX$X;;;;;;;;x$$XXXXXX$$$+;;;;;;;xXXXXXXXXX;;;;;;;;+X$$$+;;;;;;;x$$$$$$$$$$$$$$$$$X+;;;;;;x$$$+;;;;;;;x$$x;;;;;;;+X$$X;;;;;;;;xXXXXXXXXx;;;;;;;;x$$$XX$$XX$$XXX$XXXXXX$$$;;;;;;;;x$$XXXXXX$$$x;;;;;;;+XX$XXXXXX+;;;;;;;+X$$$$X$$$$$XXxxx+++;;;;;;;;;;;;;;;;;;;;;;;;;;;:
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX$X;;;;;;;;X$$XXXXXX$$X;;;;;;;;X$$$$$$$$$X;;;;;;;;x$$$+;;;;;;;x$$$$$$$$$$$$$$$$$x+;;;;;;x$$$x;;;;;;xX$$x;;;;;;;+X$$x;;;;;;;+$$$$$X$$$$x;;;;;;;+$$$$$X$$$XX$$XXXXXXXX$$$+;;;;;;;x$$XX$$$$$$x;;;;;;;;x$$$$$$$$$x+;;;;;;;+$$$XXXx+;;;;;;;;;;;;;;;;;;;;;;+;;;;;;;;;;;;;;;
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX$$XXXXXXXX$$$XXXXXX$$$XXXXXXX$$$$$$$$$$$$XXXXXXXXX$$$$XXXXXXX$$$$$$$$$$$$$$$$$$$XXXXXXX$$$$$XXXXXX$$$$$XXXXXXX$$$$XXXXXXXX$$$XXX$$$$$$XXXXXXXX$$$$XXX$XXXXX$XXXXXXX$$$XXXXXxXX$$$$$$$$$$$$XXXXXXXX$$$$$$$$$$$XXXXXxxxx$x+;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;:
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX$$$XXX$$XXXXXXXXXX$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$XXXXX$$$$XXXXXXXXXXXX$$$X$$$$$$$$$$$$$$$$$$$$$$$$$$$$$X$$$$$XXxx++;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;:
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$X$$XX$$XXXXXXXXXXXXXXXXXXXXXXX$$$$$$$$$$$$$$$$$$$$$$$$$$XXX$Xxx+;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;+;;;;;;;;;;;;;;
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$XX$$XXX$$XXXXXXXXXXXXXX$$$$$$$$$$$$$$$$$$$$$$$$$$$$$XXxxx+;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;+;;;;;;;;;;;;;;
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$XXXXXXXXXXXXXXXXXX$$$$$$$$$$$$$$$$$$$$$$$$XX$$$Xxx++;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;+;;;;;;;;;;;;;;
XX$$XX$$XX$XXXXXXXXXXX$XX$XX$XX$XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$XXXXXXXXXXXXXXXXX$$$$$$$$$$$$$$$$$$$$$$$$XXXXx+;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
XXXXXXX$XX$$$$$$$$$$$XXXXXXXXXXXX$XX$XX$XX$XXXXXXXXXXXXXXXXXXXXXX$XX$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$XXXXXXXXXX$$$$$$$$$$$$$$$$$$$$$X$XXxxx+;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
X$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$XX$XX$XX$X$$$$$$$$$$$$$$$$$$$$$$$$XX$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$XXXXX$$$$$$$$$$$$$$$$$$$$$$X$$Xxx;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
XXXXXXXXXXX$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$XXXXX$$$$$$$$$$$$$$$$$$$$$X$XXx;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;+;;;;;;;;;;;;+;;;;;;;++;;;;;;;;;;;;
xXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$X$$$$$$$$$$$$$$$$$$$$$$XXXXXXx;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;+;;;;;;;;;;;++;;;;;;+x;;;;;;;;;;;;
xXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$XXXXXx;;;;:;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;+;;;;;;;;;;;;;;;;;+;;;;;+x;;;;;;;;;;;;
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$XXXXXXXXXx;;;:;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;+x;;;;;;;;;;;;
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$XXXX$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$XXXXXXXXXX+;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;+x;;;;+;;;;;;
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$XXXXXXXXXXXXXXXXXX$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$X$$$$$$$$$$$$$$$$$$$$$$XXXXXXXXXXXx+;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;+;;;;;;;;;;;;;;;;;;;;;+x;;;;+;;;;;;
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$XXXXXXXXXXXxxxxxxxxxxXXXXXX$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$XX$$$$$$$$$$$$$$$$$$$$$XXXXXXXXXXXXx+;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;+x+;;++;;;;;;
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX$$$$$$$$$$$$$$$$$$$$$$$$$$$$XXXXxxxxxxxxxxxxxxxxxxxxxxxXXXXXX$$$$$$$$$$X$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$X$$$$$$$$$$$$$$$$$$$$$$XXXXXXXXXXXXXxxx+;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;:;;;;;;;;+;;;;;;;;;;++;;;;;;;;+;++xx+;;;;;;
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX$$$$$$$$$$$$$$$$$$$$$$$$$$XXXXxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$X$$$$$$$$$$$$$$$$$$$$$$$XXXXXXXXXXXXxxxXx+;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;+++;;;;;;;;;;x+++;;;+;+++x;;;;;;:
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX$$$$$$$$$$$$$$$$$$$$$$$$$XXXXxxxxxxxxxxxxxxxxxxxxxxxx+++xxxxxxxxXXXxxxxxxXXXXxXxxxxxxxxXXXXXXXXXXX$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$XXXXXXXXXxxxxxxXx;;;;;;::;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;++;;;;;;;;;;;;;+++;+x;;;++;;;;;;;
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX$$$$$$$$$$$$$$$$$$$$$$$XXXXxxxxxxxxxxxxxxxxxx+++++++++++++++xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxXXXXX$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$XXXXXXXXXxxxxxxxXxx;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;++;;;;;;;;;;;;;+x+++++xx+;;;;;;;
XX$$XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX$$$$$$$$$$$$$$$$$$$$XXXXXxxxxxxxxxxxxxxxxx++++++++++++++++++++++++xxx++++++++xx+x+xxxxxxxxxxxxxXXXXX$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$XXXXXXXxxxxxxxxxxXX;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;x;;;;;;;;;;;;;;x;+x+;+x+;;;;;;:
XX$$XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX$$$$$$$$$$$$$$$$$$$$XXXXxxxxxxxxxxxxxx++++++++++++++++++++++;;;+++;;+++;;++++++++++++++xxxxxxxxxxXXXX$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$XXXXXXXXxxxxxxxxxxXXx;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;+x+;;;;;;;;;;;;+;;;xxxx;;;;;;;;
XX$XXX$$XX$XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX$$$$$$$$$$$$$$$$$$$XXXXXxxxxxxxxxxxx++x++++++++++++++;;;;;;;;;;;;;;;;;;;;;;;;;;;;+++++++++xxxxxxxxXXXX$$$$$$$$$X$$$$$$$$$$$$$$$$$$X$$$$$$$$$$$$$$$$$$$$$$$XXXXXxxxxxxxxxxxxxxXx+;;;;;;;;;;;;;;;;;;:;;;;;;;;;xx;;;;;;;;++;;;;;;;;;;;++;;;xxxxx;;;;;;;
XX$$XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX$$$$$$$$$$$$$$$$$$$XXXXXxxxxxxxxxx++x++++++++++++++;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;++++++++xxxxxxxxxXXXX$$$$$$$$$$$$$$$$$$$$$$$$XXXXX$$$$$$$$$$$$$$$$$$$$$$XXXXXxxxxxxxxxxxxxxX$x;;;;;::;;;;;;;;;;;;:;;;;;;;++x+;;;;;;;++;;;;;;:;;;;++;;++xx++++;;;;
XXX$XXX$XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX$$$$$$$$$$$$$$$$$$$XXXXxxxxxxxxxxx++++++++++++++++++++;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;++++++++xxxxxxxxXXX$$$$$$$$$$$$$$$$$$$$$$$XXXXXXX$$$$$$$$$$$$$$$$$$$$$$XXxxxxxxxxxxxxxxxxxxXXx;;;;;;;;;;;;;;;;;;;;;;;;;+;;+xx+;;;;;++;;;;;;;;;;++;;;xxx;;+x;;;;
XXX$XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX$$$$$$$$$$$$$$$$$$XXXXxxxxxxxxxx++++++++++++++;;+++;;;;;;;;+++++;;;;;;;;;;;;;;;;;;;;;+++++++xxxxxxxxxXXX$$$$$$$$$$$$$$$$$$$$$$$XXXXXXX$$$$$$$$$$$$$$$$$$$$$XXXxxxxxxxxxxxxxxxxxxXXxx+;;;;;;;;;;;;;;;;;;;;;;;+;+xxxx+;;;;+;;;;;;;;;;++;;;xx+;;;;x;;;
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX$$$$$$$$$$$$$$$$$$XXXxxxxxxxxxxx++++++++++++;;;;;;;;;;;;;;;;;;;;;;;+;;;;;;;;;;;;;;++++++++++++xxxxxxxxXXX$$$$$$$$$$$$$$$$$$$$$$XXXXXXXX$$$$$$$$$$$$$$$$$$$$$$XXxxxxxxxxxxxxxxxxxxxXXxx+xxXxx+;;;;;;;;;;;;;;;;;;+++xx;;;;;+;;;;;;;;;;+;;;x+;;;;;;++;
XX$XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX$$$$$$$$$$$$$$$$$$XXXXXxxxxxxxxx+++++++++++++;;;;;+;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;++++++++++++xxxxxxxxXXXX$$$$$$$$$$$$$$$$$$$$$$XXXXXXXX$$$$$$$$$$$$$$$$$$$$$$XXxxxxxxxxxxxxxxxxxxxXXXxxx+++;;;;;;;;;;;;;;;;+;;+;;;+++;;;;++;;;;;;;;++;;xx;;;;;;++;
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX$$$$$$$$$$$$$$$$$$$XXXXxxxxxxxxx+x+++++++++;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;++++++++++xxxxxxxxXXXX$$$$$$$$$$$$$$$$$$$$XXXXX$XXXXX$$$$$$$$$$$$$$$$$$$$$$$XXxxxxxxxxxxxxxxxxxxxXXx+;;;;;;;;;;;;;;;;;;;++;;+xxxxxx++;++;;;;;;;;+xxxxxx++;;;;;x
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX$$$$$$$$$$$$$$$$$$XXXxxxxxxxxx+++++++++++++;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;++++++++++++xxxxxxxXXX$$$$$$$$$$$$$$$$$$$$$XXXXXXXXXXX$$$$$$$$$$$$$$$$$$$$$$$$XxxxxxxxxxxxxxxxxxxxxXx+;;;;;;;;;;;;;;;;+++x++++xxxxxx;+x+x;;;;;;;+xxxxxXxxx++;+x
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX$$$$$$$$$$$$$$$$XXXxxxxxxxxxx++++++++++++;+;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;++++++++++xxxxxxxXXXX$$$$$$$$$$$$$$$$$$$XXXXXXXXXXXXXX$$$$$$$$$$$$$$$$$$$$$$$XxxxxxxxxxxxxxxxxxxxxX$x+;;;;;;;;;;;;;;;+x+xxxxxxx+++xxxxx+;;;;;;+xxxxxxxxxxxxxx
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXxxXXXXXXXXXXXXXXXX$$$$$$$$$$$$$XXXXxxxxxxxxxx+++++++++++++;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;++++++++++xxxxxxxXXXX$$$$$$$$$$$$$$$$$$XXXXXXXXXXXXXXXX$$$$$$$$$$$$$$$$$$$$$$$$XxxxxxxxxxxxxxxxxxxxxXxx++xx+;;;;;;;;;+xxxxxxxxxxxxxxxxxx+;;;;;;xXxxxxxxxxxxXx
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXxXXxXXxXXXxxXXXXXXXXXXXXXXXX$$$$$$$$$$$$XXXxxxxxxxxxx+++++++++++++;++;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;++++++++++xxxxxxXXXX$$$$$$$$$$$$$$$$$$XXXXXXXXXXXXXXXxXX$$$$$$$$$$$$$$$$$$$$$$$XXxxxxxxxxxxxxxxxxxxxXXx+xXXxx;;;;;;+xxxxx++++xxXxxxx+xxx+;;;+xXXxxxxxxxxxXXx
xXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXxxxxxxXXXXXXxxxxxxxxxxxxxXXXXXXXXXXXXXXX$$$$$$$$$$$XXXxxxxxxxxxxx++++++++++++;++;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;++++++++++xxxxxxXXX$$$$$$$$$$$$$$$$$XXXXXXXXXXXXXXXXXxxX$$$$$$$$$$$$$$$$$$$$$$$$$XxxxxxxxxxxxxxxxxxxxxXXxxx+;;;;;;+xxxxx++xxXXXxxxXXXxxxxx++xxxXXxxxxxxxXXxx
xXXXXXXXXXXXXXXXXXXXXxxxxxxxxxxxxXXXXxxxxxxxXXxXXxXXxxXXXXXXXXXXXXXXXXXxX$$$$$$$$$$Xxxxxxxxxxxxxxx+++++++++;;++;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;+++++++++++xxxxxxXXX$$$$$$$$$$$$$$$$$XXXXXXXXXXXXXXXXXXxxxX$$$$$$$$$$$$$$$$$$$$$$$$XXxxxxxxxxxxxxxxxxxxxXXx+;;;;;;;+xxxxxxx++xXXxxxxx+;xXxxxxxxxxXXxxxxxxxxxx
xXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXxxxX$$$$$$$$XXxxxxxxxxxxxxxx++++++++++;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;++++++++xxxxxxxXXX$$$$$$$$$$$$$XXXXXXXXXXXXXXXXXXXXXXXXxxX$$$$$$$$$$$$$$$$$$$$$$$$$XxxxxxxxxxxxxxxxxxxxXXX+;;;;;;xxxxxxxxxxxxxxxxxxxxxxXxxxxxxxxXxxxxxxxxxx
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXxxXxxX$$$$$$$Xxxxxxxxxxxxxxxxxxxxxx++++;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;+++++++++xxxxxxXXX$$$$$$$$$$$XXxxxXXXXXXXXXXXXXXXXXXXXXXxxxX$$$$$$$$$$$$$$$$$$$$$$$$XXxxxxxxxxxxxxxxxxxxxXXx++xxxXXxxxxxxxXxxxxxxxxXXXXXxxxxxXXxXXXxxxxXXxX
xXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXxxXxxXXxxxXxxxX$$$$$XXxxxxxxxxxXXXXXXXXXXXxxxxxxx++;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;++++++++xxxxxXXX$$$$$$$$$$XxxX$XXXXXXXXXXXXXXXXXXXXXXXXXxxX$$$$$$$$$$$$$$$$$$$$$$$$$XxxxxxxxxxxxxxxxxxxxxXXxxxxxxxXXxxxxxx+xxxxxXxxXXXXxxxXXXXXXXxxxxXXxX
xxXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXxXXXXXXXXXXXXXXXXXXXXXxxxxxxxxxxXXXxxxXxxxxX$$$$XXxxxxxxxXXXX$XXXXXXXXXXXXXXxxxxxxx++;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;+++++++++++xxxxXXX$$$$$$$$$$x;;;xxxxxxXXXXXXXXXXXXXXXXXXXXXxxxX$$$$$$$$$$$$$$$$$$$$$$$$XXxxxxxxxxxxxxxxxxxxxX$x+++xXxXxxxXxxxx++xxXXXXXXXXxxxXXXXXXxxxxxxxx
xxXXXxXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXxxxxxxxxxxxxxXxxxxxxxxX$$$$XxxxxxxxxxxxxxxxxxXXXXXXXXXXXXXXXXxxxxx++++;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;+++++++xxxxxX$X$$$X$$$$x;;;;;;;;;;xXXXXXXXXXXXXXXXXXXXXXxxxx$$$$$$$$$$$$$$$$$$$$$$$$XXxxxxxxxxxxxxxxxxxxxXXx++xxxXXXXXXXXxxxxXXXXXXXXXXXXXXXXxXXxxxxxxX
xXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXxXXxXxxXxXXXXXXXXXXXxxxxxxxxxxxxxXxxxxxxxxX$$$XXxxxxxxxxxxxxxxxxxxxxxxxxXX$$X$XXXXXXXxxxx+++;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;+++++xxxxxX$$$$$$$$$x;;;;;;;;;;;;xXXXXXXXXXXXXXXXXXXXXXxxxxX$$$$$$$$$$$$$$$$$$$$$$$$XxxxxxxxxXxxxxxxxxxxxXXxxxXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXxxxxxx
xxXXXXxXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXxxxxxxxxxxxxxxxxxxxxxxxX$$$Xxxxxxxxxxxxxxxxxxxxxxx+++xxxxxxXXXXXXXXXxxx++++++;;;;;;;;;;;;;;;;;;;;;;;;;;+++++++++++++++xxxxX$$$$$X$Xx;;;;;;;;;;;;;+xXXXXXXXXXXXXXXXXXXXXXXxxxX$$$$$$$$$$$$$$$$$$$$$$$$XXxxxxxxxxxxxxXxxxxxxX$xxXXXXXxXXXXXXXXXXXXXXXXXXX$$XXXXXXxxXXXx
xxXXXxxXXXxXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXxXXXxxxxxxxxxxxxxxxxxxxxxxxX$$XXxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxXxxxxxx+++++;+;;;;;;;;;+;;;;+++xxxxxxxxxxxxxxxxxxxxxxxxxxX$$$$$$xx;;;;;;;;;;;;;;;+xXXXXXXXXXXXXXXXXXXXXXXxxxxX$$$$$$$$$$$$$$$$$$$$$$$XXxxxxxxxXxxxxxxxxxxxXXXxxXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
xXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXxxxXXXxxxxxxxxxxxxxxxxxxxxxxxxxxxX$$XxxxxxxxxxxxxxxxXXX$$$XXX$$$$$XXX$$XXxxxxxxxxxxxx+++;;;;++++++++++xxxXXXXXXXXXXXXXXXXXXXXXxxxxxxX$$$$Xx;;;;;;;+;;+;;;;;;;;xXXXXXXXXXXXXXXXXXxXXXXXxxxX$$$$$$$$$$$$$$$$$$$$$$$$XxxxxxxxxxxxxxxxxxxxxX$xxxXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX$XX
xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxXXXXxxxxxxx++xxxxxxxxXXXXXxxXXXXXXXX$XXXXXXxxxxxxxxx+++;;;+;++++++xxxxxXXXXXXXXXXXXXXXXXXXXXXXXXxxxX$$$$x;;;;;;;+;++x++++;++;+xXXXXXXXXXXXXXXXXXXXXXXXxxxxX$$$$$$$$$$$$$$$$$$$$$$$XXxxxxxxxxxxxxxxxxxxxX$xxxXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX$$$
xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxX$$xxxxxxx++++++xxxxxxxxxxxxxxxxxxXXxxxxxXXXxxxxxxxxx++;;;;++++xxxxxxxxxxxxXxxXXXXxxxxxxxxxxxXXXxxxX$$Xx+x+;;;;;;;++++++x+;+;+xxXXXXXXXXXXXXXXXXXXXXXXxxxxxX$$$$$$$$$$$$$$$$$$$$$$$$XxxxxxxxxxxxxxXXxxxxXXXxxXXXXXXXXXXXXXXXXXXXXXX$XXXXXXX$$$
xxxxxxxxxxxxxxxxxxxxx++++++++++++++++;++++;;;;;;+++xxxxxxxxxxxxxxxxxxxxxxxxxX$Xxxxxxx++++++++xxxxxxxxx+++++++x+++++xxxxxxxxxxxxx+;;;;;;;;+xxxxxxxxxxxxXXxxxx+++xxxxxxxxxxxXxxxxX$$x++xx+;;;+++++x+++++++;+xxXXXXXXXXXXXXXXXXXXXXXXXXxxxxx$$$$$$$$$$$$$$$$$$$$$$$$XxxxxxxxxxxxxxxxXXxxxX$xxXXXXXXXXXXXXXXXXXXXXXXXXXXXXX$$$
;;;+++++++++++++++++++;;;;;;;;;;++;;;;;;;;;;;+++++++xxxxxxxxxxxxxxxxxxxxxxxXXXXxxxxxx++++++++++++xxxxxx+++++++++++xx++++++++++xx++;;;;;;;+xxxxxXXXXXX$$XXXXXXXxxxxxxxxxxxxxxxxX$XXxxxxxx++++x+;;++++;+++x++xxXXXXXXXXXXXXXXXXXXXXXXXxxxxxx$$$$$$$$$$$$$$$$$$$$$$$$XXxxxxxxxxxxxxxxxxxxxXXXxXXXXXXXXXXXXXXXXXXXXXXXXXXX$$$$
;;;++++xx+++++++++++++++;;;;;;;;;;;;;;;+++;;;+;++++xxxxxxxxxxxxxxxxxxxxxxxxXXXXxxxxx+++++++++++++++++xxxxxx+++xxxxxxxx++++++++x++;;;;;;;++xx+xxxxxxxxXXXXX$$XXX$XXXXXxxxxxxxxxX$Xxx+xXxx++++x+++xxx+;;;;xxxxxxXXXXXXXXXXXXXXXXXXXXXXXXxxxxxX$$$$$$$$$$$$$$$$$$$$$$$XXxxxxxxxxxxxXxxxxxxxXXXxXXXXXXXXXXXXXXXXXXXXXXXX$$$$XX
;;;++++++xxxx++++++++++;;;;;;;;;;;;;;;;;;+;;;;;;;++xxxxxxxxxxxxxxxxxxxxxxxxxXXxxxxxx+++++;;;;;;;;+++++++++++++++++++++++;;+++++++;;;;;;;+++++++++++++xxxxXXXxxxxXX$$$XXxxxxxxx$$xx++xXxx+xxxxxx+xxxxx;;;;++xxxxXXXXXXXXXXXXXXXXXXXXXXXxXxxxxx$$$$$$$$$$$$$$$$$$$$$$$$XxxxxxXxxxxxxXxxxxxxxX$xXXXXXXXXXXXXXXXXXXXXXXX$$$$$X
;;;+++++++++++;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;+xxxxxxxxxxxxxxxxxxxxxxxxxXXxxxxxx+++++;;;;;;;;;;;;;;;;;;;;+;;;;;;;;;;+;;+++++;;;;;;;;;++++++++++++++++++xxxxxxxxxxxxxxxx+xxXXxxxxxXXXxxxxxxxxxxxxxx+;+++xxxxxxXXXXXXXXXXXXXXXXXXXxXXxxxxxxx$$$$$$$$$$$$$$$$$$$$$$$XXxxxxxxxxxxxxxxxxxXxx$$XXXXXXXXXXXXX$$$XXXXXXXX$$$XX
;;;;+++++++++++;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;+xxxxxxxxxxxxxxxxxxxxxxxxxXXxxxxxx+++++;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;++++++;;;;;;;;;;;;;;;+++xxxxxx++++++++xxxxxxx+++++xxXXxx$XxxXXXxxxxxxxxxxxxxxxxxxxxxxxxXXXXXXXXXXXXXXXXXXXXxxxxxxxxxxX$$$$$$$$$$$$$$$$$$$$$$$XxxxxxxxxxxxXxxXXxxxxXX$XXXXXXXXXX$X$$$XXXXXXXXX$XX
;;;;+++x+++++++++++;;;;;+;;;++;;;;;;;;;;;;;;;;;;;;+xxxxxxxxxxxxxxxxxxxxxxxxxXxxxxxxx+++++;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;+++++++;;;;;;;;;;+;;;;;;;;+++xxxxxxxxxxxxxxxx++++++xxXxxX$$$XXXXxxxxxxxxxxxxxxxxxxxxxxxxxxXXXXXXXXXXXXXXXXXXXXxxxxxxxxxX$$$$$$$$$$$$$$$$$$$$$$$XxxxxxxxxxxxxxxxxxxxxXX$XX$XXXXXX$X$$$X$XXXXXXXXXX
;;;;++++++++++++++xx++++++++xxx+;;;;;+;;;;;;;;;;;;xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx++++++;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;+++++++;;;;;;;;;;;;;;;;;;;;;;+++++xxxxxxxxx++++++++xXXxx$$$$$XXXXxxxxxxxxxxxxxxxxxxxxxxxxxXXXXXXXXXXXXXXXXXXXXXxxxxxxxxxx$$$$$$$$$$$$$$$$$$$$$$$XXxXxxxxxxxxXxxXXxxxxxX$XXXXXXXX$X$$$$XXXXXXXXXXX
;;;;;++++++++;;+;+x+;;+;;;;;;+x+;;++;;;;;;;;;;++;+xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx+++++;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;+++++++;;;;;;;;;;;;;;;;;;;;;;;;;;;+++++++++;;++++++xXxxx$$$$$$XXXXXxxXXxxxxxxxxxxxxxxxxxxxxXXXXXXXXXXXXXXXXXXXXXxxxxxxxx+x$$$$$$$$$$$$$$$$$$$$$$$XXxxxxxxxXxxXxxxXxxxxxXX$XXXXXX$$$$$$XXXXXXXXXXX
;;;;;+++++++;;++;+x+;;++;;;;;+xx+;++;;;;;;;;;;;;;+xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx+++++;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;+++++++;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;+++xxXxxx$$$$X$$XXXXXXxxxxxxxxxxxxxxxxxxxxxxxXXXXXXXXXXXXXXXXXXXXXxxxxxxxx+xX$$$$$$$$$$$$$$$$$$$$$$$xXxxxxxXxxXxxxXXxxxxxX$$$XXXX$$$$$$XXXX$XXXXXx
;;;;++++++++++++++x+;;;++;;;;+x++;;;;++++;;;;+++++xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx++++;;;;;;;;;;;;;;;;;;;+;;;;;;;;;;+++++;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;+++xxXxxxX$$$$$$$XXXXXXxxxxxXXXXXXXxxxxxxxxxxxXXXXXXXXXXXXXXXXXXXXxxxxxxxxx+xX$$$$$$$$$$$$$$$$$$$$$$$XxxXxxxxXxxXxxXxxxXxxx$$XXXX$$$$$$XX$X$XXXXXx
;;;;+++++++x++++++x+;;;;;;;;;+x++;;;;+++;;;;;++++xxxxxxxxxxxxxxxxxxxxxxxxxxxXXxxxxxxxxx++++++;;;;;;;;;;;;+++++++;;;++;;;++++;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;+++xxxxxXXXX$$$$$$$XXXXXxxxxxXXXxXXxxxxxxxxxxxxXXXXXXXXXXXXXXXXXXXXxxxxxxxxx+xX$$$$$$$$$$$$$$$$$$$$$$$XXxXXxxxXXXXxxxxxxXXxXXX$XXX$$XX$$$X$XXXXXXx
;;;;;+++xx++++++++++;;;;;;;;;+x+++;;++++;;;;;;+++xxxxxxxxxxxxxxxxxxxxxxxxxxxXXxxxxxxxxxx+++++++++;;;;++++++++;;+++++;;;;++++;;;;;;;;;;;;;;;;;;;++;;;;;;;;;;;;;;;;;;;;;;;;+xxx+xxXXXXX$$$X$$$XxxxxxxxXXXxxxxXxxxxxxxxxxxxXXXXXXXXXXXXXXXXXXXXxxxxxxxxx+xX$$$$$$$$$$$$$$$$$$$$$$$XxXXxxxXXxXXxXXxxxxxxxX$$xxxxxxxxxxXXX$$XXx
;;;;;;++xx++++++++xxxx+xxxxxxxx++++;+++++++;;++++xxxxxxxxxxxxxxxxxxxxxxxxxxxXXxxxxxxxxxxxx++++++++++++++++;;;;;;+++;;;;;++;;;;;;;;;;;;;;;;;;;;+++++++;;;;;;;;;;;;;;;;;++++xxx++xxXXXXX$$$$$$$xxxXXXXXXXXxXXxxxxxxxxxxxxxxXXXXXXXXXXX$$XXXXXXXxxxxxxxxx+xx$$$$$$$$$$$$$$$$$$$$$$$XXxXXXxxxxxXXxXXxxxXxxX$$XxxxxxxxxXXXXXXXx
;;;;;+++xxx++++++++++++++++++++++++;++++++++;++++xxxxxxxxxxxxxxxxxxxxxxxxxxxXXxxxx+++xxxxx++++++++++++++;;;;;;+++++;;;;++;;;;;;;;;;;;;;;;;;;;;;++++x+++;;;;;;;;;;;;;;++++xxx+;xxxxxxX$$$$$$$$$XxXxXXxxxxxXXxxxxxxxxxxxxxxxXXXXXXXXXXXX$$XXXXXXxxxxxxxxx++x$$$$$$$$$$$$$$$$$$$$$$$XXxxxxxXxxXxxXXxxXXXxxX$$XxxxxxxxXXXX$XXx
;;;;+++xxxx+++++++;;;;;;++;++++++;;;+++++++;;++++xxxxxxxxxxxxxxxxxxxxxxxxxxxXxxxxx++++xxxx++++++++++++;;;;;;;+++xxx+;;+++;;;;;;;;;;;;;;;;;;;;;;;;+++xx+++;;;;;;;;;+++++++xx+;+xxxxXX$X$$$$$$$$$XxXXXxxxxXxxxxxxxxxxxxxxxxxxxXXXXXXXxXX$$$XXXXXXxxxxxxxxx++xX$$$$$$$$$$$$$$$$$$$$$$XXxxXXXXxxxxxXXXXXXXxxxX$XxxxxxxXXXX$XXx
+;;;+++xxxx++++++++++++;++;xx++;;+;;;++++++;;++++xxxxxxxxxxxxxxxxxxxxxxxxxxxXXxxxxx+++xxx+++++++++++;;;;;;;;++++xxxxxxx+++;;;;;;;;;;;;;;;;;;;;;;;;;;+xx+++++++++++++++++xxxxxxxxxxXXX$$$$$$$$$$$XXxxxxxxxxxxxxxxxx++xx++++xxXXXXXxxxxXX$$$XXXXXXxxxxxxxxx++xX$$$$$$$$$$$$$$$$$$$$$$XXXxxxXXXXXxxXXxxxxxXXXX$Xxxx+xX$XXXXXx
+;;;+++xxxx++++++;+;;;+;++;xxx+;;;+;;+++++++++++xxxxxxxxxxxxxxxxxxxxxxxxxxxXXxxxxxx+xxxxx++++++++;;+++++++++++++xxxXXXxxx++;;;;;;;;;;;;;;;;;;++;;;;;;++xxx++++++++++xxx+xxxxxxxxxxxxX$$XX$$$$$$$$Xxxxx++xxxxxxxxxx++xx++xx++xXXXXXXXXXXX$$$XXXXXXxxxxxxxxxx+xX$$$$$$$$$$$$$$$$$$$$$$XXXXXxxXXXXXxXXXXXXxXXxX$$x++xXXX$XXXx
+;;;++++xxx+++++++++++++++;xx+++++;;+++++;++++++xxxxxxxxxxxxxxxxxxxxxxxxXX$$Xxxxxxx++xxxx+++++++++xxxxxxxxxxxxxxxxxXXXXXxxx+;;;;;;;;;;;;;+xxxx++;;++;;+xxx+++++++++xxx+xxxxxxxxxxxxX$XXX$$$$$$$$$$xxxxx+xxxxxxxxxx++++++++++xxXXXXXXXX$$$$$$XXXXXxxxxxxxxxx++xX$$$$$$$$$$$$$$$$$$$$$$XXxxxXxxXXXxXXxXXXXxXXxX$$xxx$XXXXXXx
+;;;+++xxxx+++++++++++x++++xxx+++++++xxxx+++++xxxxxxxxxxxxxxxxxxxxxxxxxX$$$$XXxxxxxxxxxxxx+++++xxxxxxxxxxxxxxxxxxxXXXXXX$Xxxx+++++++++xxxXXXxx+++;;;;;;;+xxx+++++xxx+++xxxxxxxxxxxXXX$$$XX$$$$$$$$$Xxxx+++++xxxxx++++++++++++xXXXXXX$$$$$$$$$XXXXXxxxxxxxxxxx+xX$$$$$$$$$$$$$$$$$$$$$$$XxxXXxxXXXxXXxxXXXxXXxX$$Xx$XXXXXXx
x+;;+xxxxxx++xx+xxxxx++++++xxx+++++++xxxxxx++++xxxxxxxxxxxxxxxxxxxxxxxX$$$$$XXXxxxxxxxxxxx++++xxxXXXXXXXxXXXXXxxxxXXXX$$$$$XXxxxxxxxxXX$$Xxxxxxx++;;;;;;+++xx+++++++++xxxX$xxxXXXXXXXxXXX$XXX$$$$$$$xx+x++++xxxxxx+++++++++xxxxXXX$$$$$$$$$$XXXXXXXxxxxxxxxxxx++x$$$$$$$$$$$$$$$$$$$$$$$XXxxXXxxXXxXXXXxXXXXXxx$$$XXXXXXXX
x+;;++xxxxxxxxxxxxxxxx+xxx+xxx++++++++xxxx++xxxxxxxxxxxxxxxxxxxxxxxxxX$$$$$X$XXxxxxxxxxxxxx+++xxxXXX$$XXXXXXxxxxxXXXXXXXX$$$$$$XXXX$$$XXXXXXxxxxxx+x++++++++++++++++xxxxxx$Xx;+xxXxXXXXXXXXXXXX$$$$$$xxxxx+xxxxxxx+++++xx+++xx+xXXXXX$$$$XX$$$XXXXXXXxxxxxxxxxx++X$$$$$$$$$$$$$$$$$$$$$$$XXxxxxxXXxxXXXxXXXxxXXxX$$$XXXXXX
xx++++xxXxxx++++++++++++xx+xxx+++;;++xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxX$$$$$$$XXXXxxxxxxxxxxx+++xxxxX$$$$XXXXXxxxxxXXXXXXXXX$$$$XX$$$XXXXXXXxxxxxxxxxxxxxx++++++++++++xxxxxxxX$xxxxXXxXXXXXXXXXX$$$$$$$$Xxxxxxxxxxxxxxxxxxxxxxxx+xxXXXX$$XXXX$$$XXXXXXXxxxxxxxxxxx++x$$$$$$$$$$$$$$$$$$$$$$$XxxXXxxXXxxXXxxxXXXXXxxX$$$X$XXX
x+;;++xxxxx++x+x++++++++xx+xxxxxxxxxxXXXxxxxxxxxxxxxxxxxxxxxxxxxxxxX$$$$$$$$XXXXXxxxxxxXXxxx++xxxxxX$XXXXXXxxxxxxxxxxxxxXXXXXXXXXXXX$XXXXxxxxxxxXXxxxxxxx++;++++++xxxxxxxxxxX$Xxxx;;+xxxxXXXXXXXX$$$$$$Xxxxxxxxxxxxxxxx+xxxxxxxxxxXXXXXxxX$$$$XXXXXXXXxxxxxxxxxxx++x$$$$$$$$$$$$$$$$$$$$$$$XXxxXxxXXXxXXXXXXXXXXXxX$$XXXXx
x+;+xxxxxxxxxxxx+++xxxxxxx+xxxxxxx++xxxxxXxxxxxxxxxxxxxXxxxxxxxxxxxX$$$$$$$XXXXXXXxxxxxxxxxx+++xxxXxxxxxxXXXXxxxxxxxxxxxxxxxxxx++xxxxxxxxxxxxxxxXXXXXXXxxx++++xxxxxxxxxxxxxxxX$x+;xXXx+xxXXXXXXXX$$$$$$$XXXXxxxxxxxxxxxxxxxxxxxxxxxXXXxxX$$$$$XXXXXXXXXxxxxxxxxxxx+xx$$$$$$$$$$$$$$$$$$$$$$$XXxxXxxxXXxXXxXXXXXXXxXX$$$XXX
xxxxxxxxXxxxxxxxxxxxxxxxxx++xxxxxxxxxxxxxxx++++++++++++xxXXXxxxxxxXX$$$$$$$$XXXXXXXxxxxXXxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx++++xxxxxxxxxxxxxxXX$$$XXXxxx++++xxxxxxxxXXxxxxxxX$$XXXXxxxXxXXxxXxxX$$$$$$$XXXXxxxxx+xxxxxxxxxxxxxxxxxxxxX$$$$$XXXXXXXXXXXxxxxxxxxxxx++x$$$$$$$$$$$$$$$$$$$$$$$XXxXxxxXXxXXxXxXXXXXXXxXX$$XX
Xxxxxxxxxxxxxxxxxxx++++++++++++;;;;;;;;;;;;;;;;;;;;;;;;;;+xXXXxxXXXX$$$$$$$XXXXXX$XXXxxXXXxxxxxxxx++++++++xxxx++++;;++xxxxxxXxxxxxxxxxxxxxxxXXXXXXX$XXxxxx+++xxxxxxxxX$$$xxxxxxxX$$$XXXXXxXXxxxxxX$$$$$$$$XXXxxxx+;;;;;;;+++++++xxxxxxxXX$$$XXXXXXXXXXXXXxxxxxxxxxxx++x$$$$$$$$$$$$$$$$$$$$$$$XxxXXxxXXxXXXxXXXXXXxXXXX$XX
xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx+;;;;;;;;;;;;;;;;;;;;;;;;;;;;++xxxxX$$$$$$$XXXXXX$$XXXxxXXXXxxxxx+++;;++++++xxxxxx+;;;;;;;;;+;;;;;;;;;;+++xxXxxxxxxXXXxxxx+xxxxxxxxxXX$$$$xxxxxxx$$$XXXXXxXXxXXXXXXX$$$$$$$XXXxxx+;;;;;;;;;;;;+xxxxxxxXXXXXXXXXXXXXXXXXXXXxxxxxxxxxxx++x$$$$$$$$$$$$$$$$$$$$$$$XXXxXxxXXxXXXxxXxxxXXXXX$$$
xxXXxxXxxxxXXXxXXXXXXXxXXXxXXXxx+;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;x$$$$$$$XXXXXXX$$XXXxxXXXxxxxx+++;++++++++xxxxxxx+x+;;;;;;;;;;;;;;;;+++xxxx+;++xxXXXxxxxxxxxxxxxX$$$$$$$XxxxxxxX$$$XXXXXXXXXXXXXX$$$$$$$$XXXxxxxxxxxxx+++xxxxxxxxxxXXXXXXXXXXXXXXXXXXXXXxxxxxxxxxxx++x$$$$$$$$$$$$$$$$$$$$$$$XXxxxxXXxXxXXXXXXXxXXxXXX$
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXxxxx;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;xX$$$$$$XXXXXXXX$$XXXxxxXXXxxxxx++++++++++++xxx+++xxxxxxxxx+++;xxxxx+xx+++++;;;++xxxxxxxxxxxxxXX$$$$$$X$$XxxxxxxX$$XXXXXXXXXXXXXXX$$$$$$$$XXxxxxxxxxxxxxxxxxxxxxxxxXXXXXXXXXXXXXXXXXXXXXXxxxxxxxxxxx++x$$$$$$$$$$$$$$$$$$$$$$$XXxXXxxxXXxXXXXXXXXXXXXXX
xXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXxxxx+;;;;;;;;;;;;;;;;;;;++++++xxx+xx$$$$$$$$XXXXXXXXX$$XXXxxXXXxxxxx+++++++++++++++++++++++++++xxxx+++++++++++;;;;++xxxxxxxxXXXxxXX$$$$$$$$$$$XxxxxxxX$$$$$XXXXXXXXXXX$$$$$$$$$XXXxXXxxxxxxxxxxxxxxxxxxXXXXXXXXXXXXXXXXXXXXXXxxxxxxxxxxxx++x$$$$$$$$$$$$$$$$$$$$$$$XXXxxXXxXXXXXXXXXXXXXXXX
xxXXXXXXXXXXXXXXXXXXXXXXXXXXXXXxxxx+;;;;;;;;;;;+xxxxxXXXXXXX$$$$$X$$$$$$$$$XXXXXXXXXX$$$XXxXXXXxxxxx+++++++++++++++++++;;;;;;;;;;;+++++++++;;;;;;++xxxxxxxXXXXXX$$$$$$$$$$X$$$XxxxxxxX$$$XXXXXXXXXXXXX$$$$$$$$XXXXXXXXX$xxxxxxxxxxxxxxxXXXXXXXXXXXXXXXXXXXXXXxxxxxxxxxxxx++x$$$$$$$$$$$$$$$$$$$$$$$XXXXXXxXXXXXXXXXXXXXXXX
xxXXXXxXXXXXXXXXXXXXXXXXXXXXXXXXxx;;;;;;;;;;;xxxxxXX$$$$XXxxxxxx+xX$$$$$$$$XXXXXXXXXX$$$XXXXXXXXxxxx+++++++++++++++++++++++;;;;;;;+++++++;;;;;;++++xxxxxXXXXXXX$$$$$$$$$$$$$$$$$xxxxxxxX$XxxxxxxXXxxxXX$$$$$$$$XXXXXxXXxxxxxxxxxxxXXXxxXXXXXXXXXXXXXXXXXXXXXXXxxxxxxxxxxxx++x$$$$$$$$$$$$$$$$$$$$$$$XXXXXXXXXXXXXXXXXXXXXX
xXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXxxxxxxxxxxxxxxxxxxxxxx+;;;;;;;;;;x$$$$$$$$$XXXXXXXXXXX$$$XXXXXXXxxxx++++++++++++++++++++++++++++++++++++;;;+++;;++xxxxxXXXXXX$$$$$$$$$$$$$$$$$$$$XxxxxxxxxxxxxxxxxxxxxxX$$$$$$$XXXXXXxxxxxxxxxxxxXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXxxxxxxxxxxxx++x$$$$$$$$$$$$$$$$$$$$$$$XXXXXXXXxxXXXXXXXXXXX
xXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXxxx++++++++++++++++++;;;;;;;;;++X$$$$$$$$$$XXXXXXXXXXX$$$$XXXXXXxxxx++;;;++++++++++xxxxxxxxxxxxxx++++++++;;;;;;+xxxxxXXXXX$$$$$$$$$$$$$$$$$$$$$$XXXXXxxxxxxxxxxxxxxxxxxX$$$$$$$XXXXxxxxxxxxxxxxXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXxxxxxxxxxxxx++x$$$$$$$$$$$$$$$$$$$$$$$XXXXXXXXXxXXXXXXXXXX
XXXXXXXXXX$$$$$$$$$$$$$$$$$$$$XXxxxxxxxxxxxxxxxxxxxxx+++++++xxxX$$$$XX$$$$$$$XXXXXXXXX$$$$$$$XXXXXxxx++;;;;;+++++++xxxxxXXXxxxxxxxxxx++;;;;;;;++xxxxxXXXXXx$$$$$$$$$$$$$$$$$$$$XX$$XXXXXXXxxxxxxxxxxxxxxX$$$$$XXXxx++xx+xxxxxxxXXXXXxxxxXXXXXXXXXXXXXXXXXXXXXXXXXxxxxxxxxxxxx++x$$$$$$$$$$$$$$$$$$$$$$$XXxXxXXXXXXXXXXXXXX
XX$$$$$$$$$$$$$$$$$$$$XX$$$$XXXXxxxxxxxxxxxxxxxxxxxxxxxxxXX$$$$$$$$$$$$$$$$$$$XXXXXXXX$$$$$$$$$XXXxxxx+;;;;;;;;+++++xxxxxxxxxxxxxxxx+++;;;;;;+++xxxxXXXXxxx$$$$$$$$$$$$$$$$$$$$$$XX$$$XXXXXXXXXXXXxxxXXXXXXXXXXxxxxxxxx+xxxxxxxxxxxxXXXXXXXXXXXXXXXXXXXXXXXXXXXXXxxxxxxxxxxxxx++x$$$$$$$$$$$$$$$$$$$$$$$XXXxXXXXXXXXXXXXXX
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXxxxxxxxxxxxxxxxxxxxxxXXX$$$$$$$$$$$$$$$$$X$$$$$$$XXXXXXXXX$$$$$$$$$$XXxx++++;;;;;++++++xxxxxxxxxxxxx++++;;;;;;;;+xxxXXXXXxxxX$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$XXXXXXXXXXXXXXXXXXxxxxx++++x+xxxxxxxxxxxxxxxxxXXXXXXXXXXXXXXXXXXXXXXXXxxxxxxxxxxxxx++X$$$$$$$$$$$$$$$$$$$$$$$XXXXXXXXXXXXXXXXX
xXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXxxxxxxxxxxxxxxXX$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$XXXXXX$$$$$$$$$$Xxxxx++;;;;;;;;;+++++++xxx+++++;;;;;;+;;+++xxxxX$XXxxxxX$$$$$$$$$$$$$$$X$$$$$$$$$$$$$$$$$$$$$$$$$XXXXXXXXXXxx++xxx++xx++xxxxxxxxxxxxxxxXXXXXXXXXXXXXXXXXXXXXXXXXxxxxxxxxxxxxx+xX$$$$$$$$$$$$$$$$$$$$$$$XXXXXXXXXXXXXXXX
xXXXXXXXXXXXXXXXXXXXXXXXXxxxxxxxxxXXxxxxXXX$$$$$$$$$$$$$$$$$$$$$$$$X$$$$$$$$$$$$$$$XXXXXXX$$$$$$$$$XXxxxx++;;;;;;;;;+++++++++++++;;;;;;;;++++xxxxX$$Xxxxxx$$$$$$$$$$$$$$$$$$$$$$$$$XXX$$$$$$$$$$$$$$$$$$$$$Xxxx++++x++x++xxxxxxxxxxxxxxxxxxXXXXXXXXXXXXXXXXXXXXXXXXxxxxxxxxxxxxxx+xX$$$$$$$$$$$$$$$$$$$$$$$XXXXXXXXXXXXXXX
xxXXxxXXxxXXXXXXXXXXXxxxxXXXXxxXxxxxXX$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$X$$$$$$$$$$$$$$$XXXXXXX$$$$$$$$$XXxxx++;;;;;;;;;;;;;;;+++;;;;;;;;;;;;;++xxxxXXXxxxxxxx$$$$$$$$$$$$$$$$$$$$$$$$$XXXxxxxxxxxXX$$$$$$$$$$$Xx+++++++++xxxxxxxxxxxxxxxxxxxxxXXXXXXXXXXXXXXXXXXXXXXXXXxxxxxxxxxxxxxx++$$$$$$$$$$$$$$$$$$$$$$$$XXXXXXXXXXXXXX
xxXXXXxxXXxxXXXxxxxxxxxxxXXXxxXXX$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$XXxxXXXX$$$$$$$$$XXxxx+++;;;;;;;;;;;;;;;;;;;;;;;++++xxxxxXXXxxxxxxxxx$$$$$$$$$$$$$$$$XX$$$$$$$$$XXXXXxx+;;;;++xx$$$$$x++++++++x++++xxxxxxxxxxxxxxxxxxxXXXXXXXXXXXXXXXXXXXXXXXXXXXxxxxxxxxxxxxx+xX$$$$$$$$$$$$$$$$$$$$$$$XXXXXXXXXXXXX
XXXX$$XXXXX$XXXXXXXXXXXxxxxXXX$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$XXXxxxxXXX$$$$$$$$XXxxxxx+++;;;;;;;;;;;;;;;;;;;;;++xxxxXXXXxxxxxxxxxX$$$$$$$$$$$$$$$$X$$$$$$$$$$XXXXXXXXxxxx+;;;++xxx+++++++++++++xX$$XXxxxxxxxxxxxxxxxXXXXXXXXXXXXXXXXXXXXXXXXXXxxxxxxxxxxxxx++xX$$$$$$$$$$$$$$$$$$$$$$$XXXXXXXXXXXX
XX$$$$$XXXX$$$$$$$$$$$$XXXX$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$XXxxxxxxxXXX$$$$$$$Xxxxxxx++++;;;;;;;;;;;+;++++xxxxxXXXXxxxxxxxxxxxX$$$$$$$$$$$$$$$$$$$$$$$$$$$$$XXX$$XXXXXXxxxx++++++++++++++++xX$$$$$$$XXXxxxxxxxxxxxXXXXXXXXXXXXXXXXXXXXXXXXXXxxxxxxxxxxxxxxxx$$$$$$$$$$$$$$$$$$$$$$$XXXXXXXXXXXX
X$$$$$$$$$$X$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$X$$$$$$$$$$XXXxxxxxxxXXXX$$$$$$XXXxxxxxxx++++;++;++++++xxxxxxXXXxxxxxxxxxxxxxxX$$$$$$$$$$$$$$$$$$$$$$$X$$$$$$$X$$$$$XXXXXXXxxxxx++++++++++xX$$$$$$$$$$$$XXXxxxxxxxXXXXXXXXXXXXXXXXXXXXXXXXXXXxxxxxxxxxxxxx++x$$$$$$$$$$$$$$$$$$$$$$$XXXXXXXXXXX
$$$$$$$X$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$XXX$$$$$$$$XxxxxxxxxxxXXX$$$$$$$$XXXXXxxxxxxxxx++xxxxxxXXXXXXxxxxxx+++xxxxxxX$$$$$$$$$$$$$$$$$$$$$$$$X$$$$$$$$$$$$$$XXXXXXXXXXXXxxxxxx++xX$$$$$$$$$$$$$$$$$$XXxxxXXXXXXXXXXXXXXXXXXXXXXXXXXXXxxxxxxxxxxxxx++x$$$$$$$$$$$$$$$$$$$$$$$XXXXXXXXXX
XXXXXXXX$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$XX$$$$$$$$$$$$$$$$$$$$$$$$X$$$$$$$$$$$$$$$$$$XXxxxxxxxXX$$$$$$$$$$$$$$$XXxxxxxxxxxxxxxXXXXX$XXxxxxxx++++xxx+xxX$$$$$$$$$$$$$$$$$$$$$X$$$X$$$$$$$XXXXX$$$$XXxXXXXXXXXXXX$$XXX$$$$$$$$$$$$$$$$$$$$$$$XXXXXXXXXXXXXXXXXXXXXXXXXXXXXxxxxxxxxxxxxxx++X$$$$$$$$$$$$$$$$$$$$$$$XXXXXXXXX
X$$$$$$$X$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$XxX$$$$$$$$$$$$$$$$$$$$$$$$$XX$$$$$$$$$$$$$$$$$$$$$$XX$X$$$$$$$$$$$$$$$$$$$$$XXXXXXXX$$$$XXXxxxxxxx+++++++xxxxx$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$XXXXXXXXXX$XXXXX$XXXXXXXXXXXXXXXX$$$$$$$$$$$$$$$$$$$$$$XXXXXXXXXXXXXXXXXXXXXXXXXXXxxxxxxxxxxxxxxxxx$$$$$$$$$$$$$$$$$$$$$$$XXXXXXXX
X$$$$$$X$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$XXX$$$$$$$$$$$$$$$$$$$$$$$$XXX$$$$$$$$$$$$$$$$$X$$$XX$$$$$$$$$$$$$$$$$$$$$$$$$$$$$XXXXXxxxxxx+++++++++++xxxxxX$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$XXXXXXXXXXXXXXXXX$XXXXXXXXXXXX$XXXXX$$$$$$$$$$$$$$$$$$$XXXXXXXXXXXXXXXXXXXXXXXXXXXxxxxxxxxxxxxxxxxxX$$$$$$$$$$$$$$$$$$$$$$$XXXXXXX
X$$$$$$X$$$$$$$$$$$$$$$$$$$$$$$$$$X$$$$$$$X$$$$$XXX$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$X$$$$$$$$XXXXXXXXXX$$$$$$$$$XXXXXXXXXXxxxxxxxxx+++++++++++++xxxX$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$XXXXXXxXXXXXXXXXXX$$XXXXXXXXXXXXXXXXXXXXX$$$$$$$$$$$$$$$XXXXXXXXXXXXXXXXXXXXXXXXXXXXxxxxxxxxxxxxxx+x$$$$$$$$$$$$$$$$$$$$$$$$XXXXXX
XX$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$X$$$$$$$XX$$$XXXX$$$$$XX$$$$$$$$$$$$$$$$$$$$$$$$$XXXXXXXXXXXXXXXXX$$$$$$$$$$$$$XXXXXxxxxxxxxxxxxxx+++++++++++++xxxX$$$$$$$$$$$$$$$$$$$$$$$$$$$X$$$$$$$$$XXXXXXXXxxxxxxXXX$XXXXXXXXXXXXXXXXXXXXXXXXXXX$$$$$$$$$$XXXXXXXXXXXXXXXXXXXX$XXXXXXxxxxxxxxxxxxxxx+xX$$$$$$$$$$$$$$$$$$$$$$$XXXXX
XX$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$XXXX$$$$$$$$$XXXX$$$XXXXXX$$$$$$$$$$$$$$$$$$$XX$$$$$$XXXXXX$$$$$$$$$$$$$$$$$$$XXXXxxxxxxxxxxxxx+++++++++++++++xxxX$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$XXXX$$XXXXXXXxxxxxxxxXXX$XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX$$$$$XXX$$XXXXXXXXXXXXXXXXXXXXXXxxxxxxxxxxxxxxx+x$$$$$$$$$$$$$$$$$$$$$$$$XXXX
XX$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$XxXXX$$X$$$XXxxXXXXXXxxxxXX$$$X$$$$$$X$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$XXXXX$$$XXXxxxxxxxxxxxx++++++++++++++++xxx$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$XXXXXXXXXXXXXxxXxxxxxXXxxXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX$XXXXX$$$$$XX$XX$XXXX$XXX$$XXXXXxxxxxxxxxxxxxxxxx$$$$$$$$$$$$$$$$$$$$$$$XXXX
XXXX$XXXX$X$X$$$$$$$XXXX$$$$$$XxxX$XXXXxX$X$$$XxxxXXXxXXxxxxxXXXXXX$$$$X$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$XXXXXX$$$XXxxxxxxxxxx++++++++++++++++xxxXX$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$XXXXXXXXXXXXxXXXXxxxXXxxxXXXXX$$$$$XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX$$X$$$$XXXX$$X$$XXXXXXXxxxxxxxxxxxxxxx+xX$$$$$$$$$$$$$$$$$$$$$$$XXX
XXX$$$XXXXX$$$$$$$$$$$$$$$$$$$XXXX$XXxXXXX$$XXXxxxXXXXX$X$$XXXXXXXX$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$XXXXxxxXX$XXXxxxxxxxxx++++++++;;;;++++xxxX$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$XXXXXXXXXXxxxXXXXXXXXXXXXXXXXX$$$$$$XXXXXXXXxXxxXXXXXXXXXXXXXXXXxXXXXXX$XX$$$$$$$X$XXXXX$$$XXxxxxxxxxxxxxxxx++X$$$$$$$$$$$$$$$$$$$$$$$XX
XX$$$$$$XXXXXX$$$$$$$$$$$XXXXX$$$$$$$$XXXX$XXX$$XXX$X$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$XXXXXXXXXxxxX$$$XXxx+++++++++++;;;+++;;+xxX$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$XXXXXXXXXXXX$$$XXXXX$$XXXXXXXXXXX$$$$XXXXXXXXXxxxxxxxxxxxxxxxxxxXXxxXXxxX$$$$XXXX$$$$$$$$XXXXXXXxxxxxxxxxxxxxx+xX$$$$$$$$$$$$$$$$$$$$$$$X
X$$$$$$X$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$X$$$XXXXXXXXxxxXXX$$Xxxxxx++++++++++;;;+++xxxX$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$XXX$$$$XX$$$$$$XXXXXXXXXXXXXXXX$$$$XXXxXXXXxxxxxxxxxxxxxxxXXxXXXxxXXxxXXX$$$X$$XXX$$$$$$XXXXxxxxxxxxxxxxxxx+x$$$$$$$$$$$$$$$$$$$$$$$$
XX$$$$$X$$$$$$$$$XXX$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$XXX$$$$$$$$$$$XXX$XXX$$$$$$XXXXXXXXXX$$X$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$XXxXX$$$$$$$$$$$$$$$$$$$XxxxxxxxxxxxxxXX$$$$$$$$$xxXX$X$$$$$$$$$$$$$$$$$XXXXXXx$$$$$$$$XxX$$$$$$$$$$$$$$$$$$$$$$$
$$$$$XXX$$$$$XXXXXXXX$$$$$X$$$$x+;;;;;;;+x$$$$$$$$x+;;;;;;;;;;;;+xX$$$$$$$$$$Xx;;;;;;;;+x$$$$XXXXX$$Xx;;;;;;;;xX$$$$$$x;;;;;;;;;;;;+x$$$$$$x;;;;;x$$$$$$x+;;;;;$$$$$$$$$$$$$Xx;;;;;;;;;;;;;;;;X$X;;;;;;x$XXxx$$x;;;;;x$$x+;;;;+$$Xxxxxxxxxxxxxx$$x;;;;;;+XXxxxXX$$x+;;;;;;;;;++x$$$XxxxX$x;;;;++$$x$$$$$$$$$$$$$$$$$$$$$$$
$$$$$$XX$XX$$$$$$$$$XXX$$$$$x;;;;;;;;;;;;;;;x$$$$$x+;;;;;;;;;;;;;;;;x$$$$$$x;;;;;;;;;;;;;;+X$$X$$$x;;;;;;;;;;;;;+x$$$$x;;;;;;;;;;;;;;;x$$$$x;;;;;x$$$$$$x+;;;;;$$$$$$$$$$$$$$x;;;;;;;;;;;;;;;;X$$x;;;;;+$$XxX$x;;;;;;X$$x;;;;;;$$XxxxxxxxxxXxxX$x;;;;;;;;+$$xxxX$$x;;;;;;;;;;;;;;;xX$Xxx$x;;;;;+XXxxX$$$$$$$$$$$$$$$$$$$$$
XX$$XXXXXXX$$$$XXXX$$$$$$$$x;;;;;;++x++;;;;;;+$$$$x+;;;;;+++++;;;;;;;X$$$x+;;;;;;+xxx+;;;;;;x$$$Xx;;;;;+xXXx;;;;;;x$$$x;;;;;;;++;;;;;;;x$$$x;;;;;x$$$$$$x+;;;;;$$$$$$$$$$$$$$x;;;;;;;;++;;+;;+X$$X+;;;;;x$$X$Xx;;;;;x$$$x+;;;;;$$XxxxxxxxxxxxX$X+;;;;;;;;;xXXXxxX$x;;;;;;+++;;;;;;;;XXXx$x;;;;;+XXxxx$$$$$$$$$$$$$$$$$$$$$
xXXXXXX$XXX$$XXXXX$$$$$$$$x;;;;;+x$$$$$X+;;;;;xX$$x+;;;;;X$$$$X+;;;;;x$$$+;;;;;+X$$$$$x;;;;;;x$$X+;;;;;+X$$$Xxxxx+X$X$x;;;;;+$$$$x;;;;;;$$$x;;;;;x$$$$$$x+;;;;;$$$$$$$$$$$$$$+;;;;;x$$$$$$$$$$$$$$x;;;;;;X$$$x;;;;;+$$X$x;;;;;;$$XxxxxxxxxxxXX$x;;;;;+;;;;+X$XxxX$x;;;;;;$$$$Xx;;;;;;X$X$x;;;;;+$Xx+xx$$$$$$$$$$$$$$$$$$$$
XXXXXXXXXX$XXXX$$X$$$$$$$$+;;;;;x$$XXX$$X;;;;;+x$$X+;;;;;xXXXXx;;;;;;X$$x+;;;;;X$$$$$$$x;;;;;+$$$x;;;;;;;;;+xxXX$$$Xx$x;;;;;+X$$$+;;;;;+$$$x;;;;;x$$$$$$x+;;;;;$$$$$$$$$$$$$$+;;;;;;xxxxxxxxx$$XX$$x;;;;;x$$$+;;;;+X$Xx$x;;;;;;$$XxxxxxxxxxxX$x;;;;;xX;;;;;x$$xxX$x;;;;;;$$$$$X+;;;;;x$$$x;;;;;+$Xxx+xX$$$$$$$$$$$$$$$$$$$
XXXXXXXXXXXXXXX$X$XXX$X$$X;;;;;;x$XXXXX$X;;;;;;x$$x;;;;;;;;;;;;;;;;+x$$$x;;;;;;$$$$$$$$x;;;;;+$$$$x+;;;;;;;;;;;;+xX$X$x;;;;;;;;;;;;;;;;x$$$x;;;;;x$$$$$$x+;;;;;$$$$$$$$$$$$$$x;;;;;;;;;;;;;;+$$XXX$X+;;;;;x$x;;;;;x$XxX$x+;;;;;$$XxxxxxxxxxX$$;;;;;xX$x;;;;;x$XX$$x;;;;;;$$$X$X+;;;;;x$$$x;;;;;x$Xxxxxx$$$$$$$$$$$$$$$$$$$
XXXXXXXXXXXXXXXXXX$$$$$$$$+;;;;;x$$$XXX$X;;;;;+X$$x+;;;;;;;;;;;;;;x$$$$$x;;;;;;$$$$$$$$x;;;;;+$$$$$$Xx+;;;;;;;;;;;xX$$x;;;;;;;;;;;;;;+x$$$$x;;;;;x$$$$$$x+;;;;;$$$$$$$$$$$$$$+;;;;;;;;;;;;;+x$$XXX$$x;;;;;xX;;;;;+X$XxX$x;;;;;;$$XXXXXXXXXX$Xx;;;;;X$$x+;;;;+$$XX$x;;;;;;$$$X$X+;;;;;x$$$x;;;;;xXXxxxx+x$$$$$$$$$$$$$$$$$$
XXXXXXXXXXXXXXXXXXXXXXX$$$x;;;;;+$$$$$$Xx;;;;;xX$$x+;;;;;xXx+;;;;;x$$$$$X+;;;;;xX$$$$$X+;;;;;x$$$xxxxxX$$$Xx+;;;;;+x$$x;;;;;;xxxxxxxX$$$$$$x;;;;;+x$$$$$x;;;;;;$$$$$$$$$$$$$Xx;;;;;x$$$$$$$$$$$$XxX$$x;;;;;;;;;;;x$XXXX$x+;;;;;X$$$$$$$$$$$$x;;;;;;;+;+;;;;;;+$$$$x;;;;;;$$$$$x;;;;;;x$$$x;;;;;+$Xxxxx+xX$$$$$$$$$$$$$$$$$
XXXxxxxxXXXXXXXXXXXXXXXX$$$+;;;;;;xxxxx;;;;;;;X$$$x+;;;;;X$$X+;;;;;x$$X$$x;;;;;;+xxxx+;;;;;;xX$$X+;;;;;xxxxx+;;;;;+X$$x;;;;;;$$$$$$$$$$$$$$$+;;;;;;xxxx+;;;;;;x$$$$$$$$$$$X$X+;;;;;+xxxxxxxxxxX$$XxX$x+;;;;;;;;;x$$xXX$$x;;;;;;++++x+++xxX$X+;;;;;;;;;;;;;;;;;x$$$x;;;;;;xxxx;;;;;;;x$$X$x;;;;;+XXxxxxx+xX$$$$$$$$$$$$$$$$
XxxxxxxxxxXXXXXXXXXXXXXXX$$Xx;;;;;;;;;;;;;;;+X$$X$x+;;;;;X$$$x;;;;;;x$$$$$X+;;;;;;;;;;;;;;;x$$$$$X;;;;;;;;;;;;;;;+X$$$x;;;;;+$$XX$$$$$$$$$$$X+;;;;;;;;;;;;;;;x$$$$$$$$$$$$X$$+;;;;;;;;;;;;;;;;x$$$XXX$x;;;;;;;;+$$$XXX$$x+;;;;;;;;;;;;;;;x$x;;;;;;+xx+xx+;;;;;+X$$x;;;;;;;;;;;;;;;+x$$$$$x;;;;;+$Xxxxxxx+x$$$$$$$$$$$$$$$$
XxxxxxxxxxxxxxxxXXXXXXXXXX$$$Xxx+;;;;;;;++xX$$$$$$X+;;;;+X$$$$x;;;;;;x$$$$$$Xx+;;;;;;;;+xxX$$$$$$$$xx+;;;;;;;;+xx$$$$$x;;;;;+$$XXX$$$$$$$$$$$$xx+;;;;;;;;;+xX$$$$$$$$$$$$XX$$x;;;;;;;;;;;;;;;;x$$$XXx$Xx;;;;;;+x$$X$XXX$x+;;;;;;;;;;;;;;+xX+;;;;+X$$$$$$X+;;;;;x$$x+;;;;;;;;;+++xxX$$$X$$x;;;;;x$Xxxxxxx+xx$$$$$$$$$$$$$$$
xxxxxxxxxxxxxxxxxxxxxxxxXXXXXXX$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$XXXX$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$XXX$$$$$$$$$$$$$$$$$$$$$$$$X$$$$$$$$$$$$XxxXXX$$$$$$$$$$$$$$$$$$$$$$$$$$$XxxXXX$$$$$$$$$$$$$$$$$$$$$$$$$$$XXXX$$$$$$$$$$xxxxxxxxxxX$$$$$$$$$$$$$$
xxxxxxxxxxxxxxxxxxxxxxxxxXxxxX$XXXXXXX$$$$XX$$$X$$$$$$$$$$$$$$$$$X$$$$$$$$XXX$$XX$$$$$$$$$$$$$$$$$$$$$$$$$$$XX$$$$$$XXXXxXXXXXXXXXX$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$XXXXXXxXXXXXXXX$$$$$$XXX$$$$$$XXXXXXXXXXXXXxxxxxxxxXXXXXXxxXXxxXXXXXXXXXXxxxxxXxxxxXXXxxXXXxxxxxXXXXX$XX$XX$XXXXXXxxxxxxxxxxxxxxxx+x$$$$$$$$$$$$$$
xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxXXxXXXXXX$$$XX$$$$$XXX$$$$$$$$$$$$$XXX$$$$$$XXXX$$$X$$$$$$$$$$$$$$$$$$$XX$$$$$XXX$$$$$$$XXXXxxxXXXXXXX$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$XXXXXXXXXxxXXXXX$$$$$$XXXX$$$$$$XXXXXxxxxxxXXxxxxxxxxXX$XXxxxxXXXxxX$XX$XXxxxxxxXXxXXxXXXXXXxxXxxxXXXXX$$$XXXXX$XXXXxxxxxxxxxxxxxxxxxX$$$$$$$$$$$$$
xxxxXxxxxxxxxxxxxxxxxxxxxxxxxxXxxxxXXXXX$$XX$$$$$$XX$$$$$$$$$$$$$$$X$$$$$$$XXXXXXXXX$$$$$$$$$$$$$$$$$$$$$$$$$XXXX$$$$XXXXXXXXxX$XXXXX$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$XXXXXXXXxxxXXxxXX$$$$$$$$$$$$$$$$XXXxXxxxxxxxxxxxxxxxxxX$$XxxxxxxXXxXXX$XXXXxxxxxXXxXXXXXXXXXXXxxXXXXXXX$$$$XXXXXXXXXxxxxxxxxxxxxxxx+xX$$$$$$$$$$$$
xxxxXXXXXXXXXxxxxxxxxxxxxxxxXXxxxxxxxxxXXXXXXXXXX$$$$$$$$$$$$$XX$$$X$$XX$XXXXXXXXXXX$$$$$$$$$$$$$$$$$$$$XX$$$$$XXXXXXXXXXXXXXXX$$$$XX$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$XXXXXXXxxxXXXxxxxX$$$$$$$XX$$$$$$$XXXXxxxxxxxxxxxXxxxxxxxX$$XxxxxxXXXXX$XXXXXxxxxXXXXXXXXXXXXXXXXXXXXxXX$$$$$$$$XXXXXxxxxxxxxxxxxxxxxxx$$$$$$$$$$$$
xxxxxxxXXXXXXXXXxxxxxxXXXXxXXxxxxxxxxxxxXXXXXXXX$XX$$$$$$$$XXXXX$$$XXXXX$XXXXXX$XXXXX$$$$$$$$$$$$$$$$$$XXXX$$X$$XXXXXX$XXXXXXXXX$$$XX$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$X$$$$XXXxXXXXXxxXXXX$$$$$$$XXX$$$$$$XXxxxxxxxxxxxxxxxxxxxxxX$$XXXXXxxXXXXXXXXXXXxXXXXXXXXXXXXXXXXXXXXXxXXX$$$$$$$$$$XXXxxxxxxxxxxxxxxxxxx$$$$$$$$$$$
xxxxxxxxxxXXXXXXXXXXXXxXXxXxxxxxxxxxxxxxxxxxxXXXXXX$$$$$$$XXXXXX$$XXXXXX$XXXXXXXXXXXXX$$$$$$$$$$$$$$$$$XXXXXXXXXXXXXXXXXXXXxXXXX$$$XXX$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$XXXX$$$X$$$$$XX$$$$$$$$$$$$XxxxxX$$XXXxxxxxxxxxxxxxxxxxxxxxXX$$XXXXXXXXXX$$$XXXXXXXXXXXXXXXXXXXXXxXXXXXXXXXX$$$$$$$$$$XXXxxxxxxxxxxxxxxxxX$$$$$$$$$$
xxxXxxxxxxXXXXXXXXXX$XXXXXxxxxxxxxxxxxxxxxxxxxxxxXX$$$$$XXXXXXXX$XXXXXXXXXXXXXXXXXXXXX$$$$$$$$$$$$$$$XXXXXXXXXXXXXXXXxxXXXXXXXXXX$$$XX$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$XXXXXXXX$XX$$$$$$$$$$$$$$$$$$XxX$$$$XXxxxxxxxxxxxxxxxxxXxxxxXX$$$XXXXXXXXX$$XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX$$$XXXX$$$XXXxxxxxxxxxxxxxxxx$$$$$$$$$$
xxxXxxxxxXXXXXXXxxXXXXXXX$XXXXXXxxxxxxxxxxxxxxxxxXX$X$$XXXXXXxxX$XxxXXXXXXXxXXxxXXXXXX$$$$$$$$$$$$$$$$XXXXXXxxXxxxxxxxxxXXXXxxXXXX$$XxX$$$$X$$$$$$$$$$$$$$$$$$$$$$$$$$XXXXXXXX$$XXXXXXX$$$X$$$$$$$$$$$XX$$$XXXxxxxxxxxxxxxxxxXXXXXXX$$$$$$$$$XXXX$$XXXXXXXXXXXXXXXXXXxXXXXXXXXXXxXXXXXXXXXXXXXXXXXxxxxxxxxxxxxxxX$$$$$$$$$
xxxXXxxxXXXXXXXXxxxXXX$$$XXXX$XXXXXXXXxxxxxxxxxxxxX$$$XXxxxxxxxXXxxxxXXXxxXXXxxXXXxXXXX$$$$$$$$$$XXXXXXxxXxxxxxxxxxxxxxxxxxxxXXXXXX$XXX$X$$$$$$$$$$$$$$$$$$$$$$$$$$$$$XxxxXXXXXXXXXXXXXXX$XX$XX$$$$$XX$$XXXX$XxxxxxxxxxxxxxxxXXXXXXX$$$$$$$$$$$$$$$$$XXXXXXX$$XXXXXXXXXXXXXXxxXXxXXX$$XXXXXXXXXXXXXxxxxxxxxxxxxxxX$$$$$$$$
xxxXXxxXXXXxXXXxxxxxX$$$XXXXX$XX$XXXXXXXXXXxxxxxxxX$$xxxxxxxxxxxxxxxxxxXXxxxxxxXxxxXXXX$$$$$$$$$$XXxxxxxxxxxxxxxxxxxxxxxxxxxxXXX$$$$$$XX$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$XXXXXXXXXXXX$$X$XXXX$$$$$$$$$$$XXX$XXX$XXxxxxxxxxxxxxxxXXXXXXX$$$$$$$$$$$$$$$$$$XXX$$$$$$$XXXXXXX$XXXXXXXXxXXXX$$XXXXXXXXXXXXxxxxxxxxxxxxxxx$$$$$$$$
xxxXXxxxxxxxXXXxxxx$$$XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxXXXXX$$$$$$$$$XXxxxxxxxxxxxxxxxxxxxxxxxX$XXxxxxxxxxX$$$$$$$XxxxxxxxxxxxxxxxxX$$$$xxxxxX$$XXXX$$xxxxx$$$$$XxxxxxX$$$$XXXXXXXXXXxxxxxxxxxXXxxXXXXX$X$$$$$$$$$$$$$$$$$$XX$$$$$$$$XXXXXXXXXXXXxXXXXXXXXXXXXXXXXXXXXXXxxxxxxxxxxxxxxxX$$$$$$$
xxxxXXxxxxxxxXXxxXX$XXxxxxxxxxxXXXXXXXXXXX$XX$$$$$$XXXXXXXXX$X$$$$$$$XXXXXXXX$$XXXXXXXXX$$X$$$$XxxxxxxxxxxxxxxxxxxxxxxxX$$x;;;;;;;;;;;;+X$$$$x+;;;;;;;;;;;;;;;+$$$X+;;;;;x$$XxX$X;;;;;xX$$Xx;;;;;x$$$$$XxXXXXXXXXXxxxxxxXXXXXX$$XXXXXXXXXX$$$$$XXX$$$$$XX$$$$$$$XXX$$XXXXXXXXXXXXXXXXXXX$XXXXXXXXXXxxxxxxxxxxxxxxxx$$$$$$$
xxxXXXxxxxxxxxxxXX$XxXxxxxxxXXXXXXXXXXXXXXX$$$$$X$$$XXXXXXXX$X$$$XXXXXXXXXX$$$$X$$$$XX$XXXXX$$$xxxxxxxxxxxxxxxxxxxXXXxX$$+;;;;;++++;;;;;;x$$$X+;;;;;;;;;;;;;;;+$$$X+;;;;;;;x$$X$X;;;;;xX$XXx;;;;;x$$$$$XXxxxXXXX$XxxxxxxXXXXXXXXxxxxxxxxxXXXXXXXXX$XXXxxxxXXX$$$$$XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXxxxxxxxxxxxxxxxxX$$$$$$
xxxXXXxxxxxxxxxX$XxXXxxxxxXXX$XXXXXXXXXXXX$$$$XXX$XXXXXXXXXX$$$$XXXXXXXXXX$$$$$$$XXXXXX$$XXX$$$xxxxXXxxxxxXXXXXXXXXXX$$$x;;;;;+x$$$Xx++++x$$$X+;;;;;xXXX$$X$$XX$$$X+;;;;;;;;xX$$X;;;;;xX$$Xx;;;;;x$$$$$XXxxxxxxXXXXxxxxXXXXXXXxxxxxxxxxxxxxxXXXxxXX$XXxxXX$$XXXXX$$XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXxxxxxxxxxxxxxxxxX$$$$$
xxX$XXXXxxxxxxx$XxXXxxxxxXXXXXxxxXXXXXXXXX$$X$$XXXXXXXXXXXXXX$$$XXXXXXXXXXX$$$$$XXXXXXX$$$XXXX$$XXXXX$$$$$X$$$X$$$X$$$$$x;;;;;;;++xxX$$$$$$$$X+;;;;;xXxXXXxxX$$$$$X+;;;;;;;;;;x$X;;;;;xXX$Xx;;;;;xX$X$$XXXxxxxxxXXX$XXXXXXXXXxxxxxxxxxxxxxxxXX$XXXX$$XXXX$$$$XXXXX$$X$XxxXXXXXXXXXXXXXXXXXXXXXXXXXXXxxxxxxxxxxxxxxxxX$$$$$
xxXXXXXXxxxxxXXXxXXxxxxxxxxxxxxxxXXXXXXXX$XX$XXxxXXXXXXXXXX$$XXX$XXXXXXXXXX$$$$$XXXXXXX$$$$XX$$$$$$XXX$$$$XXX$XXXXXX$$$$$x;;;;;;;;;;;;+xX$$$$x+;;;;;;;;;;;;;;x$$$$$+;;;;;;;;;;;xx;;;;;xXXXXx;;;;;xX$X$$$XxxxxxxxxXX$$$XxxxxxxXxxxxxxxxxxxxxxXX$$$XXX$XXXX$$$$$$$$$XXXXXXXXXXXXX$XXXXXXXXXXXXXXXXXXXXXxxxxxxxxxxxxxxxx$$$$$
xxXXXXXXxxxxXXXxXXxxxxxxxxxxxxxxxxxxxXXXX$$$XxxxXXXXXXXXX$$$$XXXXXXXXXXXXXX$$$$XXXXXXXXX$$$XX$$$$$XXX$$$$XX$$X$XXXXXX$$$$$$xx+;;;;;;;;;;;xX$$X+;;;;;;;;;;;;;+x$$$$X+;;;;+x;;;;;;;;;;;;xXXXXx;;;;;xXXxX$$XxxxxxxXXX$$$$$XxxxxxxxxxxxxxxxxxxxxxX$$$XXXX$X$$$$$$$$XXXXXXXXX$XXXX$$XX$XXXXX$XX$XX$XXXXXXXXxxxxxxxxxxxxxxxx$$$$
xXXXXXXXxxxxXXxXXXxxxxxxxxxxxxxxxxxxxXXXXXXXxxxX$XXxxXXX$$$$X$XXXXXXXXXX$XX$$$$XXXXXXXXX$$$$$$$$$$$$$$$$$XXXXXXXXXXX$$$$$$$XX$$$Xx+;;;;;;;x$$X+;;;;;X$$$$$$$$$$$$$X+;;;;x$x+;;;;;;;;;;xXXXXx;;;;;xXXxxX$XxxxXXXX$XX$$$$$xxxxxxxxxxxxxxxxxxxxXXX$$$$XX$$$$$$$$$XXXXXXXXXXXXXX$$$XX$$XXXXXX$XXX$XXXXXXXXxxxxxxxxxxxxxxxx$$$$
XXXXXXXXXxxxxxxXXXxxxxxxxxxxxxxxxxxxxxxxxxxxxxXXXxxxxX$$$$XXXXXXXXXXXXXX$XX$$$X$XXXXXXXXX$XXXXXX$$$$$$$$$$$XXXXXXXXXXX$Xx;;;;;x$$$$X+;;;;+x$$X+;;;;;xXXXXXXXXX$$$$X+;;;;x$$$x;;;;;;;;;xXXXXx;;;;;xXXxxXX$$$XXXXXXXXX$$$$XXxxxxxxxxxxxxxxxxXXXXX$$$$$$X$$$$$$$$$$XXXXXXXXXX$$$$$$XXX$X$$X$$$XXX$XXXXXXXxxxxxxxxxxxxxxxxx$$$
XXXX$$$X$XxxxX$XXxXXXXxxxxxxxxxxxxxxxxxxxxxxxX$$XxxxX$$$$XXXXXXXXXXXXXX$$$XX$$XXXXXXXXXXXX$XXX$XXX$$$$$$$$$$$$XXXXXXXX$$X+;;;;;+++;;;;;;;xX$$X+;;;;;;;;;;;;;;;+$$$X+;;;;x$$$$$x;;;;;;;xXX$$x;;;;;x$$XXXX$$$$XXXXXXXXXXX$XXXxxxxxxxxxxxxxxxXXXXX$$X$$$X$$$$$$$$$XXXXXXXXX$$$$$$XXXXXX$$$$$$$X$XXXXXXXXXXXxxxxxxxxxxxxxxxX$$
XXX$$$$$$$XXXXXXxxX$$XXxxxxxxxxxxxxxxxxxxxxxx$XxxxXX$$XXXXXXxxxXXXXxXXX$$XXX$XXXXXXXXXXXXXXXX$$$XX$$$$$$$$$$$$XXXXXXXXX$$Xx+;;;;;;;;;;;+x$$$$X+;;;;;;;;;;;;;;;;$$$X+;;;;x$$$X$$x+;;;;;xX$$Xx;;;;;x$$X$$XXXX$$XXXXXXXXXXXXXXxxxxxxxxxxXXXXxXXXX$$$$$$$$$$$$$$$$$XX$$XXX$$$$$$$$XXX$$X$$$$$$$XX$XXXXXXXXXXxxxxxxxxxxxxxxxX$$
XXX$$$$$$XXXXXXXxX$$$XXXxxxxxxxxxxxxxxxxxxxxX$xxxX$$$XxxxxxxxxxxXxxxxXX$$XXXXXXXXXXXXXXXXXXXX$$$$$X$$$$$$$$XX$$XXXXXXXXX$$$$XXxxxxxxXXX$$$$$$$XxxxxxxxxxxxxxxxX$$$$XxxXXX$$$XxX$XXxxxX$$$$$XxXXxx$$$XXXXXXX$$XXxXXXXXXXXXXXXXxxxxxxxxXXXXXXXXXX$$$$$$$$$$$$$$$$$XXXXX$$$$$$$$X$$$$$$X$$$X$$XX$XXXXXXXXXXxxxxxxxxxxxxxxxxX$
XX$$X$$$$$XXXXXxxX$$$X$XXXXXXXxxxxxxxxxxxxxX$XxxX$$XxxxxxxxxxxxxxxxxxXX$XXXXXxxxxXXXXxxxXXXXXX$$$$X$$$$$$$$XX$$XXXXXXXXXXXXX$XX$$$$X$XXXXX$$XXX$$$$$$$$$$$$$$$$$$$$$$$$$$$$$XxXX$$$$$$$$$X$$$$$$$$$XXXXXXXXXXXXXXXXxxXXXXXXXXXxxxxxxxXXXXXX$$$X$$$$$$$$$$$$$$$$XXXX$$XX$$$$$$$$$$$$$X$$XXXX$$XXXXXXXXXXXXxxxxxxxxxxxxxxxx$
$$$$X$$$$$XXXXXxxxX$$$$XXXXXXXXXXxxxxxxxxxXXXxxxXxxxxxxxxxxxxxxxxxxxxXX$XxxxxxxxxxxxxxxxxXXXX$XXXXXX$$$$$$XXXXX$XXXXXXXXXXXXXXXXXXXXXXXxxXXXXXX$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$XXXXXXX$$X$$XXX$XXXXXXXXXXXXXX$$XXXXXxXxxxXXXxXXXxxxxxxX$XXXX$$$$$$$$$$$$$$$$$$$XX$$$$XX$$$$$$$$XX$XXX$XXX$XX$XX$XXXXXXXXXXxxxxxxxxxxxxxxxxX
$$$$$$$$$X$$XxxxxxX$$$$XXX$$$XXXXXXXXxxxxxXXXxxxXxXXXxxxxxxxxxxxxxxxxxXXxxxxxxxxxxxxxxxxXXXXXXXXxxXXXX$$$$$$XXXXXXXXXXxxXXXXXXXxXXXXXXXXXXX$XXXXX$$$$$$$$$$$$$$$$$$$$$$$$$$$$$XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX$XXXXXxxxxxxxXXXxxxxxxXXX$$$$$$$$$$$$$$$$$$$$$$$$$$XX$$XXXXXXXX$$XX$$XXXXX$$X$$$$XXXXXXXXXxxxxxxxxxxxxxxxx
XXXXX$$$$$$$XXxXX$$$$$X$$X$$$$$$$XXXXxxxxXXXXXXXX$$XXXXXxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxXXXXxXXXXXXX$$$$$XXXXXXXXXXXxxxxxxXxxXxxxxXXXXXXX$XXxX$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX$XXXxxxXxxxxxxxxxxxxxxXX$$$$$$$$$$$$$$$$$$$$$$$$$$X$$XXXXXXXXX$$$XXXXX$$X$$$X$$$$$$XXXXXXXXxxxxxxxxxxxxxxx """);
  }
    else if (!encoder.testConnection()){
  Serial.println("sadece Encoder nanay baba");
    }
      else if (!mpu.testConnection()) {
    Serial.println("MPU6050 sıkıntı kardeşim ya");
      }
        else if (echoPin == NULL || trigPin == NULL) {
      Serial.println("otizmli misin sen çocuk");
        }
          else
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
