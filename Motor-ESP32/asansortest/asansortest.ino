#include <Arduino.h>

const int ER_PWM = 4;
const int ER_IN1 = 5;
const int ER_IN2 = 6;

const int EL_PWM = 12;
const int EL_IN1 = 13;
const int EL_IN2 = 14;



void setup() {
  // put your setup code here, to run once:
pinMode(ER_PWM, OUTPUT);
pinMode(ER_IN1, OUTPUT);
pinMode(ER_IN2, OUTPUT);

pinMode(EL_PWM, OUTPUT);
pinMode(EL_IN1, OUTPUT);
pinMode(EL_IN2, OUTPUT);



}

void loop() {
  // put your main code here, to run repeatedly:
yukari();
delay(3000);
dur();
delay(1000);
asagi();
delay(3000);
dur();
delay(1000);

}

void yukari() {
 digitalWrite(ER_IN1, HIGH);
 digitalWrite(ER_IN2, LOW);


 digitalWrite(EL_IN1, HIGH);
 digitalWrite(EL_IN2, LOW);

 analogWrite(ER_PWM, 200);
 analogWrite(EL_PWM, 200);

}

void asagi() {
 digitalWrite(ER_IN1, LOW);
 digitalWrite(ER_IN2, HIGH);


 digitalWrite(EL_IN1, LOW);
 digitalWrite(EL_IN2, HIGH);

 analogWrite(ER_PWM, 200);
 analogWrite(EL_PWM, 200);

}

void dur(){
 digitalWrite(ER_IN1, LOW);
 digitalWrite(ER_IN2, LOW);


 digitalWrite(EL_IN1, LOW);
 digitalWrite(EL_IN2, LOW);

 analogWrite(ER_PWM, 0);
 analogWrite(EL_PWM, 0);

}