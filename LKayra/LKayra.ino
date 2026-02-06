// 192.168.4.1
// MECHATAK - Final versiyon kodu

#define PROBOT_WIFI_AP_PASSWORD "kayra123"
#define PROBOT_WIFI_AP_SSID "MECHATAK"
#define PROBOT_WIFI_AP_CHANNEL 3

#include <probot.h>
#include <probot/io/joystick_api.hpp>
#include <probot/command.hpp>  // Includes scheduler, command, subsystem, command_group
#include <probot/command/examples/mecanum_drive.hpp>
#include <probot/devices/motors/boardoza_vnh5019_motor_controller.hpp>



const int FL_PWM = 15;
const int FL_IN1 = 16;
const int FL_IN2 = 17;

const int FR_PWM = 4;
const int FR_IN1 = 5;
const int FR_IN2 = 6;

const int RL_PWM = 7;
const int RL_IN1 = 8;
const int RL_IN2 = 9;

const int RR_PWM = 12;
const int RR_IN1 = 13;
const int RR_IN2 = 11;

const int E_PWM = 1;
const int E_IN1 = 2;
const int E_IN2 = 42;

using Scheduler = probot::command::Scheduler;

void robotInit() {
  // put your setup code here, to run once:
pinMode(FL_PWM, OUTPUT);
pinMode(FL_IN1, OUTPUT);
pinMode(FL_IN2, OUTPUT);

pinMode(FR_PWM, OUTPUT);
pinMode(FR_IN1, OUTPUT);
pinMode(FR_IN2, OUTPUT);

pinMode(RL_PWM, OUTPUT);
pinMode(RL_IN1, OUTPUT);
pinMode(RL_IN2, OUTPUT);

pinMode(RR_PWM, OUTPUT);
pinMode(RR_IN1, OUTPUT);
pinMode(RR_IN2, OUTPUT);

pinMode(E_PWM, OUTPUT);
pinMode(E_IN1, OUTPUT);
pinMode(E_IN2, OUTPUT);


}


void autonomousInit() {
  // Otonom başladığında bir kez çalışır



}

void autonomousLoop() {
  // Otonom süresince sürekli çalışır
  ileri(200);
  delay(500);
  anidur();
  delay(500);
  solon();
  delay(500);
  anidur();
  delay(500);
  sagon();
  delay(500);
  anidur();
}


void teleopInit() {
  // Teleop (manuel kontrol) başladığında bir kez çalışır

}

void  teleopLoop() {
  // Manuel modda sürekli.
auto js = probot::io::joystick_api::makeDefault();


  // Her döngüde buffer'ı temizle, yoksa veriler birikir
  probot::telemetry::clear();

  // Başlık
  probot::telemetry::println("=== JOYSTICK TEST ===");

  // Sol stick
  probot::telemetry::printf("Sol Stick:  X=%.2f  Y=%.2f\n",
                            js.getLeftX(), js.getLeftY());

  // Sağ stick
  probot::telemetry::printf("Sag Stick:  X=%.2f  Y=%.2f\n",
                            js.getRightX(), js.getRightY());

  // Tetikler
  probot::telemetry::printf("Tetikler:   LT=%.2f  RT=%.2f\n",
                            js.getLeftTriggerAxis(), js.getRightTriggerAxis());

  // Butonlar - basılı olanları göster
  probot::telemetry::print("Butonlar:   ");
  if (js.getA()) probot::telemetry::print("CARPI ");
  if (js.getB()) probot::telemetry::print("YUVARLAK ");
  if (js.getX()) probot::telemetry::print("KARE ");
  if (js.getY()) probot::telemetry::print("UCGEN ");
  if (js.getLB()) probot::telemetry::print("LB ");
  if (js.getRB()) probot::telemetry::print("RB ");
  if (js.getStart()) probot::telemetry::print("START ");
  if (js.getBack()) probot::telemetry::print("BACK ");
  probot::telemetry::println("");



  // D-Pad (POV)
  int pov = js.getPOV();
  probot::telemetry::print("D-Pad:      ");
  if (pov == -1) {
    probot::telemetry::println("-");
  } else if (pov == 0) {
    probot::telemetry::println("YUKARI");
  } else if (pov == 90) {
    probot::telemetry::println("SAG");
  } else if (pov == 180) {
    probot::telemetry::println("ASAGI");
  } else if (pov == 270) {
    probot::telemetry::println("SOL");
  } else {
    probot::telemetry::printf("%d derece\n", pov);
  }


  // Sequence numarası
  probot::telemetry::printf("Seq: %lu\n", static_cast<unsigned long>(js.getSeq()));




//Bu bölümü komple uyumlu hale getir
  // 2. Drive Forward (Trigger 5)
  else if (axis5 > -0.995) {  // or inside these conditions
    //Serial.println("FORWARD");
    speed = getSpeedTrig(axis5);
    setMotor(RL_IN1, RL_IN2, RL_PWM, 1, speed);
    setMotor(RR_IN1, RR_IN2, RR_PWM, 1, speed);
    setMotor(FL_IN1, FL_IN2, FL_PWM, 1, speed);
    setMotor(FR_IN1, FR_IN2, FR_PWM, 1, speed);
    Serial.println("FORWARD:" + String(speed));
  }

  // 3. Drive Reverse (Trigger 2)
  else if (axis2 > -0.995) {
    //Serial.println("REVERSE");
    speed = getSpeedTrig(axis2);
    setMotor(RL_IN1, RL_IN2, RL_PWM, -1, speed);
    setMotor(RR_IN1, RR_IN2, RR_PWM, -1, speed);
    setMotor(FL_IN1, FL_IN2, FL_PWM, -1, speed);
    setMotor(FR_IN1, FR_IN2, FR_PWM, -1, speed);
    Serial.println("REVERSE:" + String(speed));
  }


  else if (axis0 > 0.040) {
    //Serial.println("MOVE RIGHT");
    speed = getSpeedJoy(axis0);
    setMotor(RL_IN1, RL_IN2, RL_PWM, -1, speed);
    setMotor(FL_IN1, FL_IN2, FL_PWM, 1, speed);
    setMotor(RR_IN1, RR_IN2, RR_PWM, 1, speed);
    setMotor(FR_IN1, FR_IN2, FR_PWM, -1, speed);
    Serial.println("MOVE RIGHT:" + String(speed));
  }

  else if (axis0 < -0.040) {
    //Serial.println("MOVE LEFT");
    speed = getSpeedJoy(axis0);
    setMotor(RL_IN1, RL_IN2, RL_PWM, 1, speed);
    setMotor(FL_IN1, FL_IN2, FL_PWM, -1, speed);
    setMotor(RR_IN1, RR_IN2, RR_PWM, -1, speed);
    setMotor(FR_IN1, FR_IN2, FR_PWM, 1, speed);
    Serial.println("MOVE LEFT:" + String(speed));
  }

  // 4. Spin Left (Stick pushed far left)
  else if (axis3 < -0.045) {
    //Serial.println("360 LEFT");
    speed = getSpeedJoy(axis3);
    setMotor(RL_IN1, RL_IN2, RL_PWM, -1, speed);  // Left side backwards
    setMotor(FL_IN1, FL_IN2, FL_PWM, -1, speed);
    setMotor(RR_IN1, RR_IN2, RR_PWM, 1, speed);  // Right side forwards
    setMotor(FR_IN1, FR_IN2, FR_PWM, 1, speed);
    Serial.println("360 LEFT:" + String(speed));
  }

  // 5. Spin Right (Stick pushed far right)
  else if (axis3 > 0.045) {
    //Serial.println("360 RIGHT");
    speed = getSpeedJoy(axis3);
    setMotor(RL_IN1, RL_IN2, RL_PWM, 1, speed);  // Left side forwards
    setMotor(FL_IN1, FL_IN2, FL_PWM, 1, speed);
    setMotor(RR_IN1, RR_IN2, RR_PWM, -1, speed);  // Right side backwards
    setMotor(FR_IN1, FR_IN2, FR_PWM, -1, speed);
    Serial.println("360 RIGHT:" + String(speed));
  }

  // 10 Hz güncelleme (100ms)
  delay(100);


}


void robotEnd() {
  // Robot durdurulduğunda çalışır
delay(34);
fanidur();

}


void ileri(int pwm){

digitalWrite(FL_IN1, HIGH);
digitalWrite(FL_IN2, LOW);

digitalWrite(FR_IN1, HIGH);
digitalWrite(FR_IN2, LOW);

digitalWrite(RL_IN1, HIGH);
digitalWrite(RL_IN2, LOW);

digitalWrite(RR_IN1, HIGH);
digitalWrite(RR_IN2, LOW);

analogWrite(FL_PWM, pwm);
analogWrite(FR_PWM, pwm);
analogWrite(RL_PWM, pwm);
analogWrite(RR_PWM, pwm);

}

void geri(int pwm){

digitalWrite(FL_IN1, LOW);
digitalWrite(FL_IN2, HIGH);

digitalWrite(FR_IN1, LOW);
digitalWrite(FR_IN2, HIGH);

digitalWrite(RL_IN1, LOW);
digitalWrite(RL_IN2, HIGH);

digitalWrite(RR_IN1, LOW);
digitalWrite(RR_IN2, HIGH);

analogWrite(FL_PWM, pwm);
analogWrite(FR_PWM, pwm);
analogWrite(RL_PWM, pwm);
analogWrite(RR_PWM, pwm);

}

void sagslide(int pwm){

digitalWrite(RR_IN1, HIGH);
digitalWrite(RR_IN2, LOW);

digitalWrite(FR_IN1, LOW);
digitalWrite(FR_IN2, HIGH);

digitalWrite(FL_IN1, HIGH);
digitalWrite(FL_IN2, LOW);

digitalWrite(RL_IN1, LOW);
digitalWrite(RL_IN2, HIGH);

analogWrite(FL_PWM, pwm);
analogWrite(FR_PWM, pwm);
analogWrite(RL_PWM, pwm);
analogWrite(RR_PWM, pwm);

}

void solslide(int pwm){

digitalWrite(RR_IN1, LOW);
digitalWrite(RR_IN2, HIGH);

digitalWrite(FR_IN1, HIGH);
digitalWrite(FR_IN2, LOW);

digitalWrite(FL_IN1, HIGH);
digitalWrite(FL_IN2, LOW);

digitalWrite(RL_IN1, LOW);
digitalWrite(RL_IN2, HIGH);

analogWrite(FL_PWM, pwm);
analogWrite(FR_PWM, pwm);
analogWrite(RL_PWM, pwm);
analogWrite(RR_PWM, pwm);

}

void sol360(int pwm){

digitalWrite(FL_IN1, HIGH);
digitalWrite(FL_IN2, LOW);

digitalWrite(FR_IN1, LOW);
digitalWrite(FR_IN2, HIGH);

digitalWrite(RL_IN1, HIGH);
digitalWrite(RL_IN2, LOW);

digitalWrite(RR_IN1, LOW);
digitalWrite(RR_IN2, HIGH);

analogWrite(FL_PWM, pwm);
analogWrite(FR_PWM, pwm);
analogWrite(RL_PWM, pwm);
analogWrite(RR_PWM, pwm);

}

void sag360(int pwm){

digitalWrite(FL_IN1, HIGH);
digitalWrite(FL_IN2, LOW);

digitalWrite(FR_IN1, HIGH);
digitalWrite(FR_IN2, LOW);

digitalWrite(RL_IN1, HIGH);
digitalWrite(RL_IN2, LOW);

digitalWrite(RR_IN1, HIGH);
digitalWrite(RR_IN2, LOW);

analogWrite(FL_PWM, pwm);
analogWrite(FR_PWM, pwm);
analogWrite(RL_PWM, pwm);
analogWrite(RR_PWM, pwm);

}

void dur(){

digitalWrite(FL_IN1, LOW);
digitalWrite(FL_IN2, LOW);

digitalWrite(FR_IN1, LOW);
digitalWrite(FR_IN2, LOW);

digitalWrite(RL_IN1, LOW);
digitalWrite(RL_IN2, LOW);

digitalWrite(RR_IN1, LOW);
digitalWrite(RR_IN2, LOW);

analogWrite(FL_PWM, 0);
analogWrite(FR_PWM, 0);
analogWrite(RL_PWM, 0);
analogWrite(RR_PWM, 0);

}

void anidur(){

digitalWrite(FL_IN1, HIGH);
digitalWrite(FL_IN2, HIGH);

digitalWrite(FR_IN1, HIGH);
digitalWrite(FR_IN2, HIGH);

digitalWrite(RL_IN1, HIGH);
digitalWrite(RL_IN2, HIGH);

digitalWrite(RR_IN1, HIGH);
digitalWrite(RR_IN2, HIGH);

analogWrite(FL_PWM, 255);
analogWrite(FR_PWM, 255);
analogWrite(RL_PWM, 255);
analogWrite(RR_PWM, 255);

delay(30);

digitalWrite(FL_IN1, LOW);
digitalWrite(FL_IN2, LOW);

digitalWrite(FR_IN1, LOW);
digitalWrite(FR_IN2, LOW);

digitalWrite(RL_IN1, LOW);
digitalWrite(RL_IN2, LOW);

digitalWrite(RR_IN1, LOW);
digitalWrite(RR_IN2, LOW);

analogWrite(FL_PWM, 0);
analogWrite(FR_PWM, 0);
analogWrite(RL_PWM, 0);
analogWrite(RR_PWM, 0);

}

void sagon(){
digitalWrite(FR_IN1, HIGH);
digitalWrite(FR_IN2, LOW);

analogWrite(FR_PWM, 255);

}

void solon(){
digitalWrite(FL_IN1, HIGH);
digitalWrite(FL_IN2, LOW);

analogWrite(FL_PWM, 255);

}

void sagarka(){
digitalWrite(RR_IN1, HIGH);
digitalWrite(RR_IN2, LOW);

analogWrite(RR_PWM, 255);

}

void solarka(){
digitalWrite(RL_IN1, HIGH);
digitalWrite(RL_IN2, LOW);

analogWrite(RL_PWM, 255);

}

void eup(int pwma){

digitalWrite(E_IN1, HIGH);
digitalWrite(E_IN2, LOW);

analogWrite(E_PWM, pwma);

}

void edown(int pwma){

digitalWrite(E_IN1, LOW);
digitalWrite(E_IN2, HIGH);

analogWrite(E_PWM, pwma);

}

void elit(){

  analogWrite(RL_PWM, 255);

}

void belit(){

  analogWrite(RL_PWM, 255);
}

void fanidur(){

digitalWrite(FL_IN1, HIGH);
digitalWrite(FL_IN2, HIGH);

digitalWrite(FR_IN1, HIGH);
digitalWrite(FR_IN2, HIGH);

digitalWrite(RL_IN1, HIGH);
digitalWrite(RL_IN2, HIGH);

digitalWrite(RR_IN1, HIGH);
digitalWrite(RR_IN2, HIGH);

digitalWrite(E_IN1, HIGH);
digitalWrite(E_IN2, HIGH);

analogWrite(FL_PWM, 255);
analogWrite(FR_PWM, 255);
analogWrite(RL_PWM, 255);
analogWrite(RR_PWM, 255);
analogWrite(E_PWM, 255);

delay(30);

digitalWrite(FL_IN1, LOW);
digitalWrite(FL_IN2, LOW);

digitalWrite(FR_IN1, LOW);
digitalWrite(FR_IN2, LOW);

digitalWrite(RL_IN1, LOW);
digitalWrite(RL_IN2, LOW);

digitalWrite(RR_IN1, LOW);
digitalWrite(RR_IN2, LOW);

digitalWrite(E_IN1, LOW);
digitalWrite(E_IN2, LOW);

analogWrite(FL_PWM, 0);
analogWrite(FR_PWM, 0);
analogWrite(RL_PWM, 0);
analogWrite(RR_PWM, 0);
analogWrite(E_PWM, 0);


}


//veeee
//son
