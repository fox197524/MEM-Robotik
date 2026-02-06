#define PROBOT_WIFI_AP_PASSWORD "kayra123"
#define PROBOT_WIFI_AP_SSID "MECHATAK"
#define PROBOT_WIFI_AP_CHANNEL 3
#include <probot.h>

// http://192.168.4.1

#include <probot.h>
#include <probot/io/joystick_api.hpp>
#include <probot/command.hpp>
#include <probot/command/examples/mecanum_drive.hpp>
#include <probot/devices/motors/boardoza_vnh5019_motor_controller.hpp>

// Mecanum wheel motors
const int RL_PWM = 7; const int RL_IN1 = 8; const int RL_IN2 = 9;
const int RR_PWM = 12; const int RR_IN1 = 13; const int RR_IN2 = 11;
const int FL_PWM = 15; const int FL_IN1 = 16; const int FL_IN2 = 17;
const int FR_PWM = 4; const int FR_IN1 = 5; const int FR_IN2 = 6;

// Elevator motor pins
const int EL_PWM = 18; const int EL_IN1 = 38; const int EL_IN2 = 39;

const int E_LID = 41;
const int B_LID = 14;

struct MotorPins {
  int ina, inb, pwm, ena, enb;
};

static constexpr MotorPins PINS_FL{FL_IN1, FL_IN2, FL_PWM, -1, -1};
static constexpr MotorPins PINS_FR{FR_IN1, FR_IN2, FR_PWM, -1, -1};
static constexpr MotorPins PINS_RL{RL_IN1, RL_IN2, RL_PWM, -1, -1};
static constexpr MotorPins PINS_RR{RR_IN1, RR_IN2, RR_PWM, -1, -1};

// Elevator motor controller
static constexpr MotorPins PINS_EL{EL_IN1, EL_IN2, EL_PWM, -1, -1};
static probot::motor::BoardozaVNH5019MotorController drvEL(PINS_EL.ina, PINS_EL.inb, PINS_EL.pwm, PINS_EL.ena, PINS_EL.enb);

static probot::command::examples::MecanumDrive mecanum(&drvFL, &drvFR, &drvRL, &drvRR);
using Scheduler = probot::command::Scheduler;

void robotInit() {
  Serial.begin(115200);
  delay(100);

  // Initialize all motors
  drvFL.begin(); drvFR.begin(); drvRL.begin(); drvRR.begin();
  drvEL.begin();
  
  drvFL.setBrakeMode(true); drvFR.setBrakeMode(true);
  drvRL.setBrakeMode(true); drvRR.setBrakeMode(true);
  drvEL.setBrakeMode(true);

  mecanum.setInverted(false, true, false, true);
  mecanum.setWheelBase(30.0f);
  mecanum.setTrackWidth(28.0f);

  Scheduler::instance().registerSubsystem(&mecanum);
  Serial.println("[MecanumDriveDemo] robotInit: Mecanum + Elevator hazir");
}

void robotEnd() {
  Scheduler::instance().unregisterSubsystem(&mecanum);
  mecanum.stop();
  drvEL.stop();
  Serial.println("[MecanumDriveDemo] robotEnd: Tüm motorlar kapatildi");
}

void teleopInit() {
  Serial.println("[MecanumDriveDemo] teleopInit: "
                 "sol cubuk Y=ileri/geri X=yan, sag X=donus, "
                 "BTN13=elevator Yukarı, BTN14=elevator Aşağı");
}

void teleopLoop() {
  auto js = probot::io::joystick_api::makeDefault();

  // Mecanum drive
  float vx = js.getLeftY();
  float vy = js.getLeftX();
  float omega = js.getRightX();
  mecanum.drivePower(vx, vy, omega);

  // Elevator control - Button 13 UP, Button 14 DOWN
  if (js.getButton(13)) {
    drvEL.setPower(0.6f);  // IN1 HIGH, IN2 LOW = UP
    Serial.println("[ELEVATOR] Yukari");
  } 
  else if (js.getButton(14)) {
    drvEL.setPower(-0.6f); // IN1 LOW, IN2 HIGH = DOWN
    Serial.println("[ELEVATOR] Asagi");
  } 
  else {
    drvEL.setPower(0.0f);  // STOP
  }

  Serial.printf("[Mecanum] vx=%.2f vy=%.2f w=%.2f | "
                "[Elevator] pwr=%.2f\n",
                vx, vy, omega, drvEL.getPower());

  delay(20);
}

void autonomousInit() {
  Serial.println("[MecanumDriveDemo] autonomousInit: kare rota");
}

void autonomousLoop() {
  static uint32_t start = millis();
  uint32_t elapsed = millis() - start;
  float vx = 0.0f, vy = 0.0f, omega = 0.0f;

  if (elapsed < 2000) vx = 0.6f;
  else if (elapsed < 4000) vy = 0.6f;
  else if (elapsed < 6000) vx = -0.6f;
  else if (elapsed < 8000) vy = -0.6f;
  else start = millis();

  mecanum.drivePower(vx, vy, omega);
  delay(20);
}

void otonom() {
  // Custom autonomous function - elevator test
  drvEL.setPower(0.6f);
  delay(1000);
  drvEL.setPower(0.0f);
}
