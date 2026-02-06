// http://192.168.4.1
#define PROBOT_WIFI_AP_PASSWORD "ProBot1234"

#include <probot.h>
#include <probot/io/joystick_api.hpp>
#include <probot/command.hpp>  // Includes scheduler, command, subsystem, command_group
#include <probot/command/examples/mecanum_drive.hpp>
#include <probot/devices/motors/boardoza_vnh5019_motor_controller.hpp>

const int RL_PWM = 7; 
const int RL_IN1 = 8; 
const int RL_IN2 = 9;

const int RR_PWM = 12; 
const int RR_IN1 = 13; 
const int RR_IN2 = 11;

const int FL_PWM = 15; 
const int FL_IN1 = 16; 
const int FL_IN2 = 17;

const int FR_PWM = 4; 
const int FR_IN1 = 5; 
const int FR_IN2 = 6;

const int EL_PWM = 18; 
const int EL_IN1 = 38; 
const int EL_IN2 = 39;

const int ER_PWM = 1; 
const int ER_IN1 = 2; 
const int ER_IN2 = 42;

const int E_LID = 41;
const int B_LID = 14;

// Mecanum surus icin dort motorun pin atamalari.
struct MotorPins {
  int ina, inb, pwm, ena, enb;
};

static constexpr MotorPins PINS_FL{FL_IN1, FL_IN2, FL_PWM, -1, -1};
static constexpr MotorPins PINS_FR{FR_IN1, FR_IN2, FR_PWM, -1, -1};
static constexpr MotorPins PINS_RL{RL_IN1, RL_IN2, RL_PWM, -1, -1};
static constexpr MotorPins PINS_RR{RR_IN1, RR_IN2, RR_PWM, -1, -1};

static probot::motor::BoardozaVNH5019MotorController drvFL(PINS_FL.ina, PINS_FL.inb, PINS_FL.pwm, PINS_FL.ena, PINS_FL.enb);
static probot::motor::BoardozaVNH5019MotorController drvFR(PINS_FR.ina, PINS_FR.inb, PINS_FR.pwm, PINS_FR.ena, PINS_FR.enb);
static probot::motor::BoardozaVNH5019MotorController drvRL(PINS_RL.ina, PINS_RL.inb, PINS_RL.pwm, PINS_RL.ena, PINS_RL.enb);
static probot::motor::BoardozaVNH5019MotorController drvRR(PINS_RR.ina, PINS_RR.inb, PINS_RR.pwm, PINS_RR.ena, PINS_RR.enb);

static probot::command::examples::MecanumDrive mecanum(&drvFL, &drvFR, &drvRL, &drvRR);

using Scheduler = probot::command::Scheduler;

void robotInit() {
  Serial.begin(115200);
  delay(100);

  drvFL.begin(); drvFR.begin(); drvRL.begin(); drvRR.begin();
  drvFL.setBrakeMode(true);
  drvFR.setBrakeMode(true);
  drvRL.setBrakeMode(true);
  drvRR.setBrakeMode(true);

  mecanum.setInverted(false, true, false, true); // sag taraf ters kabloluysa duzelt
  mecanum.setWheelBase(30.0f);
  mecanum.setTrackWidth(28.0f);

  // Yeni API: Subsystem'i scheduler'a kaydet
  Scheduler::instance().registerSubsystem(&mecanum);
  Serial.println("[MecanumDriveDemo] robotInit: Mecanum suruse hazir");
}

void robotEnd() {
  Scheduler::instance().unregisterSubsystem(&mecanum);
  mecanum.stop();
  Serial.println("[MecanumDriveDemo] robotEnd: Motorlar kapatildi");
}

void teleopInit() {
  Serial.println("[MecanumDriveDemo] teleopInit:"
                 " sol cubuk Y ileri-geri, X yan, sag X donus");
}

void teleopLoop() {
  auto js = probot::io::joystick_api::makeDefault();

  float vx = js.getLeftY();    // ileri geri
  float vy = js.getLeftX();    // yan hareket
  float omega = js.getRightX();// donus

  mecanum.drivePower(vx, vy, omega);

  Serial.printf("[MecanumDriveDemo] vx=%.2f vy=%.2f w=%.2f fl=%.2f fr=%.2f rl=%.2f rr=%.2f\n",
                vx, vy, omega,
                drvFL.getPower(), drvFR.getPower(),
                drvRL.getPower(), drvRR.getPower());

  delay(20);
}

void autonomousInit() {
  Serial.println("[MecanumDriveDemo] autonomousInit: kare rota denemesi");
}

void autonomousLoop() {
  static uint32_t start = millis();
  uint32_t elapsed = millis() - start;
  float vx = 0.0f, vy = 0.0f, omega = 0.0f;

  if (elapsed < 2000) {
    vx = 0.6f; // ileri
  } else if (elapsed < 4000) {
    vy = 0.6f; // saga kay
  } else if (elapsed < 6000) {
    vx = -0.6f; // geri
  } else if (elapsed < 8000) {
    vy = -0.6f; // sola kay
  } else {
    start = millis();
  }

  mecanum.drivePower(vx, vy, omega);
  delay(20);
}
