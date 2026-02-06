// http://192.168.4.1
#define PROBOT_WIFI_AP_PASSWORD "kayra123"
#define PROBOT_WIFI_AP_CHANNEL 3
#define PROBOT_WIFI_AP_SSID "MECHATAK"



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

// Global variables for elevator control with cumulative time budget
static unsigned long elevatorUpActiveStart = 0;
static unsigned long elevatorDownActiveStart = 0;
static float elevatorUpRemainingBudget = 3000.0f; // in milliseconds
static float elevatorDownRemainingBudget = 3000.0f; // in milliseconds
static unsigned long elevatorDoorPulseStart = 0; // To manage button 6 pulse
static const float ELEVATOR_SPEED = 0.6f;
static const float ELEVATOR_DOOR_PULSE_POWER = 0.2f;
static const unsigned long ELEVATOR_DOOR_PULSE_DURATION_MS = 100;

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
  drvEL.setPower(0.0f); // Replaced drvEL.stop() with setPower(0.0f)
  Serial.println("[MecanumDriveDemo] robotEnd: Tüm motorlar kapatildi");
}

void teleopInit() {
  Serial.println("[MecanumDriveDemo] teleopInit: "
                 "sol cubuk Y=ileri/geri X=yan, sag X=donus, "
                 "BTN13=elevator Yukarı, BTN14=elevator Aşağı");
}

void teleopLoop() {
  auto js = probot::io::joystick_api::makeDefault();

  // --- Mecanum drive mapping ---
  float omega = js.getAxis(0);   // Left stick X = rotation (360° turning)
  float vy    = js.getAxis(2);   // Axis 2 = left/right drift (strafe)

  // RT forward, LT backward
  float rt = js.getTriggerRight(); // RT
  float lt = js.getTriggerLeft();  // LT
  float vx = rt - lt;              // net forward/backward

  mecanum.drivePower(vx, vy, omega);

  // --- Elevator control ---
  unsigned long currentMillis = millis();
  float elevatorMotorCommand = 0.0f;

  // --- Handle Elevator Door (Button 6 pulse) ---
  if (js.getRawButton(6)) {
    if (elevatorDoorPulseStart == 0) {
      elevatorDoorPulseStart = currentMillis;
      Serial.println("[ELEVATOR] Door pulse started.");
    }
    elevatorMotorCommand = ELEVATOR_DOOR_PULSE_POWER;
  } else {
    if (elevatorDoorPulseStart != 0 &&
        (currentMillis - elevatorDoorPulseStart) < ELEVATOR_DOOR_PULSE_DURATION_MS) {
      elevatorMotorCommand = ELEVATOR_DOOR_PULSE_POWER;
    } else {
      elevatorDoorPulseStart = 0;
    }
  }

  // --- Handle Elevator Up (BTN13) ---
  if (js.getRawButton(13) && elevatorDoorPulseStart == 0) {
    if (elevatorUpRemainingBudget > 0) {
      elevatorMotorCommand = ELEVATOR_SPEED;
      elevatorUpRemainingBudget -= 20; // deduct ~20ms per loop
    } else {
      Serial.println("[ELEVATOR] Up budget exhausted.");
    }
  }

  // --- Handle Elevator Down (BTN14) ---
  else if (js.getRawButton(14) && elevatorDoorPulseStart == 0) {
    if (elevatorDownRemainingBudget > 0) {
      elevatorMotorCommand = -ELEVATOR_SPEED;
      elevatorDownRemainingBudget -= 20;
    } else {
      Serial.println("[ELEVATOR] Down budget exhausted.");
    }
  }

  // --- Apply final elevator command ---
  drvEL.setPower(elevatorMotorCommand);

  Serial.printf("[Mecanum] vx=%.2f vy=%.2f w=%.2f | "
                "[Elevator] pwr=%.2f | UpB:%.0f DownB:%.0f\n",
                vx, vy, omega, drvEL.getPower(),
                elevatorUpRemainingBudget, elevatorDownRemainingBudget);

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
