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

  // Mecanum drive - PS Controller Mapping
  // axis 5 for forward, axis 4 for backward, axis 2 for left/right slide, axis 0 for rotation
  float vx = js.getLeftY();             // Forward/Backward (mapped to Left Y-axis)
  float vy = js.getLeftX();             // Left/Right Slide (mapped to Left X-axis, user requested axis 2)
  float omega = js.getRightX();         // 360 Turning (mapped to Right X-axis, user requested axis 0)
  // Read raw trigger axes (L2/R2) if available and use them to spin all motors
  float axis4 = 0.0f;
  float axis5 = 0.0f;
  // Many joystick implementations expose triggers as axes 4 and 5.
  // Use getRawAxis if available in the joystick API; if not, these calls
  // will cause a compile error and we'll need to adjust to the library.
  axis4 = js.getRawAxis(4);
  axis5 = js.getRawAxis(5);

  // If R2 (axis5) pressed -> spin all motors forward; L2 (axis4) -> spin backward
  const float TRIGGER_DEADZONE = 0.05f;
  if (axis5 > TRIGGER_DEADZONE) {
    float p = axis5; // scale power by trigger value
    drvFL.setPower(p);
    drvFR.setPower(p);
    drvRL.setPower(p);
    drvRR.setPower(p);
  } else if (axis4 > TRIGGER_DEADZONE) {
    float p = -axis4;
    drvFL.setPower(p);
    drvFR.setPower(p);
    drvRL.setPower(p);
    drvRR.setPower(p);
  } else {
    mecanum.drivePower(vx, vy, omega);
  }

  unsigned long currentMillis = millis();
  float elevatorMotorCommand = 0.0f; // This will be the final power for drvEL

  // --- Handle Elevator Door (Button 6) ---
  // If button 6 is pressed, or if a door pulse is active
  if (js.getRawButton(6)) {
    if (elevatorDoorPulseStart == 0) { // Start a new pulse
      elevatorDoorPulseStart = currentMillis;
      Serial.println("[ELEVATOR] Kapi aciliyor/kapaniyor (darbe)");
    }
    // If button 6 is held, keep the motor at door pulse power
    elevatorMotorCommand = ELEVATOR_DOOR_PULSE_POWER;
  } else {
    // If button 6 is released, and a pulse was active, stop it after duration
    if (elevatorDoorPulseStart != 0 && (currentMillis - elevatorDoorPulseStart) < ELEVATOR_DOOR_PULSE_DURATION_MS) {
      elevatorMotorCommand = ELEVATOR_DOOR_PULSE_POWER; // Continue pulse
    } else {
      elevatorDoorPulseStart = 0; // Reset pulse if not active or duration passed
    }
  }

  // --- Handle Elevator Up (Button 13) ---
  if (js.getRawButton(13) && elevatorDoorPulseStart == 0) { // Only if door is not active
    if (elevatorUpActiveStart == 0) { // Button just pressed, start tracking
      elevatorUpActiveStart = currentMillis;
      Serial.println("[ELEVATOR] Yukari baslatildi.");
    }

    // Calculate elapsed time since last check, or since button press
    unsigned long durationSinceLastCheck = currentMillis - elevatorUpActiveStart;

    if (elevatorUpRemainingBudget > 0) {
      elevatorMotorCommand = ELEVATOR_SPEED;
      elevatorUpRemainingBudget -= durationSinceLastCheck; // Deduct from budget
      if (elevatorUpRemainingBudget < 0) elevatorUpRemainingBudget = 0; // Cap at 0
    } else {
      Serial.println("[ELEVATOR] Yukari - Butce tukendi.");
    }
    elevatorUpActiveStart = currentMillis; // Reset for next iteration
  } else { // Button 13 not pressed
    if (elevatorUpActiveStart != 0) { // If it was moving up and button released
      unsigned long durationSinceLastCheck = currentMillis - elevatorUpActiveStart;
      elevatorUpRemainingBudget -= durationSinceLastCheck;
      if (elevatorUpRemainingBudget < 0) elevatorUpRemainingBudget = 0;
      Serial.printf("[ELEVATOR] Yukari - Buton birakildi. Kalan butce: %.0f ms\n", elevatorUpRemainingBudget);
      elevatorUpActiveStart = 0; // Reset active start time
    }
  }

  // --- Handle Elevator Down (Button 14) ---
  if (js.getRawButton(14) && elevatorDoorPulseStart == 0) { // Only if door is not active
    if (elevatorDownActiveStart == 0) { // Button just pressed, start tracking
      elevatorDownActiveStart = currentMillis;
      Serial.println("[ELEVATOR] Asagi baslatildi.");
    }

    unsigned long durationSinceLastCheck = currentMillis - elevatorDownActiveStart;

    if (elevatorDownRemainingBudget > 0) {
      elevatorMotorCommand = -ELEVATOR_SPEED; // Negative for down
      elevatorDownRemainingBudget -= durationSinceLastCheck; // Deduct from budget
      if (elevatorDownRemainingBudget < 0) elevatorDownRemainingBudget = 0; // Cap at 0
    } else {
      Serial.println("[ELEVATOR] Asagi - Butce tukendi.");
    }
    elevatorDownActiveStart = currentMillis; // Reset for next iteration
  } else { // Button 14 not pressed
    if (elevatorDownActiveStart != 0) { // If it was moving down and button released
      unsigned long durationSinceLastCheck = currentMillis - elevatorDownActiveStart;
      elevatorDownRemainingBudget -= durationSinceLastCheck;
      if (elevatorDownRemainingBudget < 0) elevatorDownRemainingBudget = 0;
      Serial.printf("[ELEVATOR] Asagi - Buton birakildi. Kalan butce: %.0f ms\n", elevatorDownRemainingBudget);
      elevatorDownActiveStart = 0; // Reset active start time
    }
  }

  // Apply the final elevator motor command unless a door pulse is active and not finished
  if (elevatorDoorPulseStart != 0 && (currentMillis - elevatorDoorPulseStart) < ELEVATOR_DOOR_PULSE_DURATION_MS) {
    // Door pulse is ongoing, elevatorMotorCommand is already set by door logic
  } else {
    // If no active door pulse, and no up/down buttons pressed (or budget exhausted), stop motor
    if (!js.getRawButton(13) && !js.getRawButton(14)) {
      elevatorMotorCommand = 0.0f;
    }
  }
  
  drvEL.setPower(elevatorMotorCommand);

  Serial.printf("[Mecanum] vx=%.2f vy=%.2f w=%.2f | "
                "[Elevator] pwr=%.2f | UpB:%.0f DownB:%.0f\n",
                vx, vy, omega, drvEL.getPower(), elevatorUpRemainingBudget, elevatorDownRemainingBudget);
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
