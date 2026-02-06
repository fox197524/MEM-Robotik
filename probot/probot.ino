// 192.168.4.1
#define PROBOT_WIFI_AP_PASSWORD "kayra123"
#define PROBOT_WIFI_AP_SSID "MECHATAK"
#define PROBOT_WIFI_AP_CHANNEL 3
#include <probot.h>

// http://192.168.4.1
#include <probot/devices/servo/servo.hpp>


// --- Servo pins (choose actual GPIOs for your hardware) ---
const int SERVO_BOTTOM_PIN = 21;  // example pin for bottom lid
const int SERVO_TOP_PIN    = 22;  // example pin for top lid

// --- Servo objects ---
probot::devices::Servo bottomLidServo(SERVO_BOTTOM_PIN);
probot::devices::Servo topLidServo(SERVO_TOP_PIN);

// --- Servo states ---
bool bottomLidOpen = false;
bool topLidOpen = false;

// --- Track button press edges ---
bool lastBtn0State = false;
bool lastBtn3State = false;

void robotInit() {
  Serial.begin(115200);
  delay(100);

  // Initialize all motors
  drvFL.begin(); drvFR.begin(); drvRL.begin(); drvRR.begin();
  drvEL.begin();

  drvFL.setBrakeMode(true); drvFR.setBrakeMode(true);
  drvRL.setBrakeMode(true); drvRR.setBrakeMode(true);
  drvEL.setBrakeMode(true);

  // Initialize servos
  bottomLidServo.begin();
  topLidServo.begin();
  bottomLidServo.write(0);   // start closed
  topLidServo.write(0);      // start closed

  mecanum.setInverted(false, true, false, true);
  mecanum.setWheelBase(30.0f);
  mecanum.setTrackWidth(28.0f);

  Scheduler::instance().registerSubsystem(&mecanum);
  Serial.println("[MecanumDriveDemo] robotInit: Mecanum + Elevator + Servos hazir");
}

void teleopLoop() {
  auto js = probot::io::joystick_api::makeDefault();

  // --- Mecanum drive mapping ---
  float omega = js.getAxis(0);   // Left stick X = rotation
  float vy    = js.getAxis(2);   // Axis 2 = strafe
  float rt    = js.getTriggerRight(); // RT forward
  float lt    = js.getTriggerLeft();  // LT backward
  float vx    = rt - lt;              // net forward/backward

  mecanum.drivePower(vx, vy, omega);

  // --- Elevator control (same as before) ---
  unsigned long currentMillis = millis();
  float elevatorMotorCommand = 0.0f;

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

  if (js.getRawButton(13) && elevatorDoorPulseStart == 0) {
    if (elevatorUpRemainingBudget > 0) {
      elevatorMotorCommand = ELEVATOR_SPEED;
      elevatorUpRemainingBudget -= 20;
    } else {
      Serial.println("[ELEVATOR] Up budget exhausted.");
    }
  } else if (js.getRawButton(14) && elevatorDoorPulseStart == 0) {
    if (elevatorDownRemainingBudget > 0) {
      elevatorMotorCommand = -ELEVATOR_SPEED;
      elevatorDownRemainingBudget -= 20;
    } else {
      Serial.println("[ELEVATOR] Down budget exhausted.");
    }
  }

  drvEL.setPower(elevatorMotorCommand);

  // --- Servo toggle logic ---
  bool btn0 = js.getRawButton(0);
  bool btn3 = js.getRawButton(3);

  // Bottom lid toggle
  if (btn0 && !lastBtn0State) { // detect rising edge
    bottomLidOpen = !bottomLidOpen;
    bottomLidServo.write(bottomLidOpen ? 180 : 0); // open fully or close
    Serial.printf("[SERVO] Bottom lid %s\n", bottomLidOpen ? "OPEN" : "CLOSED");
  }
  lastBtn0State = btn0;

  // Top lid toggle
  if (btn3 && !lastBtn3State) { // detect rising edge
    topLidOpen = !topLidOpen;
    topLidServo.write(topLidOpen ? 180 : 0);
    Serial.printf("[SERVO] Top lid %s\n", topLidOpen ? "OPEN" : "CLOSED");
  }
  lastBtn3State = btn3;

  // --- Telemetry ---
  Serial.printf("[Mecanum] vx=%.2f vy=%.2f w=%.2f | "
                "[Elevator] pwr=%.2f | UpB:%.0f DownB:%.0f | "
                "[Servos] Bottom:%s Top:%s\n",
                vx, vy, omega, drvEL.getPower(),
                elevatorUpRemainingBudget, elevatorDownRemainingBudget,
                bottomLidOpen ? "OPEN" : "CLOSED",
                topLidOpen ? "OPEN" : "CLOSED");

  delay(20);
}
