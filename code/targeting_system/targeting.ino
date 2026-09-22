/**
 * ============================================================================
 * Project Name : SENTINEL-X
 * Subsystem    : Autonomous Radar Target Tracking & Precision Alignment
 * Microcontroller: Arduino Uno (ATmega328P)
 * Description  : Implements a finite state machine (FSM) for continuous ultrasonic 
 *                sector scanning, polar-to-Cartesian trigonometric target 
 *                localization, smooth servo pitch alignment, and execution.
 * ============================================================================
 */

#include <Servo.h>
#include <SoftwareSerial.h>

// Finite State Machine (FSM) States
enum SystemState {
  STATE_STANDBY,
  STATE_SCANNING,
  STATE_TARGET_LOCKED,
  STATE_ALIGNING,
  STATE_ACTUATING
};

// System Control Variables
SystemState currentState = STATE_STANDBY;
bool systemEnabled = false;

// Hardware Pin Definitions
const int TRIG_PIN = 11;      // HC-SR04 Trigger Pin
const int ECHO_PIN = 12;      // HC-SR04 Echo Pin
const int ALERT_PIN = 13;     // Audio-Visual Warning Indicator (Buzzer/LED)

const int RADAR_SERVO_PIN = 9;   // Pan Servo (Ultrasonic Scanner)
const int PITCH_SERVO_PIN = 5;   // Pitch Alignment Servo
const int TRIGGER_SERVO_PIN = 6; // Actuator Release Servo

// Radar Sector Scan Parameters
const int SCAN_MIN_ANGLE = 10;   // Degrees
const int SCAN_MAX_ANGLE = 110;  // Degrees
const int DETECTION_THRESHOLD_CM = 30; // Target acquisition distance threshold

// Actuator Calibration Parameters (resting & active angles)
const int TRIGGER_REST_ANGLE = 100;
const int TRIGGER_ACTIVE_ANGLE = 70;
const int PITCH_CENTER_ANGLE = 90;

// Sensor & Targeting Data
long echoDurationUs = 0;
int measuredDistanceCm = 0;
int lockedTargetAngle = 0;
int lockedTargetDistance = 0;

// Servo Objects
Servo radarPanServo;
Servo pitchAimServo;
Servo triggerReleaseServo;

// Function Prototypes
void processBluetoothCommands();
void executeScanningCycle();
int measureDistance();
void calculateAndAlignTarget(int scanAngle, int distance);
void smoothPitchRotate(int targetAngle);
void triggerAlertSequence();
void executeActuationCycle();

void setup() {
  // Initialize Serial Communication for Telemetry & Bluetooth Control
  Serial.begin(9600);
  Serial.println(F("[SENTINEL-X] System Initializing..."));

  // Pin Modes
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(ALERT_PIN, OUTPUT);

  // Attach Servos
  radarPanServo.attach(RADAR_SERVO_PIN);
  pitchAimServo.attach(PITCH_SERVO_PIN);
  triggerReleaseServo.attach(TRIGGER_SERVO_PIN);

  // Set Servos to Safe Resting State
  radarPanServo.write(SCAN_MIN_ANGLE);
  pitchAimServo.write(PITCH_CENTER_ANGLE);
  triggerReleaseServo.write(TRIGGER_REST_ANGLE);

  delay(1000);
  Serial.println(F("[SENTINEL-X] Initialization Complete. System in STANDBY."));
}

void loop() {
  // Always poll for incoming Bluetooth telemetry commands
  processBluetoothCommands();

  // State Machine Execution
  switch (currentState) {
    case STATE_STANDBY:
      // Idle state waiting for enable signal ('O')
      break;

    case STATE_SCANNING:
      executeScanningCycle();
      break;

    case STATE_TARGET_LOCKED:
      Serial.println(F("[SENTINEL-X] State: TARGET_LOCKED. Calculating Kinematics..."));
      currentState = STATE_ALIGNING;
      break;

    case STATE_ALIGNING:
      calculateAndAlignTarget(lockedTargetAngle, lockedTargetDistance);
      currentState = STATE_ACTUATING;
      break;

    case STATE_ACTUATING:
      triggerAlertSequence();
      executeActuationCycle();
      
      // Return to scanning mode after engagement cycle completes
      if (systemEnabled) {
        currentState = STATE_SCANNING;
      } else {
        currentState = STATE_STANDBY;
      }
      break;
  }
}

/**
 * Polls hardware serial for incoming control characters
 */
void processBluetoothCommands() {
  if (Serial.available()) {
    char cmd = Serial.read();

    if (cmd == 'O' || cmd == 'o') {
      systemEnabled = true;
      currentState = STATE_SCANNING;
      Serial.println(F("[SENTINEL-X] COMMAND: Radar Scanning Enabled."));
    } 
    else if (cmd == 'F' || cmd == 'f') {
      systemEnabled = false;
      currentState = STATE_STANDBY;
      radarPanServo.write(SCAN_MIN_ANGLE);
      pitchAimServo.write(PITCH_CENTER_ANGLE);
      Serial.println(F("[SENTINEL-X] COMMAND: System Aborted / Standby."));
    } 
    else if (cmd == 'X' || cmd == 'x') {
      Serial.println(F("[SENTINEL-X] COMMAND: Manual Actuation Override Triggered."));
      executeActuationCycle();
    }
  }
}

/**
 * Sweeps the ultrasonic radar pan servo across the scanning arc
 */
void executeScanningCycle() {
  // Forward Sweep Arc (Min -> Max)
  for (int angle = SCAN_MIN_ANGLE; angle <= SCAN_MAX_ANGLE; angle += 2) {
    processBluetoothCommands();
    if (!systemEnabled) return;

    radarPanServo.write(angle);
    delay(25);

    measuredDistanceCm = measureDistance();

    if (measuredDistanceCm > 0 && measuredDistanceCm <= DETECTION_THRESHOLD_CM) {
      lockedTargetAngle = angle;
      lockedTargetDistance = measuredDistanceCm;
      currentState = STATE_TARGET_LOCKED;
      return;
    }
  }

  // Reverse Sweep Arc (Max -> Min)
  for (int angle = SCAN_MAX_ANGLE; angle >= SCAN_MIN_ANGLE; angle -= 2) {
    processBluetoothCommands();
    if (!systemEnabled) return;

    radarPanServo.write(angle);
    delay(25);

    measuredDistanceCm = measureDistance();

    if (measuredDistanceCm > 0 && measuredDistanceCm <= DETECTION_THRESHOLD_CM) {
      lockedTargetAngle = angle - 50; // Dynamic angle offset compensation
      lockedTargetDistance = measuredDistanceCm;
      currentState = STATE_TARGET_LOCKED;
      return;
    }
  }
}

/**
 * Measures distance in cm via ultrasonic pulse echo timing
 */
int measureDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  echoDurationUs = pulseIn(ECHO_PIN, HIGH, 30000); // 30ms timeout (~5m max range)

  if (echoDurationUs == 0) return 0; // Out of range or no echo

  return (int)(echoDurationUs * 0.0343 / 2.0);
}

/**
 * Performs coordinate transformation and aligns pitch servo to target
 */
void calculateAndAlignTarget(int scanAngleDeg, int targetDistCm) {
  float thetaScanRad = radians(scanAngleDeg);

  // Sensor-to-Actuator Coordinate Offsets (in cm)
  const float SENSOR_OFFSET_Y = 8.0; // Vertical offset
  const float SENSOR_OFFSET_X = 0.0; // Horizontal offset

  // Convert Polar Coordinates (r, theta) to Cartesian (x, y) relative to actuator pivot
  float targetX = (targetDistCm * cos(thetaScanRad)) - SENSOR_OFFSET_X;
  float targetY = (targetDistCm * sin(thetaScanRad)) - SENSOR_OFFSET_Y;

  // Compute Required Pitch Compensation Angle using arctan2
  float thetaPitchRad = atan2(targetY, targetX);
  int pitchAngleDeg = degrees(thetaPitchRad);

  Serial.print(F("[SENTINEL-X] Target Localized -> Angle: "));
  Serial.print(pitchAngleDeg);
  Serial.println(F(" deg. Aligning Servo..."));

  // Smoothly rotate pitch servo to target position
  smoothPitchRotate(pitchAngleDeg);
}

/**
 * Smoothly interpolates pitch servo movement to eliminate mechanical shock
 */
void smoothPitchRotate(int targetAngle) {
  int currentAngle = pitchAimServo.read();
  targetAngle = constrain(targetAngle + 30, 10, 170); // Constrain within safe servo bounds

  if (currentAngle < targetAngle) {
    for (int pos = currentAngle; pos <= targetAngle; pos++) {
      pitchAimServo.write(pos);
      delay(8);
    }
  } else {
    for (int pos = currentAngle; pos >= targetAngle; pos--) {
      pitchAimServo.write(pos);
      delay(8);
    }
  }
}

/**
 * Emits an audio-visual warning sequence prior to actuation
 */
void triggerAlertSequence() {
  Serial.println(F("[SENTINEL-X] Issuing Pre-Actuation Audio-Visual Alert..."));
  for (int i = 0; i < 5; i++) {
    digitalWrite(ALERT_PIN, HIGH);
    delay(120);
    digitalWrite(ALERT_PIN, LOW);
    delay(120);
  }
}

/**
 * Triggers payload release/actuation mechanism and resets to resting state
 */
void executeActuationCycle() {
  Serial.println(F("[SENTINEL-X] Executing Actuation Cycle..."));
  triggerReleaseServo.write(TRIGGER_ACTIVE_ANGLE);
  delay(500);
  triggerReleaseServo.write(TRIGGER_REST_ANGLE);
  Serial.println(F("[SENTINEL-X] Actuation Complete. Resetting to Standby."));
}
