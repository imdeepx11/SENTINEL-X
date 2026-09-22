/**
 * ============================================================================
 * Project Name : SENTINEL-X
 * Module       : Autonomous Radar Target Tracking & Alignment System
 * Description  : Performs continuous ultrasonic radar sweep, calculates target 
 *                coordinates via trigonometric transformation, aligns pitch 
 *                actuator servo, and executes response mechanism.
 * Microcontroller: Arduino Uno (ATmega328P)
 * ============================================================================
 */

#include <Servo.h>
#include <SoftwareSerial.h>

// System Operation Flag (controlled via Bluetooth serial commands)
bool runFlag = false;

// HC-SR04 Ultrasonic Sensor Pin Definitions
const int trigPin = 11;
const int echoPin = 12;

// Alert Indicator Pin (Buzzer / LED)
const int alertPin = 13;

// Servo Scanning Angle Limits (Degrees)
const int L_angle = 110;
const int R_angle = 10;

// Sensor Measurement Variables
long duration;
int distance;

// Target Engagement State
bool hasEngaged = false;

// Servo Motor Instances
Servo radarServo;    // Sweeps ultrasonic sensor (Radar scanning)
Servo aimServo;      // Controls pitch / alignment of actuator platform
Servo actuatorServo; // Triggers execution mechanism

// Forward Function Declarations
void loopCode();
bool checkForOffBT();
int calculateDistance();
void engageTarget(int radarAngleDeg, int targetDistance);
void smoothAimTo(int targetAngle);
void alertSignal();

void setup() {
  // Configure Ultrasonic Sensor Pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Configure Audio Visual Alert Output
  pinMode(alertPin, OUTPUT);

  // Initialize Hardware Serial for Telemetry & Bluetooth Communication (9600 Baud)
  Serial.begin(9600);

  // Attach Servo Motors to Designated Digital PWM Pins
  radarServo.attach(9);
  aimServo.attach(5);
  actuatorServo.attach(6);

  // Initialize Servo Positions
  radarServo.write(20);
  actuatorServo.write(100);  // Idle / Standby Position
  aimServo.write(90);        // Centered Pitch Alignment
  
  delay(1000);
}

void loop() {
  // Check for incoming Bluetooth control commands (Non-blocking)
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'O' || c == 'o') {        // Command: Activate Radar Sweep
      runFlag = true;
      Serial.println("SYSTEM_ENABLE: Scanning Started");
    } else if (c == 'F' || c == 'f') { // Command: Deactivate Scanning
      runFlag = false;
      Serial.println("SYSTEM_DISABLE: Scanning Paused");
    }
  }

  // Execute continuous radar sweep if activated
  if (runFlag) {
    loopCode();
  }
}

/**
 * Main Radar Scanning Loop
 * Sweeps the ultrasonic sensor back and forth while scanning for target objects.
 */
void loopCode() {
  if (!runFlag) return;

  // Forward Sweep: Right Angle -> Left Angle
  for (int i = R_angle; i <= L_angle; i++) {
    if (checkForOffBT()) return; // Abort scan if disable command received

    radarServo.write(i);
    delay(20);

    distance = calculateDistance();

    // Check if an object enters detection threshold (< 30 cm)
    if (distance > 0 && distance < 30) {
      Serial.println("ALERT: Target Detected within Range!");
      engageTarget(i, distance);
    }
  }

  // Reverse Sweep: Left Angle -> Right Angle
  for (int i = L_angle; i >= R_angle; i--) {
    if (checkForOffBT()) return;

    radarServo.write(i);
    delay(20);

    distance = calculateDistance();

    if (distance > 0 && distance < 30) {
      Serial.println("ALERT: Target Detected within Range!");
      engageTarget(i - 50, distance);
    }
  }
}

/**
 * Checks serial buffer for interrupt commands or manual override
 */
bool checkForOffBT() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'F' || c == 'f') {
      runFlag = false;
      Serial.println("SYSTEM_ABORT: Received Pause Command");
      return true;
    }
    if (c == 'x' || c == 'X') { // Manual Actuation Override Command
      actuatorServo.write(70);   // Trigger Actuation
      delay(500);
      actuatorServo.write(100);  // Reset to Standby
      Serial.println("MANUAL_OVERRIDE: Actuation Triggered");
    }
  }
  return false;
}

/**
 * Measures distance using ultrasonic sound pulse echo (Returns distance in cm)
 */
int calculateDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;

  return distance;
}

/**
 * Calculates target angle using trigonometric coordinate mapping and aligns actuator
 */
void engageTarget(int radarAngleDeg, int targetDistance) {
  float theta0 = radians(radarAngleDeg);

  // Sensor to Actuator Physical Offset Coordinates (in cm)
  float deltaY = 8.0; // Sensor is mounted 8 cm above actuator pivot
  float deltaX = 0.0; // Alignment along vertical axis

  // Compute Target Polar-to-Cartesian Position Relative to Actuator
  float targetX = targetDistance * cos(theta0) - deltaX;
  float targetY = targetDistance * sin(theta0) - deltaY;

  // Compute Compensation Pitch Angle
  float theta1 = atan2(targetY, targetX);
  int actuatorAngleDeg = degrees(theta1);

  // Smoothly Align Pitch Servo to Calculated Angle
  smoothAimTo(actuatorAngleDeg);

  // Issue Audio-Visual Alert before Actuation
  alertSignal();

  hasEngaged = true;

  // Execute Actuation Cycle
  actuatorServo.write(70);   // Engage Position
  delay(500);
  actuatorServo.write(100);  // Standby Position
}

/**
 * Rotates pitch servo smoothly to prevent mechanical jitter
 */
void smoothAimTo(int targetAngle) {
  int currentAngle = aimServo.read();

  if (currentAngle < targetAngle) {
    for (int i = currentAngle; i <= targetAngle + 30; i++) {
      aimServo.write(i);
      delay(5);
    }
  } else {
    for (int i = currentAngle; i >= targetAngle + 30; i--) {
      aimServo.write(i);
      delay(5);
    }
  }
}

/**
 * Generates an audio-visual warning pulse sequence prior to actuation
 */
void alertSignal() {
  for (int i = 0; i < 5; i++) {
    digitalWrite(alertPin, HIGH);
    delay(150);
    digitalWrite(alertPin, LOW);
    delay(150);
  }
}
