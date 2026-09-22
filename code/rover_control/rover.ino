/**
 * ============================================================================
 * Project Name : SENTINEL-X
 * Module       : Rover Mobility & Navigation System
 * Description  : Controls the 4WD robotic rover platform via HC-05 Bluetooth 
 *                commands and L298N motor driver.
 * Microcontroller: Arduino Uno (ATmega328P)
 * ============================================================================
 */

// Bluetooth Command Variable
char command;

// Motor Control Pin Configuration

// Right Side Motors (Front & Rear)
const int RF1 = 7; // Right Front Direction Pin 1
const int RF2 = 6; // Right Front Direction Pin 2
const int RB1 = 5; // Right Rear Direction Pin 1
const int RB2 = 4; // Right Rear Direction Pin 2

// Left Side Motors (Front & Rear)
const int LF1 = 3; // Left Front Direction Pin 1
const int LF2 = 2; // Left Front Direction Pin 2
const int LB1 = 9; // Left Rear Direction Pin 1
const int LB2 = 8; // Left Rear Direction Pin 2

// Motor Driver Enable (PWM Speed Control) Pins
const int ENA = 10; // Enable Right Motors (PWM)
const int ENB = 11; // Enable Left Motors (PWM)

// Function Prototypes
void forward();
void backward();
void left();
void right();
void stopCar();

void setup() {
  // Initialize Serial Communication for Bluetooth module (Baud Rate: 9600)
  Serial.begin(9600);

  // Configure Motor Control Pins as Outputs
  pinMode(RF1, OUTPUT); pinMode(RF2, OUTPUT);
  pinMode(RB1, OUTPUT); pinMode(RB2, OUTPUT);
  pinMode(LF1, OUTPUT); pinMode(LF2, OUTPUT);
  pinMode(LB1, OUTPUT); pinMode(LB2, OUTPUT);
  
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Ensure rover is stationary at startup
  stopCar();
}

void loop() {
  // Read incoming serial commands from HC-05 Bluetooth module
  if (Serial.available()) {
    command = toupper(Serial.read());

    switch (command) {
      case 'F':
        forward();
        break;
      case 'B':
        backward();
        break;
      case 'L':
        left();
        break;
      case 'R':
        right();
        break;
      case 'S':
        stopCar();
        break;
      default:
        // Ignore unrecognized commands
        break;
    }
  }
}

/**
 * Moves the rover forward at standard operational speed
 */
void forward() {
  analogWrite(ENA, 140); // Right side motor speed PWM
  analogWrite(ENB, 140); // Left side motor speed PWM

  // Right Side Forward
  digitalWrite(RF1, HIGH); digitalWrite(RF2, LOW);
  digitalWrite(RB1, HIGH); digitalWrite(RB2, LOW);

  // Left Side Forward
  digitalWrite(LF1, LOW);  digitalWrite(LF2, HIGH);
  digitalWrite(LB1, LOW);  digitalWrite(LB2, HIGH);
}

/**
 * Moves the rover backward at standard operational speed
 */
void backward() {
  analogWrite(ENA, 140); // Right side motor speed PWM
  analogWrite(ENB, 140); // Left side motor speed PWM

  // Right Side Backward
  digitalWrite(RF1, LOW);  digitalWrite(RF2, HIGH);
  digitalWrite(RB1, LOW);  digitalWrite(RB2, HIGH);

  // Left Side Backward
  digitalWrite(LF1, HIGH); digitalWrite(LF2, LOW);
  digitalWrite(LB1, HIGH); digitalWrite(LB2, LOW);
}

/**
 * Rotates the rover left (Differential drive turn)
 */
void left() {
  analogWrite(ENA, 180); // Right side motor speed PWM
  analogWrite(ENB, 0);   // Left side motor stopped

  // Right Side Forward
  digitalWrite(RF1, HIGH); digitalWrite(RF2, LOW);
  digitalWrite(RB1, HIGH); digitalWrite(RB2, LOW);

  // Left Side Backward
  digitalWrite(LF1, HIGH); digitalWrite(LF2, LOW);
  digitalWrite(LB1, HIGH); digitalWrite(LB2, LOW);
}

/**
 * Rotates the rover right (Differential drive turn)
 */
void right() {
  analogWrite(ENA, 0);   // Right side motor stopped
  analogWrite(ENB, 180); // Left side motor speed PWM

  // Right Side Backward
  digitalWrite(RF1, LOW);  digitalWrite(RF2, HIGH);
  digitalWrite(RB1, LOW);  digitalWrite(RB2, HIGH);

  // Left Side Forward
  digitalWrite(LF1, LOW);  digitalWrite(LF2, HIGH);
  digitalWrite(LB1, LOW);  digitalWrite(LB2, HIGH);
}

/**
 * Halts all motor rotation
 */
void stopCar() {
  digitalWrite(RF1, LOW); digitalWrite(RF2, LOW);
  digitalWrite(RB1, LOW); digitalWrite(RB2, LOW);
  digitalWrite(LF1, LOW); digitalWrite(LF2, LOW);
  digitalWrite(LB1, LOW); digitalWrite(LB2, LOW);
}
