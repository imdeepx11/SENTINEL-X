# System Architecture & Technical Specifications — Project SENTINEL-X

---

## 1. Executive Summary

**Project SENTINEL-X** is an advanced hybrid robotic platform that integrates wireless manual navigation with autonomous ultrasonic target tracking and precision micro-actuation. Built upon an ATmega328P microcontroller (Arduino Uno) architecture, SENTINEL-X combines multi-sensor telemetry, trigonometric coordinate transformations, dual-subsystem motor control, and event-driven embedded firmware.

The system serves as a practical, open-source educational model for autonomous mobile robotics, sensor fusion, real-time control loops, and wireless serial telemetry.

---

## 2. Project Objectives

- **Wireless Mobility Control**: Implement a 4-wheel differential drive robotic chassis controlled wirelessly over UART via Bluetooth (HC-05 module).
- **Ultrasonic Radar Sweeping**: Design a dynamic 180° angular scanning system utilizing an HC-SR04 ultrasonic distance sensor mounted on a high-precision servo motor.
- **Real-Time Target Detection**: Implement threshold-based target identification (< 30 cm detection boundary).
- **Trigonometric Angle Compensation**: Calculate target coordinates and angular compensation using inverse trigonometric functions (`atan2`, `sin`, `cos`) to align the secondary actuator.
- **Automated Actuation Mechanism**: Develop a dual-axis servo-driven payload release/trigger system with integrated audio-visual warning signals.
- **Embedded Subsystem Integration**: Coordinate multi-device serial communication, PWM motor drivers, power distribution, and actuation timing without blocking critical control paths.

---

## 3. System Subsystems Architecture

The SENTINEL-X system is partitioned into three core physical and logical modules:

```
+-------------------------------------------------------------------+
|                        SENTINEL-X SYSTEM                          |
+---------------------------------+---------------------------------+
                                  |
    +-----------------------------+-----------------------------+
    |                             |                             |
+---v-----------------------+ +---v-----------------------+ +---v-----------------------+
|  1. Mobility Subsystem    | | 2. Radar & Telemetry      | | 3. Precision Actuator     |
| - 4WD DC Motors           | | - HC-SR04 Ultrasonic      | | - Aim/Pitch Servo (D5)  |
| - L298N Motor Driver      | | - Pan Servo (D9)          | | - Trigger Servo (D6)    |
| - HC-05 Bluetooth Module  | | - Real-time Serial Logs   | | - Buzzer/LED Warning    |
+---------------------------+ +---------------------------+ +---------------------------+
```

### 3.1 Mobility Subsystem
- **Power Unit**: Dual H-Bridge L298N Motor Driver.
- **Actuation**: 4 × Geared DC Motors configured for differential steering.
- **Control Interface**: HC-05 Bluetooth Transceiver interfacing with Arduino hardware serial (9600 Baud).
- **Directional Commands**:
  - `F`: Forward motion (PWM 140)
  - `B`: Backward motion (PWM 140)
  - `L`: Differential Left Rotation (PWM 180 / 0)
  - `R`: Differential Right Rotation (PWM 0 / 180)
  - `S`: Total Motor Halt

### 3.2 Radar & Target Tracking Subsystem
- **Distance Measurement**: HC-SR04 Ultrasonic Distance Sensor ($2\text{ cm}$ to $400\text{ cm}$ range).
- **Angular Scanning**: SG90 Micro Servo providing controlled panning from $10^\circ$ to $110^\circ$.
- **Detection Algorithm**: Continuous range sampling at $20\text{ ms}$ step delays. Objects closer than $30\text{ cm}$ trigger the alignment and actuation routine.

### 3.3 Target Alignment & Actuation Subsystem
- **Pitch Alignment Servo**: Dynamically adjusts position based on calculated target angle.
- **Trigger Actuator Servo**: Executes rapid mechanical actuation ($70^\circ$ active, $100^\circ$ standby reset).
- **Audio-Visual Warning**: 5-cycle pulsed alert on Pin 13 before executing actuation.

---

## 4. Trigonometric Coordinate Mapping & Target Alignment

When an obstacle/target is detected at an angle $\theta_0$ (from the pan radar servo) and distance $r$, the system transforms polar coordinates to relative Cartesian space to account for the physical vertical displacement ($\Delta Y = 8\text{ cm}$) between the radar sensor and the actuator pivot:

$$\text{Target}_X = r \cdot \cos(\theta_0) - \Delta X$$

$$\text{Target}_Y = r \cdot \sin(\theta_0) - \Delta Y$$

$$\theta_{\text{actuator}} = \text{atan2}(\text{Target}_Y, \text{Target}_X)$$

The pitch servo (`aimServo`) smoothly interpolates from its current position to $\theta_{\text{actuator}}$ to eliminate mechanical instability or motor voltage spikes.

---

## 5. Hardware Specifications & Components

| Component | Part / Model | Specification / Role |
| :--- | :--- | :--- |
| **Microcontroller** | Arduino Uno (ATmega328P) | 16 MHz Clock, 32 KB Flash, 14 Digital I/O Pins |
| **Motor Driver** | L298N Dual H-Bridge | Handles 6V–12V motor power with PWM speed regulation |
| **Bluetooth Transceiver** | HC-05 Module | UART Interface, 2.4 GHz ISM Band, Default 9600 Baud |
| **Ultrasonic Sensor** | HC-SR04 | 40 kHz pulse frequency, $2\text{ cm} - 400\text{ cm}$ range |
| **Servo Motors** | TowerPro SG90 / MG996R | 3 × PWM Servos (Radar Pan, Pitch Aim, Actuation Trigger) |
| **Chassis Motors** | 4 × Geared DC BO Motors | High torque, 3V–12V operating range |
| **Power Source** | 12V Li-ion / Lead-Acid Pack | Step-down regulation to 5V for MCU logic |
| **Alert Module** | Active Piezo Buzzer + LED | Audio-visual status feedback |

---

## 6. Circuit Pin Connections

### 6.1 Rover Drive Connections
- **L298N IN1 - IN4**: Arduino Digital Pins `7`, `6`, `5`, `4` (Right Motors)
- **L298N IN5 - IN8**: Arduino Digital Pins `3`, `2`, `9`, `8` (Left Motors)
- **L298N ENA / ENB**: Arduino PWM Pins `10`, `11`
- **HC-05 TX / RX**: Arduino RX (Pin 0) / TX (Pin 1)

### 6.2 Radar & Actuator Connections
- **HC-SR04 Trig Pin**: Digital Pin `11`
- **HC-SR04 Echo Pin**: Digital Pin `12`
- **Radar Pan Servo**: Digital PWM Pin `9`
- **Pitch Aim Servo**: Digital PWM Pin `5`
- **Trigger Actuator Servo**: Digital PWM Pin `6`
- **Buzzer / Status LED**: Digital Pin `13`

---

## 7. Operational Workflow

1. **System Startup**: Initializing serial baud rate, setting servo resting states ($20^\circ$ radar, $90^\circ$ aim, $100^\circ$ standby trigger).
2. **Wireless Mobility**: Listening to incoming Bluetooth character bytes to drive the chassis independently.
3. **Radar Sweep Initiation**: Upon receiving `'O'` byte, radar servo initiates angular sweep between $10^\circ$ and $110^\circ$.
4. **Target Acquisition**: If distance $< 30\text{ cm}$, radar halts, logs telemetry to serial, and computes targeting geometry.
5. **Alignment & Warning**: Pitch servo aligns to calculated angle $\theta_{\text{actuator}}$, followed by a 5-beep audio-visual warning sequence.
6. **Actuation Cycle**: Trigger servo shifts to $70^\circ$, pauses for $500\text{ ms}$, returns to $100^\circ$ standby position.
7. **Resume Sweep**: Radar system resumes scanning until a pause `'F'` command is received.

---

## 8. Limitations & Engineering Challenges

- **Ultrasonic Beam Divergence**: Echo reflections can vary depending on surface geometry and material density.
- **Power Bus Noise**: DC motor inductive switching can cause power ripple; decoupled supply lines are recommended.
- **Serial Buffer Overlap**: Bluetooth commands during continuous loops require non-blocking polling.

---

## 9. Future Enhancements

- **LiDAR Integration**: Replacing ultrasonic sensors with 2D LiDAR for high-resolution point-cloud mapping.
- **Computer Vision**: Interfacing a Raspberry Pi camera module running OpenCV for visual target classification.
- **Autonomous Navigation**: Implementing ROS 2 (Robot Operating System) with SLAM for full path planning.
