# Low-Profile 1/10-Scale RC Car

**Control Firmware – Direct Drive, Active Suspension and Extreme-Angle Steering**

**Designed by:** AbdoAllah Mohammad (Abdo-2024)

**GitHub:** https://github.com/Abdo-2024

**Thingiverse:** https://www.thingiverse.com/Abdo-2024/designs

## Project Overview

This repository contains the ESP32 Arduino control firmware for my custom-designed, ultra-low-profile 1/10-scale RC car.

The vehicle has been designed so that the chassis, suspension and drivetrain fit below the height of the wheels, creating an extremely low centre of gravity.

The current prototype uses a direct-drive configuration and has successfully driven and drifted on smooth surfaces, including hardwood flooring and tiles.

The car features active servo-controlled suspension, extreme-angle steering and an adjustable chassis.

### Mechanical Features

- Ultra-low-profile 1/10-scale chassis.
- Direct-drive drivetrain.
- Up to 90° wheel steering to either side from straight ahead.
- 12° difference between inner and outer steering angles at full lock (90° inner, 78° outer).
- 4° front toe-out.
- Servo-operated ride-height adjustment.
- Independently controlled suspension corners.
- Adjustable overall chassis width, up to 40 mm wider.
- Adjustable overall chassis length, up to 40 mm longer.

## Firmware Features

The firmware provides:

- Four independent suspension servo outputs.
- One steering servo output.
- Six-channel iBUS receiver communication.
- MPU6500 accelerometer and gyroscope integration.
- Automatic IMU calibration at startup.
- Three selectable suspension modes.
- Manual ride-height adjustment.
- Smoothed servo movement.
- Serial debugging and sensor monitoring.

## Hardware Requirements

| Component | Specification |
|---|---|
| Microcontroller | ESP32-compatible board |
| Servos | 5 × 9 g |
| IMU | MPU6500 |
| Receiver | iBUS-compatible |
| UBEC | 8 A, 5 V |
| ESC | 60 A |
| Motor | 540 brushed motor, 30T |
| Universal-joint linkages | 10 |
| Stainless-steel rods | 6 × 3 mm diameter × 40 mm |
| Suspension units | 4 |
| Fasteners | Assorted M2, M2.5 and M3 bolts, nuts and threaded inserts |
| Battery | Suitable battery for the vehicle |

The exact ESP32 board model and battery specifications should be confirmed for individual builds.

## Arduino Libraries

The sketch uses the following libraries:

- `Wire.h` – I²C communication.
- `ESP32Servo.h` – Servo control on ESP32.
- `IBusBM.h` – iBUS receiver communication.
- `math.h` – Mathematical calculations.

Install the ESP32 Arduino board package and the required third-party libraries before compiling.

## Pin Configuration

The following GPIO assignments are defined in the source code.

| Component | ESP32 GPIO |
|---|---|
| Left-front suspension servo | GPIO 1 |
| Right-front suspension servo | GPIO 2 |
| Left-rear suspension servo | GPIO 3 |
| Right-rear suspension servo | GPIO 4 |
| Steering servo | GPIO 5 |
| MPU6500 SDA | GPIO 8 |
| MPU6500 SCL | GPIO 9 |
| iBUS receiver RX | GPIO 7 |
| UART TX | GPIO 21 |

The iBUS UART uses 115200 baud.

**Important:** Check the GPIO availability and electrical compatibility of your particular ESP32 board before wiring the system.

Use a suitably rated external servo supply with a common ground. Do not attempt to power all five servos directly from the microcontroller's logic supply.

## Radio Receiver Channel Mapping

| Channel | Function |
|---|---|
| CH1 | Manual suspension roll |
| CH2 | Manual suspension pitch |
| CH3 | Throttle input monitoring |
| CH4 | Steering |
| CH5 | Suspension mode selection |
| CH6 | Ride-height adjustment |

**Note:** Channel 3 is read by the firmware, but the supplied sketch does not directly output a throttle signal to the ESC. Motor-control wiring must be configured separately.

## Suspension Control Modes

### Mode 1 – Motion-Responsive Suspension

Activated when CH5 is below 1300.

This mode uses the MPU6500 orientation and acceleration measurements to calculate suspension adjustments.

It incorporates software spring and damping calculations to produce pitch and roll responses.

### Mode 2 – Automatic Leveling

Activated when CH5 is between 1300 and 1699.

This mode attempts to compensate for changes in vehicle pitch and roll using IMU feedback.

The suspension servos are adjusted independently to counteract measured vehicle inclination.

### Mode 3 – Manual Suspension Control

Activated when CH5 is 1700 or higher.

This mode allows manual adjustment of the vehicle's pitch and roll through the transmitter.

- CH1 controls suspension roll.
- CH2 controls suspension pitch.
- CH6 remains available for ride-height adjustment.

## Ride-Height Adjustment

The firmware supports raising and lowering the suspension using receiver channel 6.

CH6 maps to a common suspension servo-angle offset between approximately −15° and +15°.

The mechanism has been physically tested and works.

However, the current 9 g servos do not provide sufficient torque to raise the complete vehicle when its battery is installed.

A future revision will use higher-torque servos to improve lifting performance and allow more reliable ride-height adjustment under load.

## Steering Control

The steering servo is controlled using receiver channel 4.

The firmware maps the receiver input to servo commands between 30° and 140°.

The mechanical steering linkage provides the vehicle's extreme steering geometry, including up to 90° wheel steering to either side.

The firmware servo-angle range and the physical wheel steering angles are not the same measurement.

## Installation

1. Install the Arduino IDE and the ESP32 board package.
2. Install the `ESP32Servo` and `IBusBM` libraries.
3. Download the firmware from this repository.
4. Open `Drift_Car_Suspension_Code_2.ino` in Arduino IDE.
5. Select the correct ESP32 board and serial port.
6. Check the GPIO assignments against your wiring.
7. Compile and upload the firmware.
8. Place the vehicle flat and stationary during startup calibration.
9. Open Serial Monitor at 115200 baud to inspect diagnostic information.

The suspension control loop runs approximately every 10 milliseconds, and debug information is printed approximately every 100 milliseconds.

Test the servos with the vehicle supported and the wheels clear of the ground before driving.

## Testing and Known Limitations

### Successful Tests

The current direct-drive prototype has successfully:

- Driven on smooth flooring.
- Drifted on hardwood and tiles.
- Operated its steering mechanism.
- Raised and lowered its suspension using the servo mechanism.

### Drivetrain Stress-Test Failure

The direct-drive system was also stress-tested through repeated acceleration and deceleration on carpet and by manually restraining the vehicle.

Under these high-load conditions, the linkage connecting the motor to the differential shredded.

This identified a drivetrain durability limitation that will need to be addressed in future revisions.

### Suspension Servo Torque

The ride-height adjustment mechanism functions, but the current servos cannot lift the complete vehicle with the battery installed.

Higher-torque servos are planned.

### Firmware Limitations

The supplied sketch does not include direct ESC throttle output or a complete motor failsafe. The receiver and ESC should be configured and tested independently for loss-of-signal protection.

The IMU control parameters are experimental and may require tuning for different vehicle configurations.

## Future Development

### Higher-Torque Suspension Servos

Upgrade the suspension actuators to support reliable ride-height adjustment with the battery installed.

### Belt-Drive Transmission

A belt-drive configuration is currently being developed.

The transmission is associated with a separate project, so its CAD and printable files will be published separately.

**Belt-drive project:** URL

### 3D-Printed Springs and Universal Joints

Future experiments will investigate printable suspension springs and custom 3D-printed universal-joint mechanisms.

### Miniature Version

A smaller version of the drivetrain and vehicle is also under development.

Stay tuned for future updates.

## 3D-Printable Design Files

The associated chassis and mechanical design files are available through my Thingiverse project.

**Thingiverse design:** URL

## Creator and Attribution

**Original design and firmware by AbdoAllah Mohammad (Abdo-2024).**

GitHub: https://github.com/Abdo-2024

Thingiverse: https://www.thingiverse.com/Abdo-2024/designs

The associated Thingiverse design uses Creative Commons Attribution–NonCommercial–ShareAlike 4.0 (CC BY-NC-SA 4.0).

The software licence for this repository should be specified separately in a `LICENSE` file. Until then, this README does not grant a general licence to reuse or redistribute the firmware.

Please credit AbdoAllah Mohammad (Abdo-2024) when referencing this project.
