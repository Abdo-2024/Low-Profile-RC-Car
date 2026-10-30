//**************************************
// 1. Include
//**************************************
#include <Wire.h>
#include <ESP32Servo.h>
#include <IBusBM.h>
#include <math.h>

//**************************************
// 2. Global Objects and Constants
//**************************************
IBusBM ibus;
HardwareSerial IBUS_PORT(1);

// Suspension servos
Servo servoLF;
Servo servoRF;
Servo servoLR;
Servo servoRR;

// Steering servo
Servo servoSteering;

// MPU6500 registers
#define MPU_ADDR       0x68
#define PWR_MGMT_1     0x6B
#define ACCEL_XOUT_H   0x3B
#define GYRO_XOUT_H    0x43

#define ACCEL_SCALE 16384.0
#define GYRO_SCALE 131.0

// Servo pins
const int SERVO_LF_PIN = 1;
const int SERVO_RF_PIN = 2;
const int SERVO_LR_PIN = 3;
const int SERVO_RR_PIN = 4;
const int SERVO_STEERING_PIN = 5;

// Servo positions and limits
const int SERVO_CENTER = 90;
const int SERVO_MIN = 30;
const int SERVO_MAX = 140;

// Steering limits
const int STEERING_MIN = 30;
const int STEERING_MAX = 140;

//**************************************
// 3. Variables
//**************************************

// MPU variables
float pitch = 0;
float roll = 0;
float yaw = 0;

float ax = 0, ay = 0, az = 0;
float gx = 0, gy = 0, gz = 0;

float accelPitchForce = 0;
float accelRollForce = 0;

float gyroBiasX = 0;
float gyroBiasY = 0;
float gyroBiasZ = 0;

float pitchZero = 0;
float rollZero = 0;

// Receiver channels
int ch1 = 1500;
int ch2 = 1500;
int ch3 = 1000;  // Throttle
int ch4 = 1500;  // Steering
int ch5 = 1000;  // Mode switch
int ch6 = 1500;  // Ride height

int steeringAngle = SERVO_CENTER;

// Manual suspension controls
float manualPitch = 0;
float manualRoll = 0;

float rideHeightOffset = 0;

int mode = 1;

// Suspension state
float pitchSuspension = 0;
float rollSuspension = 0;

float pitchVelocity = 0;
float rollVelocity = 0;

float springStrength = 0.05;
float damping = 0.97;

// Wheel targets
float lf = SERVO_CENTER;
float rf = SERVO_CENTER;
float lr = SERVO_CENTER;
float rr = SERVO_CENTER;

// Smoothed wheel outputs
float lfSmooth = SERVO_CENTER;
float rfSmooth = SERVO_CENTER;
float lrSmooth = SERVO_CENTER;
float rrSmooth = SERVO_CENTER;

// Timing
unsigned long lastMicros = 0;
unsigned long lastLoopTime = 0;

bool debugEnabled = true;

//**************************************
// 4. Helper Functions
//**************************************

void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

int16_t read16(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2);

  if (Wire.available() < 2) {
    return 0;
  }

  return (int16_t)((Wire.read() << 8) | Wire.read());
}

float applyDeadband(float v, float db) {
  if (fabs(v) < db) return 0.0f;
  return v;
}

float lowPass(float prev, float cur, float alpha) {
  return prev + alpha * (cur - prev);
}

//**************************************
// 5. Setup Functions
//**************************************

void setupIMU() {
  Wire.begin(8, 9);  // SDA, SCL
  Wire.setClock(400000);

  delay(100);

  writeRegister(PWR_MGMT_1, 0x00);
  delay(100);

  if (debugEnabled) {
    Serial.println("MPU6500 initialized");
  }
}

void setupServos() {
  servoLF.attach(SERVO_LF_PIN, 500, 2400);
  servoRF.attach(SERVO_RF_PIN, 500, 2400);
  servoLR.attach(SERVO_LR_PIN, 500, 2400);
  servoRR.attach(SERVO_RR_PIN, 500, 2400);

  servoSteering.attach(SERVO_STEERING_PIN, 500, 2400);

  servoLF.write(SERVO_CENTER);
  servoRF.write(SERVO_CENTER);
  servoLR.write(SERVO_CENTER);
  servoRR.write(SERVO_CENTER);
  servoSteering.write(SERVO_CENTER);

  if (debugEnabled) {
    Serial.println("Servos initialized");
  }
}

void setupIBus() {
  // UART: baud rate, format, RX pin, TX pin
  IBUS_PORT.begin(115200, SERIAL_8N1, 7, 21);
  ibus.begin(IBUS_PORT, IBUSBM_NOTIMER);

  if (debugEnabled) {
    Serial.println("iBus initialized");
  }
}

//**************************************
// 6. IMU Calibration
//**************************************

void calibrateIMU() {
  if (debugEnabled) {
    Serial.println("Calibrating IMU - keep the car flat and still");
  }

  const int samples = 2000;
  float sumPitch = 0.0f;
  float sumRoll = 0.0f;

  gyroBiasX = 0;
  gyroBiasY = 0;
  gyroBiasZ = 0;

  for (int i = 0; i < samples; i++) {
    ax = read16(ACCEL_XOUT_H) / ACCEL_SCALE;
    ay = read16(ACCEL_XOUT_H + 2) / ACCEL_SCALE;
    az = read16(ACCEL_XOUT_H + 4) / ACCEL_SCALE;

    gx = read16(GYRO_XOUT_H) / GYRO_SCALE;
    gy = read16(GYRO_XOUT_H + 2) / GYRO_SCALE;
    gz = read16(GYRO_XOUT_H + 4) / GYRO_SCALE;

    gyroBiasX += gx;
    gyroBiasY += gy;
    gyroBiasZ += gz;

    sumPitch += atan2(ay, az) * 180.0f / PI;
    sumRoll += atan2(ax, az) * 180.0f / PI;

    delay(2);
  }

  gyroBiasX /= samples;
  gyroBiasY /= samples;
  gyroBiasZ /= samples;

  pitch = sumPitch / samples;
  roll = sumRoll / samples;
  yaw = 0;

  pitchZero = pitch;
  rollZero = roll;

  if (debugEnabled) {
    Serial.println("IMU calibration complete");
  }
}

//**************************************
// 7. Receiver Input Functions
//**************************************

void readIBus() {
  ch1 = ibus.readChannel(0);
  ch2 = ibus.readChannel(1);
  ch3 = ibus.readChannel(2);  // Receiver CH3: throttle
  ch4 = ibus.readChannel(3);  // Receiver CH4: steering
  ch5 = ibus.readChannel(4);  // Receiver CH5: mode
  ch6 = ibus.readChannel(5);  // Receiver CH6: ride height

  // Failsafe for invalid channel readings
  if (ch1 == 0 || ch2 == 0 || ch3 == 0 ||
      ch4 == 0 || ch5 == 0 || ch6 == 0) {
    ch1 = 1500;
    ch2 = 1500;
    ch3 = 1000;
    ch4 = 1500;
    ch5 = 1000;
    ch6 = 1500;
  }
}

//**************************************
// 8. Steering Control
//**************************************

void updateSteering() {
  steeringAngle = map(
    ch4, 1000, 2000,
    STEERING_MIN, STEERING_MAX
  );

  steeringAngle = constrain(
    steeringAngle,
    STEERING_MIN,
    STEERING_MAX
  );

  servoSteering.write(steeringAngle);
}

//**************************************
// 9. Mode and Manual Controls
//**************************************

void updateModeFromSwitch() {
  if (ch5 < 1300) {
    mode = 1;
  } else if (ch5 < 1700) {
    mode = 2;
  } else {
    mode = 3;
  }
}

void mapManualControls() {
  if (mode == 3) {
    manualRoll = map(ch1, 1000, 2000, -25, 25);
    manualPitch = map(ch2, 1000, 2000, -25, 25);
  } else {
    manualRoll = 0;
    manualPitch = 0;
  }
}

void mapRideHeightControl() {
  rideHeightOffset = map(ch6, 1000, 2000, -15, 15);
}

//**************************************
// 10. IMU Sensor Functions
//**************************************

void readIMU() {
  ax = read16(ACCEL_XOUT_H) / ACCEL_SCALE;
  ay = read16(ACCEL_XOUT_H + 2) / ACCEL_SCALE;
  az = read16(ACCEL_XOUT_H + 4) / ACCEL_SCALE;

  gx = read16(GYRO_XOUT_H) / GYRO_SCALE;
  gy = read16(GYRO_XOUT_H + 2) / GYRO_SCALE;
  gz = read16(GYRO_XOUT_H + 4) / GYRO_SCALE;
}

void updateOrientation(float dt) {
  if (dt <= 0.0f) return;

  float gxCorr = gx - gyroBiasX;
  float gyCorr = gy - gyroBiasY;
  float gzCorr = gz - gyroBiasZ;

  float accelPitch = atan2(ay, az) * 180.0f / PI;
  float accelRoll = atan2(ax, az) * 180.0f / PI;

  const float alpha = 0.99f;

  pitch = alpha * (pitch + gyCorr * dt)
        + (1.0f - alpha) * accelPitch;

  roll = alpha * (roll + gxCorr * dt)
       + (1.0f - alpha) * accelRoll;

  yaw += gzCorr * dt;
}

void calculateAccelForces() {
  accelPitchForce = ax * 18.0f;
  accelRollForce = ay * 18.0f;
}

//**************************************
// 11. Suspension Control Functions
//**************************************

void updateControl() {
  switch (mode) {
    case 1:
      controlRealisticMode();
      break;

    case 2:
      controlLevelingMode();
      break;

    case 3:
      controlManualMode();
      break;

    default:
      controlRealisticMode();
      break;
  }
}

void controlRealisticMode() {
  float pitchOut = pitch - pitchZero;
  float rollOut = roll - rollZero;

  pitchOut = applyDeadband(pitchOut, 0.5f);
  rollOut = applyDeadband(rollOut, 0.5f);

  float targetPitch = pitchOut * 1.5f + accelPitchForce;
  float targetRoll = rollOut * 1.5f + accelRollForce;

  pitchVelocity +=
    (targetPitch - pitchSuspension) * springStrength;

  rollVelocity +=
    (targetRoll - rollSuspension) * springStrength;

  pitchVelocity *= damping;
  rollVelocity *= damping;

  pitchSuspension += pitchVelocity;
  rollSuspension += rollVelocity;
}

void controlLevelingMode() {
  float pitchOut = pitch - pitchZero;
  float rollOut = roll - rollZero;

  pitchOut = applyDeadband(pitchOut, 0.5f);
  rollOut = applyDeadband(rollOut, 0.5f);

  const float levelingGain = 1.5f;

  pitchVelocity +=
    (-pitchOut * levelingGain - pitchSuspension) * springStrength;

  rollVelocity +=
    (-rollOut * levelingGain - rollSuspension) * springStrength;

  pitchVelocity *= damping;
  rollVelocity *= damping;

  pitchSuspension += pitchVelocity;
  rollSuspension += rollVelocity;
}

void controlManualMode() {
  pitchSuspension = manualPitch;
  rollSuspension = manualRoll;

  pitchVelocity = 0;
  rollVelocity = 0;
}

//**************************************
// 12. Suspension Servo Output
//**************************************

void updateServoTargets() {
  lf = SERVO_CENTER + pitchSuspension
     + rollSuspension + rideHeightOffset;

  rf = SERVO_CENTER + pitchSuspension
     - rollSuspension + rideHeightOffset;

  lr = SERVO_CENTER - pitchSuspension
     + rollSuspension + rideHeightOffset;

  rr = SERVO_CENTER - pitchSuspension
     - rollSuspension + rideHeightOffset;
}

void applyServoOutput() {
  lf = constrain(lf, SERVO_MIN, SERVO_MAX);
  rf = constrain(rf, SERVO_MIN, SERVO_MAX);
  lr = constrain(lr, SERVO_MIN, SERVO_MAX);
  rr = constrain(rr, SERVO_MIN, SERVO_MAX);

  const float servoSmooth = 0.15f;

  lfSmooth += (lf - lfSmooth) * servoSmooth;
  rfSmooth += (rf - rfSmooth) * servoSmooth;
  lrSmooth += (lr - lrSmooth) * servoSmooth;
  rrSmooth += (rr - rrSmooth) * servoSmooth;

  servoLF.write((int)lfSmooth);
  servoRF.write((int)rfSmooth);
  servoLR.write((int)lrSmooth);
  servoRR.write((int)rrSmooth);
}

//**************************************
// 13. Debug Output
//**************************************

void printDebugData() {
  Serial.print("Mode: ");
  Serial.print(mode);

  Serial.print(" | CH1: ");
  Serial.print(ch1);

  Serial.print(" CH2: ");
  Serial.print(ch2);

  Serial.print(" CH3 Throttle: ");
  Serial.print(ch3);

  Serial.print(" CH4 Steering: ");
  Serial.print(ch4);

  Serial.print(" Steering Angle: ");
  Serial.print(steeringAngle);

  Serial.print(" CH5: ");
  Serial.print(ch5);

  Serial.print(" CH6: ");
  Serial.print(ch6);

  Serial.print(" | Pitch: ");
  Serial.print(pitch, 1);

  Serial.print(" Roll: ");
  Serial.print(roll, 1);

  Serial.print(" | PitchSusp: ");
  Serial.print(pitchSuspension, 1);

  Serial.print(" RollSusp: ");
  Serial.print(rollSuspension, 1);

  Serial.print(" | LF: ");
  Serial.print(lfSmooth, 0);

  Serial.print(" RF: ");
  Serial.print(rfSmooth, 0);

  Serial.print(" LR: ");
  Serial.print(lrSmooth, 0);

  Serial.print(" RR: ");
  Serial.print(rrSmooth, 0);

  Serial.print(" | AX: ");
  Serial.print(ax, 2);

  Serial.print(" AY: ");
  Serial.print(ay, 2);

  Serial.print(" AZ: ");
  Serial.print(az, 2);

  Serial.println();
}

//**************************************
// 14. Setup
//**************************************

void setup() {
  Serial.begin(115200);
  delay(500);

  if (debugEnabled) {
    Serial.println("Booting...");
  }

  setupIMU();
  setupServos();
  setupIBus();

  calibrateIMU();

  pitchSuspension = 0;
  rollSuspension = 0;
  pitchVelocity = 0;
  rollVelocity = 0;

  lfSmooth = SERVO_CENTER;
  rfSmooth = SERVO_CENTER;
  lrSmooth = SERVO_CENTER;
  rrSmooth = SERVO_CENTER;

  lastMicros = micros();
  lastLoopTime = millis();

  if (debugEnabled) {
    Serial.println("Setup complete");
  }
}

//**************************************
// 15. Main Loop
//**************************************

void loop() {
  ibus.loop();

  unsigned long now = millis();

  if (now - lastLoopTime >= 10) {
    lastLoopTime = now;

    // Read receiver
    readIBus();

    // Steering servo follows receiver CH4
    updateSteering();

    // Suspension controls
    updateModeFromSwitch();
    mapManualControls();
    mapRideHeightControl();

    // Read IMU
    unsigned long currentMicros = micros();

    float dt = (currentMicros - lastMicros)
             / 1000000.0f;

    lastMicros = currentMicros;

    readIMU();
    updateOrientation(dt);
    calculateAccelForces();

    // Suspension control
    updateControl();

    // Suspension servo output
    updateServoTargets();
    applyServoOutput();

    // Debug output every 100 ms
    static unsigned long lastDebug = 0;

    if (debugEnabled && millis() - lastDebug >= 100) {
      lastDebug = millis();
      printDebugData();
    }
  }
}