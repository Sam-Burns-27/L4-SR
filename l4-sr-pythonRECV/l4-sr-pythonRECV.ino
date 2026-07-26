#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "BluetoothSerial.h"
#include <MPU6050.h>

// ── GY-521 / MPU-6050 ────────────────────────────────────────────────────────
// Shares I2C bus with PCA9685 (MPU-6050=0x68, PCA9685=0x40 — no conflict)
// Wiring: SDA->GPIO21, SCL->GPIO22, AD0->GND, VCC->3.3V
// XDA, XCL, INT — leave unconnected
MPU6050 mpu;

// Complementary filter state
float    imuAngleX     = 0.0f;
float    imuAngleY     = 0.0f;
unsigned long imuLastTime = 0;

// Tuning constants — adjust after calibration
const float IMU_ALPHA      = 0.96f;   // 0=trust accel only, 1=trust gyro only
const float ACCEL_SCALE    = 16384.0f;
const float GYRO_SCALE     = 131.0f;
const float GYRO_DEADBAND  = 1.5f;    // deg/s — raise if walking causes drift
const float ACCEL_DEADBAND = 0.02f;   // g

// Calibration offsets — run MPU6050 calibration sketch to get your chip's values
// then paste them here. Leave as 0 until calibrated.
//const int16_t CAL_AX = 0, CAL_AY = 0, CAL_AZ = 0;
//const int16_t CAL_GX = 0, CAL_GY = 0, CAL_GZ = 0;

const int16_t CAL_AX = -2718;  // (-2812 + -2625) / 2
const int16_t CAL_AY =  1312;  // (1250 + 1375) / 2
const int16_t CAL_AZ =   531;  // (500 + 562) / 2
const int16_t CAL_GX =   -31;  // (-62 + 0) / 2
const int16_t CAL_GY =   -31;  // (-62 + 0) / 2
const int16_t CAL_GZ =    31;  // (0 + 62) / 2

// IMU Bluetooth streaming
bool          imuStreaming = false;
unsigned long imuLastSent  = 0;
const unsigned long IMU_INTERVAL = 100; // ms — 10Hz, tune down for faster updates

BluetoothSerial SerialBT;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

const int SERVOMIN = 102;  // Pulse for 0 degrees
const int SERVOMAX = 512;  // Pulse for 180 degrees

int angleToPulse(int angle) {
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

// ── IMU update — call every loop ─────────────────────────────────────────────
// Runs complementary filter and streams over BT if IMU_ON was sent.
// Format: IMU:<angleX>,<angleY>,<gx>,<gy>,<gz>,<ax>,<ay>,<az>
// Python parses lines starting with "IMU:" silently (not logged to command log).
void updateIMU() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float axG   = ax / ACCEL_SCALE;
  float ayG   = ay / ACCEL_SCALE;
  float azG   = az / ACCEL_SCALE;
  float gxDps = gx / GYRO_SCALE;
  float gyDps = gy / GYRO_SCALE;
  float gzDps = gz / GYRO_SCALE;

  // Deadbands — suppress noise when still
  if (abs(gxDps) < GYRO_DEADBAND)  gxDps = 0;
  if (abs(gyDps) < GYRO_DEADBAND)  gyDps = 0;
  if (abs(gzDps) < GYRO_DEADBAND)  gzDps = 0;
  if (abs(axG)   < ACCEL_DEADBAND) axG   = 0;
  if (abs(ayG)   < ACCEL_DEADBAND) ayG   = 0;

  // Accelerometer tilt (stable long-term reference)
  float accelTiltX = atan2(ayG, azG) * (180.0f / PI);
  float accelTiltY = atan2(-axG, azG) * (180.0f / PI);

  // Complementary filter — blends gyro (smooth) with accel (no drift)
  unsigned long now = millis();
  float dt = (now - imuLastTime) / 1000.0f;
  imuLastTime = now;
  imuAngleX = IMU_ALPHA * (imuAngleX + gxDps * dt) + (1.0f - IMU_ALPHA) * accelTiltX;
  imuAngleY = IMU_ALPHA * (imuAngleY + gyDps * dt) + (1.0f - IMU_ALPHA) * accelTiltY;

  // Stream if enabled
  if (imuStreaming && (now - imuLastSent >= IMU_INTERVAL)) {
    imuLastSent = now;
    SerialBT.print("IMU:");
    SerialBT.print(imuAngleX, 2); SerialBT.print(",");
    SerialBT.print(imuAngleY, 2); SerialBT.print(",");
    SerialBT.print(gxDps, 2);     SerialBT.print(",");
    SerialBT.print(gyDps, 2);     SerialBT.print(",");
    SerialBT.print(gzDps, 2);     SerialBT.print(",");
    SerialBT.print(axG, 3);       SerialBT.print(",");
    SerialBT.print(ayG, 3);       SerialBT.print(",");
    SerialBT.println(azG, 3);
  }
}

// ── Command processor ─────────────────────────────────────────────────────────
// Accepts:
//   s <channel> <angle>   — individual servo (ch 0–11, angle 0–180)
//   S <angle>             — all 12 servos
//   F                     — free all servos (cut PWM — robot goes limp, support first!)
//   R                     — resume (center all to 90°, restores PWM)
//   IMU_ON                — start streaming IMU data over Bluetooth at 10Hz
//   IMU_OFF               — stop IMU stream
void processCommand(String input, Stream &out) {
  input.trim();
  if (!input.length()) return;

  int ch, angle;

  if (input.startsWith("s ")) {
    if (sscanf(input.c_str(), "s %d %d", &ch, &angle) == 2) {
      if (ch >= 0 && ch < 12 && angle >= 0 && angle <= 180) {
        pwm.setPWM(ch, 0, angleToPulse(angle));
        out.print("Set ch ");
        out.print(ch);
        out.print(" -> ");
        out.print(angle);
        out.println(" deg");
      } else {
        out.println("Invalid s range");
      }
    } else {
      out.println("Invalid s format");
    }

  } else if (input.startsWith("S ")) {
    if (sscanf(input.c_str(), "S %d", &angle) == 1) {
      if (angle >= 0 && angle <= 180) {
        for (int ch = 0; ch < 12; ch++) {
          pwm.setPWM(ch, 0, angleToPulse(angle));
        }
        out.print("Set ALL -> ");
        out.print(angle);
        out.println(" deg");
      } else {
        out.println("Invalid S range");
      }
    } else {
      out.println("Invalid S format");
    }

  } else if (input == "F") {
    // Free — cut PWM completely, servos go limp (no hold torque)
    // WARNING: robot will collapse — make sure it is supported before sending
    for (int ch = 0; ch < 12; ch++) {
      pwm.setPWM(ch, 0, 0);
    }
    out.println("Servos FREE — no torque");

  } else if (input == "R") {
    // Resume — re-center all servos to 90° and restore PWM
    for (int ch = 0; ch < 12; ch++) {
      pwm.setPWM(ch, 0, angleToPulse(90));
    }
    out.println("Servos RESUMED at 90");

  } else if (input == "IMU_ON") {
    imuStreaming = true;
    imuLastTime  = millis();
    out.println("IMU stream ON");

  } else if (input == "IMU_OFF") {
    imuStreaming = false;
    out.println("IMU stream OFF");

  } else {
    out.print("Unknown cmd: ");
    out.println(input);
  }
}


void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);  // fast-mode I2C — cuts PCA9685 command time from ~5ms to ~1ms
  pwm.begin();
  pwm.setPWMFreq(50);

  // Reduce readStringUntil timeout from 1000ms to 100ms so the loop
  // does not stall if a command arrives without a clean newline.
  Serial.setTimeout(100);

  SerialBT.begin("ESP32-Test");   // keep the same name
  //SerialBT.setPin("1234", 4);   // PIN "1234"      // NEW: require PIN 1234
  //Serial.println("Bluetooth SPP started with PIN 1234");

  // Same timeout reduction for Bluetooth SPP stream.
  SerialBT.setTimeout(100);

  // ── GY-521 init ──────────────────────────────────────────────────────────────
  mpu.initialize();
  mpu.setDLPFMode(MPU6050_DLPF_BW_42); // 42Hz low-pass — good balance of smooth vs lag
  mpu.setXAccelOffset(CAL_AX);
  mpu.setYAccelOffset(CAL_AY);
  mpu.setZAccelOffset(CAL_AZ);
  mpu.setXGyroOffset(CAL_GX);
  mpu.setYGyroOffset(CAL_GY);
  mpu.setZGyroOffset(CAL_GZ);
  if (mpu.testConnection()) {
    Serial.println("GY-521 OK");
  } else {
    Serial.println("GY-521 FAIL — check wiring (SDA=21, SCL=22, AD0=GND)");
  }
  imuLastTime = millis();

  // Center all 12 used channels at startup
  for (int ch = 0; ch < 12; ch++) {
    pwm.setPWM(ch, 0, angleToPulse(90));
  }

  Serial.println("Ready — commands: s/S/F/R/IMU_ON/IMU_OFF over USB or Bluetooth.");
  delay(50);  // increased from 30ms to give PCA9685 time to settle
}

void loop() {
  // Always update IMU — streams over BT automatically if IMU_ON was sent
  updateIMU();

  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    processCommand(input, Serial);
  }

  if (SerialBT.available()) {
    String input = SerialBT.readStringUntil('\n');
    processCommand(input, SerialBT);
  }
}