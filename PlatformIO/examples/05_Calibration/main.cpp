#include <Arduino.h>
/*
  05 — Calibration
  Massmore BNO08x Library

  The BNO08x calibrates itself while it runs, but it only writes the result to
  flash when you tell it to. This sketch walks the official procedure from
  CEVA document 1000-4044.

  HOW TO CALIBRATE
    1. Type 'c' to enable dynamic calibration of accel + gyro + mag.
    2. Magnetometer: sweep the board through a rotate the board ~180 degrees and back
       around EACH axis (roll, then pitch, then yaw), ~2 seconds per axis.
       This is what CEVA specifies - a figure of eight is BNO055 folklore.
    3. Accelerometer: hold the board in 4 to 6 clearly different orientations,
       about 1 second in each. They do not have to be exact.
    4. Gyroscope: set the board down on a stable surface for 2 to 3 seconds.
    5. Type 's' to save the calibration to flash. It now survives power cycles.

  Serial commands:
    c  start calibrating accel + gyro + mag
    m  calibrate the magnetometer only
    e  stop dynamic calibration
    s  save calibration to flash (Save DCD)
    x  erase stored calibration and reset the device
    ?  print the calibration status

  Product: https://www.massmore.shop
*/

#include <Wire.h>
#include <Massmore_BNO08x.h>

#define SDA_PIN   21
#define SCL_PIN   22
#define INT_PIN    4
#define RST_PIN    5

MassmoreBNO08x imu;
uint32_t lastPrint = 0;

void printHelp() {
  Serial.println(F("\nCommands: c=cal all  m=cal mag  e=end cal  "
                   "s=save  x=erase+reset  ?=status"));
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) { }

#if defined(ARDUINO_ARCH_ESP32)
  Wire.begin(SDA_PIN, SCL_PIN);
#else
  Wire.begin();
#endif
  Wire.setClock(100000);   // see example 01 on why not 400 kHz

  if (!imu.begin(MASSMORE_BNO08X_I2C_ADDR_DEF, Wire, INT_PIN, RST_PIN)) {
    Serial.println(F("BNO08x not found."));
    while (1) delay(100);
  }

  // Watch the rotation vector's own accuracy estimate while you calibrate,
  // and the magnetometer so you can see its accuracy field move.
  imu.enableRotationVector(20000);
  imu.enableMagnetometer(20000);
  imu.enableGameRotationVector(50000);

  Serial.println(F("Massmore BNO08x - calibration helper"));
  printHelp();
}

void loop() {
  imu.updateAll();

  if (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'c':
        imu.calibrateAll();
        Serial.println(F("> dynamic calibration ON for accel + gyro + mag"));
        Serial.println(F("  4-6 orientations, 180 deg and back on each axis, then set it down"));
        break;
      case 'm':
        imu.calibrateMagnetometer();
        Serial.println(F("> magnetometer calibration ON"));
        break;
      case 'e':
        imu.endCalibration();
        Serial.println(F("> dynamic calibration OFF"));
        break;
      case 's':
        imu.saveCalibration();
        Serial.println(F("> Save DCD sent - calibration written to flash"));
        break;
      case 'x':
        Serial.println(F("> erasing calibration and resetting..."));
        imu.clearCalibrationAndReset();
        imu.enableRotationVector(20000);
        imu.enableMagnetometer(20000);
        Serial.println(F("  done, sensors re-enabled"));
        break;
      case '?':
        imu.requestCalibrationStatus();
        delay(50);
        imu.updateAll();
        Serial.print(F("> ME calibration status byte: "));
        Serial.print(imu.getCalibrationStatus());
        Serial.println(imu.calibrationComplete() ? F("  (OK)") : F("  (pending/failed)"));
        break;
      default:
        break;
    }
  }

  if (millis() - lastPrint < 500) return;
  lastPrint = millis();

  float headingAccDeg = imu.getQuatAccuracy() * 57.2957795f;

  Serial.print(F("mag accuracy: "));
  Serial.print(MassmoreBNO08x::accuracyToString(
                 imu.getAccuracy(MASSMORE_SENSOR_MAGNETIC_FIELD)));
  Serial.print(F("   RV accuracy: "));
  Serial.print(MassmoreBNO08x::accuracyToString(
                 imu.getAccuracy(MASSMORE_SENSOR_ROTATION_VECTOR)));
  Serial.print(F("   heading error: "));
  Serial.print(headingAccDeg, 1);
  Serial.print(F(" deg"));

  if (headingAccDeg > 0.0f && headingAccDeg < 10.0f) {
    Serial.print(F("   <- good, press 's' to save"));
  }
  Serial.println();
}
