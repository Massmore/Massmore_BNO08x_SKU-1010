#include <Arduino.h>
/*
  10 — High rate orientation (Gyro-Integrated Rotation Vector)
  Massmore BNO08x Library

  The gyro-integrated rotation vector is the lowest latency orientation output
  the BNO08x has. It runs up to 1 kHz and the sensor puts it on its own SHTP
  channel (channel 5) so it cannot be stuck behind slower reports. That is what
  you want for AR/VR head tracking, gimbal stabilisation or a self-balancing
  robot.

  It gives you the quaternion AND the angular velocity in one report, with no
  timestamp overhead — which is also why it does not carry an accuracy field.

  You need the INT pin wired for this to work well. Without it the driver has
  to poll blindly and you will drop samples.

  This sketch measures the rate it actually achieves and prints it once a
  second, so you can see what your wiring and bus speed can sustain.

  Product: https://www.massmore.shop
*/

#include <Wire.h>
#include <Massmore_BNO08x.h>

#define SDA_PIN   21
#define SCL_PIN   22
#define INT_PIN    4      // required for high rates
#define RST_PIN    5

MassmoreBNO08x imu;

uint32_t sampleCount = 0;
uint32_t lastReport = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) { }

#if defined(ARDUINO_ARCH_ESP32)
  Wire.begin(SDA_PIN, SCL_PIN);
#else
  Wire.begin();
#endif
  Wire.setClock(400000);   // 400 kHz is the practical minimum for 400 Hz

  if (!imu.begin(MASSMORE_BNO08X_I2C_ADDR_DEF, Wire, INT_PIN, RST_PIN)) {
    Serial.println(F("BNO08x not found."));
    while (1) delay(100);
  }

  if (INT_PIN < 0) {
    Serial.println(F("WARNING: no INT pin. Expect dropped samples."));
  }

  // 2500 us = 400 Hz. Try 1000 for 1 kHz on SPI, or 5000 for a gentler 200 Hz.
  imu.enableGyroIntegratedRotationVector(2500);

  Serial.println(F("Massmore BNO08x - gyro-integrated rotation vector"));
  lastReport = millis();
}

void loop() {
  // Drain aggressively: at 400 Hz there is often more than one packet waiting.
  imu.updateAll(32);

  if (imu.hasNewReport(MASSMORE_SENSOR_GYRO_INTEGRATED_RV)) {
    sampleCount++;
  }

  if (millis() - lastReport >= 1000) {
    lastReport = millis();

    massmore_quat_t q = imu.getQuaternion();
    massmore_vec3_t w = imu.getAngularVelocity();
    massmore_euler_t e = imu.getEulerDeg();

    Serial.print(F("rate ")); Serial.print(sampleCount); Serial.print(F(" Hz"));
    Serial.print(F("   quat "));
    Serial.print(q.real, 3); Serial.print(' ');
    Serial.print(q.i, 3);    Serial.print(' ');
    Serial.print(q.j, 3);    Serial.print(' ');
    Serial.print(q.k, 3);
    Serial.print(F("   rpy "));
    Serial.print(e.roll, 1);  Serial.print(' ');
    Serial.print(e.pitch, 1); Serial.print(' ');
    Serial.print(e.yaw, 1);
    Serial.print(F("   omega [rad/s] "));
    Serial.print(w.x, 2); Serial.print(' ');
    Serial.print(w.y, 2); Serial.print(' ');
    Serial.println(w.z, 2);

    sampleCount = 0;
  }
}
