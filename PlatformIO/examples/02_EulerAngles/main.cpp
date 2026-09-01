#include <Arduino.h>
/*
  02 — Euler Angles (roll / pitch / yaw)
  Massmore BNO08x Library

  Turns the quaternion into the three angles most projects actually want, and
  shows the difference between the three rotation vectors:

    Rotation Vector       9-axis, absolute heading, needs a calibrated
                          magnetometer, can be disturbed by nearby metal/motors
    Game Rotation Vector  6-axis, no magnetometer, yaw slowly drifts but is
                          completely immune to magnetic interference
    Geomagnetic RV        accel + mag only, low power, lower update rate

  Change WHICH_VECTOR below to compare them on your own hardware.

  Product: https://www.massmore.shop
*/

#include <Wire.h>
#include <Massmore_BNO08x.h>

#define SDA_PIN   21
#define SCL_PIN   22
#define INT_PIN    4
#define RST_PIN    5

// 0 = Rotation Vector, 1 = Game Rotation Vector, 2 = Geomagnetic RV
#define WHICH_VECTOR 0

MassmoreBNO08x imu;
uint8_t activeReport;
uint32_t lastPrint = 0;

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

#if   WHICH_VECTOR == 1
  imu.enableGameRotationVector(10000);
  activeReport = MASSMORE_SENSOR_GAME_ROTATION_VECTOR;
  Serial.println(F("Game Rotation Vector (6-axis)"));
#elif WHICH_VECTOR == 2
  imu.enableGeomagneticRotationVector(20000);
  activeReport = MASSMORE_SENSOR_GEOMAGNETIC_RV;
  Serial.println(F("Geomagnetic Rotation Vector"));
#else
  imu.enableRotationVector(10000);
  activeReport = MASSMORE_SENSOR_ROTATION_VECTOR;
  Serial.println(F("Rotation Vector (9-axis)"));
#endif

  Serial.println(F("roll\tpitch\tyaw\theading\taccuracy"));
}

void loop() {
  imu.updateAll();                       // drain whatever the sensor queued

  if (!imu.hasNewReport(activeReport)) return;

  // Printing at the full report rate floods the terminal; 10 Hz is plenty.
  if (millis() - lastPrint < 100) return;
  lastPrint = millis();

  massmore_euler_t e = imu.getEulerDeg();

  Serial.print(e.roll, 1);   Serial.print('\t');
  Serial.print(e.pitch, 1);  Serial.print('\t');
  Serial.print(e.yaw, 1);    Serial.print('\t');
  Serial.print(imu.getHeadingDeg(), 1);   // same as yaw, but 0..360
  Serial.print('\t');
  Serial.println(MassmoreBNO08x::accuracyToString(imu.getAccuracy(activeReport)));
}
