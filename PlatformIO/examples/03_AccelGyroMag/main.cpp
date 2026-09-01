#include <Arduino.h>
/*
  03 — Accelerometer, Gyroscope, Magnetometer
  Massmore BNO08x Library

  Runs the three calibrated physical sensors at once, plus linear acceleration
  (gravity removed) and the gravity vector itself. Every enabled report has its
  own rate — the sensor hub interleaves them for you.

  Units follow the SH-2 reference manual:
    accelerometer / linear acceleration / gravity   m/s^2
    gyroscope                                       rad/s   (helper for deg/s)
    magnetic field                                  microtesla

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

  imu.enableAccelerometer(20000);        //  50 Hz
  imu.enableGyroscope(20000);            //  50 Hz
  imu.enableMagnetometer(50000);         //  20 Hz - the mag is the slow one
  imu.enableLinearAcceleration(20000);   //  50 Hz
  imu.enableGravity(50000);              //  20 Hz

  Serial.println(F("Massmore BNO08x - raw physical sensors"));
}

void loop() {
  imu.updateAll();

  if (millis() - lastPrint < 200) return;
  lastPrint = millis();

  massmore_vec3_t a  = imu.getAccel();
  massmore_vec3_t g  = imu.getGyroDeg();     // deg/s is easier to eyeball
  massmore_vec3_t m  = imu.getMag();
  massmore_vec3_t la = imu.getLinearAccel();
  massmore_vec3_t gv = imu.getGravity();

  Serial.print(F("Accel  [m/s2] "));
  Serial.print(a.x, 2); Serial.print(' ');
  Serial.print(a.y, 2); Serial.print(' ');
  Serial.print(a.z, 2);
  Serial.print(F("   ("));
  Serial.print(MassmoreBNO08x::accuracyToString(imu.getAccuracy(MASSMORE_SENSOR_ACCELEROMETER)));
  Serial.println(')');

  Serial.print(F("Gyro  [deg/s] "));
  Serial.print(g.x, 2); Serial.print(' ');
  Serial.print(g.y, 2); Serial.print(' ');
  Serial.println(g.z, 2);

  Serial.print(F("Mag      [uT] "));
  Serial.print(m.x, 2); Serial.print(' ');
  Serial.print(m.y, 2); Serial.print(' ');
  Serial.print(m.z, 2);
  Serial.print(F("   ("));
  Serial.print(MassmoreBNO08x::accuracyToString(imu.getAccuracy(MASSMORE_SENSOR_MAGNETIC_FIELD)));
  Serial.println(')');

  Serial.print(F("LinAcc [m/s2] "));
  Serial.print(la.x, 2); Serial.print(' ');
  Serial.print(la.y, 2); Serial.print(' ');
  Serial.println(la.z, 2);

  Serial.print(F("Gravity[m/s2] "));
  Serial.print(gv.x, 2); Serial.print(' ');
  Serial.print(gv.y, 2); Serial.print(' ');
  Serial.println(gv.z, 2);

  Serial.println();
}
