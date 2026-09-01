#include <Arduino.h>
/*
  09 — Raw sensor data (CSV logger)
  Massmore BNO08x Library

  The raw reports bypass the MotionEngine entirely and hand you the ADC counts
  straight off the accelerometer, gyroscope and magnetometer dies, together
  with the sensor's own microsecond timestamp. Use them when you want to run
  your own filter, log data for offline analysis, or characterise the part.

  Output is CSV, so you can paste it straight into a spreadsheet or capture it
  with a serial logger:

    micros,ax,ay,az,gx,gy,gz,gtemp,mx,my,mz

  Values are ADC counts, not physical units. If you want m/s^2 and rad/s, use
  the calibrated reports in example 03 instead.

  Product: https://www.massmore.shop
*/

#include <Wire.h>
#include <Massmore_BNO08x.h>

#define SDA_PIN   21
#define SCL_PIN   22
#define INT_PIN    4
#define RST_PIN    5

MassmoreBNO08x imu;

void setup() {
  Serial.begin(921600);       // raw logging wants a fast port
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

  // 200 Hz raw. Push higher only if your bus and print rate can keep up:
  // at 400 Hz with all three sensors you are moving ~20 kB/s over I2C.
  imu.enableRawAccelerometer(5000);
  imu.enableRawGyroscope(5000);
  imu.enableRawMagnetometer(20000);   // the mag tops out much lower

  Serial.println(F("micros,ax,ay,az,gx,gy,gz,gtemp,mx,my,mz"));
}

void loop() {
  if (!imu.update()) return;

  // Emit one CSV row per raw accelerometer sample; the other two are sampled
  // at whatever their latest value is.
  if (!imu.hasNewReport(MASSMORE_SENSOR_RAW_ACCELEROMETER)) return;

  massmore_vec3i_t a = imu.getRawAccel();
  massmore_vec3i_t g = imu.getRawGyro();
  massmore_vec3i_t m = imu.getRawMag();

  Serial.print((uint32_t)imu.getTimestampUs()); Serial.print(',');
  Serial.print(a.x); Serial.print(',');
  Serial.print(a.y); Serial.print(',');
  Serial.print(a.z); Serial.print(',');
  Serial.print(g.x); Serial.print(',');
  Serial.print(g.y); Serial.print(',');
  Serial.print(g.z); Serial.print(',');
  Serial.print(imu.getRawGyroTemperature()); Serial.print(',');
  Serial.print(m.x); Serial.print(',');
  Serial.print(m.y); Serial.print(',');
  Serial.println(m.z);
}
