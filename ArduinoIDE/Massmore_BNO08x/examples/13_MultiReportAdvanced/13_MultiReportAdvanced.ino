/*
  13 — Advanced: many reports, a callback, batching and power modes
  Massmore BNO08x Library

  This one puts the whole API together:

    * ten reports enabled at once, each at its own rate
    * a report callback so your code reacts the moment data lands, instead of
      polling flags
    * report-on-change so a slow sensor only speaks when something happens
    * batching, which lets the sensor buffer several samples and hand them over
      in one burst — fewer wakeups, less bus traffic
    * sleep and wake through the SHTP executable channel
    * per-report accuracy and timestamps

  Serial commands:
    s  put the sensor to sleep (only wake/always-on sensors keep running)
    o  wake it back up
    d  dump the current configuration of every enabled report
    r  soft reset

  Product: https://www.massmore.shop
*/

#include <Wire.h>
#include <Massmore_BNO08x.h>

#define SDA_PIN   21
#define SCL_PIN   22
#define INT_PIN    4
#define RST_PIN    5

MassmoreBNO08x imu;

uint32_t quatCount = 0;
uint32_t accelCount = 0;
uint32_t lastPrint = 0;

// The callback runs inside update(), from your own loop() context — it is not
// an ISR, so Serial and floats are safe here. Keep it short anyway.
void onReport(uint8_t reportId, void *ctx) {
  (void)ctx;
  switch (reportId) {
    case MASSMORE_SENSOR_ROTATION_VECTOR: quatCount++;  break;
    case MASSMORE_SENSOR_ACCELEROMETER:   accelCount++; break;
    case MASSMORE_SENSOR_TAP_DETECTOR:    Serial.println(F("  >> tap!"));   break;
    case MASSMORE_SENSOR_STEP_DETECTOR:   Serial.println(F("  >> step!"));  break;
    default: break;
  }
}

void configureSensors() {
  // --- streaming reports ---------------------------------------------------
  imu.enableRotationVector(10000);         // 100 Hz
  imu.enableAccelerometer(20000);          //  50 Hz
  imu.enableGyroscope(20000);              //  50 Hz
  imu.enableLinearAcceleration(50000);     //  20 Hz

  // --- batched report ------------------------------------------------------
  // 100 Hz samples, delivered in bursts every 500 ms. The sensor buffers them
  // internally, so the host is interrupted twice a second instead of a hundred
  // times. Great for battery powered loggers.
  imu.setFeature(MASSMORE_SENSOR_MAGNETIC_FIELD,
                 10000,                    // report interval: 100 Hz
                 500000,                   // batch interval: 500 ms
                 MASSMORE_FEATURE_FLAG_NONE);

  // --- report on change ----------------------------------------------------
  // The gravity vector only speaks when it moves by more than the change
  // sensitivity. Perfectly still device = zero bus traffic.
  imu.setFeature(MASSMORE_SENSOR_GRAVITY,
                 20000,                    // 50 Hz ceiling
                 0,
                 MASSMORE_FEATURE_FLAG_CHANGE_SENS_ENA,
                 100);                     // threshold, in the report's own Q8 units

  // --- wake sensors --------------------------------------------------------
  // Flagged as wake sensors, these keep running through modeSleep() and their
  // reports arrive on SHTP channel 4 instead of channel 3.
  imu.setFeature(MASSMORE_SENSOR_TAP_DETECTOR, 10000, 0,
                 MASSMORE_FEATURE_FLAG_WAKE_ENABLED);
  imu.setFeature(MASSMORE_SENSOR_SIGNIFICANT_MOTION, 200000, 0,
                 MASSMORE_FEATURE_FLAG_WAKE_ENABLED);

  // --- low rate engines ----------------------------------------------------
  imu.enableStepDetector(200000);
  imu.enableStepCounter(500000);
  imu.enableStabilityClassifier(500000);
}

void dumpConfig() {
  Serial.println(F("\n--- report configuration (read back from the device) ---"));
  const uint8_t ids[] = {
    MASSMORE_SENSOR_ROTATION_VECTOR, MASSMORE_SENSOR_ACCELEROMETER,
    MASSMORE_SENSOR_GYROSCOPE,       MASSMORE_SENSOR_MAGNETIC_FIELD,
    MASSMORE_SENSOR_GRAVITY,         MASSMORE_SENSOR_TAP_DETECTOR,
    MASSMORE_SENSOR_STEP_COUNTER
  };
  for (uint8_t i = 0; i < sizeof(ids); i++) {
    imu.requestFeature(ids[i]);
    Serial.print(F("  report 0x"));
    if (ids[i] < 16) Serial.print('0');
    Serial.print(ids[i], HEX);
    Serial.print(F("  interval "));
    uint32_t us = imu.getReportInterval(ids[i]);
    Serial.print(us);
    Serial.print(F(" us"));
    if (us) { Serial.print(F("  (")); Serial.print(1000000UL / us); Serial.print(F(" Hz)")); }
    Serial.print(F("  accuracy "));
    Serial.println(MassmoreBNO08x::accuracyToString(imu.getAccuracy(ids[i])));
  }
  Serial.println();
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

  Serial.print(F("Verification: "));
  Serial.println(MassmoreBNO08x::authToString(imu.verifyChip()));

  imu.setReportCallback(onReport);
  configureSensors();

  Serial.println(F("Massmore BNO08x - advanced multi-report demo"));
  Serial.println(F("s=sleep  o=on  d=dump config  r=reset"));
}

void loop() {
  imu.updateAll(16);

  if (Serial.available()) {
    switch (Serial.read()) {
      case 's':
        imu.modeSleep();
        Serial.println(F("> sleeping - only tap and significant motion still run"));
        break;
      case 'o':
        imu.modeOn();
        Serial.println(F("> awake"));
        break;
      case 'd':
        dumpConfig();
        break;
      case 'r':
        Serial.println(F("> soft reset"));
        imu.softReset();
        configureSensors();
        Serial.println(F("  sensors reconfigured"));
        break;
      default: break;
    }
  }

  if (millis() - lastPrint < 1000) return;
  lastPrint = millis();

  massmore_euler_t e = imu.getEulerDeg();

  Serial.print(F("rpy "));
  Serial.print(e.roll, 1);  Serial.print(' ');
  Serial.print(e.pitch, 1); Serial.print(' ');
  Serial.print(e.yaw, 1);
  Serial.print(F("   RV ")); Serial.print(quatCount);  Serial.print(F(" Hz"));
  Serial.print(F("   accel ")); Serial.print(accelCount); Serial.print(F(" Hz"));
  Serial.print(F("   steps ")); Serial.print(imu.getStepCount());
  Serial.print(F("   ")); Serial.print(imu.getStabilityString());
  Serial.print(F("   t=")); Serial.print((uint32_t)imu.getTimestampUs());
  Serial.println(F(" us"));

  quatCount = 0;
  accelCount = 0;
}
