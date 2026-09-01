/*
  07 — Step counter, activity classifier and stability
  Massmore BNO08x Library

  These are the "always on, low power" engines inside the MotionEngine. They do
  not need a magnetometer and they run at very low report rates, so they cost
  almost nothing to leave enabled.

    Step counter          cumulative step count since boot
    Step detector         fires once per step
    Activity classifier   still / walking / running / cycling / in vehicle /
                          on stairs, each with a 0..100 confidence
    Stability classifier  on table / stationary / stable / in motion
    Significant motion    one-shot "the device really moved" event

  Carry the board in your hand and walk around to see the classifier work.
  Give it 10-20 seconds — these engines deliberately average over time.

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
uint32_t stepsAtStart = 0;

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

  imu.enableStepCounter(200000);          // 5 Hz is plenty for a counter
  imu.enableStepDetector(200000);
  imu.enableStabilityClassifier(500000);  // 2 Hz
  imu.enableSignificantMotion(500000);

  // Bit n of the mask enables activity n. Here: unknown, in vehicle,
  // on bicycle, on foot, still, tilting, walking, running, on stairs.
  imu.enableActivityClassifier(500000, 0x1FF);

  Serial.println(F("Massmore BNO08x - activity monitor"));
  Serial.println(F("Pick the board up and walk with it."));
}

void loop() {
  imu.updateAll();

  // Event style reports: check them every loop so none are missed.
  if (imu.getStepDetected()) {
    Serial.println(F("[event] step"));
  }
  if (imu.getSignificantMotion()) {
    Serial.println(F("[event] significant motion"));
  }
  if (imu.hasNewReport(MASSMORE_SENSOR_STABILITY_CLASSIFIER)) {
    Serial.print(F("[event] stability -> "));
    Serial.println(imu.getStabilityString());
  }

  if (millis() - lastPrint < 1000) return;
  lastPrint = millis();

  Serial.print(F("steps: "));
  Serial.print(imu.getStepCount());
  Serial.print(F("   activity: "));
  Serial.print(imu.getActivityString());
  Serial.print(F("   stability: "));
  Serial.println(imu.getStabilityString());

  // Full confidence breakdown, only for the states that scored anything.
  Serial.print(F("  confidence:"));
  const char *names[] = { "unknown", "vehicle", "bicycle", "foot",
                          "still", "tilting", "walking", "running", "stairs" };
  for (uint8_t i = 0; i < MASSMORE_ACTIVITY_COUNT; i++) {
    uint8_t c = imu.getActivityConfidence((massmore_activity_t)i);
    if (c == 0) continue;
    Serial.print(' ');
    Serial.print(names[i]);
    Serial.print('=');
    Serial.print(c);
  }
  Serial.println();
}
