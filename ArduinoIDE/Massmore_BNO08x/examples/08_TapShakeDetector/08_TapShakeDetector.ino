/*
  08 — Gesture detectors: tap, shake, pickup, flip, tilt
  Massmore BNO08x Library

  Every one of these is an event, not a stream: the sensor sends a report only
  when the gesture happens. The library latches the flags for you, and the
  getters clear the latch when you read them, so a gesture is never reported
  twice and never lost between calls.

  Tap flags tell you which axis and direction, and whether it was a double tap.
  Shake flags tell you which axes were shaken.

  Note: how sensitive the tap detector is depends on how the board is mounted.
  A sensor glued to a thin plastic case will trigger far more easily than one
  bolted to an aluminium chassis. Tune it with the Shake Detector FRS
  configuration record (0x7D7D) if you need to.

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

  imu.enableTapDetector(10000);
  imu.enableShakeDetector(50000);
  imu.enablePickupDetector(100000);
  imu.enableFlipDetector(100000);
  imu.enableTiltDetector(100000);
  imu.enableStabilityDetector(100000);

  Serial.println(F("Massmore BNO08x - gesture detectors"));
  Serial.println(F("Tap the board, shake it, pick it up, turn it over."));
}

void loop() {
  imu.updateAll();

  uint8_t tap = imu.getTapDetector();
  if (tap) {
    Serial.print(F("[TAP]"));
    if (tap & MASSMORE_TAP_X_POS) Serial.print(F(" +X"));
    if (tap & MASSMORE_TAP_X_NEG) Serial.print(F(" -X"));
    if (tap & MASSMORE_TAP_Y_POS) Serial.print(F(" +Y"));
    if (tap & MASSMORE_TAP_Y_NEG) Serial.print(F(" -Y"));
    if (tap & MASSMORE_TAP_Z_POS) Serial.print(F(" +Z"));
    if (tap & MASSMORE_TAP_Z_NEG) Serial.print(F(" -Z"));
    Serial.println((tap & MASSMORE_TAP_DOUBLE) ? F("  (double tap)")
                                               : F("  (single tap)"));
  }

  uint16_t shake = imu.getShakeDetector();
  if (shake) {
    Serial.print(F("[SHAKE]"));
    if (shake & MASSMORE_SHAKE_X) Serial.print(F(" X"));
    if (shake & MASSMORE_SHAKE_Y) Serial.print(F(" Y"));
    if (shake & MASSMORE_SHAKE_Z) Serial.print(F(" Z"));
    Serial.println();
  }

  if (imu.getPickupDetected()) Serial.println(F("[PICKUP]  device was lifted"));
  if (imu.getFlipDetected())   Serial.println(F("[FLIP]    device was turned over"));
  if (imu.getTiltDetected())   Serial.println(F("[TILT]    significant tilt"));
  if (imu.getStabilityChanged()) {
    Serial.print(F("[STABILITY] now "));
    Serial.println(imu.getStabilityString());
  }
}
