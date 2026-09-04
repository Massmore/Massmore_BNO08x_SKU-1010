/*
  04 — Chip ID and authenticity check
  Massmore BNO08x Library

  Reads everything the part will tell you about itself:

    * Product ID Response (report 0xF8) — reset cause, firmware version,
      firmware part number and build number
    * FRS record 0x4B4B — the factory programmed serial number
    * FRS record 0xE30B — the rotation vector metadata, which proves the
      MotionEngine is really running

  WHAT THE AUTHENTICITY CHECK PROVES
  The BNO08x has no cryptographic attestation, so nothing can give you a
  mathematical proof. What verifyChip() proves is that the part on your board
  behaves exactly like genuine CEVA/Bosch silicon at the protocol level: it
  advertises its SHTP channels, answers a Product ID Request with a plausible
  factory firmware version, and holds a readable serial number in flash.
  A relabelled BNO055, a blank die, or an empty footprint fails one of these.

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

  Serial.println(F("\n=============================================="));
  Serial.println(F(" Massmore BNO08x - Chip identity & verification"));
  Serial.println(F("=============================================="));

#if defined(ARDUINO_ARCH_ESP32)
  Wire.begin(SDA_PIN, SCL_PIN);
#else
  Wire.begin();
#endif
  Wire.setClock(100000);   // see example 01 on why not 400 kHz

  imu.enableDebug(Serial);     // show what the driver sees

  if (!imu.begin(MASSMORE_BNO08X_I2C_ADDR_DEF, Wire, INT_PIN, RST_PIN)) {
    Serial.print(F("FAILED: "));
    Serial.println(MassmoreBNO08x::statusToString(imu.getLastError()));
    Serial.println(F("Nothing answered. Either the sensor is not on this bus,"));
    Serial.println(F("or the DI pad selects 0x4A instead of 0x4B."));
    while (1) delay(100);
  }

  // ---- 1. Product ID -------------------------------------------------------
  const massmore_product_id_t &id = imu.getProductID();

  Serial.println(F("\n--- Product ID (report 0xF8) ---"));
  Serial.print(F("Reset cause      : "));
  Serial.print(id.resetCause);
  Serial.print(F("  ("));
  Serial.print(imu.getResetReasonString());
  Serial.println(')');

  Serial.print(F("Firmware version : "));
  Serial.print(id.swVersionMajor); Serial.print('.');
  Serial.print(id.swVersionMinor); Serial.print('.');
  Serial.println(id.swVersionPatch);

  Serial.print(F("SW part number   : "));
  Serial.println(id.swPartNumber);

  Serial.print(F("SW build number  : "));
  Serial.println(id.swBuildNumber);

  // ---- 2. Authenticity -----------------------------------------------------
  Serial.println(F("\n--- Authenticity ---"));
  massmore_auth_t auth = imu.verifyChip();
  Serial.print(F("Result: "));
  Serial.println(MassmoreBNO08x::authToString(auth));

  switch (auth) {
    case MASSMORE_AUTH_OK:
      Serial.println(F("=> Genuine BNO08x running factory firmware."));
      break;
    case MASSMORE_AUTH_UNKNOWN_FW:
      Serial.println(F("=> Behaves like a real BNO08x, but the firmware part"));
      Serial.println(F("   number is newer than this library's table. Note the"));
      Serial.println(F("   number above and treat it as genuine."));
      break;
    default:
      Serial.println(F("=> Suspicious. Do not trust this part."));
      break;
  }

  // ---- 3. Serial number ----------------------------------------------------
  Serial.println(F("\n--- Factory serial number (FRS 0x4B4B) ---"));
  uint64_t serial = 0;
  if (imu.readSerialNumber(serial) == MASSMORE_OK) {
    // Arduino's print() has no 64-bit overload, so split it.
    Serial.print(F("Serial: 0x"));
    Serial.print((uint32_t)(serial >> 32), HEX);
    Serial.println((uint32_t)(serial & 0xFFFFFFFFUL), HEX);
  } else {
    Serial.print(F("Could not read: "));
    Serial.println(MassmoreBNO08x::statusToString(imu.getLastError()));
  }

  // ---- 4. Metadata: proof the MotionEngine is alive -------------------------
  Serial.println(F("\n--- Rotation vector metadata (FRS 0xE30B) ---"));
  uint32_t meta[16];
  uint16_t words = 0;
  if (imu.readSensorMetadata(MASSMORE_FRS_META_ROTATION_VECTOR, meta, 16, words)
        == MASSMORE_OK && words >= 8) {
    Serial.print(F("Words read     : ")); Serial.println(words);
    Serial.print(F("Version        : 0x")); Serial.println(meta[0], HEX);
    Serial.print(F("Min period (us): ")); Serial.println(meta[4]);
    Serial.print(F("Q point 1      : ")); Serial.println(meta[7] & 0xFFFF);
    Serial.print(F("Q point 2      : ")); Serial.println((meta[7] >> 16) & 0xFFFF);
    Serial.println(F("Q point 1 should be 14 and Q point 2 should be 12 on a"));
    Serial.println(F("genuine part - those are the quaternion and accuracy scales."));
  } else {
    Serial.println(F("Metadata read failed."));
  }

  Serial.println(F("\nDone."));
}

void loop() {
  delay(1000);
}
