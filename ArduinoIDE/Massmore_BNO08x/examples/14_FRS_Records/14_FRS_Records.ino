/*
  14 — FRS: reading the sensor's flash records
  Massmore BNO08x Library

  The BNO08x keeps its persistent configuration in a Flash Record System. Each
  record has a 16-bit ID and holds an array of 32-bit words. This is where the
  serial number, the calibration data, the mounting orientation and every
  sensor's metadata live.

  This sketch dumps the interesting ones. It is read-only by default — writing
  a bad orientation or calibration record will make the fusion output nonsense
  until you restore it, so the write path is commented out and guarded.

  Useful records (Datasheet Figure 1-31):
    0x4B4B  serial number
    0x2D3E  system orientation      <- this is what persistTare() writes
    0x7979  static calibration, accel/gyro/mag
    0x1F1F  dynamic calibration (DCD)
    0xD3E2  MotionEngine power management
    0x74B4  user record - 64 words of your own, untouched by the firmware
    0xE30B  rotation vector metadata

  Product: https://www.massmore.shop
*/

#include <Wire.h>
#include <Massmore_BNO08x.h>

#define SDA_PIN   21
#define SCL_PIN   22
#define INT_PIN    4
#define RST_PIN    5

MassmoreBNO08x imu;

void dumpRecord(const char *name, uint16_t recordId, uint16_t maxWords) {
  uint32_t buf[32];
  if (maxWords > 32) maxWords = 32;
  uint16_t words = 0;

  Serial.print(F("\n["));
  Serial.print(name);
  Serial.print(F("]  record 0x"));
  Serial.println(recordId, HEX);

  massmore_status_t rc = imu.readFrsRecord(recordId, buf, maxWords, words);
  if (rc != MASSMORE_OK) {
    Serial.print(F("  read failed: "));
    Serial.println(MassmoreBNO08x::statusToString(rc));
    return;
  }
  if (words == 0) {
    Serial.println(F("  (empty)"));
    return;
  }

  for (uint16_t i = 0; i < words; i++) {
    if (i % 4 == 0) {
      Serial.print(F("  ["));
      if (i < 10) Serial.print(' ');
      Serial.print(i);
      Serial.print(F("] "));
    }
    Serial.print(F("0x"));
    for (int8_t s = 28; s >= 0; s -= 4) {
      Serial.print((buf[i] >> s) & 0xF, HEX);
    }
    Serial.print(' ');
    if (i % 4 == 3) Serial.println();
  }
  if (words % 4) Serial.println();
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

  Serial.println(F("\n===== Massmore BNO08x - FRS dump ====="));

  dumpRecord("Serial number",        MASSMORE_FRS_SERIAL_NUMBER,       4);
  dumpRecord("System orientation",   MASSMORE_FRS_SYSTEM_ORIENTATION,  8);
  dumpRecord("Static cal AGM",       MASSMORE_FRS_STATIC_CAL_AGM,     16);
  dumpRecord("Dynamic cal (DCD)",    MASSMORE_FRS_DYNAMIC_CAL,        16);
  dumpRecord("ME power management",  MASSMORE_FRS_ME_POWER_MGMT,       8);
  dumpRecord("User record",          MASSMORE_FRS_USER_RECORD,        16);
  dumpRecord("RV metadata",          MASSMORE_FRS_META_ROTATION_VECTOR, 16);
  dumpRecord("Accel metadata",       MASSMORE_FRS_META_ACCELEROMETER,  16);

  /* ---------------------------------------------------------------------
     WRITING A RECORD

     The user record (0x74B4) is the only one the firmware never touches, so
     it is the safe place to keep your own serial number, board revision or
     assembly date. Uncomment to try it.

       uint32_t mine[4] = { 0x4D41534DUL, 0x4F52454DUL, 20260831UL, 0x0001UL };
       massmore_status_t rc = imu.writeFrsRecord(MASSMORE_FRS_USER_RECORD, mine, 4);
       Serial.print(F("write: "));
       Serial.println(MassmoreBNO08x::statusToString(rc));

     Do NOT write 0x7979, 0x1F1F or 0x2D3E by hand unless you know exactly
     what you are putting there. Use calibrateAll() + saveCalibration() and
     tareNow() + persistTare() instead — they build the record contents for
     you and the firmware validates them.
     --------------------------------------------------------------------- */

  Serial.println(F("\nDone."));
}

void loop() {
  delay(1000);
}
