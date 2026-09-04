/*
  01 — Basic Rotation Vector
  Massmore BNO08x Library

  The "hello world" of the BNO085/BNO086: enable the 9-axis fused rotation
  vector and print the quaternion at 100 Hz.

  Wiring (I2C) — pad names as printed on the Massmore Halley V2 board
    Halley V2      ESP32
    ---------      -----
    5V or 3Vo ->   5V or 3V3
    GND       ->   GND
    SDA       ->   GPIO 21  (see SDA_PIN below)
    SCL       ->   GPIO 22  (see SCL_PIN below)
    INT       ->   GPIO 4   (optional but recommended)
    RST       ->   GPIO 5   (optional)
    DI        ->   leave open for 0x4B, tie to GND for 0x4A  (the chip's SA0)
    BT, P0, P1 -> leave open (their pull-ups select I2C mode)

  Works on Arduino IDE with esp32 core 3.x, and on PlatformIO.
  Product: https://www.massmore.shop
*/

#include <Wire.h>
#include <Massmore_BNO08x.h>

// ---- adjust these for your board -------------------------------------------
#define SDA_PIN   21
#define SCL_PIN   22
#define INT_PIN    4   // set to -1 if not wired
#define RST_PIN    5   // set to -1 if not wired
// ----------------------------------------------------------------------------

MassmoreBNO08x imu;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) { }

  Serial.println(F("\nMassmore BNO08x - Basic Rotation Vector"));

#if defined(ARDUINO_ARCH_ESP32)
  Wire.begin(SDA_PIN, SCL_PIN);
#else
  Wire.begin();
#endif
  // Start at 100 kHz. The BNO08x stretches the clock, and ESP32 and RP2040
  // hosts are widely reported to drop bytes at 400 kHz. Raise it later.
  Wire.setClock(100000);

  if (!imu.begin(MASSMORE_BNO08X_I2C_ADDR_DEF, Wire, INT_PIN, RST_PIN)) {
    Serial.print(F("BNO08x not found: "));
    Serial.println(MassmoreBNO08x::statusToString(imu.getLastError()));
    Serial.println(F("Check wiring, and try address 0x4A if the DI pad is tied low."));
    while (1) delay(100);
  }

  Serial.println(F("BNO08x connected."));

  // 10000 us = 10 ms = 100 Hz
  imu.enableRotationVector(10000);
}

void loop() {
  if (!imu.update()) return;                                  // nothing new
  if (!imu.hasNewReport(MASSMORE_SENSOR_ROTATION_VECTOR)) return;

  massmore_quat_t q = imu.getQuaternion();

  Serial.print(F("i="));    Serial.print(q.i, 4);
  Serial.print(F("  j="));  Serial.print(q.j, 4);
  Serial.print(F("  k="));  Serial.print(q.k, 4);
  Serial.print(F("  w="));  Serial.print(q.real, 4);
  Serial.print(F("  accuracy="));
  Serial.print(q.accuracy * 57.2957795f, 1);                  // radians -> degrees
  Serial.print(F(" deg  ["));
  Serial.print(MassmoreBNO08x::accuracyToString(
                 imu.getAccuracy(MASSMORE_SENSOR_ROTATION_VECTOR)));
  Serial.println(F("]"));
}
