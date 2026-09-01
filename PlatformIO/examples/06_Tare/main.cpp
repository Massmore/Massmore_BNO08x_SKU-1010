#include <Arduino.h>
/*
  06 — Tare (defining "forward")
  Massmore BNO08x Library

  Tare tells the sensor "the way I am pointing right now is zero". It is what
  a VR headset does when you press the recenter button, and what you run once
  at the factory to cancel out how the PCB is mounted inside a product.

  Two useful flavours, both from CEVA document 1000-4045:

    Z-axis only  — recenters the heading. Does not care which way North is, so
                   the user can press it any time. This is the "recenter" button.
    All axes     — full alignment. Only meaningful when the device is level and
                   physically pointing North, because it also redefines what
                   level means. Do this once at the factory, then persist it.

  Serial commands:
    z  tare the Z axis only (recenter heading)
    a  tare all three axes (point the device North and level first!)
    p  persist the current tare into flash (FRS System Orientation record)
    c  clear the stored tare
    g  tare Z using the Game Rotation Vector as the basis

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

  imu.enableRotationVector(10000);
  imu.enableGameRotationVector(10000);

  Serial.println(F("Massmore BNO08x - tare demo"));
  Serial.println(F("z=tare Z  a=tare all  p=persist  c=clear  g=tare Z on game RV"));
  Serial.println(F("\nCalibrate first (see example 05) or the tare will bake in"
                   " a bad heading."));
}

void loop() {
  imu.updateAll();

  if (Serial.available()) {
    switch (Serial.read()) {
      case 'z':
        imu.tareNow(MASSMORE_TARE_AXIS_Z, MASSMORE_TARE_BASIS_ROTATION_VECTOR);
        Serial.println(F("> heading recentered (Z axis only)"));
        break;

      case 'a':
        imu.tareNow(MASSMORE_TARE_AXIS_ALL, MASSMORE_TARE_BASIS_ROTATION_VECTOR);
        Serial.println(F("> full tare on all three axes"));
        break;

      case 'g':
        imu.tareNow(MASSMORE_TARE_AXIS_Z, MASSMORE_TARE_BASIS_GAMING_RV);
        Serial.println(F("> Z tare using the game rotation vector as basis"));
        break;

      case 'p':
        imu.persistTare();
        Serial.println(F("> tare written to the System Orientation FRS record"));
        Serial.println(F("  it will now be the default after every reboot"));
        break;

      case 'c':
        imu.clearTare();
        Serial.println(F("> stored tare cleared (reorientation set to identity)"));
        break;

      default:
        break;
    }
  }

  if (millis() - lastPrint < 200) return;
  lastPrint = millis();

  massmore_euler_t e = imu.getEulerDeg();
  Serial.print(F("roll ")); Serial.print(e.roll, 1);
  Serial.print(F("  pitch ")); Serial.print(e.pitch, 1);
  Serial.print(F("  yaw ")); Serial.print(e.yaw, 1);
  Serial.print(F("  heading ")); Serial.print(imu.getHeadingDeg(), 1);
  Serial.println(F(" deg"));
}
