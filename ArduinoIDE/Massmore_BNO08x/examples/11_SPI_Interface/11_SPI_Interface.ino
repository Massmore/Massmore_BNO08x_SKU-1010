/*
  11 — SPI interface
  Massmore BNO08x Library

  SPI is the right choice when you want the highest report rates: it has no
  address phase, no clock stretching, and it can move a whole SHTP cargo in one
  chip select assertion. The BNO08x runs SPI at up to 3 MHz in mode 3
  (CPOL = 1, CPHA = 1).

  WIRING — pad names as printed on the Massmore Halley V2 board.
  This matters: SPI mode is latched at reset.
    Halley V2         ESP32
    ---------         -----
    3Vo         ->    3V3       (or 5V pad -> 5V, the board regulates it)
    GND         ->    GND
    SCL         ->    GPIO 18   SCK
    SDA         ->    GPIO 19   MISO
    DI          ->    GPIO 23   MOSI   (same pad as SA0 in I2C mode)
    CS          ->    GPIO 15
    INT         ->    GPIO 4    REQUIRED
    RST         ->    GPIO 5    REQUIRED
    P0          ->    GPIO 16   WAKE   (also must be HIGH at reset)
    P1          ->    3Vo
    BT          ->    leave open (its pull-up keeps it out of bootloader)

  Both P1 and P0 must be high from before the reset until after the first
  H_INTN assertion — that is what selects SPI. The library drives P0 high for
  you if you give it the wakePin. After reset, P0 doubles as the active-low
  WAKE signal.

  Note: only SDA and SCL pass through the board's 2N7002 level shifter. DI and
  CS are 3.3 V only, so drive this bus from a 3.3 V host.

  Product: https://www.massmore.shop
*/

#include <SPI.h>
#include <Massmore_BNO08x.h>

#define SPI_SCK   18     // -> Halley V2 SCL
#define SPI_MISO  19     // -> Halley V2 SDA
#define SPI_MOSI  23     // -> Halley V2 DI
#define CS_PIN    15     // -> Halley V2 CS
#define INT_PIN    4
#define RST_PIN    5
#define WAKE_PIN  16     // Halley V2 P0 (WAKE), set to -1 if you tied P0 to 3Vo

MassmoreBNO08x imu;
uint32_t lastPrint = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) { }

  Serial.println(F("\nMassmore BNO08x - SPI"));

#if defined(ARDUINO_ARCH_ESP32)
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, CS_PIN);
#else
  SPI.begin();
#endif

  // 3 MHz is the BNO08x maximum. Drop to 1 MHz if your wiring is long.
  if (!imu.beginSPI(CS_PIN, INT_PIN, RST_PIN, WAKE_PIN, SPI, 3000000UL)) {
    Serial.print(F("BNO08x not found on SPI: "));
    Serial.println(MassmoreBNO08x::statusToString(imu.getLastError()));
    Serial.println(F("Check that P1 and P0 were both HIGH during reset."));
    while (1) delay(100);
  }

  const massmore_product_id_t &id = imu.getProductID();
  Serial.print(F("Connected. Firmware "));
  Serial.print(id.swVersionMajor); Serial.print('.');
  Serial.print(id.swVersionMinor); Serial.print('.');
  Serial.print(id.swVersionPatch);
  Serial.print(F("  part ")); Serial.println(id.swPartNumber);

  // SPI comfortably handles rates that would saturate I2C.
  imu.enableRotationVector(2500);     // 400 Hz
  imu.enableAccelerometer(5000);      // 200 Hz
  imu.enableGyroscope(5000);          // 200 Hz
}

void loop() {
  imu.updateAll(32);

  if (millis() - lastPrint < 100) return;
  lastPrint = millis();

  massmore_euler_t e = imu.getEulerDeg();
  massmore_vec3_t  a = imu.getAccel();

  Serial.print(F("rpy "));
  Serial.print(e.roll, 1);  Serial.print(' ');
  Serial.print(e.pitch, 1); Serial.print(' ');
  Serial.print(e.yaw, 1);
  Serial.print(F("   accel "));
  Serial.print(a.x, 2); Serial.print(' ');
  Serial.print(a.y, 2); Serial.print(' ');
  Serial.println(a.z, 2);
}
