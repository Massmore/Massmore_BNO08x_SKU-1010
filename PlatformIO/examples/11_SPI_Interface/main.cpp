#include <Arduino.h>
/*
  11 — SPI interface
  Massmore BNO08x Library

  SPI is the right choice when you want the highest report rates: it has no
  address phase, no clock stretching, and it can move a whole SHTP cargo in one
  chip select assertion. The BNO08x runs SPI at up to 3 MHz in mode 3
  (CPOL = 1, CPHA = 1).

  WIRING — this matters, SPI mode is latched at reset
    BNO08x            ESP32
    ------            -----
    VIN         ->    3V3
    GND         ->    GND
    SCK/SCL     ->    GPIO 18   (SCK)
    MISO/SA0    ->    GPIO 19
    MOSI/SDA    ->    GPIO 23
    CS          ->    GPIO 15
    INT         ->    GPIO 4    REQUIRED
    RST         ->    GPIO 5    REQUIRED
    PS0/WAKE    ->    GPIO 16   (also must be HIGH at reset)
    PS1         ->    3V3
    BOOTN       ->    3V3 through 10k

  Both PS1 and PS0 must be high from before the reset until after the first
  H_INTN assertion — that is what selects SPI. The library drives PS0 high for
  you if you give it the wakePin. After reset, PS0 doubles as the active-low
  WAKE signal.

  Product: https://www.massmore.shop
*/

#include <SPI.h>
#include <Massmore_BNO08x.h>

#define SPI_SCK   18
#define SPI_MISO  19
#define SPI_MOSI  23
#define CS_PIN    15
#define INT_PIN    4
#define RST_PIN    5
#define WAKE_PIN  16     // PS0/WAKE, set to -1 if you tied PS0 to 3V3

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
    Serial.println(F("Check that PS1 and PS0 were both HIGH during reset."));
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
