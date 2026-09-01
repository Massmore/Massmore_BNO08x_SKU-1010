#include <Arduino.h>
/*
  12 — UART-RVC mode
  Massmore BNO08x Library

  UART-RVC is the simplest possible way to use this sensor. Strap two pins,
  connect ONE wire, and the BNO08x streams heading and acceleration at 100 Hz
  forever with no commands from the host. No I2C, no SHTP, no configuration.

  It is designed for ground robots — the R, V and C stand for Robot Vacuum
  Cleaner — and it is ideal any time you just want a heading.

  WIRING
    BNO08x            ESP32
    ------            -----
    VIN         ->    3V3
    GND         ->    GND
    TX          ->    GPIO 16   (the ESP32's RX)
    PS1         ->    GND
    PS0         ->    3V3
    BOOTN       ->    3V3 through 10k
    RST         ->    3V3 (or a GPIO)

  The sensor needs its external crystal or clock in this mode; the internal
  oscillator is not accurate enough for the UART.

  WHAT YOU GET / WHAT YOU GIVE UP
    yes: yaw, pitch, roll (0.01 deg), 3-axis acceleration, 100 Hz, one wire
    no:  quaternion, calibration control, tare, any other sensor report

  Product: https://www.massmore.shop
*/

#include <Massmore_BNO08x_RVC.h>

#define RVC_RX_PIN  16     // ESP32 pin connected to the sensor's TX
#define RVC_TX_PIN  17     // unused by RVC, but Serial1 wants a pin

MassmoreBNO08x_RVC rvc;
uint32_t lastPrint = 0;
uint8_t  lastIndex = 0;
uint32_t dropped = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) { }

#if defined(ARDUINO_ARCH_ESP32)
  Serial1.begin(115200, SERIAL_8N1, RVC_RX_PIN, RVC_TX_PIN);
#else
  Serial1.begin(115200);
#endif

  rvc.begin(Serial1);

  Serial.println(F("Massmore BNO08x - UART-RVC"));
  Serial.println(F("yaw\tpitch\troll\tax\tay\taz"));
}

void loop() {
  massmore_rvc_report_t r;

  if (rvc.read(r)) {
    // The index byte increments once per frame; a gap means we missed one.
    uint8_t expected = (uint8_t)(lastIndex + 1);
    if (lastIndex != 0 && r.index != expected) dropped++;
    lastIndex = r.index;

    if (millis() - lastPrint >= 100) {          // print at 10 Hz
      lastPrint = millis();
      Serial.print(r.yaw, 2);    Serial.print('\t');
      Serial.print(r.pitch, 2);  Serial.print('\t');
      Serial.print(r.roll, 2);   Serial.print('\t');
      Serial.print(r.accelX, 2); Serial.print('\t');
      Serial.print(r.accelY, 2); Serial.print('\t');
      Serial.print(r.accelZ, 2);

      if (dropped || rvc.getChecksumErrors()) {
        Serial.print(F("\t(dropped "));
        Serial.print(dropped);
        Serial.print(F(", bad csum "));
        Serial.print(rvc.getChecksumErrors());
        Serial.print(')');
      }
      Serial.println();
    }
  }
}
