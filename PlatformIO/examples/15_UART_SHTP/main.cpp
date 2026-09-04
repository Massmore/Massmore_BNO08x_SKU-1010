/* ไฟล์นี้สร้างจาก ArduinoIDE/Massmore_BNO08x/examples/15_UART_SHTP/15_UART_SHTP.ino
   แก้ที่ .ino ต้นฉบับ แล้วคัดลอกมาที่นี่
   (.cpp ไม่มีตัวสร้าง prototype อัตโนมัติแบบ .ino จึงต้อง include Arduino.h เอง) */
#include <Arduino.h>

/*
  15 — SHTP over UART
  Massmore BNO08x Library

  The full SH-2 protocol, every sensor report, every command — carried over two
  UART wires instead of I2C or SPI. This is NOT example 12 (UART-RVC): RVC is a
  one-way 100 Hz heading stream with no commands. SHTP-over-UART is the real
  thing, so you get quaternions, calibration, tare, FRS, sleep — all of it.

  Use it when the host has no free I2C or SPI bus, when the cable run is long
  enough that I2C gets unreliable, or when you want galvanic isolation (an
  opto-isolator on two UART lines is trivial; on I2C it is not).

  WIRING — Massmore Halley V2 (SKU-1010)
    Halley V2         ESP32
    ---------         -----
    3Vo         ->    3V3        (or 5V pad -> 5V, the board regulates it)
    GND         ->    GND
    SDA         ->    GPIO 16    sensor TX  -> ESP32 RX
    SCL         ->    GPIO 17    sensor RX  <- ESP32 TX
    P1          ->    3Vo        HIGH  ]  PS1 = 1, PS0 = 0
    P0          ->    GND        LOW   ]  selects SHTP-over-UART
    RST         ->    GPIO 5     (optional)
    INT         ->    GPIO 4     (optional)
    BT          ->    leave open (its pull-up keeps the part out of bootloader)

  IMPORTANT — use the Qwiic connector or the sensor-side 3.3V lines.
  The SDA and SCL pads on the 0.1 inch header sit behind the board's 2N7002
  level shifter, which is built for open-drain I2C. UART is push-pull and runs
  at 3 Mbit/s here; the shifter's pull-ups cannot slew that fast. Drive these
  two lines directly at 3.3 V — either through a Qwiic cable or from a 3.3 V
  host — and do not put a 5 V host on them in this mode.

  P1 and P0 are sampled while NRST is released, so strap them BEFORE power-up
  (or before the reset pulse). Changing them while the part is running does
  nothing.

  Baud rate is fixed at 3,000,000 by the BNO08x. It is not configurable.

  Product: https://www.massmore.shop
*/

#include <Massmore_BNO08x.h>

// ---- adjust these for your board -------------------------------------------
#define UART_RX_PIN  16   // ESP32 RX  <- Halley V2 SDA (sensor TX)
#define UART_TX_PIN  17   // ESP32 TX  -> Halley V2 SCL (sensor RX)
#define INT_PIN       4   // Halley V2 INT, set to -1 if not wired
#define RST_PIN       5   // Halley V2 RST, set to -1 if not wired
// ----------------------------------------------------------------------------

#define SHTP_UART_BAUD  3000000UL   // fixed by the BNO08x, do not change

MassmoreBNO08x imu;

uint32_t lastPrint = 0;
uint32_t quatCount = 0;
uint32_t accelCount = 0;

void onReport(uint8_t reportId, void *ctx) {
  (void)ctx;
  if (reportId == MASSMORE_SENSOR_ROTATION_VECTOR) quatCount++;
  else if (reportId == MASSMORE_SENSOR_ACCELEROMETER) accelCount++;
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) { }

  Serial.println(F("\nMassmore BNO08x - SHTP over UART"));

#if defined(ARDUINO_ARCH_ESP32)
  Serial1.begin(SHTP_UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
#else
  // On a non-ESP32 core, Serial1's pins are fixed by the board. Make sure the
  // core can actually clock 3 Mbaud — many 8-bit AVRs cannot.
  Serial1.begin(SHTP_UART_BAUD);
#endif

  // The library needs a Stream that is already open. It does the reset, waits
  // for the part to settle, then asks for the Product ID.
  if (!imu.beginUART(Serial1, INT_PIN, RST_PIN)) {
    Serial.print(F("No answer over UART: "));
    Serial.println(MassmoreBNO08x::statusToString(imu.getLastError()));
    Serial.println(F("Check that P1 is HIGH and P0 is LOW *at reset*, that SDA"));
    Serial.println(F("and SCL are not crossed, and that both lines run at 3.3 V"));
    Serial.println(F("without going through the board's level shifter."));
    while (1) delay(100);
  }

  const massmore_product_id_t &id = imu.getProductID();
  Serial.print(F("Connected. Firmware "));
  Serial.print(id.swVersionMajor); Serial.print('.');
  Serial.print(id.swVersionMinor); Serial.print('.');
  Serial.print(id.swVersionPatch);
  Serial.print(F("  part "));
  Serial.print(id.swPartNumber);
  Serial.print(F("  reset: "));
  Serial.println(imu.getResetReasonString());

  Serial.print(F("Authenticity: "));
  Serial.println(MassmoreBNO08x::authToString(imu.verifyChip()));

  imu.setReportCallback(onReport, nullptr);

  // Everything the control channel can do over I2C works here unchanged.
  imu.enableRotationVector(MASSMORE_INTERVAL_100HZ);
  imu.enableAccelerometer(MASSMORE_INTERVAL_100HZ);

  Serial.println(F("\nyaw\tpitch\troll\tax\tay\taz\tRV Hz\tacc Hz"));
  lastPrint = millis();
}

void loop() {
  // The UART transport parses one 0x7E-delimited frame per call, so drain it.
  imu.updateAll(16);

  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();

    massmore_euler_t e = imu.getEulerDeg();
    massmore_vec3_t  a = imu.getAccel();

    Serial.print(e.yaw, 1);   Serial.print('\t');
    Serial.print(e.pitch, 1); Serial.print('\t');
    Serial.print(e.roll, 1);  Serial.print('\t');
    Serial.print(a.x, 2);     Serial.print('\t');
    Serial.print(a.y, 2);     Serial.print('\t');
    Serial.print(a.z, 2);     Serial.print('\t');
    Serial.print(quatCount);  Serial.print('\t');
    Serial.println(accelCount);

    quatCount = accelCount = 0;
  }
}
