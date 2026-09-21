/*
  07_UART_Mode — Massmore_BNO08x (SHTP over UART, 3 Mbit/s)
  ---------------------------------------------------------------------------
  โหมด UART เต็มรูปแบบ: ใช้ API เดียวกับ I2C / SPI ทุกอย่าง (readAll, enable*,
  calibration, tare, Product ID, isGenuine) แค่เปลี่ยน begin() เป็น beginUART()
  ต่างจาก 06_UART_RVC ที่ได้แค่ yaw/pitch/roll + accel และสั่งงานชิปไม่ได้

  WIRING — โหมดถูก latch ตอน reset (ต่อ RST ไว้ driver จะ reset ให้เอง)
    Halley V2       ESP32 (Classic)   ESP32-S3 (MOMO)
    ---------       ---------------   ---------------
    3Vo         ->  3V3               3V3
    GND         ->  GND               GND
    SDA (TX)    ->  GPIO 21 (RX)      GPIO 14 (RX)
    SCL (RX)    ->  GPIO 22 (TX)      GPIO 15 (TX)
    PS1 (P1)    ->  3Vo  (HIGH)
    PS0 (P0)    ->  GND / ปล่อยลอย (LOW)
    RST         ->  GPIO 17           -          (แนะนำ; ไม่ต่อให้ตั้ง RST_PIN = -1 แล้วถอดไฟหลังเปลี่ยนโหมด)
    INT         ->  GPIO 4            -          (ไม่บังคับ)

  UART 3,000,000 baud 8N1 ตาม datasheet — Arduino Nano ทำความเร็วนี้ไม่ได้
  (ใช้ 06_UART_RVC ที่ 115200 แทน) ตัวอย่างนี้จึงรองรับเฉพาะ ESP32 / ESP32-S3

  Designed and Manufactured by Massmore — https://www.massmore.shop
*/

#include <Massmore_BNO08x.h>

// ---- กำหนดขาใน sketch เท่านั้น ----------------------------------------------------
#if defined(CONFIG_IDF_TARGET_ESP32S3)       // MOMO by Massmore (ESP32-S3 + CH343P)
  #define UART_RX_PIN  14       // ต่อ pad SDA (TX ของเซ็นเซอร์)
  #define UART_TX_PIN  15       // ต่อ pad SCL (RX ของเซ็นเซอร์)
  #define INT_PIN      -1
  #define RST_PIN      -1
#elif defined(ARDUINO_ARCH_ESP32)
  #define UART_RX_PIN  21
  #define UART_TX_PIN  22
  #define INT_PIN      4        // -1 = ไม่ต่อ
  #define RST_PIN      17       // -1 = ไม่ต่อ
#endif
#define UART_BAUD     3000000UL   // SHTP-over-UART ของ BNO08x ใช้ 3 Mbit/s เท่านั้น
// ---------------------------------------------------------------------------------

#if defined(ARDUINO_ARCH_ESP32)

Massmore_BNO08x imu;
HardwareSerial &imuSerial = Serial2;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) { }
  Serial.println(F("\nMassmore_BNO08x - 07_UART_Mode"));

  // sketch เป็นเจ้าของ UART: ขยาย RX buffer ก่อน begin() เพราะ advertisement ยาว ~290 byte
  imuSerial.setRxBufferSize(1024);
  imuSerial.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  if (!imu.beginUART(imuSerial, INT_PIN, RST_PIN)) {
    Serial.print(F("BNO08x not found: "));
    Serial.println(Massmore_BNO08x::statusToString(imu.lastError()));
    Serial.println(F("Check PS1 = HIGH, PS0 = LOW, SDA->RX, SCL->TX, then reset the sensor."));
    while (true) delay(100);
  }

  const Massmore_BNO08x_product_id_t &id = imu.getProductID();
  Serial.print(F("UART OK - "));
  Serial.print(Massmore_BNO08x::chipModelToString(imu.getChipModel()));
  Serial.print(F("  FW "));
  Serial.print(id.swVersionMajor); Serial.print('.');
  Serial.print(id.swVersionMinor); Serial.print('.');
  Serial.print(id.swVersionPatch);
  Serial.print(F("  part "));
  Serial.println(id.swPartNumber);
}

void loop() {
  Massmore_BNO08x_reading_t r;

  // API เดียวกับ 01_BasicRead ทุกอย่าง — transport เป็น UART แทน I2C
  if (!imu.readAll(r)) {
    Serial.print(F("read failed: "));
    Serial.println(Massmore_BNO08x::statusToString(imu.lastError()));
    delay(200);
    return;
  }

  Serial.print(F("heading="));  Serial.print(r.headingDeg, 1);
  Serial.print(F("  roll="));   Serial.print(r.eulerDeg.roll, 1);
  Serial.print(F("  pitch="));  Serial.print(r.eulerDeg.pitch, 1);
  Serial.print(F("  | accel="));
  Serial.print(r.accel.x, 2); Serial.print(' ');
  Serial.print(r.accel.y, 2); Serial.print(' ');
  Serial.print(r.accel.z, 2);
  Serial.print(F("  gyro="));
  Serial.print(r.gyro.x, 2); Serial.print(' ');
  Serial.print(r.gyro.y, 2); Serial.print(' ');
  Serial.print(r.gyro.z, 2);
  Serial.print(F("  mag="));
  Serial.print(r.mag.x, 1); Serial.print(' ');
  Serial.print(r.mag.y, 1); Serial.print(' ');
  Serial.print(r.mag.z, 1);
  Serial.print(F("  [RV acc: "));
  Serial.print(Massmore_BNO08x::accuracyToString(r.accuracyRV));
  Serial.println(']');

  delay(100);
}

#else   // AVR / อื่น ๆ: 3 Mbit/s เกินความสามารถของ UART

void setup() {
  Serial.begin(115200);
  Serial.println(F("\nMassmore_BNO08x - 07_UART_Mode"));
  Serial.println(F("SHTP-over-UART needs 3 Mbit/s: ESP32 / ESP32-S3 only. Use 06_UART_RVC on this board."));
}

void loop() { }

#endif
