/*
  02_CustomPins_BusRemap — Massmore_BNO08x
  ---------------------------------------------------------------------------
  ย้าย I2C ไปขาอื่นและใช้ bus ที่สอง (Wire1) บน ESP32 / ESP32-S3 ด้วย
  Arduino-ESP32 Core 3.x API — ไลบรารีไม่ hardcode ขาใด ๆ sketch เป็นผู้กำหนดทั้งหมด

    ESP32 (Classic)  : Wire1 บน GPIO 21 (SDA) / GPIO 22 (SCL)   — GPIO Matrix ย้ายได้ทุกขา
    ESP32-S3 (MOMO)  : Wire1 บน GPIO 14 (SDA) / GPIO 15 (SCL)   — GPIO Matrix ย้ายได้ทุกขา
    Arduino Nano     : AVR มี hardware I2C ชุดเดียว ขาตายตัว A4 (SDA) / A5 (SCL)
                       จึงใช้ Wire ตามปกติ (ตัวอย่างนี้ compile ผ่านและทำงานได้เหมือน 01)

  Wiring — Halley V2: 3Vo→3V3, GND→GND, SDA→<SDA pin>, SCL→<SCL pin>
           INT→<INT pin> (แนะนำ) ทำให้ driver ไม่ต้อง poll bus เปล่า ๆ
           RST→<RST pin> (แนะนำ) driver reset ชิปด้วยขานี้ตอน begin() — กู้ bus ที่ค้างได้
    ESP32 (Classic) ตัวอย่างนี้: INT = GPIO 4, RST = GPIO 17

  Designed and Manufactured by Massmore — https://www.massmore.shop
*/

#include <Wire.h>
#include <Massmore_BNO08x.h>

// ---- กำหนดขาใน sketch เท่านั้น ----------------------------------------------------
#if defined(CONFIG_IDF_TARGET_ESP32S3)       // MOMO by Massmore (ESP32-S3 + CH343P)
  #define I2C_SDA_PIN  14
  #define I2C_SCL_PIN  15
  #define INT_PIN      -1       // -1 = ไม่ต่อ (polling)
  #define RST_PIN      -1    // ไม่ต่อ RST
  // #define RST_PIN      18    // แนะนำ: ต่อ RST เข้า GPIO 18 แล้วเปิดคอมเมนต์บรรทัดนี้
                              // driver จะ reset ชิปให้ตอน begin() — กันอาการ I2C ค้าง
                              // เมื่อ upload firmware ใหม่ทับขณะชิปกำลังส่ง report
  TwoWire &imuBus = Wire1;      // bus ที่สองของ ESP32-S3
#elif defined(ARDUINO_ARCH_ESP32)
  #define I2C_SDA_PIN  21
  #define I2C_SCL_PIN  22
  #define INT_PIN      4
  #define RST_PIN      17       // -1 = ไม่ต่อ
  TwoWire &imuBus = Wire1;      // bus ที่สองของ ESP32 Classic
#else
  #define INT_PIN      2        // Nano: INT ต่อ D2 (หรือ -1 ถ้าไม่ต่อ)
  #define RST_PIN      -1
  TwoWire &imuBus = Wire;       // AVR: A4 / A5 เท่านั้น
#endif
#define I2C_ADDRESS   MASSMORE_BNO08X_I2C_ADDR_DEF
// ---------------------------------------------------------------------------------

Massmore_BNO08x imu;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) { }
  Serial.println(F("\nMassmore_BNO08x - 02_CustomPins_BusRemap"));

#if defined(ARDUINO_ARCH_ESP32)
  // Core 3.x: TwoWire::begin(sda, scl, frequency) — ใช้ได้กับทั้ง Wire และ Wire1
  imuBus.begin(I2C_SDA_PIN, I2C_SCL_PIN, 100000);
  Serial.print(F("Wire1 on SDA=")); Serial.print(I2C_SDA_PIN);
  Serial.print(F(" SCL="));         Serial.println(I2C_SCL_PIN);
#else
  imuBus.begin();
  imuBus.setClock(100000);
  Serial.println(F("Wire on A4 (SDA) / A5 (SCL)"));
#endif

  // ส่ง bus reference + INT pin เข้าไป — ไลบรารีไม่เรียก begin() ของ bus เอง
  if (!imu.begin(I2C_ADDRESS, imuBus, INT_PIN, RST_PIN)) {
    Serial.print(F("BNO08x not found: "));
    Serial.println(Massmore_BNO08x::statusToString(imu.lastError()));
    while (true) delay(100);
  }
  Serial.print(F("BNO08x connected at 0x"));
  Serial.println(imu.getI2CAddress(), HEX);
}

void loop() {
  Massmore_BNO08x_euler_t e;
  if (imu.readEulerDeg(e)) {
    Serial.print(F("roll="));   Serial.print(e.roll, 1);
    Serial.print(F("  pitch=")); Serial.print(e.pitch, 1);
    Serial.print(F("  yaw="));   Serial.println(e.yaw, 1);
  } else {
    Serial.print(F("read failed: "));
    Serial.println(Massmore_BNO08x::statusToString(imu.lastError()));
  }
  delay(100);
}
