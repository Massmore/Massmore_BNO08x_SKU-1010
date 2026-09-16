/*
  01_BasicRead — Massmore_BNO08x (Simple Blocking API)
  ---------------------------------------------------------------------------
  ตัวอย่างพื้นฐานที่สุด: ต่อ I2C, เรียก readAll() หนึ่งครั้งได้ครบทั้ง quaternion,
  Euler angles (roll / pitch / yaw / heading), accelerometer, gyroscope, magnetometer

  Wiring (I2C) — ชื่อขาตามที่พิมพ์บนบอร์ด Massmore Halley V2
    Halley V2      ESP32 (Classic)   ESP32-S3        Arduino Nano
    ---------      ---------------   --------        ------------
    3Vo / 5V   ->  3V3 / 5V          3V3 / 5V        5V (บอร์ดมี 3.3 V LDO)
    GND        ->  GND               GND             GND
    SDA        ->  GPIO 21           GPIO 8          A4
    SCL        ->  GPIO 22           GPIO 9          A5
    DI         ->  ปล่อยลอย = 0x4A (ค่า default ของบอร์ด Massmore), ต่อ 3Vo = 0x4B
    INT / RST  ->  ไม่ต้องต่อในตัวอย่างนี้ (driver ใช้ polling)

  หมายเหตุ: BNO08x ใช้ I2C clock stretching — เริ่มที่ 100 kHz จะเสถียรที่สุด

  Designed and Manufactured by Massmore — https://www.massmore.shop
*/

#include <Wire.h>
#include <Massmore_BNO08x.h>

// ---- ปรับให้ตรงกับบอร์ดของคุณ (pin ถูกกำหนดใน sketch เท่านั้น ไม่ใช่ในไลบรารี) ----
#if defined(CONFIG_IDF_TARGET_ESP32S3)
  #define I2C_SDA_PIN  8
  #define I2C_SCL_PIN  9
#elif defined(ARDUINO_ARCH_ESP32)
  #define I2C_SDA_PIN  21
  #define I2C_SCL_PIN  22
#endif
#define I2C_ADDRESS   MASSMORE_BNO08X_I2C_ADDR_DEF   // 0x4A — driver ลอง 0x4B ให้เองถ้าไม่พบ
// -------------------------------------------------------------------------------

Massmore_BNO08x imu;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) { }
  Serial.println(F("\nMassmore_BNO08x - 01_BasicRead"));

  // sketch เป็นเจ้าของ bus: เรียก Wire.begin() เองแล้วส่ง reference เข้าไลบรารี
#if defined(ARDUINO_ARCH_ESP32)
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);   // Arduino-ESP32 Core 3.x
#else
  Wire.begin();                            // AVR Nano: A4 / A5 (fixed)
#endif
  Wire.setClock(100000);

  if (!imu.begin(I2C_ADDRESS, Wire)) {
    Serial.print(F("BNO08x not found: "));
    Serial.println(Massmore_BNO08x::statusToString(imu.lastError()));
    Serial.println(F("Check wiring, power (3.3 V) and the DI pad (0x4A / 0x4B)."));
    while (true) delay(100);
  }

  Serial.print(F("BNO08x connected at 0x"));
  Serial.println(imu.getI2CAddress(), HEX);
}

void loop() {
  Massmore_BNO08x_reading_t r;

  // readAll() เป็น Blocking: enable report ที่จำเป็นให้เองที่ 50 Hz แล้วรอจนได้ค่าครบ
  if (!imu.readAll(r)) {
    Serial.print(F("read failed: "));
    Serial.println(Massmore_BNO08x::statusToString(imu.lastError()));
    delay(200);
    return;
  }

  Serial.print(F("heading="));  Serial.print(r.headingDeg, 1);
  Serial.print(F("  roll="));   Serial.print(r.eulerDeg.roll, 1);
  Serial.print(F("  pitch="));  Serial.print(r.eulerDeg.pitch, 1);
  Serial.print(F("  yaw="));    Serial.print(r.eulerDeg.yaw, 1);
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

  delay(100);   // พิมพ์ 10 Hz พอ ไม่ให้ Serial แน่น
}
