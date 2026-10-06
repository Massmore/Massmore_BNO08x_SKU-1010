/*
  06_I2C_Euler_Compass — Massmore_BNO08x (Euler angles + เข็มทิศ tilt-compensated)
  ---------------------------------------------------------------------------
  อ่าน Rotation Vector (9-axis: accel + gyro + mag) ผ่าน I2C แล้วแสดง
    - Euler angles: roll / pitch / yaw เป็นองศา
    - เข็มทิศ: heading 0..360° (0 = เหนือ, 90 = ตะวันออก, ตามเข็มนาฬิกา) + ทิศ 16 ทิศ (N, NNE, NE, …)
    - ความแม่นยำ heading ที่ชิปประเมินเอง (±องศา) และสถานะ calibration ของ RV / magnetometer

  BNO08x ทำ sensor fusion และ tilt compensation ในชิปแล้ว — เอียงบอร์ดได้โดย heading ไม่เพี้ยน
  (ต่างจากเข็มทิศ magnetometer ธรรมดาที่ต้องวางราบ)

  แกนอ้างอิง: heading คือทิศที่แกน +X (ลูกศร X บนบอร์ด) ชี้ไป เมื่อวางบอร์ดหงายขึ้น (Z ชี้ฟ้า)
  Rotation Vector ของ BNO08x อ้างอิงกรอบโลก East-North-Up: yaw = 0 เมื่อ +X ชี้ตะวันออก
  และเพิ่มขึ้นทวนเข็มนาฬิกา → heading ของแกน +X = 90° - yaw
  ติดตั้งบอร์ดหมุนไปจากทิศหน้าของอุปกรณ์ → ปรับ HEADING_OFFSET_DEG

  Calibration: ครั้งแรกหลังเปิดเครื่อง accuracy จะเป็น UNRELIABLE / LOW และ heading ยังไม่ชี้เหนือจริง
  หยิบบอร์ดหมุนเป็นเลข 8 ในอากาศ (ทุกแกน) ประมาณ 10–20 วินาที จนขึ้น MEDIUM / HIGH
  อยู่ห่างโลหะ / แม่เหล็ก / มอเตอร์ / สายไฟกระแสสูง (ดู 05_Calibration_Tare สำหรับ Save DCD)

  Wiring (I2C) — ชื่อขาตามที่พิมพ์บนบอร์ด Massmore Halley V2
    Halley V2      ESP32 (Classic)   ESP32-S3 (MOMO) Arduino Nano
    ---------      ---------------   ---------------  ------------
    3Vo / 5V   ->  3V3 / 5V          3V3 / 5V        5V (บอร์ดมี 3.3 V LDO)
    GND        ->  GND               GND             GND
    SDA        ->  GPIO 21           GPIO 14         A4
    SCL        ->  GPIO 22           GPIO 15         A5
    DI         ->  ปล่อยลอย = 0x4A (ค่า default ของบอร์ด Massmore), ต่อ 3Vo = 0x4B
    RST        ->  GPIO 17           -               -     (แนะนำ; ไม่ต่อให้ตั้ง RST_PIN = -1)
    INT        ->  ไม่ต้องต่อในตัวอย่างนี้ (driver ใช้ polling)

  Designed and Manufactured by Massmore — https://www.massmore.shop
*/

#include <Wire.h>
#include <Massmore_BNO08x.h>

// ---- ปรับให้ตรงกับบอร์ดของคุณ (pin ถูกกำหนดใน sketch เท่านั้น ไม่ใช่ในไลบรารี) ----
#if defined(CONFIG_IDF_TARGET_ESP32S3)       // MOMO by Massmore (ESP32-S3 + CH343P)
  #define I2C_SDA_PIN  14
  #define I2C_SCL_PIN  15
  // #define RST_PIN      18    // แนะนำ: ต่อ RST เข้า GPIO 18 แล้วเปิดคอมเมนต์บรรทัดนี้
                              // driver จะ reset ชิปให้ตอน begin() — กันอาการ I2C ค้าง
                              // เมื่อ upload firmware ใหม่ทับขณะชิปกำลังส่ง report
#elif defined(ARDUINO_ARCH_ESP32)
  #define I2C_SDA_PIN  21
  #define I2C_SCL_PIN  22
  #define RST_PIN      17       // -1 = ไม่ต่อ
#endif
#ifndef RST_PIN
  #define RST_PIN      -1       // ไม่ต่อ RST
#endif
#define I2C_ADDRESS   MASSMORE_BNO08X_I2C_ADDR_DEF   // 0x4A — driver ลอง 0x4B ให้เองถ้าไม่พบ

// Magnetic declination (องศา, ตะวันออก = +) แปลงทิศเหนือแม่เหล็กเป็นทิศเหนือจริง
// ประเทศไทยใกล้ 0° (กรุงเทพฯ ≈ -0.5°) — ตรวจค่าตำแหน่งของคุณที่ NOAA Magnetic Field Calculator
#define DECLINATION_DEG      0.0f
// มุมติดตั้งบอร์ดเทียบกับทิศหน้าของอุปกรณ์ (องศา ตามเข็มนาฬิกา) — ติดตรงให้ +X ชี้ไปหน้า = 0
#define HEADING_OFFSET_DEG   0.0f

#define PRINT_INTERVAL_MS    200     // พิมพ์ 5 Hz
// -------------------------------------------------------------------------------

Massmore_BNO08x imu;

uint32_t printTimer = 0;
uint32_t hintTimer  = 0;

// หมุนมุมให้อยู่ในช่วง 0..360
float wrap360(float deg) {
  while (deg <  0.0f)   deg += 360.0f;
  while (deg >= 360.0f) deg -= 360.0f;
  return deg;
}

// ชื่อทิศ 16 ทิศ ทุก 22.5°
const char *compassPoint(float headingDeg) {
  static const char *const names[16] = {
    "N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
    "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"
  };
  uint8_t idx = (uint8_t)((headingDeg + 11.25f) / 22.5f) & 0x0F;
  return names[idx];
}

// พิมพ์ float ให้กว้างเท่ากันทุกบรรทัด (อ่านง่ายใน Serial Monitor)
void printPadded(float v, uint8_t width, uint8_t decimals) {
  char buf[12];
  dtostrf(v, width, decimals, buf);
  Serial.print(buf);
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) { }
  Serial.println(F("\nMassmore_BNO08x - 06_I2C_Euler_Compass"));

  // sketch เป็นเจ้าของ bus: เรียก Wire.begin() เองแล้วส่ง reference เข้าไลบรารี
#if defined(ARDUINO_ARCH_ESP32)
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);   // Arduino-ESP32 Core 3.x
#else
  Wire.begin();                            // AVR Nano: A4 / A5 (fixed)
#endif
  Wire.setClock(100000);

  if (!imu.begin(I2C_ADDRESS, Wire, -1, RST_PIN)) {
    Serial.print(F("BNO08x not found: "));
    Serial.println(Massmore_BNO08x::statusToString(imu.lastError()));
    Serial.println(F("Check wiring, power (3.3 V) and the DI pad (0x4A / 0x4B)."));
    while (true) delay(100);
  }
  Serial.print(F("BNO08x connected at 0x"));
  Serial.println(imu.getI2CAddress(), HEX);

  // ไม่ต่อ RST: ชิปไม่ถูก reset ตอน MCU reset และยังส่ง report ที่ sketch ก่อนหน้าเปิดค้างไว้
  // (เช่น Geomagnetic RV ซึ่งจะทับ quaternion ของ Rotation Vector) → soft reset ล้างให้สะอาดก่อน
  if (RST_PIN < 0) imu.softReset();

  // Rotation Vector = 9-axis fusion ที่อ้างอิงทิศเหนือแม่เหล็ก (Game RV ไม่มี mag จึงใช้เป็นเข็มทิศไม่ได้)
  // เปิด Magnetometer ด้วยเพื่อดูสถานะ calibration ของ mag แยกต่างหาก
  imu.enableRotationVector(MASSMORE_BNO08X_INTERVAL_50HZ);
  imu.enableMagnetometer(MASSMORE_BNO08X_INTERVAL_10HZ);

  Serial.println(F("Move the board in a figure-8 until RV accuracy is MEDIUM or HIGH."));
}

void loop() {
  imu.update();                                   // Non-blocking: ดึง packet ที่ค้างอยู่ (ถ้ามี)

  // รอ Rotation Vector ชุดแรกหลังเปิด report ก่อนเริ่มพิมพ์ (ค่า cache ก่อนหน้านั้นยังไม่ใช่ของ RV)
  static bool gotRV = false;
  if (!gotRV) {
    gotRV = imu.hasNewReport(MASSMORE_BNO08X_SENSOR_ROTATION_VECTOR);
    if (!gotRV) return;
  }

  uint32_t now = millis();
  if ((uint32_t)(now - printTimer) < PRINT_INTERVAL_MS) return;
  printTimer = now;

  Massmore_BNO08x_reading_t r;
  imu.getReadings(r);                             // copy ค่าล่าสุด (ไม่แตะ bus)

  // heading ของแกน +X แบบเข็มทิศ (0 = เหนือ, ตามเข็มนาฬิกา) — ดูคำอธิบายกรอบ ENU ด้านบน
  float heading    = wrap360(90.0f - r.eulerDeg.yaw + DECLINATION_DEG + HEADING_OFFSET_DEG);
  float accuracyDg = imu.getQuatAccuracy() * 57.2957795f;   // ±องศา ที่ชิปประเมิน (0 = ยังไม่รู้)

  Serial.print(F("roll="));     printPadded(r.eulerDeg.roll,  7, 1);
  Serial.print(F("  pitch="));  printPadded(r.eulerDeg.pitch, 6, 1);
  Serial.print(F("  yaw="));    printPadded(r.eulerDeg.yaw,   7, 1);
  Serial.print(F("  | heading=")); printPadded(heading, 5, 1);
  Serial.print(' ');
  Serial.print(compassPoint(heading));
  Serial.print(F("\t+/-"));     Serial.print(accuracyDg, 0);
  Serial.print(F(" deg  [RV "));
  Serial.print(Massmore_BNO08x::accuracyToString(r.accuracyRV));
  Serial.print(F(", MAG "));
  Serial.print(Massmore_BNO08x::accuracyToString(r.accuracyMag));
  Serial.println(']');

  // เตือนให้ calibrate ทุก 5 วินาทีถ้า heading ยังไม่น่าเชื่อถือ
  if (r.accuracyRV < MASSMORE_BNO08X_ACCURACY_MEDIUM && (uint32_t)(now - hintTimer) >= 5000) {
    hintTimer = now;
    Serial.println(F("  -> heading not calibrated yet: move the board in a figure-8"));
  }
}
