#include <Arduino.h>
/*
  วัดมุมด้วย Game Rotation Vector — Massmore_BNO08x
  ---------------------------------------------------------------------------
  อ่าน roll / pitch / yaw เป็นองศา โดยใช้ Game Rotation Vector (fusion 6 แกน:
  accel + gyro) ซึ่ง "ไม่ใช้ magnetometer" จึงไม่ต้อง calibrate เลย เปิดเครื่อง
  แล้ววัดได้ทันที และไม่สะเทือนจากโลหะ / มอเตอร์ / ลำโพง / สายไฟรอบตัว

  ต่างจาก Rotation Vector (9 แกน) ที่ 05_I2C_Euler_Compass ใช้:
    - yaw = 0 คือทิศที่บอร์ดหันอยู่ตอนบูต ไม่ใช่ทิศเหนือแม่เหล็ก
    - yaw จะ drift ช้า ๆ เพราะไม่มี mag มาตรึงทิศ — กด 'z' เพื่อตั้งศูนย์ใหม่
    - roll / pitch ไม่ drift (accel เห็นแรงโลกเป็นตัวอ้างอิงเสมอ)
  ถ้าต้องการ heading อ้างอิงทิศเหนือจริง ต้องใช้ RV + calibrate mag (ดู 04, 05)

  คำสั่งทาง Serial (115200) — ส่งทีละตัวอักษร
    z  ตั้งศูนย์เฉพาะ yaw   (วางบอร์ดในท่าที่ต้องการให้เป็น 0 แล้วกด)
    a  ตั้งศูนย์ทุกแกน      (วางระดับก่อนกด → roll/pitch/yaw = 0 ทั้งหมด)
    r  ยกเลิก tare ใน RAM  (soft reset ชิป กลับไปใช้ท่าตอนบูตเป็นศูนย์)
  tare อยู่ใน RAM ของชิป หายเมื่อตัดไฟ — ไม่เขียน flash ให้สึกหรอ

  Wiring (I2C) — MOMO by Massmore (ESP32-S3 + CH343P)
    Halley V2  3Vo -> 3V3,  GND -> GND,  SDA -> GPIO 14,  SCL -> GPIO 15
    DI ปล่อยลอย = 0x4A · INT ไม่ต้องต่อ (ใช้ polling)
    RST: ถ้าต่อเข้า GPIO ให้ใส่เลขขาที่ RST_PIN — จะกันอาการ I2C ค้างหลัง
         upload firmware ใหม่ทับขณะชิปกำลัง stream report

  Designed and Manufactured by Massmore — https://www.massmore.shop
*/

#include <Wire.h>
#include <Massmore_BNO08x.h>

/* ---------------- ปรับได้ที่นี่เท่านั้น (ไม่ใช่ในไลบรารี) ---------------------- */
#if defined(CONFIG_IDF_TARGET_ESP32S3)       // MOMO by Massmore (ESP32-S3 + CH343P)
  #define I2C_SDA_PIN  14
  #define I2C_SCL_PIN  15
#elif defined(ARDUINO_ARCH_ESP32)
  #define I2C_SDA_PIN  21
  #define I2C_SCL_PIN  22
#endif
#define RST_PIN        -1        // ไม่ต่อ RST
// #define RST_PIN     18        // แนะนำ: ต่อ RST เข้า GPIO 18 แล้วเปิดคอมเมนต์บรรทัดนี้ (ลบบรรทัดบน)
                                 // driver จะ reset ชิปให้ตอน begin() — กันอาการ I2C ค้างเมื่อ upload
                                 // firmware ใหม่ทับขณะชิปกำลังส่ง report (ทดสอบแล้ว 5/5 ครั้ง)
#define I2C_ADDRESS    MASSMORE_BNO08X_I2C_ADDR_DEF   // 0x4A (driver ลอง 0x4B ให้เองถ้าไม่พบ)
#define REPORT_HZ      MASSMORE_BNO08X_INTERVAL_100HZ
#define PRINT_INTERVAL_MS  100   // พิมพ์ 10 Hz
/* -------------------------------------------------------------------------- */

Massmore_BNO08x imu;
uint32_t printTimer = 0;
bool gotGRV = false;          // ได้ Game RV ชุดแรกหลังเปิด report แล้วหรือยัง
float yawOffset = 0.0f;       // ศูนย์ของ yaw ที่ตั้งด้วย 'z' (หักในซอฟต์แวร์)

// หมุนมุมให้อยู่ในช่วง -180..+180
static float wrap180(float deg) {
  while (deg <= -180.0f) deg += 360.0f;
  while (deg >   180.0f) deg -= 360.0f;
  return deg;
}

static void enableReports() {
  // Game RV เท่านั้น: RV / Geomagnetic RV เขียน quaternion ตัวเดียวกัน ถ้าเปิดพร้อมกันค่าจะทับกัน
  imu.enableGameRotationVector(REPORT_HZ);
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) { }
  Serial.println(F("\nMassmore_BNO08x - Euler angles (Game RV, no calibration needed)"));

#if defined(ARDUINO_ARCH_ESP32)
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
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

  // ไม่ต่อ RST: ชิปไม่ถูก reset ตอน MCU reset และยังส่ง report ที่ sketch ก่อนหน้า
  // เปิดค้างไว้ (เช่น RV ซึ่งจะทับ quaternion ของ Game RV) → soft reset ล้างให้สะอาดก่อน
  if (RST_PIN < 0) imu.softReset();

  enableReports();
  Serial.println(F("z = zero yaw   a = zero all axes   r = undo tare"));
}

void loop() {
  imu.update();                                  // Non-blocking: ดึง packet ที่ค้างอยู่ (ถ้ามี)

  if (Serial.available()) {
    switch ((char)Serial.read()) {
      /* Tare เฉพาะแกน Z: ชิปไม่ทำอะไรเลยเมื่อ basis = Game RV (ทดสอบบน BNO086 v3.12.6
       * แล้ว ส่ง TARE_AXIS_Z ไปแล้ว yaw ไม่ขยับ) จึงหัก offset ในซอฟต์แวร์เอง
       * หมายเหตุ: roll/pitch ไม่ถูกแตะ ต่างจาก 'a' ที่ให้ชิปคำนวณ quaternion ใหม่ทั้งชุด */
      case 'z': yawOffset = imu.getEulerDeg().yaw;           Serial.println(F("> yaw zeroed")); break;
      /* basis ต้องเป็น GAMING_RV ให้ตรงกับ report ที่เปิดไว้ — ค่า default ของ tareNow()
       * คือ Rotation Vector (9 แกน) ซึ่งเราไม่ได้เปิด สั่งไปแล้วมุมจะไม่ขยับเลย */
      case 'a': imu.tareNow(MASSMORE_BNO08X_TARE_AXIS_ALL,
                            MASSMORE_BNO08X_TARE_BASIS_GAMING_RV);
                yawOffset = 0.0f;                            Serial.println(F("> all axes zeroed")); break;
      /* clearTare() = Set Reorientation เป็น identity quaternion ซึ่งเป็นวิธีที่ถูก
       * สำหรับยกเลิก tare — softReset ไม่ล้าง (ทดสอบแล้ว: tare รอดข้าม soft reset) */
      case 'r': imu.clearTare(); yawOffset = 0.0f;           Serial.println(F("> tare cleared")); break;
      default:  break;
    }
  }

  // รอ Game RV ชุดแรกหลังเปิด report ก่อนเริ่มพิมพ์ (ค่า cache ก่อนหน้านั้นไม่ใช่ของ Game RV)
  if (!gotGRV) {
    gotGRV = imu.hasNewReport(MASSMORE_BNO08X_SENSOR_GAME_ROTATION_VECTOR);
    if (!gotGRV) return;
  }

  uint32_t now = millis();
  if ((uint32_t)(now - printTimer) < PRINT_INTERVAL_MS) return;
  printTimer = now;

  Massmore_BNO08x_euler_t e = imu.getEulerDeg();
  char buf[64];
  snprintf(buf, sizeof(buf), "roll=%7.2f  pitch=%7.2f  yaw=%7.2f  deg",
           (double)e.roll, (double)e.pitch, (double)wrap180(e.yaw - yawOffset));
  Serial.println(buf);
}
