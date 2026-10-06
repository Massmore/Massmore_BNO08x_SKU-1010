/*
  05_Calibration_Tare — Massmore_BNO08x
  ---------------------------------------------------------------------------
  Calibration ตามขั้นตอนของ CEVA (doc 1000-4044) และ Tare (doc 1000-4045)
  ควบคุมผ่าน Serial Monitor (115200, ส่งทีละตัวอักษร)

  ทำตาม 4 ขั้นนี้ครั้งเดียวก็จบ — ตัวอย่างจะบอกท้ายบรรทัดสถานะเองว่าต้องทำอะไรต่อ
    1. กด 'c'   เปิด dynamic calibration
    2. ขยับบอร์ดตามท่าด้านล่าง จนบรรทัดสถานะขึ้น READY
    3. กด 's'   เก็บ calibration ลง flash ของชิป (อยู่ข้าม power cycle)
    4. กด 'e'   ปิด dynamic calibration

  ท่าขยับบอร์ด (ตาม datasheet ไม่ใช่ท่าเลข 8 ของ BNO055):
    magnetometer : หมุน ~180° แล้วหมุนกลับ รอบแกน roll, pitch, yaw ทีละแกน ~2 s/แกน
    accelerometer: ถือนิ่ง 4–6 ทิศทางที่ต่างกันชัดเจน ~1 s ต่อทิศ
    gyroscope    : วางนิ่งบนพื้นมั่นคง 2–3 s
  อยู่ห่างโลหะ / แม่เหล็ก / มอเตอร์ / สายไฟกระแสสูง ระหว่าง calibrate

  Tare = กำหนดว่าท่าปัจจุบันคือศูนย์ (คนละเรื่องกับ calibration) ทำหลัง calibration
    วางบอร์ดในท่าอ้างอิงก่อน แล้วกด 'z' (เฉพาะ heading) หรือ 'a' (ทุกแกน)

  คำสั่งทั้งหมด
    calibration : c=เปิด  m=เปิดเฉพาะ mag  e=ปิด  s=บันทึกลง flash  x=ลบ+reset  ?=ถามสถานะ
    tare        : z=ตั้งศูนย์ heading  a=ตั้งศูนย์ทุกแกน  p=บันทึกลง flash  t=ลบที่บันทึกไว้
    h = แสดงคำสั่งอีกครั้ง
  คำสั่ง s / p / x / t เขียน flash ของชิปซึ่งมีจำนวนรอบเขียนจำกัด — ใช้ตอน setup พอ
  ไม่ต้องใส่ใน loop

  Wiring (I2C): เหมือน 01_BasicRead (แนะนำต่อ RST — ดูบล็อกขาด้านล่าง)

  Designed and Manufactured by Massmore — https://www.massmore.shop
*/

#include <Wire.h>
#include <Massmore_BNO08x.h>

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
#define I2C_ADDRESS   MASSMORE_BNO08X_I2C_ADDR_DEF

// heading error ที่ถือว่า calibrate สำเร็จ (องศา) — CEVA แนะนำ < 10°
#define HEADING_ERR_OK_DEG   10.0f

Massmore_BNO08x imu;
uint32_t printTimer = 0;
bool calibrating = false;      // กด 'c' หรือ 'm' ไปแล้วหรือยัง
bool saved       = false;      // กด 's' ไปแล้วหรือยัง

static void printHelp() {
  Serial.println(F("STEP 1 'c' start calibration  2 move the board  3 's' save  4 'e' stop"));
  Serial.println(F("  move: mag = turn 180 deg back and forth around roll, pitch, yaw (2 s each)"));
  Serial.println(F("        accel = hold still in 4-6 different orientations (1 s each)"));
  Serial.println(F("        gyro  = leave it still on the table (2-3 s)"));
  Serial.println(F("cal : c=on m=mag-only e=off s=save x=erase+reset ?=status"));
  Serial.println(F("tare: z=zero heading a=zero all axes p=persist t=clear   h=this help"));
}

static void enableReports() {
  // RV = fusion 9 แกน ใช้ mag จึงเห็นผลของ calibration ได้ตรง ๆ
  // อย่าเปิด Game RV / Geomagnetic RV พร้อมกัน: ทุกตัวเขียน quaternion ตัวเดียวกัน
  // ในไลบรารี ค่า Euler จะสลับไปมาตามว่า report ไหนมาถึงทีหลัง
  imu.enableRotationVector(MASSMORE_BNO08X_INTERVAL_50HZ);
  imu.enableMagnetometer(MASSMORE_BNO08X_INTERVAL_50HZ);
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) { }
  Serial.println(F("\nMassmore_BNO08x - 05_Calibration_Tare"));

#if defined(ARDUINO_ARCH_ESP32)
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
#else
  Wire.begin();                            // AVR Nano: A4 / A5 (fixed)
#endif
  Wire.setClock(100000);

  if (!imu.begin(I2C_ADDRESS, Wire, -1, RST_PIN)) {
    Serial.print(F("BNO08x not found: "));
    Serial.println(Massmore_BNO08x::statusToString(imu.lastError()));
    while (true) delay(100);
  }
  Serial.print(F("BNO08x connected at 0x"));
  Serial.println(imu.getI2CAddress(), HEX);

  enableReports();
  printHelp();
}

void loop() {
  imu.updateAll();

  if (Serial.available()) {
    switch ((char)Serial.read()) {
      // ---- calibration ----
      case 'c': imu.calibrateAll();          calibrating = true; saved = false;
                Serial.println(F("> calibration ON (accel+gyro+mag) - now move the board")); break;
      case 'm': imu.calibrateMagnetometer(); calibrating = true; saved = false;
                Serial.println(F("> magnetometer calibration ON - now move the board")); break;
      case 'e': imu.endCalibration();        calibrating = false;
                Serial.println(F("> calibration OFF")); break;
      case 's': imu.saveCalibration();       saved = true;
                Serial.println(F("> saved to flash - press 'e' to stop calibration,"));
                Serial.println(F("  then place the board in its reference pose and press 'z' or 'a'")); break;
      case 'x':
        Serial.println(F("> erasing calibration + reset..."));
        imu.clearCalibrationAndReset();
        enableReports();
        calibrating = false; saved = false;
        Serial.println(F("  done, reports re-enabled")); break;
      case '?':
        imu.requestCalibrationStatus();
        delay(50);
        imu.updateAll();
        Serial.print(F("> ME calibration status = "));
        Serial.print(imu.getCalibrationStatus());
        Serial.println(imu.calibrationComplete() ? F(" (OK)") : F(" (pending/failed)"));
        break;
      // ---- tare ----
      // basis = Rotation Vector ให้ตรงกับ report ที่เปิดไว้ (เป็นค่า default ของ tareNow ด้วย)
      // ถ้าเปลี่ยนไปใช้ Game RV ต้องส่ง TARE_BASIS_GAMING_RV และ tare เฉพาะแกน Z จะไม่มีผล
      // (ทดสอบบน BNO086 fw 3.12.6: ส่ง TARE_AXIS_Z + GAMING_RV แล้ว yaw ไม่ขยับ)
      case 'z': imu.tareNow(MASSMORE_BNO08X_TARE_AXIS_Z,
                            MASSMORE_BNO08X_TARE_BASIS_ROTATION_VECTOR);
                Serial.println(F("> heading zeroed (Z only)")); break;
      case 'a': imu.tareNow(MASSMORE_BNO08X_TARE_AXIS_ALL,
                            MASSMORE_BNO08X_TARE_BASIS_ROTATION_VECTOR);
                Serial.println(F("> all axes zeroed")); break;
      case 'p': imu.persistTare(); Serial.println(F("> tare persisted to flash")); break;
      // softReset ไม่ล้าง tare (ทดสอบแล้ว: tare รอดข้าม soft reset) — ต้องใช้ clearTare()
      case 't': imu.clearTare();   Serial.println(F("> stored tare cleared")); break;
      case 'h': printHelp(); break;
      default: break;
    }
  }

  if ((uint32_t)(millis() - printTimer) < 500) return;
  printTimer = millis();

  Massmore_BNO08x_euler_t e = imu.getEulerDeg();
  float headingErrDeg = imu.getQuatAccuracy() * 57.2957795f;
  bool  good = (headingErrDeg > 0.0f && headingErrDeg < HEADING_ERR_OK_DEG);

  Serial.print(F("roll="));   Serial.print(e.roll, 1);
  Serial.print(F(" pitch=")); Serial.print(e.pitch, 1);
  Serial.print(F(" yaw="));   Serial.print(e.yaw, 1);
  Serial.print(F(" | mag="));
  Serial.print(Massmore_BNO08x::accuracyToString(imu.getAccuracy(MASSMORE_BNO08X_SENSOR_MAGNETIC_FIELD)));
  Serial.print(F(" RV="));
  Serial.print(Massmore_BNO08x::accuracyToString(imu.getAccuracy(MASSMORE_BNO08X_SENSOR_ROTATION_VECTOR)));
  Serial.print(F(" err="));
  Serial.print(headingErrDeg, 1);
  Serial.print(F(" deg | "));

  // บอกสิ่งที่ต้องทำต่อ ไม่ต้องเลื่อนขึ้นไปอ่าน help
  if (calibrating) {
    if (saved)     Serial.println(F("saved - press 'e' to stop calibration"));
    else if (good) Serial.println(F("READY - press 's' to save"));
    else           Serial.println(F("MOVE THE BOARD (press 'h' for the pattern)"));
  } else {
    if (saved)     Serial.println(F("calibrated + saved - 'z' or 'a' to set zero"));
    /* ชิปมี dynamic calibration ทำงานเบื้องหลังอยู่แล้ว ถ้าอุปกรณ์ถูกขยับในการใช้งาน
     * ปกติ err อาจต่ำกว่าเกณฑ์เองโดยยังไม่ได้กด 'c' — ไม่ต้องบังคับให้ calibrate ซ้ำ */
    else if (good) Serial.println(F("already good - 'z'/'a' to set zero, or 'c' to recalibrate"));
    else           Serial.println(F("press 'c' to start calibration"));
  }
}
