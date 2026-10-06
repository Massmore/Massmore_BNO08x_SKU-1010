/*
  10_Factory_Test — Massmore_BNO08x (Outgoing QA / QC)
  ---------------------------------------------------------------------------
  ใช้ตรวจบอร์ด Massmore BNO08x SKU-1010 ก่อนส่งลูกค้า และให้เว็บ
  Massmore Web Serial Monitor อ่านผลอัตโนมัติ — รันเองทันทีหลังบูต

  Test sequence
    1. BUS_SCAN      สแกน I2C หา 0x4A / 0x4B (เตือนถ้าเจอ 0x28 / 0x29 = bootloader)
    2. CHIP_ID       Product ID (0xF8): firmware part number ต้องตรง SH-2 build ที่รู้จัก
    3. FW_VERSION    version สมเหตุสมผล
    4. SERIAL        FRS 0x4B4B (NONE = ชิปไม่ได้ program serial จากโรงงาน ไม่ถือว่า FAIL)
    5. METADATA      Rotation Vector metadata Q point ต้องเป็น 14/12 (พิสูจน์ MotionEngine)
    6. AUTHENTICITY  isGenuine() → GENUINE / SUSPECT
    7. RANGE_ACCEL   |a| ใกล้ 9.81 m/s^2 (วางนิ่ง) และอยู่ใน full scale
    8. RANGE_GYRO    |w| ใกล้ 0 rad/s (วางนิ่ง)
    9. RANGE_MAG     |B| อยู่ในช่วงสนามแม่เหล็กโลก (ไม่เป็นศูนย์ / ไม่อิ่มตัว)
   10. RANGE_QUAT    |q| = 1.00 ± 0.02
   11. CONTINUOUS    readAll() 20 ครั้งติดกัน ไม่มี TIMEOUT / NAN, noise ของ |a| ต่ำ
   12. VERDICT

  Serial 115200 — ทุกบรรทัดที่เว็บ parse ขึ้นต้นด้วย '#' (English only):
    #MASSMORE_FACTORY_TEST v1.0
    #PRODUCT Massmore_BNO08x
    #MCU ESP32
    #CHIP <BNO085|BNO086|BNO08x>          (รุ่นชิปที่อ่านได้จาก Product ID — ข้อมูล ไม่ใช่ PASS/FAIL)
    #RESULT <TEST_NAME> <PASS|FAIL> <value>
    #VERDICT <PASS|FAIL> [<REASON>]
    [PASS] SENSOR QA PASSED - READY TO SHIP   หรือ   [FAIL] QA CHECK FAILED: <REASON>

  Default wiring (I2C / Qwiic)
    ESP32 Classic   : Halley V2  SDA -> GPIO 21,  SCL -> GPIO 22,  3Vo -> 3V3,  GND -> GND
    ESP32-S3 (MOMO) : Halley V2  SDA -> GPIO 14,  SCL -> GPIO 15,  3Vo -> 3V3,  GND -> GND
    INT / RST ไม่บังคับ (ไม่ต่อให้ตั้ง FT_INT_PIN / FT_RST_PIN = -1) · DI ปล่อยลอย (0x4A) · BT / P0 / P1 ปล่อยลอย
  Pin ถูก hardcode ไว้ "เฉพาะใน sketch นี้" ไม่ใช่ในไลบรารี

  วางบอร์ดนิ่ง ๆ บนโต๊ะระหว่างทดสอบ · พิมพ์ 'r' + Enter เพื่อทดสอบซ้ำ

  Designed and Manufactured by Massmore — https://www.massmore.shop
*/

#include <Wire.h>
#include <math.h>
#include <Massmore_BNO08x.h>

/* ---------------- Factory Test wiring (ปรับได้ที่นี่เท่านั้น) ------------------ */
#if defined(CONFIG_IDF_TARGET_ESP32S3)      // MOMO by Massmore (ESP32-S3 + CH343P)
  #define FT_SDA_PIN   14
  #define FT_SCL_PIN   15
  #define FT_INT_PIN   (-1)
  #define FT_RST_PIN   (-1)   // ไม่ต่อ RST
  // #define FT_RST_PIN   18  // แนะนำ: ต่อ RST เข้า GPIO 18 แล้วเปิดคอมเมนต์บรรทัดนี้
                              // driver จะ reset ชิปให้ตอน begin() — กันอาการ I2C ค้าง
                              // เมื่อ upload firmware ใหม่ทับขณะชิปกำลังส่ง report
  #define FT_MCU_NAME  "ESP32-S3"
#elif defined(ARDUINO_ARCH_ESP32)
  #define FT_SDA_PIN   21
  #define FT_SCL_PIN   22
  #define FT_INT_PIN   (-1)      /* ต่อ INT ให้ใส่เลข GPIO เช่น 4 */
  #define FT_RST_PIN   (-1)      /* ต่อ RST ให้ใส่เลข GPIO เช่น 17 — driver reset ชิปเองได้ */
  #define FT_MCU_NAME  "ESP32"
#elif defined(__AVR__)
  #define FT_MCU_NAME  "AVR_NANO"
#else
  #define FT_MCU_NAME  "UNKNOWN"
#endif
#ifndef FT_INT_PIN
  #define FT_INT_PIN   (-1)
#endif
#ifndef FT_RST_PIN
  #define FT_RST_PIN   (-1)
#endif
#define FT_I2C_HZ      100000UL
#define FT_SAMPLES     20
/* -------------------------------------------------------------------------- */

/* Pass criteria (physical ranges — Datasheet §6 + bench sanity) */
#define FT_ACCEL_NOMINAL   9.81f
#define FT_ACCEL_TOL       1.50f     /* |a| ต้องอยู่ใน 9.81 ± 1.5 m/s^2 เมื่อวางนิ่ง */
#define FT_GYRO_STILL_MAX  0.50f     /* rad/s เมื่อวางนิ่ง */
#define FT_MAG_MIN         5.0f      /* uT — ต่ำกว่านี้ = magnetometer ตาย */
#define FT_MAG_MAX         200.0f    /* uT — สูงกว่านี้ = อิ่มตัว / มีแม่เหล็กใกล้ */
#define FT_QUAT_TOL        0.02f
#define FT_NOISE_MAX       1.00f     /* max-min ของ |a| ใน 20 sample */

Massmore_BNO08x imu;

static bool    ftFailed = false;
static const __FlashStringHelper *ftReason = nullptr;   /* เก็บใน flash ประหยัด SRAM บน AVR */
static uint8_t ftAddr = 0;

/* ---------------- helpers ------------------------------------------------- */
static void ftResult(const __FlashStringHelper *name, bool pass) {
  Serial.print(F("#RESULT ")); Serial.print(name);
  Serial.print(pass ? F(" PASS ") : F(" FAIL "));
}
static void ftFail(const __FlashStringHelper *reason) {
  if (!ftFailed) {
    ftFailed = true;
    ftReason = reason;
  }
}
static void ftVerdict() {
  Serial.println();
  if (ftFailed) {
    Serial.print(F("#VERDICT FAIL ")); Serial.println(ftReason);
    Serial.print(F("[FAIL] QA CHECK FAILED: ")); Serial.println(ftReason);
  } else {
    Serial.println(F("#VERDICT PASS"));
    Serial.println(F("[PASS] SENSOR QA PASSED - READY TO SHIP"));
  }
  Serial.println(F("Type 'r' + Enter to run the test again."));
}
static float vlen(const Massmore_BNO08x_vec3_t &v) {
  return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}
static void printHex32(uint32_t v) {
  for (int8_t s = 28; s >= 0; s -= 4) Serial.print((v >> s) & 0xF, HEX);
}

/* ---------------- the test ------------------------------------------------ */
static void runFactoryTest() {
  ftFailed = false;
  ftReason = nullptr;
  ftAddr = 0;

  Serial.println();
  Serial.println(F("#MASSMORE_FACTORY_TEST v1.0"));
  Serial.println(F("#PRODUCT Massmore_BNO08x"));
  Serial.print(F("#MCU ")); Serial.println(F(FT_MCU_NAME));
  Serial.print(F("Library v")); Serial.println(F(MASSMORE_BNO08X_VERSION_STR));

  /* 1. BUS_SCAN ---------------------------------------------------------- */
  bool bootloader = false;
  for (uint8_t a = 0x08; a < 0x78; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.print(F("  I2C device at 0x")); Serial.println(a, HEX);
      if (a == MASSMORE_BNO08X_I2C_ADDR_LOW || a == MASSMORE_BNO08X_I2C_ADDR_HIGH) {
        if (!ftAddr) ftAddr = a;
      } else if (a == MASSMORE_BNO08X_BOOTLOADER_ADDR_LOW || a == MASSMORE_BNO08X_BOOTLOADER_ADDR_HIGH) {
        bootloader = true;
      }
    }
  }
  ftResult(F("BUS_SCAN"), ftAddr != 0);
  if (ftAddr) { Serial.print(F("0x")); Serial.println(ftAddr, HEX); }
  else if (bootloader) { Serial.println(F("BOOTLOADER_MODE")); ftFail(F("BOOTLOADER_MODE_BT_PIN_LOW")); }
  else { Serial.println(F("NO_DEVICE")); ftFail(F("NO_DEVICE_ON_I2C")); }
  if (ftFailed) { ftVerdict(); return; }

  /* begin() — Product ID handshake -------------------------------------- */
  if (!imu.begin(ftAddr, Wire, FT_INT_PIN, FT_RST_PIN)) {
    ftResult(F("CHIP_ID"), false);
    Serial.println(Massmore_BNO08x::statusToString(imu.lastError()));
    ftFail(F("NO_PRODUCT_ID_RESPONSE"));
    ftVerdict(); return;
  }

  /* 2. CHIP_ID (SH-2 firmware part number) -------------------------------- */
  const Massmore_BNO08x_product_id_t &id = imu.getProductID();
  bool idOk = imu.verifyChipID();
  ftResult(F("CHIP_ID"), idOk);
  Serial.println(id.swPartNumber);
  for (uint8_t i = 0; i < imu.getProductIDCount(); i++) {
    const Massmore_BNO08x_product_id_t &e = imu.getProductID(i);
    Serial.print(F("  part ")); Serial.print(e.swPartNumber);
    Serial.print(F(" v"));      Serial.print(e.swVersionMajor); Serial.print('.');
    Serial.print(e.swVersionMinor); Serial.print('.'); Serial.print(e.swVersionPatch);
    Serial.print(F(" build ")); Serial.println(e.swBuildNumber);
  }
  if (!idOk) ftFail(F("CHIP_ID_MISMATCH"));

  /* CHIP model (BNO085 / BNO086) — informational, never fails the test ---- */
  Serial.print(F("#CHIP "));
  Serial.println(Massmore_BNO08x::chipModelToString(imu.getChipModel()));

  /* 3. FW_VERSION --------------------------------------------------------- */
  bool fwOk = (id.swVersionMajor >= 1 && id.swVersionMajor <= 9 && id.swBuildNumber != 0);
  ftResult(F("FW_VERSION"), fwOk);
  Serial.print(id.swVersionMajor); Serial.print('.');
  Serial.print(id.swVersionMinor); Serial.print('.');
  Serial.println(id.swVersionPatch);
  if (!fwOk) ftFail(F("FW_VERSION_IMPLAUSIBLE"));

  /* 4. SERIAL ------------------------------------------------------------- */
  {
    uint64_t sn = 0;
    Massmore_BNO08x_status_t rc = imu.readSerialNumber(sn);
    bool snOk = (rc == MASSMORE_BNO08X_OK) || (rc == MASSMORE_BNO08X_ERR_BAD_RESPONSE);
    ftResult(F("SERIAL"), snOk);
    if (rc == MASSMORE_BNO08X_OK) {
      Serial.print(F("0x")); printHex32((uint32_t)(sn >> 32)); printHex32((uint32_t)sn); Serial.println();
    } else if (rc == MASSMORE_BNO08X_ERR_BAD_RESPONSE) {
      Serial.println(F("NONE"));             /* record 0x4B4B not programmed — informational */
    } else {
      Serial.println(Massmore_BNO08x::statusToString(rc));
      ftFail(F("SERIAL_READ_TIMEOUT"));
    }
  }

  /* 5. METADATA (MotionEngine signature) ---------------------------------- */
  bool meOk = imu.verifyMotionEngine();
  ftResult(F("METADATA"), meOk);
  Serial.println(meOk ? F("Q14/12") : F("Q_POINT_MISMATCH"));
  if (!meOk) ftFail(F("MOTIONENGINE_METADATA"));

  /* 6. AUTHENTICITY ------------------------------------------------------- */
  bool genuine = imu.isGenuine();
  ftResult(F("AUTHENTICITY"), genuine);
  Serial.println(genuine ? F("GENUINE") : F("SUSPECT"));
  Serial.print(F("  ")); Serial.println(Massmore_BNO08x::authToString(imu.getLastAuthResult()));
  if (!genuine) ftFail(F("AUTHENTICITY_SUSPECT"));

  /* 7-10. RANGE checks (first good sample) -------------------------------- */
  Massmore_BNO08x_reading_t r;
  bool first = false;
  for (uint8_t i = 0; i < 5 && !first; i++) first = imu.readAll(r, 500);
  if (!first) {
    ftResult(F("RANGE_ACCEL"), false); Serial.println(F("TIMEOUT"));
    ftFail(F("NO_SENSOR_REPORTS"));
    ftVerdict(); return;
  }
  /* ให้ fusion นิ่งก่อนวัด */
  for (uint8_t i = 0; i < 10; i++) imu.readAll(r, 500);

  float amag = vlen(r.accel);
  bool aOk = isfinite(amag) && fabsf(amag - FT_ACCEL_NOMINAL) <= FT_ACCEL_TOL
             && amag <= MASSMORE_BNO08X_RANGE_ACCEL_MS2;
  ftResult(F("RANGE_ACCEL"), aOk); Serial.println(amag, 2);
  if (!aOk) ftFail(F("ACCEL_OUT_OF_RANGE"));

  float gmag = vlen(r.gyro);
  bool gOk = isfinite(gmag) && gmag <= FT_GYRO_STILL_MAX;
  ftResult(F("RANGE_GYRO"), gOk); Serial.println(gmag, 3);
  if (!gOk) ftFail(F("GYRO_OUT_OF_RANGE"));

  float mmag = vlen(r.mag);
  bool mOk = isfinite(mmag) && mmag >= FT_MAG_MIN && mmag <= FT_MAG_MAX;
  ftResult(F("RANGE_MAG"), mOk); Serial.println(mmag, 1);
  if (!mOk) ftFail(F("MAG_OUT_OF_RANGE"));

  float qn = sqrtf(r.quat.i * r.quat.i + r.quat.j * r.quat.j + r.quat.k * r.quat.k + r.quat.real * r.quat.real);
  bool qOk = isfinite(qn) && fabsf(qn - 1.0f) <= FT_QUAT_TOL;
  ftResult(F("RANGE_QUAT"), qOk); Serial.println(qn, 3);
  if (!qOk) ftFail(F("QUATERNION_NOT_UNIT"));

  /* 11. CONTINUOUS -------------------------------------------------------- */
  uint8_t good = 0;
  float aMin = 1e9f, aMax = -1e9f;
  for (uint8_t i = 0; i < FT_SAMPLES; i++) {
    if (!imu.readAll(r, 500)) continue;
    float a = vlen(r.accel);
    if (!isfinite(a) || !isfinite(r.eulerDeg.yaw)) continue;
    if (a < aMin) aMin = a;
    if (a > aMax) aMax = a;
    good++;
  }
  bool cOk = (good == FT_SAMPLES) && ((aMax - aMin) <= FT_NOISE_MAX);
  ftResult(F("CONTINUOUS"), cOk);
  Serial.print(good); Serial.print('/'); Serial.println(FT_SAMPLES);
  Serial.print(F("  |a| noise (max-min) = ")); Serial.println(aMax - aMin, 3);
  if (!cOk) ftFail(good == FT_SAMPLES ? F("ACCEL_NOISE_TOO_HIGH") : F("CONTINUOUS_READ_DROPOUT"));

  imu.disableAllReports();
  ftVerdict();
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) { }

#if defined(ARDUINO_ARCH_ESP32)
  Wire.begin(FT_SDA_PIN, FT_SCL_PIN);
#else
  Wire.begin();
#endif
  Wire.setClock(FT_I2C_HZ);

  Serial.println(F("\nMassmore_BNO08x - 10_Factory_Test (keep the board still)"));
  runFactoryTest();
}

void loop() {
  if (Serial.available()) {
    int c = Serial.read();
    if (c == 'r' || c == 'R') runFactoryTest();
  }
}
