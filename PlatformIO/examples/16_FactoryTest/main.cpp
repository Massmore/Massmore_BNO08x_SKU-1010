/* ไฟล์นี้สร้างจาก ArduinoIDE/Massmore_BNO08x/examples/16_FactoryTest/16_FactoryTest.ino
   แก้ที่ .ino ต้นฉบับ แล้วคัดลอกมาที่นี่
   (.cpp ไม่มีตัวสร้าง prototype อัตโนมัติแบบ .ino จึงต้อง include Arduino.h เอง) */
#include <Arduino.h>

/**
 * 16_FactoryTest
 * --------------
 * โปรแกรมทดสอบสำหรับสายการผลิต (Factory / QC Test) ของบอร์ด
 * Massmore BNO08x SKU-1010 (BNO085 / BNO086) ต่อผ่าน I2C
 *
 * ออกแบบให้ "เสียบแล้วรันเอง" ไม่ต้องพิมพ์อะไร เหมาะกับการเปิดผ่าน
 * Web Serial Monitor แล้วให้ลูกค้าดูผลได้ทันที
 *
 * ลำดับการทำงาน
 *   STAGE 1  ตรวจว่ามีอุปกรณ์อยู่จริงไหม  -> สแกนบัส I2C หา 0x4B / 0x4A
 *                                          และเตือนถ้าเจอ 0x28/0x29 (bootloader)
 *   STAGE 2  ตรวจว่าเป็นของแท้ไหม         -> Product ID (0xF8) + verifyChip()
 *            ถ้าสองด่านนี้ไม่ผ่าน จะ "ไม่เข้า" โหมด RUN TEST เด็ดขาด
 *   STAGE 3  RUN TEST ไล่ทดสอบจุดเด่นของ BNO08x ในโหมด I2C ครบทุกหัวข้อ
 *   STAGE 4  สรุปผล PASS / FAIL / WARN + ป้ายตัดสินรวม
 *
 * การต่อสาย (ESP32 DevKit, Flash 4 MB)
 *   ชื่อขาตามที่พิมพ์บนบอร์ด Massmore Halley V2
 *   Halley V2 5V หรือ 3Vo -> 5V หรือ 3V3
 *   Halley V2 GND         -> GND
 *   Halley V2 SDA         -> GPIO 21
 *   Halley V2 SCL         -> GPIO 22
 *   Halley V2 INT         -> ไม่ต้องต่อ (ชุดทดสอบนี้ใช้แบบ polling)
 *   Halley V2 RST         -> ไม่ต้องต่อ (ใช้ soft reset ผ่าน executable channel แทน)
 *   Halley V2 DI          -> ไม่ต้องต่อ = 0x4B, ต่อลง GND = 0x4A
 *   Halley V2 BT/P0/P1    -> ไม่ต้องต่อ (pull-up ในตัวเลือกโหมด I2C ให้เอง)
 *   ถ้าต่อ INT/RST ไว้ ให้แก้ FT_INT_PIN / FT_RST_PIN ข้างล่างเป็นเลขขาจริง
 *
 * หมายเหตุสำคัญ: BNO08x ใช้ clock stretching ชุดทดสอบนี้จึงตั้ง I2C ไว้ที่
 * 100 kHz ซึ่งเป็นค่าที่เสถียรที่สุดบน ESP32 / RP2040
 *
 * Serial Monitor: 115200 8N1
 * พิมพ์ 'r' แล้ว Enter เพื่อทดสอบซ้ำโดยไม่ต้องกดปุ่ม reset
 *
 * ทุกหัวข้อจะพิมพ์บรรทัดแบบเครื่องอ่านได้ท้ายผลด้วย
 *   #RESULT,<ลำดับ>,<ชื่อ>,<PASS|FAIL|WARN>,<รายละเอียด>
 * และปิดท้ายด้วย
 *   #VERDICT,<PASS|FAIL>,<passed>,<failed>,<warned>
 * เว็บสามารถ parse บรรทัดที่ขึ้นต้นด้วย '#' ไปทำไฟ PASS/FAIL ได้เลย
 *
 * Product: https://www.massmore.shop
 */

#include <Wire.h>
#include <Massmore_BNO08x.h>

/* ------------------------------------------------------------------ */
/*  ตั้งค่าฮาร์ดแวร์                                                    */
/* ------------------------------------------------------------------ */

/* ขา I2C -- ค่าเริ่มต้นสำหรับ ESP32 DevKit
 * ESP32-S3 / C3 / S2 ส่วนใหญ่ใช้ SDA=8 SCL=9 ให้แก้ตรงนี้ตามบอร์ดของคุณ */
#ifndef FT_SDA_PIN
#define FT_SDA_PIN 21
#endif
#ifndef FT_SCL_PIN
#define FT_SCL_PIN 22
#endif

/* -1 = ไม่ได้ต่อสาย ชุดทดสอบใช้ polling และ soft reset แทนได้ทั้งคู่ */
#ifndef FT_INT_PIN
#define FT_INT_PIN (-1)
#endif
#ifndef FT_RST_PIN
#define FT_RST_PIN (-1)
#endif

/* BNO08x ยืด clock (clock stretching) 100 kHz คือค่าที่ปลอดภัยที่สุด */
#define FT_I2C_FREQ_HZ  100000UL

MassmoreBNO08x imu;

/* ------------------------------------------------------------------ */
/*  ตัวนับผลและตัวช่วยพิมพ์                                             */
/* ------------------------------------------------------------------ */

enum ft_result_t { FT_PASS = 0, FT_FAIL = 1, FT_WARN = 2 };

static uint8_t ft_index  = 0;   /* ลำดับหัวข้อที่ทดสอบไปแล้ว */
static uint8_t ft_passed = 0;
static uint8_t ft_failed = 0;
static uint8_t ft_warned = 0;

static uint8_t ft_addr       = 0;      /* address ที่เจอจริงบนบัส */
static bool    ft_bootloader = false;  /* เจอ 0x28/0x29 = ขา BT (BOOTN) ถูกดึงลง */

/* ข้อมูลตัวชิปที่เก็บไว้ใช้ตอนพิมพ์รายงานสรุปท้ายสุด */
static massmore_auth_t ft_auth     = MASSMORE_AUTH_NO_RESPONSE;
static uint64_t        ft_serial   = 0;
static bool            ft_serialOk = false;

/* เก็บผลทุกหัวข้อไว้ เพื่อพิมพ์ซ้ำเป็นตารางรายงานตอนจบ
 * 32 ช่องพอสำหรับ 24 หัวข้อ และเหลือที่ให้เพิ่มในอนาคต */
#define FT_MAX_ITEMS 32
#define FT_DETAIL_LEN 96

typedef struct {
  const char *name;                 /* ชี้ไป string literal จึงไม่ต้องคัดลอก */
  uint8_t     res;                  /* ft_result_t */
  char        detail[FT_DETAIL_LEN];
} ft_item_t;

static ft_item_t ft_items[FT_MAX_ITEMS];

/* ตัวนับจำนวน report แต่ละชนิด นับจาก callback จึงได้จำนวนจริงต่อ report
 * ไม่ใช่ต่อแพ็กเก็ต (หนึ่งแพ็กเก็ต SHTP อาจมีหลาย report เมื่อทำ batching) */
static uint16_t ft_count[0x40];

static void ftOnReport(uint8_t id, void *ctx) {
  (void)ctx;
  if (id < 0x40 && ft_count[id] < 0xFFFF) ft_count[id]++;
}

static void ftCountReset() {
  for (uint8_t i = 0; i < 0x40; i++) ft_count[i] = 0;
}

static uint16_t ftCount(uint8_t id) {
  return (id < 0x40) ? ft_count[id] : 0;
}

/** ดูด packet จากเซนเซอร์ต่อเนื่องเป็นเวลา ms มิลลิวินาที */
static void ftPump(uint32_t ms) {
  uint32_t t0 = millis();
  while ((millis() - t0) < ms) imu.updateAll(16);
}

static void ftLine(char c) {
  for (uint8_t i = 0; i < 62; i++) Serial.print(c);
  Serial.println();
}

/** หัวข้อทดสอบ พิมพ์ "[nn] ชื่อหัวข้อ" */
static void ftStage(const char *name) {
  ft_index++;
  Serial.println();
  Serial.print(F("["));
  if (ft_index < 10) Serial.print('0');
  Serial.print(ft_index);
  Serial.print(F("] "));
  Serial.println(name);
}

/**
 * คัดลอกข้อความผลลัพธ์แบบไม่ตัดกลางตัวอักษร UTF-8
 * ภาษาไทยหนึ่งตัวกิน 3 ไบต์ ถ้าตัดดื้อ ๆ ที่ขอบบัฟเฟอร์จะได้ตัวประหลาดท้ายบรรทัด
 */
static void ftCopyDetail(char *dst, size_t cap, const char *src) {
  if (!dst || cap == 0) return;
  if (!src) { dst[0] = '\0'; return; }

  size_t n = 0;
  while (src[n] && n < cap - 1) n++;

  if (src[n]) {                       /* ตัดจริง ไม่ได้จบสตริงพอดี */
    /* ถอยกลับให้พ้น continuation byte (10xxxxxx) แล้วทิ้งตัวที่ไม่ครบไปเลย */
    while (n > 0 && ((uint8_t)src[n] & 0xC0) == 0x80) n--;
  }
  memcpy(dst, src, n);
  dst[n] = '\0';
}

/** ปิดท้ายหัวข้อ: นับผล + พิมพ์บรรทัดคนอ่าน + บรรทัดเครื่องอ่าน */
static void ftReport(const char *name, ft_result_t res, const char *detail) {
  const char *tag;
  switch (res) {
    case FT_PASS: tag = "PASS"; ft_passed++; break;
    case FT_FAIL: tag = "FAIL"; ft_failed++; break;
    default:      tag = "WARN"; ft_warned++; break;
  }

  /* เก็บไว้พิมพ์เป็นตารางรายงานตอนจบ */
  if (ft_index >= 1 && ft_index <= FT_MAX_ITEMS) {
    ft_item_t *it = &ft_items[ft_index - 1];
    it->name = name;
    it->res  = (uint8_t)res;
    ftCopyDetail(it->detail, FT_DETAIL_LEN, detail);
  }

  Serial.print(F("     -> "));
  Serial.print(tag);
  if (detail && detail[0]) {
    Serial.print(F("  ("));
    Serial.print(detail);
    Serial.print(')');
  }
  Serial.println();

  Serial.print(F("#RESULT,"));
  Serial.print(ft_index);
  Serial.print(',');
  Serial.print(name);
  Serial.print(',');
  Serial.print(tag);
  Serial.print(',');
  Serial.println(detail ? detail : "");
}

static void ftHex8(uint8_t v) {
  Serial.print(F("0x"));
  if (v < 0x10) Serial.print('0');
  Serial.print(v, HEX);
}

static float ftMag3(const massmore_vec3_t &v) {
  return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

static void ftPrintVec(const char *label, const massmore_vec3_t &v,
                       const char *unit) {
  Serial.print(F("     "));
  Serial.print(label);
  Serial.print(F(" x="));  Serial.print(v.x, 3);
  Serial.print(F(" y="));  Serial.print(v.y, 3);
  Serial.print(F(" z="));  Serial.print(v.z, 3);
  Serial.print(' ');
  Serial.print(unit);
  Serial.print(F("   |v|="));
  Serial.println(ftMag3(v), 3);
}

/** ระดับความแม่นที่ชิปรายงานมากับ report แต่ละชนิด (0..3) */
static void ftPrintAccuracy(const char *label, uint8_t sensorId) {
  Serial.print(F("     "));
  Serial.print(label);
  Serial.print(F(" accuracy = "));
  Serial.print((int)imu.getAccuracy(sensorId));
  Serial.print(F(" ("));
  Serial.print(MassmoreBNO08x::accuracyToString(imu.getAccuracy(sensorId)));
  Serial.println(')');
}

/* ================================================================== */
/*  STAGE 1 -- ค้นหาอุปกรณ์บนบัส I2C                                    */
/* ================================================================== */

/**
 * สแกนบัส I2C ทั้ง 126 address
 * ตั้ง ft_addr เป็น address ของ BNO08x ที่เจอ (0x4B ก่อน แล้วค่อย 0x4A)
 * ตั้ง ft_bootloader ถ้าเจอ 0x28/0x29 ซึ่งแปลว่าชิปบูตเข้า DFU เพราะขา BT (BOOTN)
 * @return จำนวนอุปกรณ์ทั้งหมดที่ตอบ ACK
 */
static uint8_t ftScanBus() {
  uint8_t count = 0;
  ft_addr       = 0;
  ft_bootloader = false;

  Serial.println(F("     สแกน address 0x01..0x7E"));
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() != 0) continue;

    count++;
    Serial.print(F("       พบอุปกรณ์ที่ "));
    ftHex8(addr);

    if (addr == MASSMORE_BNO08X_I2C_ADDR_HIGH) {
      Serial.print(F("   <-- BNO08x (ขา DI = 1, ค่าจากโรงงาน)"));
      ft_addr = addr;
    } else if (addr == MASSMORE_BNO08X_I2C_ADDR_LOW) {
      Serial.print(F("   <-- BNO08x (ขา DI = 0)"));
      if (ft_addr == 0) ft_addr = addr;
    } else if (addr == MASSMORE_BNO08X_BOOTLOADER_ADDR_LOW ||
               addr == MASSMORE_BNO08X_BOOTLOADER_ADDR_HIGH) {
      Serial.print(F("   <-- BOOTLOADER (DFU) ของ BNO08x !"));
      ft_bootloader = true;
    }
    Serial.println();
  }
  if (count == 0) Serial.println(F("       ไม่พบอุปกรณ์ใดเลยบนบัสนี้"));
  return count;
}

static bool ftStageDetect() {
  ftStage("DEVICE DETECT -- ค้นหา BNO08x บนบัส I2C");
  Serial.print(F("     SDA=GPIO"));
  Serial.print(FT_SDA_PIN);
  Serial.print(F("  SCL=GPIO"));
  Serial.print(FT_SCL_PIN);
  Serial.print(F("  clock="));
  Serial.print((uint32_t)(FT_I2C_FREQ_HZ / 1000UL));
  Serial.print(F(" kHz  INT="));
  if (FT_INT_PIN < 0) Serial.print(F("ไม่ต่อ")); else Serial.print(FT_INT_PIN);
  Serial.print(F("  RST="));
  if (FT_RST_PIN < 0) Serial.println(F("ไม่ต่อ")); else Serial.println(FT_RST_PIN);

  uint8_t total = ftScanBus();
  char detail[128];

  if (ft_bootloader && ft_addr == 0) {
    snprintf(detail, sizeof(detail),
             "เจอเฉพาะ bootloader 0x28/0x29 - ขา BT ถูกดึงลง");
    ftReport("DEVICE_DETECT", FT_FAIL, detail);
    Serial.println();
    Serial.println(F("     ชิปบูตเข้าโหมด DFU แทนโหมดใช้งานปกติ"));
    Serial.println(F("     ขา BT (BOOTN) ต้องถูกดึงขึ้น 3V3 หรือปล่อยให้ pull-up ในตัวทำงาน"));
    Serial.println(F("     นี่คือสาเหตุอันดับหนึ่งของอาการ \"บอร์ดสำเร็จรูปใช้ได้ PCB เองไม่ได้\""));
    return false;
  }

  if (ft_addr == 0) {
    snprintf(detail, sizeof(detail),
             "ไม่พบ 0x4B/0x4A (ตอบ ACK %u ตัว)", (unsigned)total);
    ftReport("DEVICE_DETECT", FT_FAIL, detail);
    Serial.println();
    Serial.println(F("     สิ่งที่ต้องตรวจ:"));
    Serial.println(F("       - สาย SDA/SCL สลับกันหรือหลุดหรือไม่"));
    Serial.println(F("       - มี pull-up 2.2k-4.7k ขึ้นไฟ 3V3 ทั้งสองเส้นหรือไม่"));
    Serial.println(F("         (datasheet CEVA ระบุ 2-4 kOhm ไม่ใช่ 10k)"));
    Serial.println(F("       - จ่ายไฟที่ขา 5V หรือ 3Vo และ GND ร่วมกับ ESP32 แล้วหรือยัง"));
    Serial.println(F("       - ขา BT (BOOTN) ปล่อยลอยหรือถูกดึงลงกราวด์หรือไม่"));
    Serial.println(F("       - ขา P0/P1 ต้องเป็น 0/0 จึงจะเป็นโหมด I2C"));
    return false;
  }

  snprintf(detail, sizeof(detail), "addr=0x%02X ACK, พบทั้งหมด %u ตัว",
           ft_addr, (unsigned)total);
  ftReport("DEVICE_DETECT", FT_PASS, detail);
  if (ft_bootloader) {
    Serial.println(F("     * เจอ bootloader address ด้วย -- ปกติจะไม่เห็นพร้อมกัน ตรวจขา BT"));
  }
  return true;
}

/* ================================================================== */
/*  STAGE 2 -- ตรวจว่าเป็น BNO08x ของแท้                                */
/* ================================================================== */

static bool ftStageIdentity() {
  ftStage("IDENTITY -- Product ID (0xF8) และตรวจว่าเป็นของแท้");

  if (!imu.begin(ft_addr, Wire, (int8_t)FT_INT_PIN, (int8_t)FT_RST_PIN)) {
    char detail[128];
    snprintf(detail, sizeof(detail), "begin() ล้มเหลว: %s",
             MassmoreBNO08x::statusToString(imu.getLastError()));
    ftReport("IDENTITY", FT_FAIL, detail);
    Serial.println(F("     มีอุปกรณ์ตอบ ACK แต่ไม่คุย SHTP -- ไม่ใช่ BNO08x หรือชิปค้าง"));
    return false;
  }

  imu.setReportCallback(ftOnReport, nullptr);

  const massmore_product_id_t &id = imu.getProductID();
  Serial.print(F("     reset cause      = "));
  Serial.print(id.resetCause);
  Serial.print(F("  ("));
  Serial.print(imu.getResetReasonString());
  Serial.println(')');

  Serial.print(F("     firmware version = "));
  Serial.print(id.swVersionMajor); Serial.print('.');
  Serial.print(id.swVersionMinor); Serial.print('.');
  Serial.println(id.swVersionPatch);

  Serial.print(F("     SW part number   = "));
  Serial.print(id.swPartNumber);
  Serial.println(F("   (10003606 = SH-2 image ของ BNO085/086)"));

  Serial.print(F("     SW build number  = "));
  Serial.println(id.swBuildNumber);

  /* ชิปตอบ Product ID หลายชุด ชุดละหนึ่งอิมเมจเฟิร์มแวร์ พิมพ์ให้ครบ */
  uint8_t nImages = imu.getProductIDCount();
  Serial.print(F("     อิมเมจที่ชิปแจ้ง  = "));
  Serial.print(nImages);
  Serial.println(F(" ชุด"));
  for (uint8_t i = 0; i < nImages; i++) {
    const massmore_product_id_t &e = imu.getProductID(i);
    Serial.print(F("       ["));
    Serial.print(i);
    Serial.print(F("] v"));
    Serial.print(e.swVersionMajor); Serial.print('.');
    Serial.print(e.swVersionMinor); Serial.print('.');
    Serial.print(e.swVersionPatch);
    Serial.print(F("  part "));  Serial.print(e.swPartNumber);
    Serial.print(F("  build ")); Serial.println(e.swBuildNumber);
  }

  massmore_auth_t auth = imu.verifyChip();
  ft_auth = auth;
  Serial.print(F("     verifyChip()     = "));
  Serial.println(MassmoreBNO08x::authToString(auth));

  char detail[128];
  switch (auth) {
    case MASSMORE_AUTH_OK:
      snprintf(detail, sizeof(detail),
               "BNO08x แท้ fw %u.%u.%u part=%lu",
               (unsigned)id.swVersionMajor, (unsigned)id.swVersionMinor,
               (unsigned)id.swVersionPatch, (unsigned long)id.swPartNumber);
      ftReport("IDENTITY", FT_PASS, detail);
      return true;

    case MASSMORE_AUTH_UNKNOWN_FW:
      snprintf(detail, sizeof(detail),
               "ตอบถูกทุกอย่าง แต่ part=%lu ยังไม่อยู่ในตาราง",
               (unsigned long)id.swPartNumber);
      ftReport("IDENTITY", FT_WARN, detail);
      Serial.println(F("     เป็นเฟิร์มแวร์รุ่นใหม่กว่าตารางในไลบรารี ถือว่าเป็นของแท้"));
      return true;

    default:
      snprintf(detail, sizeof(detail), "%s",
               MassmoreBNO08x::authToString(auth));
      ftReport("IDENTITY", FT_FAIL, detail);
      Serial.println(F("     มีอุปกรณ์ตอบที่ address นี้ แต่ไม่ใช่ BNO08x ของแท้"));
      return false;
  }
}

/* ================================================================== */
/*  STAGE 3 -- RUN TEST ทีละหัวข้อ                                      */
/* ================================================================== */

/** แปลง float เป็นสตริงทศนิยม 2 ตำแหน่ง โดยไม่ต้องพึ่ง %f ใน snprintf
 *  (บาง toolchain ตัด float formatting ออกเพื่อประหยัดแฟลช) */
static const char *ftF2(float v, char *buf, size_t n) {
  bool neg = (v < 0.0f);
  if (neg) v = -v;
  long whole = (long)v;
  long frac  = (long)((v - (float)whole) * 100.0f + 0.5f);
  if (frac >= 100) { frac -= 100; whole += 1; }
  snprintf(buf, n, "%s%ld.%02ld", neg ? "-" : "", whole, frac);
  return buf;
}

/* ---- 03  serial number จาก FRS ------------------------------------ */
static void ftTestSerialNumber() {
  ftStage("SERIAL NUMBER -- อ่านเลขซีเรียลจากโรงงาน (FRS 0x4B4B)");

  uint64_t serial = 0;
  char detail[128];

  if (imu.readSerialNumber(serial) != MASSMORE_OK) {
    snprintf(detail, sizeof(detail), "ไม่มี record 0x4B4B ในชิปตัวนี้ (%s)",
             MassmoreBNO08x::statusToString(imu.getLastError()));
    ftReport("SERIAL_NUMBER", FT_WARN, detail);
    Serial.println(F("     BNO08x หลายล็อตไม่ได้โปรแกรมเลขซีเรียลมาจากโรงงาน"));
    Serial.println(F("     ไม่ใช่ความเสียหายของบอร์ด และไม่กระทบการใช้งาน"));
    Serial.println(F("     ตัวพิสูจน์ว่า FRS อ่านได้จริงอยู่ที่หัวข้อ FRS_METADATA ถัดไป"));
    return;
  }

  ft_serial   = serial;
  ft_serialOk = true;

  uint32_t hi = (uint32_t)(serial >> 32);
  uint32_t lo = (uint32_t)(serial & 0xFFFFFFFFUL);
  Serial.print(F("     serial = 0x"));
  Serial.print(hi, HEX);
  Serial.println(lo, HEX);

  if (serial == 0) {
    ftReport("SERIAL_NUMBER", FT_WARN, "อ่านได้แต่เป็นศูนย์ทั้งหมด");
    return;
  }
  snprintf(detail, sizeof(detail), "serial=0x%08lX%08lX",
           (unsigned long)hi, (unsigned long)lo);
  ftReport("SERIAL_NUMBER", FT_PASS, detail);
}

/* ---- 04  metadata = พิสูจน์ว่า MotionEngine ทำงานจริง ---------------- */
static void ftTestMetadata() {
  ftStage("FRS METADATA -- อ่าน metadata ของ Rotation Vector (0xE30B)");

  uint32_t meta[16];
  uint16_t words = 0;
  char detail[128];

  if (imu.readSensorMetadata(MASSMORE_FRS_META_ROTATION_VECTOR, meta, 16, words)
        != MASSMORE_OK || words < 8) {
    snprintf(detail, sizeof(detail), "อ่าน metadata ไม่ได้ (words=%u)",
             (unsigned)words);
    ftReport("FRS_METADATA", FT_FAIL, detail);
    return;
  }

  uint16_t revision = (uint16_t)(meta[3] & 0xFFFF);
  Serial.print(F("     words = "));      Serial.print(words);
  Serial.print(F("   version = 0x"));    Serial.print(meta[0], HEX);
  Serial.print(F("   revision = "));     Serial.println(revision);

  /* ตำแหน่งของคู่ Q point ขยับตาม revision ของ metadata record จึงมองทั้ง
   * word 7 และ word 8 แทนที่จะ hard-code ช่องเดียวแล้วเดาว่าถูก
   * ส่วน min period ก็อยู่คนละ word ตาม revision เหมือนกัน จึงไม่พิมพ์ตัวเลข
   * ที่ยังไม่รู้แน่ว่าอ่านจากช่องไหน */
  int8_t qWord = -1;
  for (uint8_t w = 7; w <= 8 && w < words && w < 16; w++) {
    if ((uint16_t)(meta[w] & 0xFFFF) == MASSMORE_Q_QUAT &&
        (uint16_t)((meta[w] >> 16) & 0xFFFF) == MASSMORE_Q_QUAT_ACC) {
      qWord = (int8_t)w;
      break;
    }
  }

  Serial.print(F("     word7 = 0x")); Serial.print(meta[7], HEX);
  if (words > 8) { Serial.print(F("   word8 = 0x")); Serial.print(meta[8], HEX); }
  Serial.println();
  Serial.println(F("     มองหาคู่ Q point 14 (quaternion) กับ 12 (accuracy)"));

  if (qWord < 0) {
    snprintf(detail, sizeof(detail), "ไม่พบคู่ Q point 14/12 (rev=%u words=%u)",
             (unsigned)revision, (unsigned)words);
    ftReport("FRS_METADATA", FT_WARN, detail);
    return;
  }
  snprintf(detail, sizeof(detail), "Q=14/12 ที่ word %d rev=%u MotionEngine ทำงาน",
           (int)qWord, (unsigned)revision);
  ftReport("FRS_METADATA", FT_PASS, detail);
}

/* ---- 05  ชนิดออสซิลเลเตอร์ ----------------------------------------- */
static void ftTestOscillator() {
  ftStage("OSCILLATOR -- ถามชนิดออสซิลเลเตอร์ (command 10)");

  char detail[128];
  if (imu.requestOscillatorType() != MASSMORE_OK) {
    ftReport("OSCILLATOR", FT_WARN, "ไม่ตอบภายในเวลา (เฟิร์มแวร์เก่าอาจไม่รองรับ)");
    return;
  }
  uint8_t osc = imu.getOscillatorType();
  Serial.print(F("     oscillator type = "));
  Serial.print(osc);
  Serial.println(F("   (0 = internal, 1 = external crystal)"));
  snprintf(detail, sizeof(detail), "type=%u", (unsigned)osc);
  ftReport("OSCILLATOR", FT_PASS, detail);
}

/* ---- 06  คิวข้อผิดพลาดในตัวชิป -------------------------------------- */
static void ftTestErrorQueue() {
  ftStage("ERROR QUEUE -- อ่านคิว error ภายในชิป (command 1)");

  char detail[128];
  if (imu.requestErrorList() != MASSMORE_OK) {
    ftReport("ERROR_QUEUE", FT_WARN, "ส่งคำสั่งไม่สำเร็จ");
    return;
  }
  uint8_t n = imu.getErrorCount();
  Serial.print(F("     error count = "));
  Serial.println(n);
  snprintf(detail, sizeof(detail), "errors=%u", (unsigned)n);
  ftReport("ERROR_QUEUE", n == 0 ? FT_PASS : FT_WARN, detail);
}

/* ---- 07  soft reset ผ่าน executable channel ------------------------ */
static void ftTestSoftReset() {
  ftStage("SOFT RESET -- reset ผ่าน executable channel (ช่อง 1)");

  char detail[128];
  if (imu.softReset() != MASSMORE_OK) {
    snprintf(detail, sizeof(detail), "softReset() ล้มเหลว: %s",
             MassmoreBNO08x::statusToString(imu.getLastError()));
    ftReport("SOFT_RESET", FT_FAIL, detail);
    return;
  }

  if (imu.requestProductID(500) != MASSMORE_OK) {
    ftReport("SOFT_RESET", FT_FAIL, "reset แล้วชิปไม่กลับมาตอบ Product ID");
    return;
  }

  const massmore_product_id_t &id = imu.getProductID();
  Serial.print(F("     reset cause หลัง reset = "));
  Serial.print(id.resetCause);
  Serial.print(F("  ("));
  Serial.print(imu.getResetReasonString());
  Serial.println(')');

  snprintf(detail, sizeof(detail), "กลับมาพร้อมใช้งาน cause=%u",
           (unsigned)id.resetCause);
  ftReport("SOFT_RESET", FT_PASS, detail);
}

/* ---- 08  Set Feature / Get Feature ไป-กลับ -------------------------- */
static void ftTestFeatureConfig() {
  ftStage("FEATURE CONFIG -- Set Feature (0xFD) แล้วอ่านกลับด้วย Get Feature (0xFE)");

  char detail[128];
  const uint32_t want = 10000UL;   /* 100 Hz */

  if (imu.enableRotationVector(want) != MASSMORE_OK) {
    ftReport("FEATURE_CONFIG", FT_FAIL, "ส่ง Set Feature ไม่ผ่าน");
    return;
  }

  /* ให้ชิปเริ่มส่งรายงานจริงก่อน แล้วค่อยถามกลับ ไม่อย่างนั้นบางเฟิร์มแวร์
   * ตอบ Get Feature ด้วย interval = 0 เพราะยังไม่ทันตั้งค่าเสร็จ */
  ftCountReset();
  ftPump(300);
  uint16_t n = ftCount(MASSMORE_SENSOR_ROTATION_VECTOR);

  massmore_status_t rc = imu.requestFeature(MASSMORE_SENSOR_ROTATION_VECTOR);
  uint32_t got = imu.getReportInterval(MASSMORE_SENSOR_ROTATION_VECTOR);

  Serial.print(F("     ตั้งไว้ "));
  Serial.print(want);
  Serial.print(F(" us   ชิปตอบกลับ "));
  Serial.print(got);
  Serial.print(F(" us   ระหว่างนั้นได้ "));
  Serial.print(n);
  Serial.println(F(" report"));

  imu.disableReport(MASSMORE_SENSOR_ROTATION_VECTOR);
  delay(20);

  if (rc != MASSMORE_OK) {
    ftReport("FEATURE_CONFIG", FT_FAIL, "ชิปไม่ตอบ Get Feature Response");
    return;
  }
  if (got == 0) {
    /* คำสั่งไป-กลับบนช่อง control สำเร็จแล้ว และเซ็นเซอร์ส่งข้อมูลจริง
     * ค่าที่ echo กลับมาเป็น 0 เป็นพฤติกรรมของเฟิร์มแวร์ ไม่ใช่บอร์ดเสีย */
    if (n > 0) {
      snprintf(detail, sizeof(detail),
               "ตอบ Get Feature แต่ echo interval=0 (sensor ส่งจริง n=%u)",
               (unsigned)n);
      ftReport("FEATURE_CONFIG", FT_WARN, detail);
    } else {
      ftReport("FEATURE_CONFIG", FT_FAIL, "echo interval=0 และไม่มี report เข้ามาเลย");
    }
    return;
  }
  snprintf(detail, sizeof(detail), "set=%luus get=%luus (n=%u)",
           (unsigned long)want, (unsigned long)got, (unsigned)n);
  ftReport("FEATURE_CONFIG", FT_PASS, detail);
}

/* ---- 09  accelerometer: ต้องวัดแรงโน้มถ่วงได้ ----------------------- */
static void ftTestAccelerometer() {
  ftStage("ACCELEROMETER -- วัดความเร่ง 100 Hz และตรวจขนาดแรงโน้มถ่วง");

  char detail[128], b1[16];
  ftCountReset();
  imu.enableAccelerometer(MASSMORE_INTERVAL_100HZ);
  ftPump(700);
  uint16_t n = ftCount(MASSMORE_SENSOR_ACCELEROMETER);
  massmore_vec3_t a = imu.getAccel();
  imu.disableReport(MASSMORE_SENSOR_ACCELEROMETER);

  Serial.print(F("     ได้ "));
  Serial.print(n);
  Serial.println(F(" report ใน 700 ms"));
  ftPrintVec("accel ", a, "m/s^2");
  ftPrintAccuracy("accel ", MASSMORE_SENSOR_ACCELEROMETER);

  if (n == 0) {
    ftReport("ACCELEROMETER", FT_FAIL, "ไม่มี report เข้ามาเลย");
    return;
  }
  float mag = ftMag3(a);
  if (mag < 7.0f || mag > 12.5f) {
    snprintf(detail, sizeof(detail), "|a|=%s m/s2 ไม่ใกล้ 9.81 (n=%u)",
             ftF2(mag, b1, sizeof(b1)), (unsigned)n);
    ftReport("ACCELEROMETER", FT_WARN, detail);
    Serial.println(F("     ถ้าบอร์ดกำลังเคลื่อนไหวอยู่ ค่านี้จะเพี้ยนได้ วางนิ่งแล้วลองใหม่"));
    return;
  }
  snprintf(detail, sizeof(detail), "|a|=%s m/s2 (n=%u)",
           ftF2(mag, b1, sizeof(b1)), (unsigned)n);
  ftReport("ACCELEROMETER", FT_PASS, detail);
}

/* ---- 10  gyroscope: วางนิ่งต้องได้ใกล้ศูนย์ ------------------------- */
static void ftTestGyroscope() {
  ftStage("GYROSCOPE -- วัดความเร็วเชิงมุม 100 Hz ขณะวางนิ่ง");

  char detail[128], b1[16];
  ftCountReset();
  imu.enableGyroscope(MASSMORE_INTERVAL_100HZ);
  ftPump(700);
  uint16_t n = ftCount(MASSMORE_SENSOR_GYROSCOPE);
  massmore_vec3_t g   = imu.getGyro();
  massmore_vec3_t gd  = imu.getGyroDeg();
  imu.disableReport(MASSMORE_SENSOR_GYROSCOPE);

  Serial.print(F("     ได้ "));
  Serial.print(n);
  Serial.println(F(" report ใน 700 ms"));
  ftPrintVec("gyro  ", g,  "rad/s");
  ftPrintVec("gyro  ", gd, "deg/s");
  ftPrintAccuracy("gyro  ", MASSMORE_SENSOR_GYROSCOPE);

  if (n == 0) {
    ftReport("GYROSCOPE", FT_FAIL, "ไม่มี report เข้ามาเลย");
    return;
  }
  float mag = ftMag3(g);
  if (mag > 0.35f) {
    snprintf(detail, sizeof(detail), "|w|=%s rad/s สูงเกินขณะวางนิ่ง",
             ftF2(mag, b1, sizeof(b1)));
    ftReport("GYROSCOPE", FT_WARN, detail);
    Serial.println(F("     ปกติเกิดจากบอร์ดถูกขยับระหว่างทดสอบ วางนิ่งแล้วพิมพ์ r ใหม่"));
    return;
  }
  snprintf(detail, sizeof(detail), "|w|=%s rad/s (n=%u)",
           ftF2(mag, b1, sizeof(b1)), (unsigned)n);
  ftReport("GYROSCOPE", FT_PASS, detail);
}

/* ---- 11  magnetometer: ต้องเจอสนามแม่เหล็กโลก ---------------------- */
static void ftTestMagnetometer() {
  ftStage("MAGNETOMETER -- วัดสนามแม่เหล็ก และเทียบกับสนามโลก");

  char detail[128], b1[16];
  ftCountReset();
  imu.enableMagnetometer(MASSMORE_INTERVAL_100HZ);
  ftPump(900);
  uint16_t n = ftCount(MASSMORE_SENSOR_MAGNETIC_FIELD);
  massmore_vec3_t m = imu.getMag();
  imu.disableReport(MASSMORE_SENSOR_MAGNETIC_FIELD);

  Serial.print(F("     ได้ "));
  Serial.print(n);
  Serial.println(F(" report ใน 900 ms"));
  ftPrintVec("mag   ", m, "uT");
  ftPrintAccuracy("mag   ", MASSMORE_SENSOR_MAGNETIC_FIELD);
  Serial.println(F("     สนามแม่เหล็กโลกอยู่ราว 25-65 uT แล้วแต่ละติจูด"));

  if (n == 0) {
    ftReport("MAGNETOMETER", FT_FAIL, "ไม่มี report เข้ามาเลย");
    return;
  }
  float mag = ftMag3(m);
  if (mag < 15.0f || mag > 90.0f) {
    snprintf(detail, sizeof(detail), "|B|=%s uT อยู่นอกช่วงสนามโลก",
             ftF2(mag, b1, sizeof(b1)));
    ftReport("MAGNETOMETER", FT_WARN, detail);
    Serial.println(F("     มักเกิดจากมีเหล็ก ลำโพง มอเตอร์ หรือสายไฟกระแสสูงอยู่ใกล้"));
    Serial.println(F("     หรือยังไม่ได้ calibrate แม่เหล็ก (accuracy = 0)"));
    return;
  }
  snprintf(detail, sizeof(detail), "|B|=%s uT (n=%u)",
           ftF2(mag, b1, sizeof(b1)), (unsigned)n);
  ftReport("MAGNETOMETER", FT_PASS, detail);
}

/* ---- ตัวช่วยของกลุ่ม rotation vector ------------------------------- */

static float ftQuatNorm(const massmore_quat_t &q) {
  return sqrtf(q.i * q.i + q.j * q.j + q.k * q.k + q.real * q.real);
}

static void ftPrintQuat(const massmore_quat_t &q) {
  Serial.print(F("     quat i="));  Serial.print(q.i, 4);
  Serial.print(F(" j="));           Serial.print(q.j, 4);
  Serial.print(F(" k="));           Serial.print(q.k, 4);
  Serial.print(F(" w="));           Serial.print(q.real, 4);
  Serial.print(F("   |q|="));       Serial.println(ftQuatNorm(q), 4);
}

static void ftPrintEuler() {
  massmore_euler_t e = imu.getEulerDeg();
  Serial.print(F("     roll="));    Serial.print(e.roll, 1);
  Serial.print(F(" deg  pitch="));  Serial.print(e.pitch, 1);
  Serial.print(F(" deg  yaw="));    Serial.print(e.yaw, 1);
  Serial.print(F(" deg  heading=")); Serial.print(imu.getHeadingDeg(), 1);
  Serial.println(F(" deg"));
}

/**
 * ทดสอบ rotation vector หนึ่งชนิด: เปิด -> ดูด -> ตรวจ |q| ~ 1 -> ปิด
 * @param zeroIsWarn true = ไม่มี report ให้ตอบ WARN แทน FAIL
 *                   (ใช้กับ geomagnetic RV ที่บางเฟิร์มแวร์ให้อัตราต่ำมาก)
 */
static void ftTestRotationVector(const char *title, const char *tag,
                                 uint8_t sensorId, uint32_t intervalUs,
                                 uint32_t pumpMs, bool zeroIsWarn) {
  ftStage(title);

  char detail[128], b1[16];
  ftCountReset();
  imu.enableReport(sensorId, intervalUs);
  ftPump(pumpMs);
  uint16_t n = ftCount(sensorId);
  massmore_quat_t q = imu.getQuaternion();
  imu.disableReport(sensorId);

  Serial.print(F("     ได้ "));
  Serial.print(n);
  Serial.print(F(" report ใน "));
  Serial.print(pumpMs);
  Serial.println(F(" ms"));
  ftPrintQuat(q);
  ftPrintEuler();
  if (sensorId == MASSMORE_SENSOR_ROTATION_VECTOR ||
      sensorId == MASSMORE_SENSOR_GEOMAGNETIC_RV) {
    Serial.print(F("     heading accuracy = "));
    Serial.print(q.accuracy * 57.2957795f, 2);
    Serial.println(F(" deg"));
  }
  ftPrintAccuracy("RV    ", sensorId);

  if (n == 0) {
    ftReport(tag, zeroIsWarn ? FT_WARN : FT_FAIL, "ไม่มี report เข้ามาเลย");
    return;
  }
  float norm = ftQuatNorm(q);
  if (norm < 0.95f || norm > 1.05f) {
    snprintf(detail, sizeof(detail), "|q|=%s ไม่ใช่ unit quaternion",
             ftF2(norm, b1, sizeof(b1)));
    ftReport(tag, FT_FAIL, detail);
    return;
  }
  snprintf(detail, sizeof(detail), "|q|=%s (n=%u)",
           ftF2(norm, b1, sizeof(b1)), (unsigned)n);
  ftReport(tag, FT_PASS, detail);
}

/* ---- 15  gravity + linear acceleration ----------------------------- */
static void ftTestGravityLinear() {
  ftStage("GRAVITY / LINEAR ACC -- แยกแรงโน้มถ่วงออกจากความเร่งจริง");

  char detail[128], b1[16], b2[16];
  ftCountReset();
  imu.enableGravity(MASSMORE_INTERVAL_100HZ);
  imu.enableLinearAcceleration(MASSMORE_INTERVAL_100HZ);
  ftPump(800);
  uint16_t ng = ftCount(MASSMORE_SENSOR_GRAVITY);
  uint16_t nl = ftCount(MASSMORE_SENSOR_LINEAR_ACCELERATION);
  massmore_vec3_t gv = imu.getGravity();
  massmore_vec3_t la = imu.getLinearAccel();
  imu.disableReport(MASSMORE_SENSOR_GRAVITY);
  imu.disableReport(MASSMORE_SENSOR_LINEAR_ACCELERATION);

  Serial.print(F("     gravity n="));  Serial.print(ng);
  Serial.print(F("   linear accel n=")); Serial.println(nl);
  ftPrintVec("gravity", gv, "m/s^2");
  ftPrintVec("linacc ", la, "m/s^2");

  if (ng == 0 || nl == 0) {
    ftReport("GRAVITY_LINACC", FT_FAIL, "มี report ไม่ครบทั้งสองชนิด");
    return;
  }

  float mg = ftMag3(gv);
  float ml = ftMag3(la);
  ftF2(mg, b1, sizeof(b1));
  ftF2(ml, b2, sizeof(b2));

  if (mg < 8.3f || mg > 11.3f) {
    snprintf(detail, sizeof(detail), "|g|=%s m/s2 ผิดจาก 9.81 มาก", b1);
    ftReport("GRAVITY_LINACC", FT_WARN, detail);
    return;
  }
  if (ml > 1.5f) {
    snprintf(detail, sizeof(detail), "|g|=%s ok แต่ |linacc|=%s สูงขณะวางนิ่ง",
             b1, b2);
    ftReport("GRAVITY_LINACC", FT_WARN, detail);
    Serial.println(F("     ปกติเกิดจากบอร์ดถูกขยับ หรือ fusion ยังไม่เข้าที่"));
    return;
  }
  snprintf(detail, sizeof(detail), "|g|=%s  |linacc|=%s", b1, b2);
  ftReport("GRAVITY_LINACC", FT_PASS, detail);
}

/* ---- 16  gyro-integrated RV: ช่อง SHTP หมายเลข 5 อัตราสูง ---------- */
static void ftTestGyroIntegratedRV() {
  ftStage("GYRO-INTEGRATED RV -- quaternion อัตราสูงบนช่อง SHTP 5");

  char detail[128], b1[16];
  ftCountReset();
  imu.enableGyroIntegratedRotationVector(5000);   /* 200 Hz */
  ftPump(600);
  uint16_t n = ftCount(MASSMORE_SENSOR_GYRO_INTEGRATED_RV);
  massmore_quat_t q  = imu.getQuaternion();
  massmore_vec3_t av = imu.getAngularVelocity();
  imu.disableReport(MASSMORE_SENSOR_GYRO_INTEGRATED_RV);

  Serial.print(F("     ได้ "));
  Serial.print(n);
  Serial.print(F(" report ใน 600 ms  (~"));
  Serial.print((uint32_t)n * 1000UL / 600UL);
  Serial.println(F(" Hz)"));
  ftPrintQuat(q);
  ftPrintVec("angvel ", av, "rad/s");
  Serial.println(F("     report นี้เป็นจุดเด่นของ BNO08x: latency ต่ำสุด ไม่ผ่าน queue ปกติ"));

  if (n == 0) {
    ftReport("GYRO_INTEGRATED_RV", FT_FAIL, "ช่อง 5 ไม่ส่งข้อมูลเลย");
    return;
  }
  if (n < 20) {
    snprintf(detail, sizeof(detail), "ได้แค่ %u report ใน 600 ms", (unsigned)n);
    ftReport("GYRO_INTEGRATED_RV", FT_WARN, detail);
    Serial.println(F("     อัตราต่ำกว่าที่ควร มักเกิดจาก I2C ช้าหรือสายยาวเกิน"));
    return;
  }
  snprintf(detail, sizeof(detail), "n=%u |q|=%s", (unsigned)n,
           ftF2(ftQuatNorm(q), b1, sizeof(b1)));
  ftReport("GYRO_INTEGRATED_RV", FT_PASS, detail);
}

/* ---- 17  raw reports: ADC counts ตรงจากตัวเซนเซอร์ ------------------ */
static void ftTestRawReports() {
  ftStage("RAW REPORTS -- ค่า ADC ดิบของ accel / gyro / mag");

  char detail[128];
  ftCountReset();

  /* raw report ออกก็ต่อเมื่อเซ็นเซอร์ตัวนั้นกำลังทำงานอยู่จริง เปิดเฉพาะ raw
   * อย่างเดียวจะไม่ได้อะไรเลย ต้องเปิดตัวที่ผ่านการประมวลผลคู่กันด้วย */
  imu.enableAccelerometer(MASSMORE_INTERVAL_100HZ);
  imu.enableGyroscope(MASSMORE_INTERVAL_100HZ);
  imu.enableMagnetometer(MASSMORE_INTERVAL_100HZ);
  imu.enableRawAccelerometer(MASSMORE_INTERVAL_100HZ);
  imu.enableRawGyroscope(MASSMORE_INTERVAL_100HZ);
  imu.enableRawMagnetometer(MASSMORE_INTERVAL_100HZ);
  ftPump(1200);
  uint16_t na = ftCount(MASSMORE_SENSOR_RAW_ACCELEROMETER);
  uint16_t ng = ftCount(MASSMORE_SENSOR_RAW_GYROSCOPE);
  uint16_t nm = ftCount(MASSMORE_SENSOR_RAW_MAGNETOMETER);
  massmore_vec3i_t ra = imu.getRawAccel();
  massmore_vec3i_t rg = imu.getRawGyro();
  massmore_vec3i_t rm = imu.getRawMag();
  int16_t temp = imu.getRawGyroTemperature();
  imu.disableReport(MASSMORE_SENSOR_RAW_ACCELEROMETER);
  imu.disableReport(MASSMORE_SENSOR_RAW_GYROSCOPE);
  imu.disableReport(MASSMORE_SENSOR_RAW_MAGNETOMETER);
  imu.disableReport(MASSMORE_SENSOR_ACCELEROMETER);
  imu.disableReport(MASSMORE_SENSOR_GYROSCOPE);
  imu.disableReport(MASSMORE_SENSOR_MAGNETIC_FIELD);

  Serial.print(F("     raw accel n=")); Serial.print(na);
  Serial.print(F("  x="));  Serial.print(ra.x);
  Serial.print(F(" y="));   Serial.print(ra.y);
  Serial.print(F(" z="));   Serial.println(ra.z);
  Serial.print(F("     raw gyro  n=")); Serial.print(ng);
  Serial.print(F("  x="));  Serial.print(rg.x);
  Serial.print(F(" y="));   Serial.print(rg.y);
  Serial.print(F(" z="));   Serial.println(rg.z);
  Serial.print(F("     raw mag   n=")); Serial.print(nm);
  Serial.print(F("  x="));  Serial.print(rm.x);
  Serial.print(F(" y="));   Serial.print(rm.y);
  Serial.print(F(" z="));   Serial.println(rm.z);
  Serial.print(F("     gyro die temperature (raw counts) = "));
  Serial.println(temp);

  uint8_t got = (na ? 1 : 0) + (ng ? 1 : 0) + (nm ? 1 : 0);
  if (got == 0) {
    /* เฟิร์มแวร์บางบิลด์ไม่ปล่อย raw report ออกทาง I2C เลย เป็นข้อจำกัดของ
     * อิมเมจ ไม่ใช่บอร์ดเสีย และไม่กระทบการใช้งานปกติที่ใช้ค่าที่ผ่าน fusion */
    ftReport("RAW_REPORTS", FT_WARN, "เฟิร์มแวร์ตัวนี้ไม่ปล่อย raw report ออกมา");
    Serial.println(F("     ไม่กระทบการใช้งานปกติ ค่าที่ผ่าน fusion ยังครบทุกตัว"));
    return;
  }
  if (got < 3) {
    snprintf(detail, sizeof(detail), "ได้ %u จาก 3 ชนิด (a=%u g=%u m=%u)",
             (unsigned)got, (unsigned)na, (unsigned)ng, (unsigned)nm);
    ftReport("RAW_REPORTS", FT_WARN, detail);
    return;
  }
  if (ra.x == 0 && ra.y == 0 && ra.z == 0) {
    ftReport("RAW_REPORTS", FT_WARN, "raw accel เป็นศูนย์ทั้งสามแกน");
    return;
  }
  snprintf(detail, sizeof(detail), "ครบ 3 ชนิด (a=%u g=%u m=%u) temp=%d",
           (unsigned)na, (unsigned)ng, (unsigned)nm, (int)temp);
  ftReport("RAW_REPORTS", FT_PASS, detail);
}

/* ---- 18  uncalibrated reports พร้อมค่า bias ------------------------- */
static void ftTestUncalibrated() {
  ftStage("UNCALIBRATED -- gyro/mag แบบยังไม่หัก bias พร้อมค่า bias");

  char detail[128];
  ftCountReset();
  imu.enableGyroscopeUncalibrated(MASSMORE_INTERVAL_100HZ);
  imu.enableMagnetometerUncalibrated(MASSMORE_INTERVAL_100HZ);
  ftPump(800);
  uint16_t ng = ftCount(MASSMORE_SENSOR_GYROSCOPE_UNCAL);
  uint16_t nm = ftCount(MASSMORE_SENSOR_MAGNETIC_FIELD_UNCAL);
  massmore_vec3_t gb = imu.getGyroBias();
  massmore_vec3_t mb = imu.getMagBias();
  imu.disableReport(MASSMORE_SENSOR_GYROSCOPE_UNCAL);
  imu.disableReport(MASSMORE_SENSOR_MAGNETIC_FIELD_UNCAL);

  Serial.print(F("     gyro uncal n=")); Serial.print(ng);
  Serial.print(F("   mag uncal n="));    Serial.println(nm);
  ftPrintVec("gyroBias", gb, "rad/s");
  ftPrintVec("magBias ", mb, "uT");

  if (ng == 0 && nm == 0) {
    ftReport("UNCALIBRATED", FT_FAIL, "ไม่มี report ทั้งสองชนิด");
    return;
  }
  if (ng == 0 || nm == 0) {
    snprintf(detail, sizeof(detail), "ขาดไปหนึ่งชนิด (g=%u m=%u)",
             (unsigned)ng, (unsigned)nm);
    ftReport("UNCALIBRATED", FT_WARN, detail);
    return;
  }
  snprintf(detail, sizeof(detail), "g=%u m=%u report", (unsigned)ng, (unsigned)nm);
  ftReport("UNCALIBRATED", FT_PASS, detail);
}

/* ---- 19  หลาย report พร้อมกัน: ทดสอบ throughput ของบัส -------------- */
static void ftTestMultiReportRate() {
  ftStage("MULTI-REPORT RATE -- เปิด 3 sensor พร้อมกันแล้ววัดอัตราจริง");

  char detail[128];
  ftCountReset();
  imu.enableAccelerometer(MASSMORE_INTERVAL_100HZ);
  imu.enableGyroscope(MASSMORE_INTERVAL_100HZ);
  imu.enableRotationVector(MASSMORE_INTERVAL_50HZ);
  ftPump(1000);
  uint16_t na = ftCount(MASSMORE_SENSOR_ACCELEROMETER);
  uint16_t ng = ftCount(MASSMORE_SENSOR_GYROSCOPE);
  uint16_t nr = ftCount(MASSMORE_SENSOR_ROTATION_VECTOR);
  imu.disableReport(MASSMORE_SENSOR_ACCELEROMETER);
  imu.disableReport(MASSMORE_SENSOR_GYROSCOPE);
  imu.disableReport(MASSMORE_SENSOR_ROTATION_VECTOR);

  Serial.println(F("     ใน 1 วินาที (เป้าหมาย 100 / 100 / 50 Hz)"));
  Serial.print(F("       accel = ")); Serial.print(na); Serial.println(F(" Hz"));
  Serial.print(F("       gyro  = ")); Serial.print(ng); Serial.println(F(" Hz"));
  Serial.print(F("       RV    = ")); Serial.print(nr); Serial.println(F(" Hz"));

  if (na == 0 || ng == 0 || nr == 0) {
    snprintf(detail, sizeof(detail), "มี report ขาด (a=%u g=%u rv=%u)",
             (unsigned)na, (unsigned)ng, (unsigned)nr);
    ftReport("MULTI_REPORT_RATE", FT_FAIL, detail);
    return;
  }
  if (na < 60 || ng < 60 || nr < 30) {
    snprintf(detail, sizeof(detail), "อัตราต่ำกว่าเป้า a=%u g=%u rv=%u",
             (unsigned)na, (unsigned)ng, (unsigned)nr);
    ftReport("MULTI_REPORT_RATE", FT_WARN, detail);
    Serial.println(F("     ลองสั้นสายลง หรือเพิ่ม pull-up ให้แรงขึ้น"));
    return;
  }
  snprintf(detail, sizeof(detail), "a=%u g=%u rv=%u Hz",
           (unsigned)na, (unsigned)ng, (unsigned)nr);
  ftReport("MULTI_REPORT_RATE", FT_PASS, detail);
}

/* ---- 20  เอนจินตรวจจับเหตุการณ์ทั้งชุด ------------------------------ */
static void ftTestEventEngines() {
  ftStage("EVENT ENGINES -- step / stability / activity / tap / shake / gesture");

  char detail[128];
  uint8_t okCmd = 0;
  ftCountReset();

  if (imu.enableStepCounter(MASSMORE_INTERVAL_10HZ)          == MASSMORE_OK) okCmd++;
  if (imu.enableStepDetector(MASSMORE_INTERVAL_10HZ)         == MASSMORE_OK) okCmd++;
  if (imu.enableStabilityClassifier(MASSMORE_INTERVAL_10HZ)  == MASSMORE_OK) okCmd++;
  if (imu.enableActivityClassifier(MASSMORE_INTERVAL_10HZ)   == MASSMORE_OK) okCmd++;
  if (imu.enableTapDetector(MASSMORE_INTERVAL_100HZ)         == MASSMORE_OK) okCmd++;
  if (imu.enableShakeDetector(MASSMORE_INTERVAL_10HZ)        == MASSMORE_OK) okCmd++;
  if (imu.enableSignificantMotion(MASSMORE_INTERVAL_10HZ)    == MASSMORE_OK) okCmd++;
  if (imu.enableFlipDetector(MASSMORE_INTERVAL_10HZ)         == MASSMORE_OK) okCmd++;
  if (imu.enablePickupDetector(MASSMORE_INTERVAL_10HZ)       == MASSMORE_OK) okCmd++;
  if (imu.enableTiltDetector(MASSMORE_INTERVAL_10HZ)         == MASSMORE_OK) okCmd++;
  if (imu.enableStabilityDetector(MASSMORE_INTERVAL_10HZ)    == MASSMORE_OK) okCmd++;
  if (imu.enablePocketDetector(MASSMORE_INTERVAL_10HZ)       == MASSMORE_OK) okCmd++;
  if (imu.enableCircleDetector(MASSMORE_INTERVAL_10HZ)       == MASSMORE_OK) okCmd++;

  /* stability กับ activity classifier ต้องสะสมข้อมูลหลายวินาทีถึงจะเลิกตอบ
   * Unknown จึงให้เวลามากกว่าหัวข้ออื่น */
  ftPump(2500);

  uint16_t nStab = ftCount(MASSMORE_SENSOR_STABILITY_CLASSIFIER);
  uint16_t nAct  = ftCount(MASSMORE_SENSOR_ACTIVITY_CLASSIFIER);
  uint16_t nStep = ftCount(MASSMORE_SENSOR_STEP_COUNTER);

  Serial.print(F("     ส่ง Set Feature สำเร็จ "));
  Serial.print(okCmd);
  Serial.println(F(" / 13 เอนจิน"));
  Serial.print(F("     stability  = "));
  Serial.print(imu.getStabilityString());
  Serial.print(F("   (n=")); Serial.print(nStab); Serial.println(')');
  Serial.print(F("     activity   = "));
  Serial.print(imu.getActivityString());
  Serial.print(F("  confidence="));
  Serial.print(imu.getActivityConfidence(imu.getActivity()));
  Serial.print(F("%  (n=")); Serial.print(nAct); Serial.println(')');
  Serial.print(F("     step count = "));
  Serial.print(imu.getStepCount());
  Serial.print(F("   (n=")); Serial.print(nStep); Serial.println(')');
  Serial.print(F("     tap flags = "));
  ftHex8(imu.getTapDetector());
  Serial.print(F("   shake flags = "));
  ftHex8((uint8_t)imu.getShakeDetector());
  Serial.println();
  Serial.println(F("     เอนจินเหล่านี้รอเหตุการณ์จริง ไม่เคาะไม่เขย่าก็ไม่มีค่าออก"));

  imu.disableReport(MASSMORE_SENSOR_STEP_COUNTER);
  imu.disableReport(MASSMORE_SENSOR_STEP_DETECTOR);
  imu.disableReport(MASSMORE_SENSOR_STABILITY_CLASSIFIER);
  imu.disableReport(MASSMORE_SENSOR_ACTIVITY_CLASSIFIER);
  imu.disableReport(MASSMORE_SENSOR_TAP_DETECTOR);
  imu.disableReport(MASSMORE_SENSOR_SHAKE_DETECTOR);
  imu.disableReport(MASSMORE_SENSOR_SIGNIFICANT_MOTION);
  imu.disableReport(MASSMORE_SENSOR_FLIP_DETECTOR);
  imu.disableReport(MASSMORE_SENSOR_PICKUP_DETECTOR);
  imu.disableReport(MASSMORE_SENSOR_TILT_DETECTOR);
  imu.disableReport(MASSMORE_SENSOR_STABILITY_DETECTOR);
  imu.disableReport(MASSMORE_SENSOR_POCKET_DETECTOR);
  imu.disableReport(MASSMORE_SENSOR_CIRCLE_DETECTOR);

  if (okCmd < 13) {
    snprintf(detail, sizeof(detail), "ส่งคำสั่งได้ %u/13", (unsigned)okCmd);
    ftReport("EVENT_ENGINES", FT_FAIL, detail);
    return;
  }
  if (nStab == 0 && nAct == 0) {
    ftReport("EVENT_ENGINES", FT_WARN, "เปิดครบ 13 แต่ยังไม่มี classifier report");
    return;
  }
  snprintf(detail, sizeof(detail), "13/13 stability=%s activity=%s",
           imu.getStabilityString(), imu.getActivityString());
  ftReport("EVENT_ENGINES", FT_PASS, detail);
}

/* ---- 21  คำสั่ง calibration ของ MotionEngine ------------------------ */
static void ftTestCalibrationCommand() {
  ftStage("CALIBRATION CMD -- เปิด/ปิด dynamic calibration ของ MotionEngine");

  char detail[128];

  /* คำสั่ง ME calibrate สั่งกับ MotionEngine ที่กำลังทำงาน เปิด rotation vector
   * ไว้ก่อนเพื่อให้ fusion เดินอยู่ ไม่อย่างนั้นบางเฟิร์มแวร์ตอบ status ไม่เป็นศูนย์ */
  imu.enableRotationVector(MASSMORE_INTERVAL_100HZ);
  ftPump(400);

  massmore_status_t rc1 = imu.calibrateAll();
  delay(60);
  massmore_status_t rc2 = imu.requestCalibrationStatus();
  uint8_t st = imu.getCalibrationStatus();

  Serial.print(F("     calibrateAll()            -> "));
  Serial.println(MassmoreBNO08x::statusToString(rc1));
  Serial.print(F("     requestCalibrationStatus()-> "));
  Serial.println(MassmoreBNO08x::statusToString(rc2));
  Serial.print(F("     status byte = "));
  Serial.print(st);
  Serial.println(F("   (0 = ชิปรับคำสั่งแล้ว)"));

  ftPrintAccuracy("accel ", MASSMORE_SENSOR_ACCELEROMETER);
  ftPrintAccuracy("gyro  ", MASSMORE_SENSOR_GYROSCOPE);
  ftPrintAccuracy("mag   ", MASSMORE_SENSOR_MAGNETIC_FIELD);
  Serial.println(F("     ท่า calibrate ตาม datasheet Figure 3-2:"));
  Serial.println(F("       accel  วาง 4-6 ท่าที่ต่างกัน ค้างท่าละ ~1 วินาที"));
  Serial.println(F("       gyro   วางนิ่งบนโต๊ะ 2-3 วินาที"));
  Serial.println(F("       mag    หมุน 180 องศาแล้วกลับ ทีละแกน ~2 วินาที/แกน"));

  massmore_status_t rc3 = imu.endCalibration();
  delay(40);
  imu.disableReport(MASSMORE_SENSOR_ROTATION_VECTOR);

  if (rc1 != MASSMORE_OK || rc2 != MASSMORE_OK || rc3 != MASSMORE_OK) {
    snprintf(detail, sizeof(detail), "คำสั่งไม่ผ่าน (%s / %s / %s)",
             MassmoreBNO08x::statusToString(rc1),
             MassmoreBNO08x::statusToString(rc2),
             MassmoreBNO08x::statusToString(rc3));
    ftReport("CALIBRATION_CMD", FT_FAIL, detail);
    return;
  }
  if (st != 0) {
    snprintf(detail, sizeof(detail), "ชิปตอบ status=%u", (unsigned)st);
    ftReport("CALIBRATION_CMD", FT_WARN, detail);
    return;
  }
  ftReport("CALIBRATION_CMD", FT_PASS, "ME รับคำสั่ง calibrate และ stop ครบ");
}

/* ---- 22  คำสั่ง tare ------------------------------------------------ */
static void ftTestTareCommand() {
  ftStage("TARE CMD -- สั่ง tare แกน Z แล้วล้างค่ากลับ");

  char detail[128];
  imu.enableRotationVector(MASSMORE_INTERVAL_100HZ);
  ftPump(400);

  float before = imu.getHeadingDeg();
  massmore_status_t rc1 = imu.tareNow(MASSMORE_TARE_AXIS_Z,
                                      MASSMORE_TARE_BASIS_ROTATION_VECTOR);
  delay(120);
  ftPump(300);
  float after = imu.getHeadingDeg();

  Serial.print(F("     heading ก่อน tare = ")); Serial.print(before, 1);
  Serial.print(F(" deg   หลัง tare = "));       Serial.print(after, 1);
  Serial.println(F(" deg"));
  Serial.print(F("     tareNow()   -> "));
  Serial.println(MassmoreBNO08x::statusToString(rc1));

  massmore_status_t rc2 = imu.clearTare();
  delay(120);
  Serial.print(F("     clearTare() -> "));
  Serial.println(MassmoreBNO08x::statusToString(rc2));
  Serial.println(F("     (ชุดทดสอบนี้ไม่เรียก persistTare() จึงไม่เขียนลงแฟลช)"));

  imu.disableReport(MASSMORE_SENSOR_ROTATION_VECTOR);

  if (rc1 != MASSMORE_OK || rc2 != MASSMORE_OK) {
    snprintf(detail, sizeof(detail), "tare=%s clear=%s",
             MassmoreBNO08x::statusToString(rc1),
             MassmoreBNO08x::statusToString(rc2));
    ftReport("TARE_CMD", FT_FAIL, detail);
    return;
  }
  ftReport("TARE_CMD", FT_PASS, "tare และ clear ผ่านทั้งคู่");
}

/* ---- 23  sleep / wake ผ่าน executable channel ---------------------- */
static void ftTestSleepWake() {
  ftStage("SLEEP / WAKE -- สั่ง sleep แล้วปลุกกลับผ่านช่อง executable");

  char detail[128];
  massmore_status_t rc1 = imu.modeSleep();
  delay(300);
  massmore_status_t rc2 = imu.modeOn();
  delay(300);

  Serial.print(F("     modeSleep() -> "));
  Serial.println(MassmoreBNO08x::statusToString(rc1));
  Serial.print(F("     modeOn()    -> "));
  Serial.println(MassmoreBNO08x::statusToString(rc2));

  ftCountReset();
  imu.enableRotationVector(MASSMORE_INTERVAL_100HZ);
  ftPump(700);
  uint16_t n = ftCount(MASSMORE_SENSOR_ROTATION_VECTOR);
  imu.disableReport(MASSMORE_SENSOR_ROTATION_VECTOR);

  Serial.print(F("     หลังปลุก ได้ "));
  Serial.print(n);
  Serial.println(F(" report ใน 700 ms"));

  if (rc1 != MASSMORE_OK || rc2 != MASSMORE_OK) {
    ftReport("SLEEP_WAKE", FT_FAIL, "ส่งคำสั่ง executable ไม่ผ่าน");
    return;
  }
  if (n == 0) {
    ftReport("SLEEP_WAKE", FT_WARN, "ปลุกแล้วยังไม่มีข้อมูลกลับมาใน 700 ms");
    return;
  }
  snprintf(detail, sizeof(detail), "ปลุกกลับมาแล้ว n=%u", (unsigned)n);
  ftReport("SLEEP_WAKE", FT_PASS, detail);
}

/* ---- 24  ความต่อเนื่องของสตรีม: timestamp และ sequence number ------- */
static void ftTestStreamIntegrity() {
  ftStage("STREAM INTEGRITY -- ตรวจ timestamp และ sequence number ว่าไม่ตกหล่น");

  char detail[128];
  imu.enableRotationVector(MASSMORE_INTERVAL_50HZ);
  imu.clearNewFlags();

  uint16_t got = 0, gaps = 0, back = 0;
  uint8_t  prevSeq = 0;
  uint64_t prevTs  = 0;
  bool     first   = true;
  uint32_t t0 = millis();

  while (got < 120 && (millis() - t0) < 4000) {
    if (!imu.update()) continue;
    if (!imu.hasNewReport(MASSMORE_SENSOR_ROTATION_VECTOR)) continue;

    uint8_t  s  = imu.getSequenceNumber();
    uint64_t ts = imu.getTimestampUs();
    if (!first) {
      if ((uint8_t)(prevSeq + 1) != s) gaps++;
      if (ts < prevTs) back++;
    }
    first   = false;
    prevSeq = s;
    prevTs  = ts;
    got++;
  }
  imu.disableReport(MASSMORE_SENSOR_ROTATION_VECTOR);

  Serial.print(F("     เก็บได้ "));   Serial.print(got);
  Serial.print(F(" report   ช่วงที่ sequence ขาด = ")); Serial.print(gaps);
  Serial.print(F("   timestamp ถอยหลัง = "));           Serial.println(back);

  if (got < 20) {
    snprintf(detail, sizeof(detail), "เก็บได้แค่ %u report", (unsigned)got);
    ftReport("STREAM_INTEGRITY", FT_FAIL, detail);
    return;
  }
  if (back > 0) {
    /* timestamp อ้างอิงนาฬิกาโฮสต์ตอนที่แพ็กเก็ตเข้ามา ถ้า loop() ถูกงานอื่น
     * แย่งไปนาน ๆ ลำดับอาจสลับได้เล็กน้อย เป็นเรื่องจังหวะของโฮสต์ ไม่ใช่บอร์ดเสีย
     * ตัวชี้ขาดว่าข้อมูลตกหล่นจริงคือ sequence number ด้านล่าง */
    snprintf(detail, sizeof(detail), "timestamp ถอยหลัง %u ครั้ง (sequence ขาด %u)",
             (unsigned)back, (unsigned)gaps);
    ftReport("STREAM_INTEGRITY", FT_WARN, detail);
    return;
  }
  if (gaps > 0) {
    snprintf(detail, sizeof(detail), "sequence ขาด %u ครั้งจาก %u report",
             (unsigned)gaps, (unsigned)got);
    ftReport("STREAM_INTEGRITY", FT_WARN, detail);
    Serial.println(F("     ข้อมูลตกหล่นบนบัส I2C มักมาจากสายยาวหรือ pull-up อ่อน"));
    return;
  }
  snprintf(detail, sizeof(detail), "%u report ต่อเนื่อง ไม่มีขาด", (unsigned)got);
  ftReport("STREAM_INTEGRITY", FT_PASS, detail);
}

/* ================================================================== */
/*  ลำดับการรันทั้งชุด                                                  */
/* ================================================================== */

static void ftBanner() {
  Serial.println();
  ftLine('=');
  Serial.println(F("  MASSMORE BNO08x 9-AXIS IMU  --  FACTORY TEST"));
  Serial.println(F("  SKU-1010   BNO085 / BNO086 Sensor Fusion (I2C)"));
  ftLine('=');
  Serial.print(F("  library   : Massmore_BNO08x v"));
  Serial.println(F(MASSMORE_BNO08X_VERSION_STR));
  Serial.print(F("  board     : ESP32 DevKit (Flash 4 MB)"));
#if defined(ARDUINO_ARCH_ESP32)
  Serial.print(F("   CPU "));
  Serial.print(getCpuFrequencyMhz());
  Serial.print(F(" MHz"));
#endif
  Serial.println();
  Serial.print(F("  interface : I2C  SDA=GPIO"));
  Serial.print(FT_SDA_PIN);
  Serial.print(F("  SCL=GPIO"));
  Serial.print(FT_SCL_PIN);
  Serial.print(F("  clock="));
  Serial.print((uint32_t)(FT_I2C_FREQ_HZ / 1000UL));
  Serial.println(F(" kHz"));
  Serial.print(F("  address   : 0x4B (DI=1) หรือ 0x4A (DI=0) -- สแกนหาเอง"));
  Serial.println();
  Serial.print(F("  built     : "));
  Serial.print(F(__DATE__));
  Serial.print(' ');
  Serial.println(F(__TIME__));
  ftLine('=');
  Serial.println(F("  วางบอร์ดนิ่ง ๆ บนโต๊ะระหว่างทดสอบ จะได้ผลที่แม่นที่สุด"));
  ftLine('=');
}

/* ------------------------------------------------------------------ */
/*  รายงานสรุปท้ายสุด                                                   */
/* ------------------------------------------------------------------ */

/** เครื่องหมายหน้าแต่ละหัวข้อ กว้าง 3 ตัวอักษรเท่ากันทุกแบบ */
static void ftMark(uint8_t res) {
  switch (res) {
    case FT_PASS: Serial.print(F("[✓]")); break;   /* ถูก */
    case FT_FAIL: Serial.print(F("[✗]")); break;   /* ผิด */
    default:      Serial.print(F("[!]"));      break;   /* เตือน */
  }
}

/** พิมพ์ชื่อหัวข้อแล้วเติมช่องว่างให้ครบ 20 ตัว เพื่อให้คอลัมน์ตรงกัน */
static void ftPadName(const char *s) {
  uint8_t n = 0;
  while (s && s[n]) { Serial.print(s[n]); n++; }
  while (n < 20) { Serial.print(' '); n++; }
}

/** เลขลำดับสองหลัก */
static void ftNum2(uint8_t v) {
  if (v < 10) Serial.print('0');
  Serial.print(v);
}

/** ส่วนที่ 1 ของรายงาน: เจอ address อะไร ชิปอะไร ของแท้ไหม */
static void ftReportDevice() {
  const massmore_product_id_t &id = imu.getProductID();

  Serial.println(F("  [ อุปกรณ์ที่ตรวจพบ ]"));

  Serial.print(F("    I2C address   : "));
  if (ft_addr) {
    ftHex8(ft_addr);
    Serial.print(F("   (ขา DI = "));
    Serial.print(ft_addr == MASSMORE_BNO08X_I2C_ADDR_HIGH ? F("HIGH") : F("LOW"));
    Serial.println(')');
  } else if (ft_bootloader) {
    Serial.println(F("เจอแต่ bootloader 0x28/0x29 -- ขา BT ถูกดึงลง"));
  } else {
    Serial.println(F("ไม่พบอุปกรณ์บนบัสเลย"));
  }

  Serial.print(F("    Chip          : "));
  if (id.valid) {
    Serial.println(F("BNO085 / BNO086 (SH-2 MotionEngine)"));
  } else {
    Serial.println(F("ไม่ทราบ -- ไม่ตอบ Product ID"));
  }

  Serial.print(F("    Firmware      : "));
  if (id.valid) {
    Serial.print(id.swVersionMajor); Serial.print('.');
    Serial.print(id.swVersionMinor); Serial.print('.');
    Serial.println(id.swVersionPatch);
  } else {
    Serial.println('-');
  }

  Serial.print(F("    Part number   : "));
  if (id.valid) Serial.println(id.swPartNumber); else Serial.println('-');

  Serial.print(F("    Build number  : "));
  if (id.valid) Serial.println(id.swBuildNumber); else Serial.println('-');

  Serial.print(F("    Images        : "));
  Serial.print(imu.getProductIDCount());
  Serial.println(F(" ชุด (ชิปแจ้งเฟิร์มแวร์มาหลายอิมเมจ ปกติ)"));

  Serial.print(F("    Serial number : "));
  if (ft_serialOk) {
    Serial.print(F("0x"));
    Serial.print((uint32_t)(ft_serial >> 32), HEX);
    Serial.println((uint32_t)(ft_serial & 0xFFFFFFFFUL), HEX);
  } else {
    Serial.println(F("ไม่ได้โปรแกรมมาจากโรงงาน (ไม่กระทบการใช้งาน)"));
  }

  Serial.print(F("    Reset cause   : "));
  Serial.println(id.valid ? imu.getResetReasonString() : "-");

  Serial.print(F("    Authentic     : "));
  switch (ft_auth) {
    case MASSMORE_AUTH_OK:
      ftMark(FT_PASS);
      Serial.println(F(" ของแท้ -- part number ตรงตารางเฟิร์มแวร์โรงงาน"));
      break;
    case MASSMORE_AUTH_UNKNOWN_FW:
      ftMark(FT_WARN);
      Serial.println(F(" น่าจะแท้ -- ตอบถูกทุกอย่าง แต่ part number ใหม่กว่าตารางในไลบรารี"));
      break;
    default:
      ftMark(FT_FAIL);
      Serial.print(F(" ไม่ผ่าน -- "));
      Serial.println(MassmoreBNO08x::authToString(ft_auth));
      break;
  }
  Serial.println(F("                    (เป็นการตรวจระดับโปรโตคอล ไม่ใช่ลายเซ็นดิจิทัล)"));
}

/** ส่วนที่ 2 ของรายงาน: ไล่ผลทีละหัวข้อพร้อมเครื่องหมายถูก/ผิด */
static void ftReportItems() {
  Serial.print(F("  [ ผลรายฟังก์ชัน "));
  Serial.print(ft_index);
  Serial.println(F(" หัวข้อ ]"));

  uint8_t n = ft_index;
  if (n > FT_MAX_ITEMS) n = FT_MAX_ITEMS;

  for (uint8_t i = 0; i < n; i++) {
    Serial.print(F("    "));
    ftMark(ft_items[i].res);
    Serial.print(' ');
    ftNum2((uint8_t)(i + 1));
    Serial.print(' ');
    ftPadName(ft_items[i].name);
    Serial.println(ft_items[i].detail);
  }
}

/** ส่วนที่ 3 ของรายงาน: นับผ่าน / เตือน / ไม่ผ่าน */
static void ftReportTotals() {
  Serial.println(F("  [ สรุป ]"));

  Serial.print(F("    "));  ftMark(FT_PASS);
  Serial.print(F(" ผ่าน      "));
  if (ft_passed < 10) Serial.print(' ');
  Serial.print(ft_passed);
  Serial.println(F(" หัวข้อ"));

  Serial.print(F("    "));  ftMark(FT_WARN);
  Serial.print(F(" เตือน     "));
  if (ft_warned < 10) Serial.print(' ');
  Serial.print(ft_warned);
  Serial.println(F(" หัวข้อ   (สภาพแวดล้อมตอนทดสอบ ไม่ใช่บอร์ดเสีย)"));

  Serial.print(F("    "));  ftMark(FT_FAIL);
  Serial.print(F(" ไม่ผ่าน   "));
  if (ft_failed < 10) Serial.print(' ');
  Serial.print(ft_failed);
  Serial.println(F(" หัวข้อ"));

  Serial.print(F("        รวมทั้งหมด "));
  Serial.print(ft_index);
  Serial.println(F(" หัวข้อ"));

  /* ย้ำรายชื่อหัวข้อที่มีปัญหา เพื่อไม่ต้องเลื่อนหาเอง */
  uint8_t n = ft_index;
  if (n > FT_MAX_ITEMS) n = FT_MAX_ITEMS;

  if (ft_failed) {
    Serial.print(F("    หัวข้อที่ไม่ผ่าน :"));
    for (uint8_t i = 0; i < n; i++) {
      if (ft_items[i].res != FT_FAIL) continue;
      Serial.print(' ');
      ftNum2((uint8_t)(i + 1));
      Serial.print('-');
      Serial.print(ft_items[i].name);
    }
    Serial.println();
  }
  if (ft_warned) {
    Serial.print(F("    หัวข้อที่เตือน   :"));
    for (uint8_t i = 0; i < n; i++) {
      if (ft_items[i].res != FT_WARN) continue;
      Serial.print(' ');
      ftNum2((uint8_t)(i + 1));
      Serial.print('-');
      Serial.print(ft_items[i].name);
    }
    Serial.println();
  }
}

/** บรรทัดสำหรับให้เว็บ parse: ข้อมูลอุปกรณ์หนึ่งบรรทัดจบ */
static void ftMachineDeviceLine() {
  const massmore_product_id_t &id = imu.getProductID();

  Serial.print(F("#DEVICE,0x"));
  if (ft_addr < 0x10) Serial.print('0');
  Serial.print(ft_addr, HEX);
  Serial.print(',');
  if (id.valid) {
    Serial.print(id.swVersionMajor); Serial.print('.');
    Serial.print(id.swVersionMinor); Serial.print('.');
    Serial.print(id.swVersionPatch);
  }
  Serial.print(',');
  if (id.valid) Serial.print(id.swPartNumber);
  Serial.print(',');
  if (id.valid) Serial.print(id.swBuildNumber);
  Serial.print(',');
  if (ft_serialOk) {                    /* เว้นว่างถ้าชิปไม่มี record นี้ */
    Serial.print(F("0x"));
    Serial.print((uint32_t)(ft_serial >> 32), HEX);
    Serial.print((uint32_t)(ft_serial & 0xFFFFFFFFUL), HEX);
  }
  Serial.print(',');
  switch (ft_auth) {
    case MASSMORE_AUTH_OK:         Serial.println(F("GENUINE"));            break;
    case MASSMORE_AUTH_UNKNOWN_FW: Serial.println(F("GENUINE_UNKNOWN_FW")); break;
    case MASSMORE_AUTH_NO_RESPONSE:Serial.println(F("NO_RESPONSE"));        break;
    default:                       Serial.println(F("SUSPECT"));            break;
  }
}

static void ftSummary(bool gatePassed) {
  bool overall = gatePassed && (ft_failed == 0);

  Serial.println();
  ftLine('=');
  Serial.println(F("  รายงานผลการทดสอบ  /  TEST REPORT"));
  ftLine('=');

  ftReportDevice();
  ftLine('-');
  ftReportItems();
  ftLine('-');
  ftReportTotals();
  ftLine('=');

  if (overall) {
    Serial.println(F("   #####    #     #####  #####"));
    Serial.println(F("   #    #  # #   #      #     "));
    Serial.println(F("   #####  #####   ####   #### "));
    Serial.println(F("   #      #   #       #      #"));
    Serial.println(F("   #      #   #  #####  ##### "));
    Serial.println();
    Serial.println(F("  >>> บอร์ดนี้ผ่านการทดสอบทุกหัวข้อ พร้อมส่งมอบ <<<"));
    if (ft_warned) {
      Serial.println(F("  * มีหัวข้อ WARN -- ส่วนใหญ่เกิดจากสภาพแวดล้อมรอบตัว ไม่ใช่ความเสียหาย"));
    }
  } else {
    Serial.println(F("   #####    #    #  #####"));
    Serial.println(F("   #       # #   #  #    "));
    Serial.println(F("   ####   ##### #  #  ####"));
    Serial.println(F("   #      #   # #   # #   "));
    Serial.println(F("   #      #   # #    #####"));
    Serial.println();
    if (!gatePassed) {
      Serial.println(F("  >>> ไม่ผ่านด่านตรวจอุปกรณ์ จึงไม่ได้เข้าโหมด RUN TEST <<<"));
    } else {
      Serial.println(F("  >>> บอร์ดนี้ไม่ผ่าน -- ดูหัวข้อที่ขึ้น [✗] ด้านบน <<<"));
    }
  }
  ftLine('=');

  ftMachineDeviceLine();

  Serial.print(F("#VERDICT,"));
  Serial.print(overall ? F("PASS") : F("FAIL"));
  Serial.print(',');
  Serial.print(ft_passed);
  Serial.print(',');
  Serial.print(ft_failed);
  Serial.print(',');
  Serial.println(ft_warned);

  Serial.println();
  Serial.println(F("พิมพ์ 'r' แล้วกด Enter เพื่อทดสอบซ้ำ"));
}

static void runFactoryTest() {
  ft_index = ft_passed = ft_failed = ft_warned = 0;
  ft_auth     = MASSMORE_AUTH_NO_RESPONSE;
  ft_serial   = 0;
  ft_serialOk = false;
  memset(ft_items, 0, sizeof(ft_items));
  ftCountReset();

  ftBanner();

#if defined(ARDUINO_ARCH_ESP32)
  Wire.begin(FT_SDA_PIN, FT_SCL_PIN, FT_I2C_FREQ_HZ);
#else
  Wire.begin();
  Wire.setClock(FT_I2C_FREQ_HZ);
#endif

  Serial.println();
  Serial.println(F("### STAGE 1-2 : ตรวจอุปกรณ์ก่อนเข้าโหมดทดสอบ ###"));

  /* ด่านที่ 1 -- ต้องเจอ address ก่อน */
  if (!ftStageDetect()) {
    ftSummary(false);
    return;
  }

  /* ด่านที่ 2 -- ต้องเป็น BNO08x ของแท้ */
  if (!ftStageIdentity()) {
    ftSummary(false);
    return;
  }

  Serial.println();
  ftLine('-');
  Serial.println(F("  ผ่านด่านตรวจอุปกรณ์ -> เข้าสู่โหมด RUN TEST"));
  ftLine('-');
  Serial.println(F("### STAGE 3 : RUN TEST ###"));

  ftTestSerialNumber();
  ftTestMetadata();
  ftTestOscillator();
  ftTestErrorQueue();
  ftTestSoftReset();
  ftTestFeatureConfig();

  ftTestAccelerometer();
  ftTestGyroscope();
  ftTestMagnetometer();

  ftTestRotationVector("ROTATION VECTOR -- fusion 9 แกน (accel + gyro + mag)",
                       "ROTATION_VECTOR", MASSMORE_SENSOR_ROTATION_VECTOR,
                       MASSMORE_INTERVAL_100HZ, 800, false);
  ftTestRotationVector("GAME ROTATION VECTOR -- fusion 6 แกน ไม่ใช้แม่เหล็ก",
                       "GAME_RV", MASSMORE_SENSOR_GAME_ROTATION_VECTOR,
                       MASSMORE_INTERVAL_100HZ, 800, false);
  ftTestRotationVector("GEOMAGNETIC RV -- accel + mag ประหยัดพลังงาน",
                       "GEOMAGNETIC_RV", MASSMORE_SENSOR_GEOMAGNETIC_RV,
                       MASSMORE_INTERVAL_10HZ, 1200, true);

  ftTestGravityLinear();
  ftTestGyroIntegratedRV();
  ftTestRawReports();
  ftTestUncalibrated();
  ftTestMultiReportRate();
  ftTestEventEngines();
  ftTestCalibrationCommand();
  ftTestTareCommand();
  ftTestSleepWake();
  ftTestStreamIntegrity();

  /* คืนเซนเซอร์สู่สภาพเงียบก่อนจบ เผื่อผู้ใช้กด r ทดสอบซ้ำ */
  imu.disableAllReports();

  Serial.println();
  Serial.println(F("### STAGE 4 : SUMMARY ###"));
  ftSummary(true);
}

/* ================================================================== */
/*  setup / loop                                                       */
/* ================================================================== */

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 2000) { delay(10); }
  delay(300);            /* เผื่อเวลาให้ Serial Monitor บนเว็บเปิดทัน */

  runFactoryTest();
}

void loop() {
  if (Serial.available()) {
    char c = (char)Serial.read();
    if (c == 'r' || c == 'R') {
      runFactoryTest();
    }
  }
  delay(20);
}
