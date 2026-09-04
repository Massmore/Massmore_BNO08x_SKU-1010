# Massmore BNO08x

ไลบรารี Arduino / PlatformIO สำหรับเซ็นเซอร์ **BNO085 / BNO086** 9-axis Sensor Fusion IMU
(CEVA / Bosch SH-2 MotionEngine) — รองรับครบทุกฟังก์ชันตั้งแต่พื้นฐานถึงขั้นสูง

A complete Arduino / PlatformIO driver for the **BNO085 / BNO086** 9-axis sensor-fusion IMU.

**by Massmore** · [www.massmore.shop](https://www.massmore.shop) · [หน้าสินค้า / Product page](https://www.massmore.shop/products/2141d3bf-9d0f-4837-badf-a36bcda61638)

[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

---

## จุดเด่น / Highlights

| | |
|---|---|
| **3 อินเทอร์เฟซ** | I2C, SPI และ UART (SHTP) ในคลาสเดียว + คลาสแยกสำหรับ UART-RVC |
| **ครบทุก report** | ทุก sensor report ที่ SH-2 รองรับ — quaternion, accel, gyro, mag, tap, step, activity classifier, gyro-integrated RV 1 kHz |
| **ตรวจสอบชิปแท้** | อ่าน Product ID + Serial number จาก FRS + ตรวจ firmware part number |
| **Calibration / Tare** | ครบตามขั้นตอนของ CEVA (เอกสาร 1000-4044 และ 1000-4045) |
| **Non-blocking** | ไม่มี `delay()` ในลูปหลัก รองรับขา INT |
| **ไม่ใช้ heap** | ไม่มี `new` / `malloc` เลย — ปลอดภัยกับ MCU เล็ก |
| **ไม่ต้องพึ่งไลบรารีอื่น** | ใช้แค่ `Wire.h` และ `SPI.h` |

รองรับ / Tested on:

- Arduino IDE + **esp32 core 3.x** (ESP32, ESP32-S2, ESP32-S3, ESP32-C3, ESP32-C6)
- PlatformIO + `platform = espressif32` (board definition 3.x ขึ้นไป)
- AVR (Uno / Mega / Nano), SAMD, RP2040, STM32

---

## ติดตั้ง / Installation — Arduino IDE

โฟลเดอร์นี้เป็น **ไลบรารี Arduino มาตรฐาน** ติดตั้งได้ 2 วิธี

**วิธีที่ 1 — Add .ZIP Library**

1. บีบอัดโฟลเดอร์นี้เป็น `Massmore_BNO08x_ArduinoIDE.zip`
2. Arduino IDE → **Sketch → Include Library → Add .ZIP Library…**
3. เลือกไฟล์ zip

**วิธีที่ 2 — copy เข้าโฟลเดอร์ libraries โดยตรง**

copy โฟลเดอร์นี้ไปวางที่:

| ระบบปฏิบัติการ | ตำแหน่ง |
|---|---|
| Windows | `Documents\Arduino\libraries\` |
| macOS | `~/Documents/Arduino/libraries/` |
| Linux | `~/Arduino/libraries/` |

แล้วปิด-เปิด Arduino IDE ใหม่

### เปิดตัวอย่าง

**File → Examples → Massmore_BNO08x → 01_BasicRotationVector**

มีตัวอย่างให้ 16 ชุด เริ่มจาก 01 ก่อน

### ตั้งค่าบอร์ด ESP32 core 3.x

ถ้ายังไม่ได้ลง ESP32 core:

1. **File → Preferences → Additional Board Manager URLs** ใส่
   `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
2. **Tools → Board → Boards Manager…** ค้นหา `esp32` แล้วติดตั้ง
   **esp32 by Espressif Systems เวอร์ชัน 3.x**
3. เลือกบอร์ดของคุณ แล้วตั้ง **Upload Speed** ที่ **460800**
   (ไดรเวอร์ USB-serial บน macOS หลายตัวไม่นิ่งที่ 921600 · ถ้ายังไม่ผ่านให้ลดเหลือ 115200)

ไลบรารีนี้ใช้ได้ทั้ง core 2.x และ 3.x และใช้กับ AVR / SAMD / RP2040 / STM32 ได้ด้วย

### สำหรับ VS Code + PlatformIO

ใช้แพ็กเกจอีกโฟลเดอร์: **`Massmore_BNO08x_PlatformIO`**

---

## การต่อสาย / Wiring

### I2C (ง่ายที่สุด — เริ่มจากอันนี้)

ชื่อขาในตารางคือชื่อที่พิมพ์อยู่บนบอร์ด **Massmore Halley V2 (SKU-1010)**

| Halley V2 | ESP32 | หมายเหตุ |
|---|---|---|
| `5V` | 5V | บอร์ดมี regulator ในตัว |
| `3Vo` | 3V3 | จ่าย 3.3V ตรงก็ได้ เลือกอย่างใดอย่างหนึ่ง |
| `GND` | GND | |
| `SDA` | GPIO 21 | ผ่าน level shifter จึงรับ 5V logic ได้ |
| `SCL` | GPIO 22 | ผ่าน level shifter จึงรับ 5V logic ได้ |
| `INT` | GPIO 4 | ไม่บังคับ แต่**แนะนำมาก** — 3.3V เท่านั้น |
| `RST` | GPIO 5 | ไม่บังคับ — 3.3V เท่านั้น |
| `DI` | ไม่ต่อ | ปล่อยลอย = `0x4B` · ต่อลง GND = `0x4A` |
| `BT` `P0` `P1` | ไม่ต่อ | pull-up ในตัวเลือกโหมด I2C ให้เอง |

I2C address คือ **0x4B** เมื่อขา `DI` ต่อไฟหรือปล่อยลอย (ค่าเริ่มต้น)
และ **0x4A** เมื่อ `DI` ลงกราวด์ · ขา `DI` คือขาที่ datasheet เรียกว่า SA0/ADDR

### SPI (เร็วที่สุด)

| Halley V2 | ESP32 | หน้าที่ในโหมด SPI |
|---|---|---|
| `SCL` | GPIO 18 | SCK |
| `SDA` | GPIO 19 | MISO |
| `DI` | GPIO 23 | MOSI (ขาเดียวกับ SA0 ในโหมด I2C) |
| `CS` | GPIO 15 | chip select |
| `INT` | GPIO 4 | **บังคับ** |
| `RST` | GPIO 5 | **บังคับ** |
| `P0` | GPIO 16 | WAKE · ต้อง HIGH ตอน reset |
| `P1` | `3Vo` | ต้อง HIGH ตอน reset |
| `BT` | ไม่ต่อ | pull-up ในตัวกันไม่ให้เข้า bootloader |

> **สำคัญ:** โหมด SPI จะถูกล็อกตอนปล่อย reset โดยดูสถานะขา `P0`/`P1`
> ทั้งสองขาต้องเป็น HIGH ตั้งแต่ก่อน reset จนหลัง INT ทำงานครั้งแรก
>
> ขา `DI` และ `CS` **ไม่ผ่าน level shifter** จึงรับได้แค่ 3.3V — ใช้โฮสต์ 3.3V เท่านั้น

### UART-RVC (สายเส้นเดียว)

| Halley V2 | ESP32 | |
|---|---|---|
| `SDA` | GPIO 16 (RX) | ในโหมดนี้ขา `SDA` คือ TX ของเซ็นเซอร์ |
| `P1` | GND | |
| `P0` | `3Vo` | |
| `BT` | ไม่ต่อ | |

ได้ yaw / pitch / roll + accel ที่ 100 Hz ทันที ไม่ต้องสั่งอะไรเลย
ต้องใช้คริสตัลภายนอกในโหมดนี้ (คริสตัลภายในไม่แม่นพอสำหรับ UART)

### UART-SHTP (โปรโตคอลเต็มผ่าน UART)

| Halley V2 | ESP32 | |
|---|---|---|
| `SDA` | GPIO 16 (RX) | TX ของเซ็นเซอร์ |
| `SCL` | GPIO 17 (TX) | RX ของเซ็นเซอร์ |
| `P1` | `3Vo` | |
| `P0` | GND | |
| `BT` | ไม่ต่อ | |
| `INT` / `RST` | GPIO 4 / 5 | ไม่บังคับ |

ต่างจาก UART-RVC ตรงที่ได้ **ทุก report และทุกคำสั่ง** เหมือน I2C/SPI
บอดเรตตายตัวที่ **3,000,000** เปลี่ยนไม่ได้ · เพราะเร็วขนาดนี้และเป็นสัญญาณ push-pull
ให้ต่อที่ **3.3V ตรง ๆ** (ผ่าน Qwiic หรือโฮสต์ 3.3V) อย่าผ่าน level shifter 2N7002
ของบอร์ดซึ่งออกแบบมาสำหรับ I2C แบบ open-drain

---

## เริ่มต้นใช้งาน / Quick start

```cpp
#include <Wire.h>
#include <Massmore_BNO08x.h>

MassmoreBNO08x imu;

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);          // ESP32: SDA, SCL
  Wire.setClock(400000);

  // address, bus, INT pin, RST pin  (-1 = ไม่ได้ต่อ)
  if (!imu.begin(0x4B, Wire, 4, 5)) {
    Serial.println("ไม่พบเซ็นเซอร์");
    while (1);
  }

  imu.enableRotationVector(10000);   // 10000 us = 100 Hz
}

void loop() {
  if (!imu.update()) return;

  if (imu.hasNewReport(MASSMORE_SENSOR_ROTATION_VECTOR)) {
    massmore_euler_t e = imu.getEulerDeg();
    Serial.printf("roll %.1f  pitch %.1f  yaw %.1f\n", e.roll, e.pitch, e.yaw);
  }
}
```

---

## ตัวอย่างทั้งหมด / Examples

| # | ตัวอย่าง | เนื้อหา |
|---|---|---|
| 01 | `BasicRotationVector` | อ่าน quaternion 100 Hz — เริ่มที่นี่ |
| 02 | `EulerAngles` | roll / pitch / yaw และเปรียบเทียบ rotation vector ทั้ง 3 แบบ |
| 03 | `AccelGyroMag` | accel + gyro + mag + linear accel + gravity พร้อมกัน |
| 04 | `ChipIDVerify` | **ตรวจสอบชิปแท้** — Product ID, serial number, metadata |
| 05 | `Calibration` | ขั้นตอน calibration ตามคู่มือ CEVA + บันทึกลงแฟลช |
| 06 | `Tare` | ตั้งทิศ "หน้า" — recenter heading และบันทึกถาวร |
| 07 | `StepCounterActivity` | นับก้าว + activity classifier + stability |
| 08 | `TapShakeDetector` | tap / double tap / shake / pickup / flip / tilt |
| 09 | `RawSensorData` | ข้อมูล ADC ดิบพร้อม timestamp เป็น CSV |
| 10 | `HighRateGyroRV` | gyro-integrated RV 400 Hz–1 kHz สำหรับ AR/VR และ gimbal |
| 11 | `SPI_Interface` | ใช้งานผ่าน SPI 3 MHz |
| 12 | `UART_RVC` | โหมดสายเดียว 100 Hz |
| 13 | `MultiReportAdvanced` | 10 reports พร้อมกัน + callback + batching + sleep/wake |
| 14 | `FRS_Records` | อ่าน/เขียน flash records ของเซ็นเซอร์ |
| 15 | `UART_SHTP` | SH-2 เต็มรูปแบบผ่าน UART 3 Mbaud ต่างจาก 12 ที่เป็น RVC อย่างเดียว |
| 16 | `FactoryTest` | ทดสอบบอร์ดครบทุกฟีเจอร์ในโหมด I2C แล้วออกรายงานสรุป PASS/FAIL |

---

## API สำคัญ / Key API

### เริ่มต้น

```cpp
bool begin(uint8_t address = 0x4B, TwoWire &wire = Wire,
           int8_t intPin = -1, int8_t rstPin = -1);
bool beginSPI(int8_t cs, int8_t intPin, int8_t rstPin,
              int8_t wakePin = -1, SPIClass &spi = SPI, uint32_t hz = 3000000);
bool beginUART(Stream &serial, int8_t intPin = -1, int8_t rstPin = -1);
```

### ลูปหลัก

```cpp
bool     update();                  // อ่าน 1 packet — ไม่ block
uint8_t  updateAll(uint8_t max=16); // ดึงทุก packet ที่ค้างอยู่
bool     hasNewReport(uint8_t id);  // เช็คแล้วเคลียร์ flag
void     setReportCallback(void (*cb)(uint8_t reportId, void *ctx), void *ctx);
```

### เปิดเซ็นเซอร์

ทุกฟังก์ชันรับค่า **report interval หน่วยไมโครวินาที** (10000 µs = 100 Hz):

```cpp
imu.enableRotationVector(10000);          // 9-axis, มีทิศเหนือจริง
imu.enableGameRotationVector(10000);      // 6-axis, ไม่ใช้ mag, ไม่โดนแม่เหล็กรบกวน
imu.enableARVRStabilizedRotationVector(10000);
imu.enableGyroIntegratedRotationVector(1000);   // สูงสุด 1 kHz
imu.enableAccelerometer(20000);
imu.enableGyroscope(20000);
imu.enableMagnetometer(50000);
imu.enableTapDetector(10000);
imu.enableStepCounter(200000);
imu.enableActivityClassifier(500000, 0x1FF);
// ... และอีกกว่า 30 แบบ ดูใน Massmore_BNO08x.h
```

ควบคุมเต็มรูปแบบด้วย `setFeature()`:

```cpp
imu.setFeature(MASSMORE_SENSOR_MAGNETIC_FIELD,
               10000,      // report interval (µs)
               500000,     // batch interval (µs) — รวมส่งเป็นชุด ประหยัดไฟ
               MASSMORE_FEATURE_FLAG_WAKE_ENABLED,
               0,          // change sensitivity
               0);         // sensor-specific config word
```

### อ่านค่า

```cpp
massmore_quat_t  q = imu.getQuaternion();     // i, j, k, real, accuracy
massmore_euler_t e = imu.getEulerDeg();       // roll, pitch, yaw (องศา)
float heading      = imu.getHeadingDeg();     // 0..360

massmore_vec3_t a  = imu.getAccel();          // m/s²
massmore_vec3_t g  = imu.getGyroDeg();        // deg/s
massmore_vec3_t m  = imu.getMag();            // µT

uint32_t steps     = imu.getStepCount();
uint8_t  tap       = imu.getTapDetector();    // อ่านแล้วเคลียร์
const char *act    = imu.getActivityString();

massmore_accuracy_t acc = imu.getAccuracy(MASSMORE_SENSOR_ROTATION_VECTOR);
uint64_t t_us      = imu.getTimestampUs();
```

### ตรวจสอบชิปแท้

```cpp
massmore_auth_t a = imu.verifyChip();
Serial.println(MassmoreBNO08x::authToString(a));

const massmore_product_id_t &id = imu.getProductID();
// id.swVersionMajor/Minor/Patch, id.swPartNumber, id.swBuildNumber, id.resetCause

uint64_t serial;
imu.readSerialNumber(serial);   // จาก FRS record 0x4B4B
```

**การตรวจสอบนี้พิสูจน์อะไรได้:** BNO08x ไม่มีระบบ cryptographic attestation
ดังนั้นไม่มีไลบรารีไหนพิสูจน์ความแท้ได้ 100% ในเชิงคณิตศาสตร์
สิ่งที่ `verifyChip()` พิสูจน์ได้คือ **ชิ้นส่วนบนบอร์ดของคุณทำงานเหมือน BNO08x ของแท้ทุกประการในระดับโปรโตคอล** —
ประกาศ SHTP advertisement ถูกต้อง ตอบ Product ID Request ด้วยเวอร์ชันเฟิร์มแวร์ที่สมเหตุสมผล
และมี serial number อยู่ในแฟลช ของปลอม ชิปที่ถูกแปะฉลากใหม่ (เช่น BNO055 ที่พิมพ์ว่า BNO086)
ไดที่ยังไม่ได้โปรแกรม หรือบอร์ดที่ไม่มีเซ็นเซอร์ จะตกหนึ่งในขั้นตอนเหล่านี้

ค่าที่ส่งกลับ:

| ค่า | ความหมาย |
|---|---|
| `MASSMORE_AUTH_OK` | ตอบถูกต้องทุกอย่าง และ firmware part number ตรงกับที่รู้จัก |
| `MASSMORE_AUTH_UNKNOWN_FW` | เป็น BNO08x จริง แต่ part number ใหม่กว่าตารางในไลบรารี — **ไม่ใช่ความผิดพลาด** |
| `MASSMORE_AUTH_BAD_VERSION` | ตอบมา แต่เลขเวอร์ชันไม่สมเหตุสมผล |
| `MASSMORE_AUTH_NO_RESPONSE` | ไม่ตอบเลย — ไม่ใช่ BNO08x หรือต่อสาย/แอดเดรสผิด |
| `MASSMORE_AUTH_BAD_RESPONSE` | ตอบมาแต่ข้อมูลผิดรูปแบบ |

### Calibration

```cpp
imu.calibrateAll();          // เปิด dynamic calibration ทั้ง accel + gyro + mag
// ... แกว่งเลข 8 / วางทั้ง 6 ด้าน / วางนิ่ง ...
imu.saveCalibration();       // บันทึกลงแฟลช (Save DCD)
imu.endCalibration();
imu.clearCalibrationAndReset();
```

ดูขั้นตอนละเอียดในตัวอย่าง `05_Calibration`

### Tare (ตั้งทิศหน้า)

```cpp
imu.tareNow(MASSMORE_TARE_AXIS_Z);    // recenter เฉพาะ heading — ปุ่ม recenter
imu.tareNow(MASSMORE_TARE_AXIS_ALL);  // จัดแนวเต็ม — ต้องหันหน้าไปทิศเหนือและวางระนาบ
imu.persistTare();                    // บันทึกถาวรลง FRS
imu.clearTare();
```

### จัดการพลังงาน

```cpp
imu.modeSleep();   // เหลือแค่ wake sensors ทำงาน
imu.modeOn();
imu.softReset();
imu.hardwareReset();
```

---

## เลือก Rotation Vector แบบไหนดี

| แบบ | เซ็นเซอร์ที่ใช้ | ข้อดี | ข้อเสีย |
|---|---|---|---|
| **Rotation Vector** | accel + gyro + mag | heading อ้างอิงทิศเหนือจริง ไม่ดริฟต์ | โดนมอเตอร์/เหล็ก/ลำโพงรบกวน ต้อง calibrate mag |
| **Game Rotation Vector** | accel + gyro | ไม่โดนแม่เหล็กรบกวนเลย นิ่งมาก | yaw ดริฟต์ช้า ๆ ไม่รู้ทิศเหนือ |
| **Geomagnetic RV** | accel + mag | ประหยัดไฟที่สุด | เรตต่ำ ตอบสนองช้า |
| **AR/VR Stabilized RV** | accel + gyro + mag | เหมือน RV แต่ตัดการกระตุกออก | ดีเลย์เพิ่มเล็กน้อย |
| **Gyro-Integrated RV** | gyro (integrate) | เร็วสุด ถึง 1 kHz ดีเลย์ต่ำสุด | ดริฟต์ ต้องมี RV ตัวอื่นคอยแก้ |

**เลือกยังไง:** หุ่นยนต์/โดรนที่ต้องรู้ทิศเหนือ → Rotation Vector ·
gimbal หรือของที่อยู่ใกล้มอเตอร์ → Game Rotation Vector ·
AR/VR head tracking → Gyro-Integrated RV + ARVR Stabilized RV

---

## แก้ปัญหา / Troubleshooting

| อาการ | สาเหตุที่พบบ่อย |
|---|---|
| `begin()` คืน false | แอดเดรสผิด — ลอง `0x4A` · ไม่ได้ต่อ 3.3V · ไม่มี pull-up บน I2C |
| ค่านิ่งไม่ขยับ | ยังไม่ได้เรียก `enableXxx()` หรือลืมเรียก `update()` ในลูป |
| heading ดริฟต์หรือกระโดด | ต้อง calibrate magnetometer (ตัวอย่าง 05) หรือมีเหล็ก/มอเตอร์ใกล้เกินไป |
| ข้อมูลหายที่เรตสูง | ต่อขา INT ด้วย · ตั้ง `Wire.setClock(400000)` · ใช้ `updateAll()` แทน `update()` |
| SPI ไม่ทำงาน | ขา `P0`/`P1` ต้อง HIGH ตอน reset · ต้องต่อทั้ง `INT` และ `RST` |
| UART-RVC เงียบ | ขา `P1` ต้องลงกราวด์ `P0` ต้องขึ้นไฟ · ต้องใช้คริสตัลภายนอก |
| UART-SHTP เงียบ | ขา `P1` ต้องขึ้นไฟ `P0` ต้องลงกราวด์ ตั้งแต่ก่อน reset · `SDA`/`SCL` สลับกัน · ต่อผ่าน level shifter ที่ 3 Mbaud ไม่ได้ |
| ค่าดูแปลก ๆ ตอนบูต | เซ็นเซอร์ต้องใช้เวลาราว 90 ms หลัง reset — `begin()` รอให้แล้ว |

เปิด debug เพื่อดูว่าไดรเวอร์เห็นอะไร:

```cpp
imu.enableDebug(Serial);
```

---

## หมายเหตุทางเทคนิค / Implementation notes

- ไลบรารีนี้เขียนโปรโตคอล SH-2 / SHTP ขึ้นใหม่ทั้งหมดจากเอกสารสาธารณะของ CEVA
  **ไม่มีซอร์สโค้ดของ CEVA หรือ Bosch อยู่ในนี้** จึงไม่ต้องพึ่งไดรเวอร์ `sh2` ภายนอก
- ความยาวของแต่ละ report อ่านมาจาก **SHTP advertisement ที่เซ็นเซอร์ประกาศเองตอนบูต**
  (TLV tag `0x81`) ไม่ได้ hard-code ไว้ ทำให้รองรับเฟิร์มแวร์เวอร์ชันใหม่ได้อัตโนมัติ
  มีตารางสำรองไว้เผื่อพลาด advertisement
- Q-point scaling ทุกค่าอ้างอิงจาก SH-2 Reference Manual §6.5:
  accel Q8 · gyro Q9 · mag Q4 · quaternion Q14 · accuracy Q12 · angular velocity Q10
- Timestamp คำนวณจาก base timestamp (report `0xFB`) รวมกับ delay ในแต่ละ report
  ตามที่ระบุใน datasheet §1.3.5.3

### เอกสารอ้างอิง

| เอกสาร | รหัส |
|---|---|
| BNO08X Datasheet | CEVA 1000-3927 |
| SH-2 Reference Manual | CEVA 1000-3625 |
| Sensor Hub Transport Protocol | CEVA 1000-3535 |
| Sensor Calibration Procedure | CEVA 1000-4044 |
| Tare Function Usage Guide | CEVA 1000-4045 |

---

---

## ทดสอบบนเครื่อง PC / Host tests

ไลบรารีนี้มีชุดทดสอบที่รันบน PC ได้เลย ไม่ต้องมีบอร์ดหรือเซ็นเซอร์:

```bash
cd test && make
```

ชุดทดสอบจะป้อน SHTP packet สังเคราะห์เข้าไปในไดรเวอร์ผ่าน mock I2C
แล้วตรวจว่าค่าที่ decode ออกมาตรงกับตัวเลขในเอกสารของ CEVA — **98 assertions**
ครอบคลุมตั้งแต่ Q-point scaling, การถอด quaternion เป็น Euler,
การเข้ารหัสคำสั่ง Set Feature / Tare / Calibration ไปจนถึงการอ่าน FRS
และตัวอย่าง UART-RVC ที่ยกมาจาก datasheet โดยตรง

ดูรายละเอียดใน [`test/README.md`](test/README.md)

## License

MIT — ดู [LICENSE](LICENSE)

Assembled by **Massmore** · [www.massmore.shop](https://www.massmore.shop)
