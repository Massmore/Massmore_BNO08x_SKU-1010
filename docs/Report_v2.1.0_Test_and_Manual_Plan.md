# Massmore_BNO08x v2.1.0 / v2.1.1 — Hardware Test Report & User Manual Plan

| รายการ | ค่า |
|---|---|
| ผลิตภัณฑ์ | Massmore Halley V2 — BNO085 / BNO086 9-DOF IMU (SKU-1010) |
| ไลบรารี | `Massmore_BNO08x` **v2.1.0** (รอบแรก) และ **v2.1.1** (หัวข้อ 12) by Massmore |
| ช่วงทดสอบ | 2026-09-21 ถึง 2026-09-22 (v2.1.0) · 2026-09-27 (v2.1.1 — หัวข้อ 12) |
| ชิปที่ใช้ทดสอบ | **BNO086** — SH-2 part `10004563`, FW `3.12.6` build `62` |
| วัตถุประสงค์ | (1) ทดสอบไลบรารีกับฮาร์ดแวร์จริงทุกโหมด (2) แก้ bug ที่พบ (3) เป็นวัตถุดิบสำหรับเขียนคู่มือผู้ใช้ฉบับละเอียด |

> เอกสารนี้เป็น **รายงานผลทดสอบ + แผนคู่มือ** — ทุกผลลัพธ์ในเอกสารเป็น output จริงที่ copy จาก Serial Monitor
> ส่วนที่ "ยังไม่ได้ทดสอบกับฮาร์ดแวร์" ระบุไว้ชัดเจนในหัวข้อ 9

---

## สารบัญ

1. [สรุปผล](#1-สรุปผล)
2. [อุปกรณ์และสภาพแวดล้อมทดสอบ](#2-อุปกรณ์และสภาพแวดล้อมทดสอบ)
3. [พื้นฐานที่ต้องรู้ก่อนต่อสาย: Pad บนบอร์ดและการเลือกโหมด](#3-พื้นฐานที่ต้องรู้ก่อนต่อสาย-pad-บนบอร์ดและการเลือกโหมด)
4. [การตั้งค่า Arduino IDE](#4-การตั้งค่า-arduino-ide)
5. [ขั้นตอนทดสอบแต่ละโหมด (Step by Step + ผลจริง)](#5-ขั้นตอนทดสอบแต่ละโหมด-step-by-step--ผลจริง)
6. [Factory Test (08) และ Firmware สำเร็จรูป](#6-factory-test-08-และ-firmware-สำเร็จรูป)
7. [ปัญหาที่พบระหว่างทดสอบ สาเหตุ และการแก้ไข](#7-ปัญหาที่พบระหว่างทดสอบ-สาเหตุ-และการแก้ไข)
8. [สิ่งที่เปลี่ยนใน v2.1.0](#8-สิ่งที่เปลี่ยนใน-v210)
9. [สิ่งที่ยังไม่ได้ทดสอบ / ข้อจำกัด](#9-สิ่งที่ยังไม่ได้ทดสอบ--ข้อจำกัด)
10. [Plan to User Manual — โครงคู่มือที่เสนอ](#10-plan-to-user-manual--โครงคู่มือที่เสนอ)
11. [ภาคผนวก](#11-ภาคผนวก)
12. [รอบทดสอบเพิ่มเติม 2026-09-27 (v2.1.1)](#12-รอบทดสอบเพิ่มเติม-2026-09-27-v211)

---

## 1. สรุปผล

### 1.1 ผลทดสอบรวม

| MCU | โหมด | PS1 / PS0 | ตัวอย่าง | ผล | ตัวเลขสำคัญ |
|---|---|---|---|---|---|
| ESP32 DevKit | I2C | LOW / LOW | 01, 02, 03, 04 | **PASS** | RV 100 Hz (03), loop ≈ 36,000 รอบ/s |
| ESP32 DevKit | I2C | LOW / LOW | 09_Factory_Test | **PASS** | `#VERDICT PASS`, `#CHIP BNO086` |
| ESP32 DevKit | SPI | HIGH / HIGH | 06_SPI_Advance | **PASS** | RV 395–400 Hz (ตั้ง 400 Hz) |
| ESP32 DevKit | UART-SHTP 3 Mbaud | HIGH / LOW | 08_UART_Mode | **PASS** | 469 / 469 ครั้งใน 60 s, error 0 |
| ESP32 DevKit | UART-RVC 115200 | LOW / HIGH | 07_UART_RVC | **PASS** | 30 s ไม่มี frame หาย, checksum error 0 |
| ESP32-S3 MOMO | I2C | LOW / LOW | 01, 08 | **PASS** | 600 / 600 sample ไม่มีค่าเพี้ยน |
| ทุก MCU | — | — | 8 ตัวอย่าง × 3 บอร์ด (ชุดก่อนเพิ่ม `05_I2C_Euler_Compass`) | **Compile PASS** | Arduino IDE (ESP32 Core 3.3.12) 24/24 + PlatformIO (Core 3.3.11) 24/24, warning จากไลบรารี = 0 |

### 1.2 Bug ที่พบและแก้ในรอบนี้ (รายละเอียดหัวข้อ 7)

| # | โหมด | อาการที่ผู้ใช้เห็น | สถานะ |
|---|---|---|---|
| B1 | I2C | heading / roll / pitch กระโดดเป็นบางครั้ง (~1 ใน 87 ครั้ง) | แก้แล้ว — ทดสอบ 600 ครั้ง ผิด 0 |
| B2 | I2C | `begin()` หาชิปไม่เจอหลัง MCU reset ทั้งที่ต่อถูก | แก้แล้ว (probe ซ้ำ) |
| B3 | I2C | หลังอัปโหลดโปรแกรมใหม่ bus ค้าง SDA/SCL = LOW, `begin()` ค้าง ~20 s | แก้แล้ว (ใช้ RST + แจ้ง error ทันที) |
| B4 | I2C | FW_VERSION บางรอบเป็น 1.10.10 บางรอบเป็น 3.12.6 | แก้แล้ว |
| B5 | UART-SHTP | ต่อขา INT แล้ว `beginUART()` ล้มเหลวทุกครั้ง | แก้แล้ว |
| B6 | UART-SHTP | `readAll()` timeout ~50% เมื่อมี `delay()` ใน loop | แก้แล้ว — 469/469 |
| B7 | SPI | ต่อ P0 เข้า 3V3 ตามคอมเมนต์เดิม → `No device found` | แก้เอกสาร + บังคับขา WAKE |
| B8 | UART-RVC | ตัวอย่างเดิมใช้ TX = GPIO17 ชนกับ RST และไม่ reset ชิปหลังเปลี่ยนโหมด | แก้ตัวอย่างแล้ว |

### 1.3 สิ่งที่ผู้เขียนคู่มือต้องเน้น (ข้อสรุปจากการทดสอบ)

1. **ขา `RST` ควรต่อเสมอ** ถ้าทำได้ — แก้ปัญหา bus ค้างหลังอัปโหลด และทำให้การเปลี่ยนโหมด P0/P1 ไม่ต้องถอดไฟ
2. **โหมดถูกอ่านตอน reset เท่านั้น** — เปลี่ยน P0/P1 แล้วต้อง reset ชิป (ขา RST หรือถอดไฟเซ็นเซอร์) ไม่ใช่แค่กด reset ที่ MCU
3. **SPI ต้องต่อ P0 เข้า GPIO (WAKE)** ห้ามต่อ 3V3 ตรง ๆ
4. **UART-SHTP ใช้ 3,000,000 baud** และต้องขยาย RX buffer ก่อน `begin()` — Arduino Nano ใช้โหมดนี้ไม่ได้
5. **ESP32-S3 MOMO** ใช้ Serial ผ่าน CH343P → ตั้ง **USB CDC On Boot = Disabled**

---

## 2. อุปกรณ์และสภาพแวดล้อมทดสอบ

### 2.1 ฮาร์ดแวร์

| อุปกรณ์ | รายละเอียด | Port บน Mac |
|---|---|---|
| เซ็นเซอร์ | Massmore Halley V2 (SKU-1010) — ชิป BNO086 | — |
| MCU #1 | **MOMO by Massmore** — ESP32-S3 + USB-UART CH343P | `/dev/cu.usbmodem5B3D0336211` |
| MCU #2 | **ESP32 DevKit (Classic)** — USB-UART | `/dev/cu.usbserial-130` |
| ไฟเลี้ยงเซ็นเซอร์ | 3V3 จากบอร์ด MCU → pad `3Vo` | — |

### 2.2 ซอฟต์แวร์

| รายการ | เวอร์ชัน | หมายเหตุ |
|---|---|---|
| Arduino IDE | 2.x (arduino-cli ที่มากับ IDE) | ใช้ build / upload ทั้งหมดในรอบนี้ |
| Arduino-ESP32 core | **2.0.17** (ทดสอบบอร์ดจริง) → **3.3.12** (compile, 2026-09-22) | ทดสอบฮาร์ดแวร์ทั้งหมดบน 2.0.17 · หลังอัปเดต core 3.x compile ใหม่ทั้งหมดผ่าน ยังไม่ได้รันบนบอร์ด |
| Arduino AVR core | 1.8.8 | compile Nano เท่านั้น |
| esptool | 4.5.1 (มากับ ESP32 core) | merge / flash firmware |
| PlatformIO Core | 6.2.0 + pioarduino 55.03.311 (Core 3.3.11) | ต้องรันด้วย `DYLD_FALLBACK_LIBRARY_PATH=/usr/lib` บนเครื่องทดสอบ (Python ของ PlatformIO หา `liblzma` ของ Homebrew ไม่เจอ) — ไม่เกี่ยวกับไลบรารี |

### 2.3 ค่า Serial

- Serial Monitor ของทุกตัวอย่าง: **115200 baud**, line ending ใดก็ได้
- ต้อง **ปิด Serial Monitor ก่อน Upload** (ถ้าเปิดค้าง upload / โปรแกรมอื่นจะเปิดพอร์ตไม่ได้ — `Resource busy`)

---

## 3. พื้นฐานที่ต้องรู้ก่อนต่อสาย: Pad บนบอร์ดและการเลือกโหมด

### 3.1 Pad บนบอร์ด Halley V2 และหน้าที่ในแต่ละโหมด

| Pad บนบอร์ด | I2C | SPI | UART-SHTP | UART-RVC |
|---|---|---|---|---|
| `3Vo` | ไฟ 3.3 V | ไฟ 3.3 V | ไฟ 3.3 V | ไฟ 3.3 V |
| `GND` | GND | GND | GND | GND |
| `SDA` | SDA | **MISO** (ชิป → MCU) | **TX** ของชิป (→ RX ของ MCU) | **TX** ของชิป (→ RX ของ MCU) |
| `SCL` | SCL | **SCK** | **RX** ของชิป (← TX ของ MCU) | ไม่ใช้ |
| `DI` | เลือก address (ลอย = 0x4A) | **MOSI** (MCU → ชิป) | ไม่ใช้ | ไม่ใช้ |
| `CS` | ไม่ใช้ | **CS** | ไม่ใช้ | ไม่ใช้ |
| `INT` | ไม่บังคับ | **จำเป็น** | ไม่บังคับ | ไม่ใช้ |
| `RST` | แนะนำ | **จำเป็น** | แนะนำ | แนะนำ |
| `P0` (PS0) | LOW | **WAKE → GPIO** (HIGH ตอน reset) | LOW | HIGH |
| `P1` (PS1) | LOW | HIGH | HIGH | LOW |
| `BT` | ปล่อยลอย | ปล่อยลอย | ปล่อยลอย | ปล่อยลอย |

> `BT` ห้ามดึง LOW ตอน reset — ชิปจะเข้า bootloader (address 0x28/0x29)

### 3.2 ตารางเลือกโหมด (อ่านตอน reset เท่านั้น)

| PS1 (`P1`) | PS0 (`P0`) | โหมด | ความเร็ว | ตัวอย่างที่ใช้ |
|---|---|---|---|---|
| LOW | LOW | **I2C** (ค่าเริ่มต้นจากโรงงาน) | 100 kHz (แนะนำ) | 01, 02, 03, 04, 08 |
| HIGH | HIGH | **SPI** Mode 3 | ≤ 3 MHz | 05 |
| HIGH | LOW | **UART-SHTP** | 3,000,000 baud | 07 |
| LOW | HIGH | **UART-RVC** | 115200 baud, 100 Hz | 06 |

**ขั้นตอนเปลี่ยนโหมดที่ถูกต้อง (ยืนยันจากการทดสอบ):**

1. ถอดไฟเซ็นเซอร์ หรือเตรียมขา RST ต่อกับ MCU
2. ต่อ `P0` / `P1` ตามตาราง
3. **reset ชิป** — ต่อ RST แล้วตัวอย่างจะ reset ให้เองตอน `begin()` (ทุกตัวอย่างทำให้แล้วเมื่อตั้งค่าขา RST) หรือถอดไฟเซ็นเซอร์แล้วเสียบใหม่
4. อัปโหลดตัวอย่างของโหมดนั้น

> กด reset ที่ MCU **ไม่ได้** reset เซ็นเซอร์ — ชิปจะยังอยู่ในโหมดเดิม

---

## 4. การตั้งค่า Arduino IDE

### 4.1 ติดตั้งไลบรารี

1. Arduino IDE → **Sketch → Include Library → Add .ZIP Library…**
2. เลือก `ArduinoIDE/Massmore_BNO08x.zip`
3. ตรวจ: **File → Examples → Massmore_BNO08x** ต้องเห็น 9 ตัวอย่าง `01_BasicRead` … `09_Factory_Test`
4. ถ้าเคยติดตั้ง v2.0.0 ไว้ ให้ลบโฟลเดอร์ `Documents/Arduino/libraries/Massmore_BNO08x` เดิมก่อน

### 4.2 Board settings ที่ใช้ทดสอบ

| รายการ | ESP32 DevKit | ESP32-S3 MOMO |
|---|---|---|
| Board | ESP32 Dev Module | ESP32S3 Dev Module |
| Upload Speed | **460800** (921600 ขึ้น `Invalid head of packet`) | 921600 |
| USB CDC On Boot | — | **Disabled** (Serial ออกทาง CH343P / UART0) |
| Flash Size | 4MB | ค่าเริ่มต้น |
| Partition | Default | Default |

> MOMO: ถ้าตั้ง USB CDC On Boot = Enabled, Serial Monitor จะเงียบ เพราะ `Serial` ย้ายไปที่ USB native แทน CH343P

---

## 5. ขั้นตอนทดสอบแต่ละโหมด (Step by Step + ผลจริง)

### 5.1 I2C บน ESP32-S3 (MOMO) — ตัวอย่าง 01 และ 09

**Wiring**

| Halley V2 | MOMO (ESP32-S3) |
|---|---|
| `3Vo` | 3V3 |
| `GND` | GND |
| `SDA` | GPIO 14 |
| `SCL` | GPIO 15 |
| `P0`, `P1`, `DI`, `BT`, `INT`, `RST` | ไม่ต่อ |

**Step by step**

1. ต่อสายตามตาราง (Qwiic ก็ได้ถ้าบอร์ดมี)
2. เปิด `File → Examples → Massmore_BNO08x → 01_BasicRead`
3. ตัวอย่างเลือกขาให้เองตาม MCU (`CONFIG_IDF_TARGET_ESP32S3` → SDA 14 / SCL 15) ไม่ต้องแก้
4. เลือก Board = ESP32S3 Dev Module, USB CDC On Boot = Disabled, Port = MOMO
5. Upload → เปิด Serial Monitor 115200

**ผลจริง (01_BasicRead)**

```text
Massmore_BNO08x - 01_BasicRead
BNO08x connected at 0x4A
heading=50.8  roll=-4.5  pitch=0.8  yaw=50.8  | accel=-0.14 -0.79 9.64  gyro=0.00 0.00 0.00  mag=-60.3 46.1 -35.7  [RV acc: Unreliable]
heading=50.8  roll=-4.5  pitch=0.8  yaw=50.8  | accel=-0.14 -0.79 9.68  gyro=0.00 0.00 0.00  mag=-60.3 46.8 -35.8  [RV acc: Unreliable]
```

**เกณฑ์ผ่าน:** `connected at 0x4A`, accel.z ≈ 9.6–9.8 เมื่อวางราบ, gyro ≈ 0 เมื่อนิ่ง, roll/pitch สอดคล้องกับ accel
(`RV acc: Unreliable` เป็นปกติก่อน calibrate magnetometer — ดูตัวอย่าง 04)

**ปัญหาที่พบในขั้นนี้:** รอบแรก SDA/SCL อ่านได้ LOW ค้าง → `No device found` แก้โดยถอดไฟเซ็นเซอร์เสียบใหม่ (สาเหตุ: หัวข้อ 7, B3)

**ผลจริง (09_Factory_Test บน MOMO)**

```text
#MASSMORE_FACTORY_TEST v1.0
#PRODUCT Massmore_BNO08x
#MCU ESP32-S3
#RESULT BUS_SCAN PASS 0x4A
#RESULT CHIP_ID PASS 10004563
#CHIP BNO086
#RESULT FW_VERSION PASS 3.12.6
#RESULT SERIAL PASS NONE
#RESULT METADATA PASS Q14/12
#RESULT AUTHENTICITY PASS GENUINE
#RESULT RANGE_ACCEL PASS 9.68
#RESULT RANGE_GYRO PASS 0.000
#RESULT RANGE_MAG PASS 93.4
#RESULT RANGE_QUAT PASS 1.000
#RESULT CONTINUOUS PASS 20/20
#VERDICT PASS
[PASS] SENSOR QA PASSED - READY TO SHIP
```

(ใน I2C scan ของ MOMO พบอุปกรณ์อื่นที่ 0x40 และ 0x70 ด้วย — เป็นอุปกรณ์บนบอร์ด MOMO ไม่ใช่ปัญหา)

---

### 5.2 I2C บน ESP32 DevKit — ตัวอย่าง 01–04

**Wiring**

| Halley V2 | ESP32 DevKit | หมายเหตุ |
|---|---|---|
| `3Vo` | 3V3 | |
| `GND` | GND | |
| `SDA` | GPIO 21 | |
| `SCL` | GPIO 22 | |
| `INT` | GPIO 4 | ใช้ใน 02, 03 (01/04 ใช้ polling) |
| `RST` | GPIO 17 | แนะนำ — ทุกตัวอย่าง reset ชิปให้ตอน `begin()` |
| `P0`, `P1` | ไม่ต่อ (LOW) | |

**ขาที่ตัวอย่างใช้บน ESP32 (กำหนดไว้ต้นไฟล์ .ino)**

| ตัวอย่าง | Bus | SDA / SCL | INT | RST |
|---|---|---|---|---|
| 01_BasicRead | `Wire` | 21 / 22 | -1 (polling) | 17 |
| 02_CustomPins_BusRemap | `Wire1` | 21 / 22 | 4 | 17 |
| 03_NonBlocking_Multitask | `Wire` | 21 / 22 | 4 | 17 |
| 04_Calibration_Tare | `Wire` | 21 / 22 | -1 | 17 |

ไม่ได้ต่อ RST: แก้ `#define RST_PIN 17` เป็น `-1` (ถ้า bus ค้างต้องถอดไฟเซ็นเซอร์เอง)

**Step by step**

1. ต่อสายตามตาราง
2. Board = ESP32 Dev Module, **Upload Speed = 460800**
3. เปิดตัวอย่าง → Upload → Serial Monitor 115200

**ผลจริง**

01_BasicRead:
```text
Massmore_BNO08x - 01_BasicRead
BNO08x connected at 0x4A
heading=359.9  roll=-1.8  pitch=1.7  yaw=-0.1  | accel=-0.29 -0.30 9.58  gyro=0.00 0.00 0.00  mag=36.2 -21.6 -32.7  [RV acc: Unreliable]
```

02_CustomPins_BusRemap:
```text
Massmore_BNO08x - 02_CustomPins_BusRemap
Wire1 on SDA=21 SCL=22
BNO08x connected at 0x4A
roll=-1.8  pitch=1.7  yaw=-0.1
```

03_NonBlocking_Multitask (บรรทัด `[stats]` คือหัวใจของตัวอย่างนี้):
```text
Massmore_BNO08x - 03_NonBlocking_Multitask
Rotation Vector + Accelerometer @ 100 Hz. loop() never blocks.
yaw=0.0 pitch=0.0 roll=0.0 | az=0.00          <- บรรทัดแรกเป็น 0 เพราะยังไม่มี report — ปกติ
yaw=-0.1 pitch=1.9 roll=-1.5 | az=9.57
[stats] loop/s=20427  RV reports/s=38         <- วินาทีแรกไม่เต็ม — ปกติ
[stats] loop/s=36811  RV reports/s=100
[stats] loop/s=36312  RV reports/s=100
[stats] loop/s=36296  RV reports/s=101
```

**เกณฑ์ผ่าน 03:** `RV reports/s` ≈ 100 และ `loop/s` เป็นหลักหมื่น = `loop()` ไม่ถูก block

04_Calibration_Tare (พิมพ์ทุก 500 ms):
```text
roll=0.0 pitch=0.0 yaw=0.0 | mag acc: Unreliable  RV acc: Unreliable  heading err=0.0 deg
```
คำสั่ง (พิมพ์ใน Serial Monitor ทีละตัวอักษร): `c` calibrate ทั้งหมด, `m` เฉพาะ mag, `e` ปิด, `s` Save DCD, `x` ลบ calibration, `?` สถานะ, `z` tare heading, `a` tare ทุกแกน, `p` persist tare, `t` clear tare, `h` help

**ทดสอบความทนทาน:** อัปโหลดตัวอย่าง 04 → 01 → 03 → 02 → 09 ทับกันขณะชิปกำลังส่งข้อมูล 2 รอบ — **ขึ้นทุกครั้ง** (ก่อนแก้ B3 ตัวอย่าง 04 ค้างทุกครั้ง)

---

### 5.3 SPI บน ESP32 DevKit — ตัวอย่าง 06

**Wiring**

| Halley V2 | ESP32 DevKit | หมายเหตุ |
|---|---|---|
| `3Vo` | 3V3 | |
| `GND` | GND | |
| `SCL` (SCK) | GPIO 18 | |
| `SDA` (MISO) | GPIO 19 | |
| `DI` (MOSI) | GPIO 23 | |
| `CS` | GPIO 5 | |
| `INT` | GPIO 4 | **จำเป็น** |
| `RST` | GPIO 17 | **จำเป็น** |
| `P0` (WAKE) | **GPIO 16** | **จำเป็น — ห้ามต่อ 3V3** |
| `P1` | 3V3 | |

**Step by step**

1. ต่อสายตามตาราง — จุดที่พลาดบ่อยคือ `P0`: ต้องต่อเข้า GPIO 16 ไม่ใช่ 3V3
2. เปิด `06_SPI_Advance` → Upload → Serial Monitor 115200
3. driver ดึง GPIO 16 = HIGH ระหว่าง reset (ชิปจึงเข้าโหมด SPI) แล้วใช้ขาเดียวกันเป็น WAKE ต่อ

**ผลจริง**

```text
Massmore_BNO08x - 06_SPI_Advance
Connected. Firmware 3.12.6  part 10004563
rpy -1.1 1.9 -0.0  gyro(dps) 0.0 0.0 0.0
[rate] Rotation Vector = 130 Hz               <- วินาทีแรกไม่เต็ม — ปกติ
[rate] Rotation Vector = 400 Hz
[rate] Rotation Vector = 397 Hz
[rate] Rotation Vector = 399 Hz
```

อัตราที่วัดได้ 15 วินาที: `400 397 397 400 398 400 399 400 399 399 400 398 395 397` Hz — reset ซ้ำ 2 รอบก็ขึ้นทุกครั้ง

**ปัญหาที่พบในขั้นนี้:** ต่อ `P0` เข้า 3V3 ตามคอมเมนต์เดิมของตัวอย่าง → `BNO08x not found on SPI: No device found` (หัวข้อ 7, B7)

---

### 5.4 UART-SHTP 3 Mbaud บน ESP32 DevKit — ตัวอย่าง 08 (ใหม่ใน v2.1.0)

**Wiring**

| Halley V2 | ESP32 DevKit | หมายเหตุ |
|---|---|---|
| `3Vo` | 3V3 | |
| `GND` | GND | |
| `SDA` (TX ของชิป) | GPIO 21 = **RX** ของ ESP32 | |
| `SCL` (RX ของชิป) | GPIO 22 = **TX** ของ ESP32 | |
| `INT` | GPIO 4 | ไม่บังคับ |
| `RST` | GPIO 17 | แนะนำ |
| `P1` | **3V3 (HIGH)** | |
| `P0` | ไม่ต่อ / GND (LOW) | |

**จุดสำคัญ**

- ความเร็ว **3,000,000 baud** 8N1 ตาม datasheet — ค่าอื่นใช้ไม่ได้
- sketch ต้องเรียก `Serial2.setRxBufferSize(1024)` **ก่อน** `Serial2.begin()` (advertisement ตอน reset ยาว ~290 byte)
- ใช้ API เดียวกับ I2C ทุกอย่าง แค่เปลี่ยน `begin()` → `beginUART(Serial2, INT, RST)`
- Arduino Nano ใช้โหมดนี้ไม่ได้ (UART ทำ 3 Mbaud ไม่ได้) — ตัวอย่างจะพิมพ์แจ้งให้ใช้ 06 แทน

**ผลจริง**

```text
Massmore_BNO08x - 08_UART_Mode
UART OK - BNO086  FW 3.12.6  part 10004563
heading=360.0  roll=-1.1  pitch=1.3  | accel=-0.20 -0.17 9.63  gyro=-0.00 0.00 0.00  mag=42.2 2.6 -31.9  [RV acc: Unreliable]
heading=360.0  roll=-1.1  pitch=1.3  | accel=-0.20 -0.18 9.63  gyro=0.00 0.00 0.00  mag=43.3 2.6 -31.9  [RV acc: Unreliable]
```

- 60 วินาที: **469 บรรทัดสำเร็จ / 0 ล้มเหลว**
- ทดสอบ FSM API 20 วินาที (RV + accel 100 Hz): `RV/s = 100` ทุกวินาที, `ACC/s ≈ 127`, sample ผิดปกติ 0
- ยืนยันที่ระดับ byte: ชิปส่ง frame `7E 01 1C 01 …` (SHTP over UART) ออกทาง pad `SDA`

---

### 5.5 UART-RVC 115200 บน ESP32 DevKit — ตัวอย่าง 07

**Wiring**

| Halley V2 | ESP32 DevKit | หมายเหตุ |
|---|---|---|
| `3Vo` | 3V3 | |
| `GND` | GND | |
| `SDA` (TX ของชิป) | GPIO 21 (RX) | สายเส้นเดียวที่จำเป็น |
| `SCL` | GPIO 22 | ไม่ใช้ในโหมดนี้ ต่อทิ้งไว้ได้ |
| `RST` | GPIO 17 | แนะนำ — ตัวอย่าง reset ชิปให้เข้าโหมด RVC |
| `P1` | LOW (ไม่ต่อ / GND) | |
| `P0` | **3V3 (HIGH)** | |

**Step by step**

1. สลับ `P0` เป็น HIGH, `P1` เป็น LOW
2. เปิด `07_UART_RVC` → Upload → Serial Monitor 115200
3. ตัวอย่างจะ pulse RST (GPIO 17) หนึ่งครั้ง → ชิปเริ่ม stream 100 Hz เองโดยไม่ต้องส่งคำสั่ง

**ผลจริง**

```text
Massmore_BNO08x - 07_UART_RVC
yaw	pitch	roll	ax	ay	az
0.00	-0.93	1.53	-0.24	-0.16	9.68
0.00	-0.92	1.52	-0.22	-0.14	9.65
0.06	-0.84	1.39	-0.24	-0.13	9.63
```

- 30 วินาที: 295 บรรทัด (พิมพ์ 10 Hz จาก frame 100 Hz), **ไม่มีข้อความ `(dropped …, bad csum …)`** = frame ครบและ checksum ถูกทุก frame
- โหมดนี้ได้แค่ yaw/pitch/roll + accel — ไม่มี quaternion, calibration, tare

---

## 6. Factory Test (08) และ Firmware สำเร็จรูป

### 6.1 การใช้งาน

- ต่อแค่ `3Vo`, `GND`, `SDA`→21, `SCL`→22 (ESP32) — **INT/RST ไม่ต้องต่อ** (firmware สำเร็จรูปตั้ง `FT_INT_PIN` / `FT_RST_PIN` = -1)
- วางบอร์ดนิ่งบนโต๊ะ, ห่างแม่เหล็ก
- เปิด Serial 115200 → ทดสอบอัตโนมัติหลังบูต · พิมพ์ `r` + Enter เพื่อทดสอบซ้ำ

### 6.2 Flash firmware สำเร็จรูป (ESP32 DevKit)

```bash
esptool.py --chip esp32 --port /dev/cu.usbserial-130 --baud 460800 \
  write_flash -z 0x0 firmware/bin/Massmore_BNO08x_FactoryTest_ESP32.bin
```

ผลจริงหลัง flash ไฟล์ merged:

```text
Massmore_BNO08x - 09_Factory_Test (keep the board still)

#MASSMORE_FACTORY_TEST v1.0
#PRODUCT Massmore_BNO08x
#MCU ESP32
#RESULT BUS_SCAN PASS 0x4A
#RESULT CHIP_ID PASS 10004563
  part 10004563 v3.12.6 build 62
#CHIP BNO086
#RESULT FW_VERSION PASS 3.12.6
#RESULT SERIAL PASS NONE
#RESULT METADATA PASS Q14/12
#RESULT AUTHENTICITY PASS GENUINE
  OK - genuine BNO08x factory firmware
#RESULT RANGE_ACCEL PASS 9.61
#RESULT RANGE_GYRO PASS 0.000
#RESULT RANGE_MAG PASS 48.9
#RESULT RANGE_QUAT PASS 1.000
#RESULT CONTINUOUS PASS 20/20
  |a| noise (max-min) = 0.042

#VERDICT PASS
[PASS] SENSOR QA PASSED - READY TO SHIP
Type 'r' + Enter to run the test again.
```

### 6.3 ความหมายของแต่ละบรรทัดและเกณฑ์

| บรรทัด | ตรวจอะไร | เกณฑ์ PASS | FAIL reason |
|---|---|---|---|
| `BUS_SCAN` | scan I2C 0x08–0x77 หา 0x4A / 0x4B | พบ | `NO_DEVICE_ON_I2C`, `BOOTLOADER_MODE_BT_PIN_LOW` (พบ 0x28/0x29) |
| `CHIP_ID` | SH-2 Product ID (report 0xF8) part number | 10003606 / 10004095 / 10004563 | `NO_PRODUCT_ID_RESPONSE`, `CHIP_ID_MISMATCH` |
| `#CHIP` | รุ่นชิป BNO085 / BNO086 / BNO08x | **ข้อมูลเท่านั้น** ไม่มีผลต่อ PASS/FAIL | — |
| `FW_VERSION` | version major 1–9, build ≠ 0 | สมเหตุสมผล | `FW_VERSION_IMPLAUSIBLE` |
| `SERIAL` | serial จาก FRS 0x4B4B | อ่านได้ หรือ `NONE` (ไม่ได้ program จากโรงงาน) | `SERIAL_READ_TIMEOUT` |
| `METADATA` | RV metadata Q point = 14/12 | ตรง | `MOTIONENGINE_METADATA` |
| `AUTHENTICITY` | รวมผลด้านบน | GENUINE | `AUTHENTICITY_SUSPECT` |
| `RANGE_ACCEL` | \|a\| เมื่อวางนิ่ง | 9.81 ± 1.5 m/s² | `ACCEL_OUT_OF_RANGE` |
| `RANGE_GYRO` | \|ω\| เมื่อวางนิ่ง | ≤ 0.5 rad/s | `GYRO_OUT_OF_RANGE` |
| `RANGE_MAG` | \|B\| | 5–200 µT | `MAG_OUT_OF_RANGE` |
| `RANGE_QUAT` | \|q\| | 1 ± 0.02 | `QUATERNION_NOT_UNIT` |
| `CONTINUOUS` | อ่านต่อเนื่อง 20 sample | 20/20, noise \|a\| ≤ 1.0 | `NO_SENSOR_REPORTS` |

> Parser ของ Web Serial Monitor ต้อง **ข้ามบรรทัด `#` ที่ไม่รู้จัก** ได้ (บรรทัด `#CHIP` เพิ่มใน v2.1.0)

---

## 7. ปัญหาที่พบระหว่างทดสอบ สาเหตุ และการแก้ไข

เรียงตามลำดับที่พบ — แต่ละข้อมีหลักฐานจากการวัดจริง เหมาะใช้เขียนบท Troubleshooting

### B1 — I2C: ค่ามุมกระโดดเป็นบางครั้ง

- **อาการ:** บน MOMO heading ส่วนใหญ่ 50.8° แต่บางบรรทัดเป็น 1.0° และ roll/pitch ไม่ตรงกับ accel (~1 ใน 87 ครั้ง)
- **หลักฐาน:** quaternion ที่ผิด `i/j/k` ถูก แต่ `real = 0.0121` และ `accuracy = 0` (ปกติ real ≈ 0.90)
- **สาเหตุ:** packet ที่ยาวเกินหนึ่ง I2C chunk (เช่น 195 byte) ต้องอ่านหลายรอบ บางครั้งชิปยังไม่พร้อมและตอบ header ว่าง `0000` → driver เดิมอ่าน byte ศูนย์เป็นข้อมูล ท้าย quaternion จึงกลายเป็น 0
- **แก้:** ตรวจ header ของทุก chunk — header ว่างให้รอแล้วอ่านใหม่ (สูงสุด 10 ครั้ง), ทิ้ง continuation ที่เหลือจาก packet ที่ล้มเหลว, ไม่อ่านเกินความยาวจริง
- **ผลหลังแก้:** 600 / 600 sample ไม่มีค่าเพี้ยน (min/max ของ yaw, pitch, roll, q.real คงที่)

### B2 — I2C: `begin()` หาชิปไม่เจอครั้งแรก

- **อาการ:** `[massmore] no ACK at 0x4A` ทั้งที่ bus ปกติ (SDA=1, SCL=1)
- **สาเหตุ:** BNO08x อาจ NACK ครั้งแรกขณะหลับ/ยุ่ง แต่ driver เดิม probe ครั้งเดียว
- **แก้:** probe ซ้ำสูงสุด 10 ครั้ง ห่าง 10 ms

### B3 — I2C: bus ค้างหลังอัปโหลดโปรแกรม (สำคัญที่สุดสำหรับคู่มือ)

- **อาการ:** หลังอัปโหลด / reset MCU → `No device found` หรือโปรแกรมค้างหลังพิมพ์ชื่อตัวอย่าง; วัดขาได้ **SDA = 0 และ SCL = 0** แม้เปิด pull-up ภายใน
- **หลักฐาน:** probe ได้ `rc=5` (timeout) ครั้งละ ~1 s; ถอดไฟเซ็นเซอร์แล้วหาย; pulse RST แล้วหาย
- **สาเหตุ:** MCU ถูก reset กลางการอ่าน I2C ขณะชิปกำลัง stream → BNO08x กด SDA และ SCL ค้าง (SCL ค้างด้วย จึงแก้ด้วยการส่ง 9 clock ไม่ได้)
- **แก้:**
  - ตัวอย่างบน ESP32 ส่ง `RST_PIN = 17` ให้ `begin()` → driver reset ชิปก่อนใช้ทุกครั้ง
  - driver ตรวจ timeout (`rc=5`) แล้วคืน `ERR_IO` + ข้อความ *"Bus I/O error (I2C held low? use RST pin or power-cycle)"* ทันที แทนการค้าง ~20 s
- **ผลหลังแก้:** อัปโหลดทับกัน 5 ตัวอย่าง × 2 รอบ ขึ้นทุกครั้ง

### B4 — FW_VERSION ไม่คงที่

- **อาการ:** Factory Test บางรอบ `FW_VERSION 1.10.10` บางรอบ `3.12.6`
- **สาเหตุ:** หลัง hardware reset ชิปตอบ Product ID 2 รายการ (`10004563 v3.12.6 build 62` และ `10003606 v1.10.10 build 404`) driver เดิมเก็บรายการหลังสุดเป็นค่าหลัก
- **แก้:** เก็บ firmware image ที่รู้จักตัว **แรก** เป็นค่าหลัก → ได้ 3.12.6 ทุกรอบ

### B5 — UART-SHTP: ต่อ INT แล้วใช้ไม่ได้

- **อาการ:** `beginUART=0 (No device found)` ทั้งที่ sniff ได้ frame ครบ
- **สาเหตุ:** `update()` รอ INT = LOW ก่อนอ่าน แต่ใน UART ข้อมูลอยู่ใน RX buffer ของ MCU แล้ว และ INT ไม่ได้บอกสถานะนั้น
- **แก้:** โหมด UART ไม่รอ INT → ต่อ/ไม่ต่อ INT ใช้ได้ทั้งคู่

### B6 — UART-SHTP: `readAll()` timeout เมื่อมี `delay()`

- **อาการ:** ตัวอย่าง 08 (UART-SHTP) รอบแรก 25 สำเร็จ / 47 `read failed: Timeout`
- **หลักฐาน:** ก่อนเรียก `readAll()` RX buffer มีข้อมูลค้าง 1019–1024 byte (เต็ม); ขยาย buffer เป็น 4096 ก็ยังเต็มและยัง fail ~50%
- **สาเหตุ:** ระหว่าง `delay(100)` ชิป stream ต่อจน buffer ล้น → frame ขาด → parser ตามไม่ทัน
- **แก้:** Blocking API (`readAll`, `readEulerDeg`, `readHeadingDeg`) ทิ้งข้อมูลเก่าใน RX buffer ก่อนรอค่าใหม่ → 469 / 469

### B7 — SPI: ต่อ P0 เข้า 3V3 แล้วส่งคำสั่งไม่ได้

- **อาการ:** `BNO08x not found on SPI: No device found` ทั้งที่อ่าน advertisement ได้ถูกต้อง
- **หลักฐาน (ทดลองคุม bus เอง):**
  - ส่งคำสั่งตอน INT = HIGH → ชิปตอบ `FFFF` (ไม่ฟัง) → ไม่มี response
  - รอ 1 s → ชิปไม่ยก INT เองเลยเมื่อว่าง
  - ส่งคำสั่งตอน INT = LOW → ได้ Product ID (`F8`) กลับมา
- **สาเหตุ:** SPI ชิปรับคำสั่งเฉพาะตอนถูกปลุกด้วย WAKE (`P0` = LOW) — ต่อ 3V3 ตรง ๆ จึงปลุกไม่ได้
- **แก้:** แก้คอมเมนต์ตัวอย่าง 06 (SPI), `beginSPI()` คืน `ERR_BAD_PARAM` ทันทีถ้าไม่ได้ส่ง `wakePin` → ต่อ `P0` เข้า GPIO 16 แล้วได้ ~400 Hz

### B8 — UART-RVC: ตัวอย่างเดิมไม่ตรงกับการต่อสาย

- **อาการ/ความเสี่ยง:** ตัวอย่างเดิมใช้ RX = 16, TX = 17 (ชน RST = 17) และไม่ reset ชิป → ชิปค้างในโหมดเดิมหลังสลับ P0/P1
- **แก้:** ESP32 ใช้ RX 21 / TX 22, S3 ใช้ 14 / 15, pulse RST 17 ตอนเริ่ม

### ปัญหาด้านเครื่องมือ (ไม่ใช่ไลบรารี) — ควรอยู่ใน FAQ

| อาการ | สาเหตุ | แก้ |
|---|---|---|
| Upload ESP32: `Invalid head of packet (0xE0)` | USB-UART ไม่นิ่งที่ 921600 | Upload Speed 460800 |
| `Resource busy` / upload ไม่ได้ | Serial Monitor เปิดค้าง | ปิด Serial Monitor ก่อน upload |
| MOMO: Serial Monitor เงียบ | USB CDC On Boot = Enabled | ตั้ง Disabled |
| PlatformIO: `Python's lzma module is unavailable` | Python ของ PlatformIO ลิงก์กับ `liblzma` ของ Homebrew (`/opt/homebrew/opt/xz`) ที่ไม่มีในเครื่อง | รัน `DYLD_FALLBACK_LIBRARY_PATH=/usr/lib pio run` (ใช้ liblzma ของ macOS) หรือ `brew install xz` |

---

## 8. สิ่งที่เปลี่ยนใน v2.1.0

### 8.1 API ใหม่

| Function | คืนค่า | ใช้ทำอะไร |
|---|---|---|
| `getChipModel()` | `Massmore_BNO08x_chip_t` (`MASSMORE_BNO08X_CHIP_BNO085` / `_BNO086` / `_UNKNOWN`) | ระบุรุ่นชิปจาก part number (10004563 / 10004095 = BNO086) และ report เฉพาะ BNO086 (0x2B–0x2D) ใน advertisement |
| `chipModelToString(c)` | `"BNO085"` / `"BNO086"` / `"BNO08x"` | ข้อความสำหรับแสดงผล |

```cpp
Serial.println(Massmore_BNO08x::chipModelToString(imu.getChipModel()));   // BNO086
```

### 8.2 พฤติกรรมที่เปลี่ยน

| Function | v2.0.0 | v2.1.0 |
|---|---|---|
| `begin()` I2C | probe 1 ครั้ง, bus ค้าง = ค้าง ~20 s | probe ≤ 10 ครั้ง, bus ค้าง = `ERR_IO` ทันที |
| อ่าน I2C packet ยาว | header ว่างถูกอ่านเป็นข้อมูล | รอ/อ่านใหม่, ทิ้ง continuation ที่ไม่สมบูรณ์ |
| `getProductID()` | รายการหลังสุด | firmware ที่รู้จักตัวแรก (คงที่) |
| `update()` บน UART | รอ INT | ไม่รอ INT |
| `readAll()` / `readEulerDeg()` / `readHeadingDeg()` บน UART | อ่าน backlog เก่า | ทิ้ง backlog แล้วรอค่าใหม่ |
| `beginSPI(…, wakePin = -1)` | ยอมรับ แล้ว fail แบบ timeout | `ERR_BAD_PARAM` ทันที |
| `statusToString(ERR_IO)` | `Bus I/O error` | `Bus I/O error (I2C held low? use RST pin or power-cycle)` |

### 8.3 ตัวอย่าง

| ตัวอย่าง | การเปลี่ยนแปลง |
|---|---|
| 01, 04 | ESP32: RST = 17 · S3: SDA 14 / SCL 15 · คอมเมนต์อธิบายเหตุผลที่ควรต่อ RST |
| 02 | ESP32: Wire1 บน 21/22 (เดิม 25/26), INT 4, RST 17 · S3: Wire1 บน 14/15 |
| 03 | ESP32: INT 4, RST 17 · S3: 14/15, INT -1 |
| 05 | P0 = WAKE จำเป็น (ลบคำแนะนำ "ต่อ 3Vo") |
| 06 | ESP32 RX 21 / TX 22, S3 14/15, pulse RST ตอนเริ่ม, ตาราง PS0/PS1 ชัดเจน |
| **08_UART_Mode** | **ใหม่** — SHTP-over-UART 3 Mbaud |
| **09_Factory_Test** | ย้ายจาก 07 → 08 → 09 · พิมพ์ `#CHIP` + รายการ Product ID ทั้งหมด · S3 ใช้ 14/15 |

### 8.4 ไฟล์ในโปรเจกต์

| ไฟล์ | การเปลี่ยนแปลง |
|---|---|
| `ArduinoIDE/Massmore_BNO08x.zip` | build ใหม่ v2.1.0 (26 ไฟล์) |
| `PlatformIO/lib/Massmore_BNO08x/` | sync ตรงกับ ArduinoIDE ทุกไฟล์ · `library.json` 2.1.0 |
| `PlatformIO/src/main.cpp` | ตัววัดมุม Euler (Game RV) ตั้งแต่ v2.1.1 — ก่อนหน้านี้ = 09_Factory_Test |
| `firmware/bin/*.bin` | build ใหม่ v2.1.0 (arduino-cli + esptool merge) |
| `firmware/manifest.json` | version 2.1.0 |
| `firmware/README.md` | ผลจริง, วิธี build ด้วย arduino-cli |
| `README.md` | What's new, hardware-tested matrix, pin mapping, troubleshooting |
| cleanup | ลบ `.DS_Store` ทั้งหมด, ลบ build cache `PlatformIO/.pio` |

---

## 9. สิ่งที่ยังไม่ได้ทดสอบ / ข้อจำกัด

| รายการ | สถานะ | ความเสี่ยง / สิ่งที่ควรทำก่อนเขียนคู่มือส่วนนั้น |
|---|---|---|
| ไฟล์ `firmware/bin` v2.1.0 ล่าสุด | build ด้วย PlatformIO `esp32dev` (Core 3.3.11) **ยังไม่ได้ flash ทดสอบ** | ตัวที่ผ่าน PASS คือ build บน Core 2.0.17 — ควร flash ตรวจอีกครั้ง |
| Arduino-ESP32 Core 3.x บนบอร์ดจริง | compile ผ่านทั้งหมด (Arduino 3.3.12, PlatformIO 3.3.11) แต่ทดสอบฮาร์ดแวร์บน 2.0.17 เท่านั้น | ควรรันซ้ำอย่างน้อย 01, 05, 07, 08 บน Core 3.x — driver I2C ของ Core 3.x เปลี่ยนจาก 2.x (timeout / clock stretching) |
| ESP32-S3: ตัวอย่าง 02, 03, 04, 05, 06, 07 | compile ผ่าน ยังไม่ได้รันบนบอร์ด | ขา S3 ของ SPI (12/13/11/10, INT 4, RST 5, WAKE 6) ยังไม่ยืนยัน |
| Arduino Nano ทุกโหมด | compile เท่านั้น (Factory Test ใช้ flash 80%) | ต้องทดสอบจริง + level shifter สำหรับ INT/RST/CS/DI/P0/P1 |
| คำสั่ง calibration / tare ใน 04 (`c m e s x ? z a p t`) | ยังไม่ได้ส่งคำสั่งทดสอบ (ยืนยันเฉพาะการเริ่มต้นและการแสดงผล) | ทดสอบตามขั้นตอน CEVA 1000-4044 ก่อนเขียนบท Calibration |
| ชิป BNO085 | ไม่มีบอร์ดทดสอบ | `getChipModel()` ของ BNO085 อิงข้อมูล advertisement — ควรยืนยันกับบอร์ดจริง |
| INT pin บน I2C (02/03) ประสิทธิภาพเทียบ polling | ใช้งานได้ ยังไม่ได้วัดเปรียบเทียบ | — |
| `docs/Article_BNO08x.md`, `BNO08x_User_Guide.docx/.pdf` | **เอกสารเก่า** ใช้ API v1 (`MassmoreBNO08x`, `getLastError()`) | ควรแทนที่ด้วยคู่มือใหม่ตามแผนหัวข้อ 10 |

---

## 10. Plan to User Manual — โครงคู่มือที่เสนอ

กลุ่มผู้อ่าน: ผู้เริ่มต้น Arduino ถึงวิศวกร — เขียนแบบ "ทำตามได้ทีละขั้น" ทุกบทมี (1) สิ่งที่ต้องมี (2) การต่อสาย (3) การตั้งค่า (4) โค้ด (5) ผลที่ควรเห็น (6) ถ้าไม่ได้ผล

### บทที่ 1 — รู้จักบอร์ด Massmore Halley V2
- จุดเด่น BNO085/BNO086 (sensor fusion ในชิป, ไม่ต้องเขียน filter เอง)
- ความต่าง BNO085 vs BNO086 (BNO086 มี report 0x2B–0x2D) และวิธีดูว่าบอร์ดเป็นรุ่นไหน (`#CHIP` / `getChipModel()`)
- รูปที่ต้องมี: ด้านหน้า/หลังบอร์ด (`docs/images/halley-v2-front-back.png`), ขนาด (`halley-v2-dimensions.png`)
- ข้อมูลจากรายงานนี้: หัวข้อ 3.1

### บทที่ 2 — Pinout และการเลือกโหมด
- ตาราง pad × โหมด (หัวข้อ 3.1) และตาราง PS0/PS1 (หัวข้อ 3.2)
- กล่องเตือน: โหมดอ่านตอน reset เท่านั้น / BT ห้าม LOW / ขาอื่นนอกจาก SDA-SCL เป็น 3.3 V เท่านั้น
- รูปที่ต้องถ่ายใหม่: การต่อ P0/P1 ทั้ง 4 แบบ (ภาพ close-up jumper wire บน pad)

### บทที่ 3 — ติดตั้งซอฟต์แวร์
- Arduino IDE: ติดตั้ง ESP32 board package, Add .ZIP Library, เปิด Examples (หัวข้อ 4)
- ภาพหน้าจอที่ต้องมี: Boards Manager, Add .ZIP, เมนู Examples ที่เห็น 9 ตัวอย่าง, เมนู Tools ของ ESP32 (Upload Speed 460800) และ S3 (USB CDC On Boot Disabled)
- PlatformIO (ทางเลือก): เปิดโฟลเดอร์ `PlatformIO/`, เลือก env
- วิธีอัปเดตจาก v2.0.0 (ลบโฟลเดอร์เก่า)

### บทที่ 4 — เริ่มต้นเร็ว: I2C + 01_BasicRead
- ใช้หัวข้อ 5.1 (MOMO) และ 5.2 (ESP32) — ตาราง wiring, step by step, ผลจริง, เกณฑ์ผ่าน
- อธิบายแต่ละค่าที่พิมพ์ออกมา (heading, roll, pitch, accel, gyro, mag, RV accuracy)
- ภาพ: การต่อ Qwiic / jumper กับ ESP32 (`halley-v2-esp32-i2c-wiring.png`) + ภาพหน้าจอ Serial Monitor
- กล่อง "ถ้าขึ้น No device found" → ลิงก์ไปบท Troubleshooting (B2, B3)

### บทที่ 5 — ย้ายขาและใช้ bus ที่สอง (02)
- ESP32 / S3 GPIO Matrix, Wire1, ส่ง INT/RST
- เหตุผลที่ควรต่อ RST (B3) — ใส่ภาพ "SDA/SCL ค้าง LOW" แบบง่าย

### บทที่ 6 — Non-blocking API สำหรับงาน multitask (03)
- ลำดับ `enable*()` → `update()` → `isDataReady()` → `getReadings()`
- อ่านบรรทัด `[stats]` (หัวข้อ 5.2) — loop/s และ RV/s หมายถึงอะไร
- ตารางอัตรา report `MASSMORE_BNO08X_INTERVAL_*` (1–1000 Hz, หน่วย µs)

### บทที่ 7 — Calibration และ Tare (04)
- ขั้นตอน CEVA: mag หมุน 180° ทีละแกน, accel 4–6 ทิศ, gyro วางนิ่ง → รอ accuracy Medium/High → กด `s`
- คำสั่งทั้งหมด (หัวข้อ 5.2)
- **ต้องทดสอบคำสั่งจริงก่อนเขียน** (หัวข้อ 9) — เก็บภาพหน้าจอ accuracy เปลี่ยนจาก Unreliable → High

### บทที่ 8 — SPI ความเร็วสูง (05)
- หัวข้อ 5.3 ทั้งหมด + กล่องเตือนใหญ่เรื่อง P0 = WAKE (B7)
- อธิบายว่าทำไม SPI ได้ 400 Hz แต่ I2C แนะนำ ≤ 100–200 Hz

### บทที่ 9 — UART แบบเต็ม (07) และ UART-RVC (06)
- ตารางเปรียบเทียบ: UART-SHTP (API เต็ม, 3 Mbaud, ESP32 เท่านั้น) vs RVC (สายเส้นเดียว, 115200, ได้แค่ ypr + accel, Nano ใช้ได้)
- หัวข้อ 5.4 และ 5.5 — wiring, การสลับ P0/P1, ผลจริง
- จุดที่ผู้ใช้พลาดบ่อย: สลับ TX/RX (pad SDA = TX ของชิป), ลืม `setRxBufferSize()`, baud ผิด

### บทที่ 10 — ตรวจของแท้และ Factory Test (08)
- อธิบายแต่ละบรรทัดและเกณฑ์ (หัวข้อ 6.3)
- วิธี flash firmware สำเร็จรูป (esptool / web flasher) + ผลจริง (หัวข้อ 6.2)
- สิ่งที่การตรวจพิสูจน์ได้ / ไม่ได้ (BNO08x ไม่มี cryptographic attestation)

### บทที่ 11 — API Reference
- ยกจาก README หัวข้อ 7 + API ใหม่หัวข้อ 8.1 + ตาราง error code (`statusToString`)

### บทที่ 12 — Troubleshooting / FAQ
- ตารางจากหัวข้อ 7 (B1–B8) เขียนใหม่ในรูป "อาการ → ตรวจอะไร → แก้อย่างไร"
- ตารางปัญหาเครื่องมือ (Upload speed, Resource busy, CDC On Boot, PlatformIO lzma)

### ภาคผนวกคู่มือ
- ตาราง MCU compatibility (ESP32 / S3 / Nano) + ขาที่ใช้ในทุกตัวอย่าง (หัวข้อ 11.1)
- Changelog v2.0.0 → v2.1.0 (หัวข้อ 8)

### Checklist ก่อนปล่อยคู่มือ

- [ ] Flash `firmware/bin` v2.1.0 แล้วได้ `#VERDICT PASS` + `Library v2.1.0`
- [ ] ทดสอบคำสั่ง calibration / tare ใน 04
- [ ] ทดสอบตัวอย่าง 02–07 บน ESP32-S3 (อย่างน้อย SPI และ UART)
- [x] Compile บน Arduino-ESP32 Core 3.x (Arduino IDE 3.3.12 + PlatformIO 3.3.11) — 48/48 ผ่าน
- [ ] ทดสอบบอร์ดจริงบน Arduino-ESP32 Core 3.x
- [ ] ทดสอบ Nano (I2C + RVC) พร้อม level shifter
- [ ] ถ่ายภาพ wiring ทุกโหมด + ภาพหน้าจอ Serial Monitor ทุกตัวอย่าง
- [ ] Parser ของ Web Serial Monitor รองรับบรรทัด `#CHIP`
- [ ] แทนที่ `docs/Article_BNO08x.md` และ `BNO08x_User_Guide` เดิม (API v1)

---

## 11. ภาคผนวก

### 11.1 ขาที่ทุกตัวอย่างใช้ (ค่าใน .ino ปัจจุบัน)

| ตัวอย่าง | ESP32 DevKit | ESP32-S3 MOMO | Arduino Nano |
|---|---|---|---|
| 01_BasicRead | SDA 21, SCL 22, RST 17 | SDA 14, SCL 15 | A4, A5 |
| 02_CustomPins_BusRemap | Wire1 SDA 21, SCL 22, INT 4, RST 17 | Wire1 SDA 14, SCL 15 | A4, A5, INT D2 |
| 03_NonBlocking_Multitask | SDA 21, SCL 22, INT 4, RST 17 | SDA 14, SCL 15 | A4, A5, INT D2 |
| 04_Calibration_Tare | SDA 21, SCL 22, RST 17 | SDA 14, SCL 15 | A4, A5 |
| 06_SPI_Advance | SCK 18, MISO 19, MOSI 23, CS 5, INT 4, RST 17, WAKE 16 | SCK 12, MISO 13, MOSI 11, CS 10, INT 4, RST 5, WAKE 6 | D13, D12, D11, CS D10, INT D2, RST D3, WAKE D4 |
| 07_UART_RVC | RX 21 (TX 22 ไม่ใช้), RST 17 | RX 14 (TX 15 ไม่ใช้) | SoftwareSerial RX D2 |
| 08_UART_Mode | RX 21, TX 22, INT 4, RST 17 | RX 14, TX 15 | ไม่รองรับ |
| 09_Factory_Test | SDA 21, SCL 22 | SDA 14, SCL 15 | A4, A5 |

### 11.2 ขนาดโปรแกรม (Flash) และผล compile

Arduino IDE (ESP32 Core **3.3.12**, AVR 1.8.8) และ PlatformIO (pioarduino 55.03.311 = Core **3.3.11**) — ผ่านทุกช่อง, warning จากไลบรารี = 0

| ตัวอย่าง | Arduino ESP32 | Arduino ESP32-S3 | Arduino Nano | PIO `esp32dev` | PIO `esp32-s3-devkitc-1` | PIO `nano` |
|---|---|---|---|---|---|---|
| 01_BasicRead | 309,056 (23%) | 340,338 (25%) | 20,558 (66%) | 323,576 | 356,430 | 20,558 |
| 02_CustomPins_BusRemap | 308,552 (23%) | 339,822 (25%) | 19,638 (63%) | 322,964 | 355,842 | 19,638 |
| 03_NonBlocking_Multitask | 308,536 (23%) | 339,806 (25%) | 19,948 (64%) | 323,052 | 355,962 | 19,948 |
| 04_Calibration_Tare | 310,164 (23%) | 341,410 (26%) | 20,910 (68%) | 324,496 | 357,262 | 20,910 |
| 06_SPI_Advance | 309,744 (23%) | 341,206 (26%) | 19,676 (64%) | 324,748 | 357,626 | 19,676 |
| 07_UART_RVC | 297,108 (22%) | 305,293 (23%) | 6,818 (22%) | 311,220 | 319,721 | 5,858 |
| 08_UART_Mode | 308,808 (23%) | 340,102 (25%) | 2,838 (9%) stub | 323,160 | 356,046 | 1,648 stub |
| 09_Factory_Test | 313,068 (23%) | 343,910 (26%) | 24,836 (80%) | 326,952 | 359,570 | 24,836 |

หน่วย: byte · Nano ใน PIO ใช้ `-Wall` (ไม่ใส่ `-Wextra` เพราะ AVR core เองมี warning)

### 11.3 Firmware ที่ build ในรอบนี้

| ไฟล์ | Offset | ขนาด | SHA-256 |
|---|---|---|---|
| `Massmore_BNO08x_FactoryTest_ESP32.bin` (merged) | `0x0` | 398,688 B | `6419d58ec89ff155563041862972fffa5a5af5bcb89ea5d49bf2ce93ef620d40` |
| `Massmore_BNO08x_FactoryTest_ESP32_app.bin` | `0x10000` | 333,152 B | `f048f51aa795aef8255a8c1f0ef600b495b6159a1a144e7e76f7e559fb4cd2a1` |

Build: PlatformIO `esp32dev`, pioarduino 55.03.311 (Core 3.3.11), `firmware.factory.bin` · 4 MB · DIO

### 11.4 คำสั่งที่ใช้ build / flash (อ้างอิง)

```bash
# PlatformIO ทุก env (macOS เครื่องทดสอบต้องใส่ DYLD_FALLBACK_LIBRARY_PATH)
cd PlatformIO && DYLD_FALLBACK_LIBRARY_PATH=/usr/lib pio run -e esp32dev -e esp32-s3-devkitc-1 -e nano

# compile ตัวอย่าง (arduino-cli ที่มากับ Arduino IDE)
arduino-cli compile -b esp32:esp32:esp32 --library ArduinoIDE/Massmore_BNO08x \
  ArduinoIDE/Massmore_BNO08x/examples/01_BasicRead

# upload (ESP32 DevKit ต้องลดความเร็ว)
arduino-cli upload -b esp32:esp32:esp32:UploadSpeed=460800 -p /dev/cu.usbserial-130 <build-dir>

# ESP32-S3 MOMO (CDC On Boot = Disabled เป็นค่าเริ่มต้นของ FQBN นี้)
arduino-cli compile -b esp32:esp32:esp32s3 --library ArduinoIDE/Massmore_BNO08x <sketch>
```

### 11.5 โครงสร้างโปรเจกต์หลัง cleanup

```text
Massmore_BNO08x_SKU-1010/
├── README.md
├── .gitignore
├── ArduinoIDE/
│   ├── Massmore_BNO08x.zip            ← ไฟล์ติดตั้งใน Arduino IDE (v2.1.0)
│   └── Massmore_BNO08x/               ← ต้นฉบับไลบรารี
│       ├── library.properties  keywords.txt  LICENSE
│       ├── src/  (Massmore_BNO08x.h/.cpp, _Defs.h, _RVC.h)
│       └── examples/ 01 … 08
├── PlatformIO/
│   ├── platformio.ini
│   ├── src/main.cpp                   ← ตัววัดมุม Euler (Game RV) ตั้งแต่ v2.1.1
│   └── lib/Massmore_BNO08x/           ← สำเนาเดียวกับ ArduinoIDE + library.json
├── firmware/
│   ├── README.md  manifest.json
│   └── bin/  Massmore_BNO08x_FactoryTest_ESP32(.bin | _app.bin)
└── docs/
    ├── Report_v2.1.0_Test_and_Manual_Plan.md   ← เอกสารนี้
    ├── Article_BNO08x.md  BNO08x_User_Guide.docx/.pdf   ← เอกสารเก่า (API v1)
    └── images/
```

---

## 12. รอบทดสอบเพิ่มเติม 2026-09-27 (v2.1.1)

ทดสอบ I2C บน **ESP32-S3 MOMO** ต่อจากรอบ v2.1.0 (ซึ่งบน MOMO ทดสอบไว้แค่ตัวอย่าง 01 และ 09)
ชิปตัวเดิม: BNO086 part `10004563` FW `3.12.6` build `62` · SDA 14 / SCL 15 · 100 kHz
build ผ่าน PlatformIO (pioarduino 55.03.311 = Core 3.3.11) ทั้งหมด

### 12.1 ผลทดสอบ

| ตัวอย่าง | ผล | ค่าที่วัดได้ |
|---|---|---|
| `09_Factory_Test` | **PASS** | `#VERDICT PASS` ครบ 11 ข้อ · `#CHIP BNO086` · \|a\| noise 0.040 m/s² |
| `01_BasicRead` | **PASS** | `roll=-1.32 pitch=-20.02 yaw=0.47` นิ่งที่ทศนิยม 2 ตำแหน่ง · ไม่พบอาการ quaternion real ≈ 0 อีก (bug B1 ของ v2.1.0 หายจริง) |
| `03_NonBlocking_Multitask` | **PASS** | `loop/s=640`, `RV reports/s=100` ตรงกับ `INTERVAL_100HZ` |
| `04_Calibration_Tare` | **PASS** | `?` → `ME calibration status = 0 (OK)` · `z` → yaw 147.2° → -0.8° · heading err 15.9° → 2.9° |
| `05_I2C_Euler_Compass` | **PASS** | heading + ทิศ 16 ทิศ ทำงาน (accuracy Unreliable ก่อน calibrate mag เป็นปกติ) |
| Compile matrix | **PASS** | 9 ตัวอย่าง × 3 บอร์ด PlatformIO = **27/27** build, warning = 0 (`-Wall -Wextra`) |

ยังไม่ได้ทดสอบบน MOMO: `02` (Wire1), `06` SPI, `07` UART-RVC, `08` UART-SHTP — ต้องเดินสายเพิ่ม

### 12.2 B6 — ตัวอย่าง 04 เปิด RV และ Game RV พร้อมกัน (แก้แล้ว)

- **อาการ:** `enableReports()` ของตัวอย่าง 04 เปิด Rotation Vector, magnetometer และ Game Rotation Vector พร้อมกัน
- **สาเหตุ:** ทั้ง RV, Game RV, Geomagnetic RV และ ARVR variant เขียนลง `_quat` ตัวเดียวกันใน `parseInputReports()` ค่า Euler ที่พิมพ์จึงเป็นของ report ที่มาถึงทีหลัง สลับไปมาระหว่าง fusion 9 แกนกับ 6 แกน
- **แก้:** ตัด `enableGameRotationVector()` ออกจากตัวอย่าง 04 และเขียนคอมเมนต์เตือนไว้ใน `enableReports()` (โค้ดไลบรารีไม่เปลี่ยน)

### 12.3 B7 — `tareNow()` ต้องส่ง basis ให้ตรงกับ report ที่เปิดไว้

- **อาการ:** ใช้ Game RV แล้วสั่ง `tareNow(TARE_AXIS_ALL)` ชิปตอบรับแต่มุมไม่ขยับเลย
- **สาเหตุ:** ค่า default ของ `basis` คือ `TARE_BASIS_ROTATION_VECTOR` (9 แกน) ซึ่งไม่ได้เปิดใช้ ชิปจึง tare vector ที่ไม่มีใครอ่าน
- **แก้:** ส่ง `TARE_BASIS_GAMING_RV` เมื่อใช้ Game RV → ทุกแกนเป็น 0.00 ถูกต้อง
- **ข้อจำกัดของชิป:** `TARE_AXIS_Z` + `TARE_BASIS_GAMING_RV` ไม่มีผลใด ๆ (ทดสอบ BNO086 fw 3.12.6) ถ้าต้องตั้งศูนย์เฉพาะ yaw บน Game RV ให้หัก offset ในซอฟต์แวร์ — `PlatformIO/src/main.cpp` ทำแบบนี้

### 12.4 B8 — `softReset()` ไม่ล้าง tare

- **อาการ:** tare แล้วสั่ง `softReset()` มุมยังเป็น 0 ทุกแกน
- **แก้:** ใช้ `clearTare()` (Set Reorientation เป็น identity quaternion) ซึ่งไม่เขียน flash ด้วย

### 12.5 อาการ I2C ค้างเมื่อ upload ทับขณะชิปกำลังส่ง report

ยืนยันสาเหตุและวิธีแก้ของ B3 (v2.1.0) เพิ่มเติมด้วยการวัดระดับสัญญาณจริง:

| สภาพ | SDA | SCL | กู้ด้วย 20 clock + STOP | สแกนเจอชิป |
|---|---|---|---|---|
| ปกติ | 1 | 1 | — | เจอ 0x4A |
| ชิปกด SDA ค้าง | **0** | 1 | SDA กลับเป็น 1 | ต้อง reset ชิปก่อน |
| เซ็นเซอร์ไม่มีไฟ / สายหลุด | **0** | **0** | ไม่ได้ผล | ไม่เจอ |

- ตัวกระตุ้นคือ **การ reset MCU กลางการรับส่ง I2C** ซึ่งเกิดทุกครั้งที่ upload firmware และทุกครั้งที่เปิด Serial Monitor (toggle DTR/RTS) ยิ่ง report rate สูงยิ่งเจอบ่อย (ที่ 100 Hz เจอแทบทุกครั้ง)
- **วิธีแก้ที่ได้ผล 100%:** ต่อขา RST เข้า GPIO แล้วส่งให้ `begin()` — ทดสอบ reset กลางสตรีม 100 Hz **5/5 ครั้ง ชิปกลับมาทุกครั้ง** ทุกตัวอย่างจึงเพิ่มบรรทัดคอมเมนต์ `RST_PIN 18` ไว้ให้เปิดใช้
- ถ้าไม่ต่อ RST: ตัดไฟเซ็นเซอร์แล้วจ่ายใหม่ (ถอด USB ~5 s) เป็นวิธีเดียวที่กู้ได้ในกรณีที่สัญญาณถูกกดค้างทั้งสองเส้น

---

Massmore Biz Co., Ltd. · [www.massmore.shop](https://www.massmore.shop)
