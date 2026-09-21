# Massmore_BNO08x — Factory Test Firmware

เฟิร์มแวร์ที่ compile แล้วของตัวอย่าง [`08_Factory_Test`](../ArduinoIDE/Massmore_BNO08x/examples/08_Factory_Test/)
สำหรับ **ESP32 (Classic) DevKit — Primary Factory Test MCU** ใช้ตรวจบอร์ด Massmore BNO08x SKU-1010
ก่อนส่งลูกค้า (Outgoing QA/QC) และให้ **Massmore Web Serial Monitor** อ่านผลอัตโนมัติ

## Files

| File | Flash offset | Use |
|---|---|---|
| `bin/Massmore_BNO08x_FactoryTest_ESP32.bin` | `0x0` | **Merged image** (bootloader + partitions + app) — ไฟล์เดียวจบ ใช้กับ web flasher / esptool |
| `bin/Massmore_BNO08x_FactoryTest_ESP32_app.bin` | `0x10000` | Application เท่านั้น (ใช้เมื่อไม่ต้องการเขียนทับ bootloader / partition table) |
| `manifest.json` | — | Manifest สำหรับ ESP Web Tools |

Build (v2.1.0): PlatformIO env `esp32dev`, pioarduino `55.03.311` (Arduino-ESP32 Core 3.3.11), partition `default.csv`, 4 MB flash, DIO

## Wiring (Primary MCU = ESP32 Classic, I2C / Qwiic)

| Halley V2 | ESP32 DevKit | Note |
|---|---|---|
| `3Vo` (หรือ `5V`) | 3V3 (หรือ 5V) | |
| `GND` | GND | |
| `SDA` | GPIO 21 | |
| `SCL` | GPIO 22 | I2C 100 kHz |
| `INT`, `RST` | ไม่ต้องต่อ | test ใช้ polling (ถ้าต่อ ให้แก้ `FT_INT_PIN` / `FT_RST_PIN` แล้ว build ใหม่) |
| `DI` | ไม่ต้องต่อ = `0x4A` · ต่อ `3Vo` = `0x4B` | scan หาให้เอง |
| `BT`, `P0`, `P1` | ไม่ต้องต่อ | `BT` ห้ามดึง LOW |

**วางบอร์ดนิ่ง ๆ บนโต๊ะ** ระหว่างทดสอบ (มี range check ของ accel / gyro)

## Flashing

**1. esptool (command line)** — ไฟล์เดียวจบ

```bash
esptool.py --chip esp32 --port /dev/cu.usbserial-0001 --baud 921600 \
  write_flash -z 0x0 bin/Massmore_BNO08x_FactoryTest_ESP32.bin
```

ถ้าอัพโหลดไม่นิ่ง (เช่น `Invalid head of packet`) ให้ลด `--baud` เป็น `460800` หรือ `115200` · Windows ใช้ `--port COM5`

**2. PlatformIO** — เปิดโฟลเดอร์ `PlatformIO/` แล้ว `pio run -e esp32dev -t upload` (main.cpp คือ Factory Test อยู่แล้ว)

**3. Web flasher (ESP Web Tools)** — วางโฟลเดอร์ `firmware/` บนเว็บ HTTPS แล้วใช้

```html
<script type="module" src="https://unpkg.com/esp-web-tools@10/dist/web/install-button.js"></script>
<esp-web-install-button manifest="firmware/manifest.json"></esp-web-install-button>
```

## Serial Output Format (115200 8N1, English only)

ทุกบรรทัดที่เว็บ parse ขึ้นต้นด้วย `#` · บรรทัดอื่นเป็น human-readable

```text
#MASSMORE_FACTORY_TEST v1.0
#PRODUCT Massmore_BNO08x
#MCU ESP32
#RESULT <TEST_NAME> <PASS|FAIL> <value>      (one line per sub-test)
#VERDICT <PASS|FAIL> [<REASON>]              (exactly once, last # line)
[PASS] SENSOR QA PASSED - READY TO SHIP   |   [FAIL] QA CHECK FAILED: <REASON>
```

| TEST_NAME | PASS criteria | Value |
|---|---|---|
| `BUS_SCAN` | พบ ACK ที่ 0x4A หรือ 0x4B (0x28/0x29 = bootloader → FAIL) | address |
| `CHIP_ID` | SH-2 Product ID part number ตรง 10003606 / 10004095 / 10004563 | part number |
| `FW_VERSION` | major 1..9, build ≠ 0 | `major.minor.patch` |
| `SERIAL` | อ่าน FRS 0x4B4B ได้ หรือ record ว่าง (`NONE`) — timeout = FAIL | hex / `NONE` |
| `METADATA` | Rotation Vector metadata Q point = 14 / 12 (MotionEngine ทำงานจริง) | `Q14/12` |
| `AUTHENTICITY` | `isGenuine()` | `GENUINE` / `SUSPECT` |
| `RANGE_ACCEL` | \|a\| = 9.81 ± 1.5 m/s² (วางนิ่ง) | m/s² |
| `RANGE_GYRO` | \|ω\| ≤ 0.5 rad/s (วางนิ่ง) | rad/s |
| `RANGE_MAG` | 5 ≤ \|B\| ≤ 200 µT | µT |
| `RANGE_QUAT` | \|q\| = 1.00 ± 0.02 | norm |
| `CONTINUOUS` | 20/20 `readAll()` สำเร็จ, ไม่มี NAN, noise \|a\| (max−min) ≤ 1.0 m/s² | `n/20` |

พิมพ์ `r` + Enter เพื่อทดสอบซ้ำโดยไม่ต้อง reset

## Expected Report (real passing board)

ผลจริงจากบอร์ด Massmore BNO08x (ชิป BNO086) บน ESP32 DevKit — flash ไฟล์ merged ที่ `0x0` แล้วเปิด Serial 115200 (ทดสอบจริง 2026-09-22 — บรรทัด `Library` แสดงเวอร์ชันของไฟล์ bin ปัจจุบัน):

```text
Massmore_BNO08x - 08_Factory_Test (keep the board still)

#MASSMORE_FACTORY_TEST v1.0
#PRODUCT Massmore_BNO08x
#MCU ESP32
Library v2.1.0
  I2C device at 0x4A
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

- `#CHIP` (`BNO085` / `BNO086` / `BNO08x`) เป็นข้อมูลเท่านั้น ไม่ใช่เงื่อนไข PASS/FAIL — parser ของเว็บต้องข้ามบรรทัดที่ไม่รู้จักได้
- ค่า `RANGE_*` ขึ้นกับตำแหน่งวางบอร์ดและสนามแม่เหล็กรอบตัว — ดูเกณฑ์ใน `08_Factory_Test.ino`
- ถ้า MCU ถูก reset ขณะชิปกำลังส่งข้อมูล ชิปอาจกด SDA/SCL ค้าง → `BUS_SCAN FAIL` ให้ถอดไฟเซ็นเซอร์แล้วทดสอบใหม่

## Rebuild

**PlatformIO (วิธีหลัก — ใช้ build ไฟล์ปัจจุบัน, Arduino-ESP32 Core 3.3.11):**

```bash
cd PlatformIO
pio run -e esp32dev
cp .pio/build/esp32dev/firmware.factory.bin ../firmware/bin/Massmore_BNO08x_FactoryTest_ESP32.bin
cp .pio/build/esp32dev/firmware.bin         ../firmware/bin/Massmore_BNO08x_FactoryTest_ESP32_app.bin
```

> macOS: ถ้า PlatformIO ขึ้น `Python's lzma module is unavailable` ให้รันด้วย `DYLD_FALLBACK_LIBRARY_PATH=/usr/lib pio run -e esp32dev`

**Arduino IDE / arduino-cli (ทางเลือก):**

```bash
arduino-cli compile -b esp32:esp32:esp32 --library ArduinoIDE/Massmore_BNO08x \
  --build-path build/fw ArduinoIDE/Massmore_BNO08x/examples/08_Factory_Test
# build/fw/08_Factory_Test.ino.merged.bin = merged image สำหรับ offset 0x0 (Core 3.x สร้างให้เอง)
```
