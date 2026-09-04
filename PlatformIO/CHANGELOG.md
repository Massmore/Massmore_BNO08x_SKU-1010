# Changelog

All notable changes to the Massmore BNO08x library.
Format follows [Keep a Changelog](https://keepachangelog.com/), versioning
follows [Semantic Versioning](https://semver.org/).

## [1.0.2] — 2026-09-02

สองบั๊กแรกเจอตอนรัน `16_FactoryTest` กับบอร์ดจริงเป็นครั้งแรก

### Fixed

- **base timestamp เป็นเลขมีเครื่องหมาย แต่โค้ดอ่านเป็น `uint32_t`** —
  รายงานถูกสร้างก่อนที่แพ็กเก็ตจะถูกส่ง ค่า base delta จึงติดลบเป็นปกติ
  พออ่านเป็น unsigned ค่า `-120` กลายเป็น `4294967176` ทำให้ `getTimestampUs()`
  กระโดดไปมาและถอยหลัง แก้เป็น `int32_t` แล้ว
- **`getTimestampUs()` ไม่มีจุดอ้างอิง** — base delta กับ delay เป็นระยะเวลา
  *เทียบกับตอนที่แพ็กเก็ตถูกส่ง* ไม่ใช่เวลาสัมบูรณ์ ตอนนี้ไดรเวอร์จับ `micros()`
  ตอนรับแพ็กเก็ตแล้วบวกเข้าไป ค่าที่ได้จึงเทียบกับ `millis()`/`micros()` ได้ตรง ๆ
  และเดินหน้าอย่างเดียว
- **เก็บ Product ID Response ได้ชุดเดียว** — ชิปตอบคำขอเดียวด้วยหลายชุด
  ชุดละหนึ่งอิมเมจเฟิร์มแวร์ และ SH-2 application ไม่ได้มาเป็นชุดแรกเสมอ
  ของเดิมคว้าชุดไหนก็ได้ที่มาถึงก่อน ทำให้บอร์ดแท้ที่ตอบ part 10004563 มาก่อน
  ถูก `verifyChip()` ตัดสินเป็น `UNKNOWN_FW` ทั้งที่มี 10003606 ตามมาด้วย
  ตอนนี้เก็บครบทุกชุด เลือกชุดที่เป็น SH-2 application เป็นตัวหลัก และ
  `verifyChip()` ผ่านถ้าชุดใดชุดหนึ่งตรงตาราง

### Added

- `getProductIDCount()` และ `getProductID(index)` สำหรับไล่ดูทุกอิมเมจที่ชิปแจ้ง

### Changed

- `16_FactoryTest` ปรับเกณฑ์ตัดสินหลังทดสอบกับบอร์ดจริง:
  `SERIAL_NUMBER` ที่ไม่มี record `0x4B4B` เป็น **WARN** ไม่ใช่ FAIL (หลายล็อต
  ไม่ได้โปรแกรมเลขซีเรียลมาจากโรงงาน) · `RAW_REPORTS` เปิดเซ็นเซอร์ปกติคู่กับ raw
  ตามที่ชิปต้องการ และเป็น WARN ถ้าเฟิร์มแวร์ไม่ปล่อย raw ออกมา ·
  `FEATURE_CONFIG` รอให้รายงานเดินก่อนค่อยถามกลับ และถือว่า echo `interval=0`
  เป็น WARN เมื่อเซ็นเซอร์ส่งข้อมูลจริง · `FRS_METADATA` มองหาคู่ Q point 14/12
  ทั้ง word 7 และ 8 เพราะตำแหน่งขยับตาม revision ของ record และเลิกพิมพ์
  min period ที่ยังไม่รู้แน่ว่าอ่านจากช่องไหน · `CALIBRATION_CMD` เปิด rotation
  vector ให้ MotionEngine เดินก่อนสั่ง · `EVENT_ENGINES` ให้เวลา 2.5 วินาที
  เพื่อให้ classifier เลิกตอบ Unknown · `STREAM_INTEGRITY` ให้ sequence number
  เป็นตัวชี้ขาด ส่วน timestamp ที่สลับลำดับเป็น WARN
- ข้อความผลลัพธ์ในรายงานตัดแบบรู้ขอบตัวอักษร UTF-8 แล้ว ภาษาไทยไม่ขาดกลางตัว

### Added (ตัวอย่างและเฟิร์มแวร์ที่มากับรุ่นนี้)

- **ตัวอย่างที่ 16 `16_FactoryTest`** — ชุดทดสอบสำหรับสายการผลิต / QC บนบัส I2C
  (`SDA = GPIO 21`, `SCL = GPIO 22`, 100 kHz) รันเองทันทีหลังบูต ไม่ต้องพิมพ์อะไร
  ด่านที่ 1 สแกนบัสหา `0x4B` / `0x4A` และเตือนถ้าเจอ `0x28` / `0x29`
  ซึ่งแปลว่าขา BOOTN ถูกดึงลงจนชิปเข้าโหมด DFU · ด่านที่ 2 อ่าน Product ID (0xF8)
  แล้วเรียก `verifyChip()` — ถ้าสองด่านนี้ไม่ผ่านจะไม่เข้าโหมด RUN TEST
  จากนั้นไล่ทดสอบต่ออีก 22 หัวข้อครอบคลุมจุดเด่นของ BNO08x ในโหมด I2C ทั้งหมด
  (FRS, metadata, oscillator, error queue, soft reset, set/get feature, accel, gyro,
  mag, rotation vector ทั้งสามชนิด, gravity/linear accel, gyro-integrated RV บนช่อง 5,
  raw reports, uncalibrated reports, throughput หลาย report พร้อมกัน, เอนจินตรวจจับ
  เหตุการณ์ 13 ตัว, calibration, tare, sleep/wake และความต่อเนื่องของ timestamp
  กับ sequence number) แล้วสรุป PASS / FAIL / WARN
  ทุกหัวข้อพิมพ์บรรทัด `#RESULT,...` และปิดท้ายด้วย `#VERDICT,...` ให้เว็บ parse ต่อได้
- **โฟลเดอร์ `firmware/`** — เฟิร์มแวร์ของ `16_FactoryTest` ที่คอมไพล์แล้วสำหรับ
  ESP32 DevKit (WROOM-32, Flash 4 MB) พร้อม `manifest.json` สำหรับ ESP Web Tools
  ตาราง offset และ SHA-256 ครบ ลูกค้าแฟลชผ่านเว็บแล้วดู Serial Monitor ได้เลย
- **ตัวอย่างที่ 15 `15_UART_SHTP`** — SH-2 เต็มรูปแบบผ่าน UART (ต่างจากตัวอย่าง 12
  ที่เป็น UART-RVC ทางเดียว) ใช้ `beginUART()` ที่ 3,000,000 บอด ซึ่งเป็นค่าตายตัวของชิป
  ได้ quaternion, calibration, tare, FRS และทุกคำสั่งเหมือน I2C/SPI
  พร้อมคำเตือนว่าสัญญาณเร็วขนาดนี้ต้องต่อ 3.3V ตรง ๆ ไม่ควรผ่าน level shifter 2N7002
  ของบอร์ดซึ่งออกแบบมาสำหรับ I2C แบบ open-drain

### Changed (ชื่อขา ลำดับตัวอย่าง และรายงานผล)

- **เปลี่ยนชื่อขาในโค้ดและ README ทั้งหมดให้ตรงกับซิลค์สกรีนบอร์ด Massmore Halley V2**
  `SA0`/`ADDR` -> **`DI`** · `PS0` -> **`P0`** · `PS1` -> **`P1`** · `BOOTN` -> **`BT`** ·
  `VIN`/`VCC` -> **`5V`** หรือ **`3Vo`** ส่วนชื่อในเอกสารของ CEVA ยังกำกับไว้ในวงเล็บ
  เพื่อให้เทียบกับ datasheet ได้ · README หลักเพิ่มตารางเทียบชื่อขาบนบอร์ดกับชื่อในเอกสาร
  และตารางสถานะ `P1`/`P0` ของทั้งสี่โหมด (I2C / SPI / UART-SHTP / UART-RVC)
- ระบุชัดว่ามีเพียง `SDA` กับ `SCL` ที่ผ่าน level shifter 2N7002 บนบอร์ด
  ขา `INT` `RST` `DI` `CS` `BT` `P0` `P1` เป็น 3.3V ล้วน
- **`16_FactoryTest` ปิดท้ายด้วยรายงานสรุปแทนบรรทัดนับผลสั้น ๆ** — บอกในหน้าจอเดียวว่า
  เจอ address อะไร ชิปอะไร firmware/part/build/serial อะไร ของแท้หรือไม่ แล้วไล่ผล
  ทุกหัวข้อด้วยเครื่องหมาย `[✓]` ผ่าน · `[!]` เตือน · `[✗]` ไม่ผ่าน ปิดท้ายด้วยจำนวน
  ผ่าน/เตือน/ไม่ผ่าน และรายชื่อหัวข้อที่มีปัญหา จึงไม่ต้องเลื่อนขึ้นไปอ่านย้อน
- **เพิ่มบรรทัด `#DEVICE,...` สำหรับให้เว็บ parse** — address, firmware version,
  part number, build number, serial number และสถานะของแท้ ในบรรทัดเดียว ออกท้ายสุด
  พร้อม `#VERDICT`
- **สลับลำดับตัวอย่าง** ให้ `FactoryTest` อยู่ท้ายสุด: `15_UART_SHTP` · `16_FactoryTest`
- **ลด upload speed เป็น 460800** ทั้งใน `platformio.ini` และคำแนะนำใน README
  เพราะไดรเวอร์ USB-serial บน macOS หลายตัวไม่นิ่งที่ 921600

## [1.0.1] — 2026-09-01

Corrections found while writing the documentation, all traceable to primary
CEVA sources. No API changes.

### Fixed

- **Two sensors in one sketch now work.** The report-length table learned from
  the SHTP advertisement lived in a file-scope static, so a second
  `MassmoreBNO08x` object shared and corrupted the first one's table. It is now
  a per-instance member. (The Adafruit and SparkFun libraries have the same
  class of bug, which is why "two BNO08x on one bus" never works with them.)
- **Reset settle time raised from 120 ms to 300 ms** when no INT pin is wired.
  No CEVA document specifies a boot-to-ready time — the "~90 ms" figure quoted
  around the internet, and previously in this library's comments, has no source.
  The datasheet only states that the part asserts H_INTN when its reset routine
  completes, so waiting on that pin is still the preferred path and is
  unchanged. 300 ms is the value both the Adafruit and SparkFun drivers use.

### Changed

- **Calibration instructions corrected throughout.** Datasheet Figure 3-2
  specifies: accelerometer = 4-6 unique orientations held ~1 s each; gyroscope =
  stationary surface for 2-3 s; magnetometer = rotate ~180 degrees and back
  about EACH of roll, pitch and yaw, ~2 s per axis. The figure-of-eight motion
  previously described in example 05 and in the code comments is BNO055
  folklore and does not calibrate this part.
- **Examples now default to 100 kHz I2C instead of 400 kHz.** The BNO08x uses
  clock stretching; 400 kHz is within spec but is widely reported to drop bytes
  on ESP32 and RP2040 hosts. Example 10 (high-rate) still uses 400 kHz with a
  note on what to do if reports stall.
- Example 06 now warns that a tare applied before the magnetometer has resolved
  north will be invalidated as soon as it does.

## [1.0.0] — 2026-08-31

First public release.

### Added

**Transports**
- SHTP over I2C, with chunked reads that respect the no-repeated-start rule
- SHTP over SPI at up to 3 MHz, mode 3, with WAKE handshake
- SHTP over UART at 3 Mbit/s with RFC-1662 style byte stuffing
- Separate `MassmoreBNO08x_RVC` class for the 100 Hz UART-RVC output stream

**Sensor reports**
- Rotation vector, game rotation vector, geomagnetic rotation vector
- AR/VR stabilised rotation vector and game rotation vector
- Gyro-integrated rotation vector on its own SHTP channel, up to 1 kHz
- Accelerometer, gyroscope, magnetometer (calibrated and uncalibrated)
- Linear acceleration, gravity
- Raw accelerometer, gyroscope and magnetometer with sensor timestamps
- Tap, shake, flip, pickup, tilt, pocket, circle, sleep and stability detectors
- Step counter, step detector, significant motion
- Stability classifier and personal activity classifier with confidences
- Heart rate monitor
- External environmental sensors: pressure, ambient light, humidity,
  proximity, temperature

**Configuration**
- Full Set Feature command: report interval, batch interval, feature flags,
  change sensitivity and the sensor-specific configuration word
- Get Feature request, so the host can read back what the device is doing
- Report lengths learned at runtime from the device's SHTP advertisement,
  with a built-in fallback table

**Calibration and orientation**
- ME dynamic calibration for accelerometer, gyroscope, magnetometer and
  planar accelerometer
- Save DCD to flash, periodic auto-save, clear DCD and reset
- Tare now / persist tare / clear tare, with per-axis and per-basis control

**Device management**
- Product ID request and decode
- `verifyChip()` authenticity check against Product ID, version plausibility
  and a table of known factory firmware part numbers
- Factory serial number read from FRS record 0x4B4B
- FRS record read and write with the full handshake
- Sensor metadata read
- Soft reset, hardware reset, executable sleep and on
- Oscillator type and error list queries

**Ergonomics**
- Euler angle helpers in radians and degrees, plus a 0..360 compass heading
- Per-report accuracy tracking and reconstructed microsecond timestamps
- `hasNewReport()` test-and-clear flags and an optional report callback
- Latching event getters so a gesture is never missed or double-counted
- Typed error codes with human readable strings
- Optional debug output to any `Stream`

**Packaging**
- 14 worked examples from a five line quick start to a ten-report
  batching and power management demo
- Separate PlatformIO and Arduino IDE distributions
