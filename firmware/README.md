# Firmware สำเร็จรูป — `16_FactoryTest`

ไฟล์ในโฟลเดอร์นี้คือ **เฟิร์มแวร์ที่คอมไพล์แล้ว** ของตัวอย่าง
[`16_FactoryTest`](../ArduinoIDE/Massmore_BNO08x/examples/16_FactoryTest/)
สำหรับ **ESP32 DevKit (ESP32-WROOM-32, Flash 4 MB)**

ลูกค้าที่ไม่มี Arduino IDE หรือ VS Code สามารถแฟลชไฟล์นี้ผ่านเว็บได้ทันที
แล้วเปิด Serial Monitor 115200 ดูผลการทดสอบบอร์ดได้เลย

| หัวข้อ | ค่า |
|---|---|
| บอร์ด | ESP32 DevKit / WROOM-32, Flash 4 MB |
| PlatformIO env | `esp32dev` |
| Arduino ESP32 core | 3.3.11 (pioarduino `55.03.311`) |
| Partition scheme | `default.csv` (app 1.25 MB × 2 + SPIFFS) |
| Serial | 115200 8N1 |
| ขา I2C | SDA = GPIO **21**, SCL = GPIO **22** |
| I2C clock | 100 kHz (BNO08x ใช้ clock stretching) |
| I2C address | สแกนหาเอง — `0x4B` (ขา `DI` = 1) หรือ `0x4A` (ขา `DI` = 0) |
| INT / RST | ไม่ต้องต่อ |

---

## ไฟล์ในโฟลเดอร์นี้

```
firmware/
├── manifest.json                 <- manifest สำหรับ ESP Web Tools
├── esp32dev/
│   ├── merged-firmware.bin       <- ไฟล์เดียวจบ แฟลชที่ offset 0x0
│   ├── bootloader.bin            <- แฟลชที่ 0x1000
│   ├── partitions.bin            <- แฟลชที่ 0x8000
│   ├── boot_app0.bin             <- แฟลชที่ 0xE000
│   └── firmware.bin              <- แฟลชที่ 0x10000 (แอปพลิเคชันล้วน)
└── README.md
```

### ตารางออฟเซ็ต

| ไฟล์ | Offset | ขนาด (ไบต์) |
|---|---|---|
| `merged-firmware.bin` | `0x0` | 426,592 |
| `bootloader.bin` | `0x1000` | 23,520 |
| `partitions.bin` | `0x8000` | 3,072 |
| `boot_app0.bin` | `0xE000` | 8,192 |
| `firmware.bin` | `0x10000` | 361,056 |

`merged-firmware.bin` คือไฟล์ 4 ตัวข้างบนรวมกันไว้แล้วในไฟล์เดียว
(ช่วง `0x0`–`0x1000` เติมด้วย `0xFF`) จึงแฟลชที่ `0x0` ได้ตรง ๆ

### SHA-256

```
7ce108e007bbe8bc4688ba44e74a89ccead570fa9e330ebe76f2c627fc631ae3  merged-firmware.bin
b2306bbe4af1424d5ea454030d7482774befacaaea068b1add2a621f4328aef8  bootloader.bin
148b959cbff1c38aa8e1d5c0ba9d612c54997b945e56a63f41223eef650653a1  partitions.bin
f94c5d786a7a8fab06ac5d10e33bf37711a6697636dc037559ea19cc410a17f0  boot_app0.bin
58369f25c4d5f894ffa9470a3d1b877ae2fb6682b2d2d4db26b159b5b71eea8a  firmware.bin
```

---

## วิธีที่ 1 — แฟลชผ่านเว็บ (ESP Web Tools)

วางไฟล์ทั้งโฟลเดอร์นี้ไว้บนเว็บ แล้วใส่แท็กนี้ในหน้าเพจ

```html
<script type="module"
        src="https://unpkg.com/esp-web-tools@10/dist/web/install-button.js">
</script>

<esp-web-install-button manifest="firmware/manifest.json">
  <button slot="activate">แฟลชเฟิร์มแวร์ Factory Test</button>
  <span slot="unsupported">เบราว์เซอร์นี้ไม่รองรับ — ใช้ Chrome หรือ Edge</span>
  <span slot="not-allowed">ต้องเปิดหน้านี้ผ่าน HTTPS หรือ localhost</span>
</esp-web-install-button>
```

ข้อกำหนดของ Web Serial: ต้องเป็น **Chrome / Edge** และหน้าเว็บต้องเป็น
**HTTPS** (หรือ `localhost`)

## วิธีที่ 2 — esptool (คอมมานด์ไลน์)

ไฟล์เดียวจบ

```bash
esptool.py --chip esp32 --port /dev/cu.usbserial-0001 --baud 460800 \
  write_flash -z 0x0 esp32dev/merged-firmware.bin
```

หรือแยกทีละส่วน (ใช้เมื่อไม่อยากลบพื้นที่ส่วนอื่นของแฟลช)

```bash
esptool.py --chip esp32 --port /dev/cu.usbserial-0001 --baud 460800 \
  write_flash -z \
  0x1000  esp32dev/bootloader.bin \
  0x8000  esp32dev/partitions.bin \
  0xe000  esp32dev/boot_app0.bin \
  0x10000 esp32dev/firmware.bin
```

> บอร์ดบางรุ่นต้องกดปุ่ม **BOOT** ค้างไว้ตอนเริ่มอัปโหลด
> ใช้ 460800 เพราะไดรเวอร์ USB-serial บน macOS หลายตัวไม่นิ่งที่ 921600
> ถ้ายังไม่ผ่าน ให้ลดความเร็วเป็น `--baud 115200`

---

## การต่อสาย

ชื่อขาคือชื่อที่พิมพ์บนบอร์ด Halley V2

| Halley V2 | ESP32 DevKit |
|---|---|
| `5V` หรือ `3Vo` | 5V หรือ 3V3 |
| `GND` | GND |
| `SDA` | GPIO 21 |
| `SCL` | GPIO 22 |
| `INT` | ไม่ต้องต่อ (ชุดทดสอบใช้ polling) |
| `RST` | ไม่ต้องต่อ (ใช้ soft reset ผ่าน executable channel) |
| `DI` | ไม่ต้องต่อ = `0x4B` · ต่อลง GND = `0x4A` |
| `BT` | ปล่อยไว้ตามบอร์ด **ห้ามดึงลงกราวด์** ไม่งั้นชิปจะเข้า bootloader |
| `P0` `P1` | ไม่ต้องต่อ (ต้องเป็น 0/0 จึงจะเป็นโหมด I2C) |

pull-up ของ SDA/SCL ควรอยู่ที่ **2.2k–4.7k** ต่อไฟ 3V3 ตามที่ datasheet ของ CEVA
ระบุไว้ (2–4 kΩ) ไม่ใช่ 10k แบบที่บอร์ด breakout หลายเจ้าให้มา

**วางบอร์ดนิ่ง ๆ บนโต๊ะระหว่างทดสอบ** เพราะหลายหัวข้อวัดค่าจริงจากเซนเซอร์

---

## สิ่งที่จะเห็นใน Serial Monitor

โปรแกรมรันเองทันทีหลังบูต ไม่ต้องพิมพ์อะไร

```
==============================================================
  MASSMORE BNO08x 9-AXIS IMU  --  FACTORY TEST
  SKU-1010   BNO085 / BNO086 Sensor Fusion (I2C)
==============================================================
  library   : Massmore_BNO08x v1.0.2
  board     : ESP32 DevKit (Flash 4 MB)   CPU 240 MHz
  interface : I2C  SDA=GPIO21  SCL=GPIO22  clock=100 kHz
  address   : 0x4B (DI=1) หรือ 0x4A (DI=0) -- สแกนหาเอง
  ...

### STAGE 1-2 : ตรวจอุปกรณ์ก่อนเข้าโหมดทดสอบ ###

[01] DEVICE DETECT -- ค้นหา BNO08x บนบัส I2C
     สแกน address 0x01..0x7E
       พบอุปกรณ์ที่ 0x4B   <-- BNO08x (ขา DI = 1, ค่าจากโรงงาน)
     -> PASS  (addr=0x4B ACK, พบทั้งหมด 1 ตัว)

[02] IDENTITY -- Product ID (0xF8) และตรวจว่าเป็นของแท้
     reset cause      = 1  (Power on reset)
     firmware version = 3.2.17
     SW part number   = 10003606   (10003606 = SH-2 image ของ BNO085/086)
     verifyChip()     = OK - genuine BNO08x factory firmware
     -> PASS  (BNO08x แท้ fw 3.2.17 part=10003606)
```

ถ้า **STAGE 1 หรือ 2 ไม่ผ่าน โปรแกรมจะไม่เข้าโหมด RUN TEST** และพิมพ์ FAIL
พร้อมรายการสิ่งที่ต้องตรวจ (สาย, pull-up, ไฟเลี้ยง, ขา `BT`)

เมื่อผ่านด่านตรวจอุปกรณ์แล้วจะไล่ทดสอบต่ออีก 22 หัวข้อ:

| # | หัวข้อ | ตรวจอะไร |
|---|---|---|
| 03 | `SERIAL_NUMBER` | เลขซีเรียลจากโรงงานใน FRS `0x4B4B` |
| 04 | `FRS_METADATA` | metadata ของ Rotation Vector — ต้องเจอคู่ Q point 14/12 |
| 05 | `OSCILLATOR` | ชนิดออสซิลเลเตอร์ (command 10) |
| 06 | `ERROR_QUEUE` | คิว error ภายในชิป (command 1) |
| 07 | `SOFT_RESET` | reset ผ่าน executable channel แล้วชิปกลับมา |
| 08 | `FEATURE_CONFIG` | Set Feature แล้วอ่านกลับด้วย Get Feature |
| 09 | `ACCELEROMETER` | ขนาดเวกเตอร์ต้องใกล้ 9.81 m/s² |
| 10 | `GYROSCOPE` | วางนิ่งแล้วต้องใกล้ศูนย์ |
| 11 | `MAGNETOMETER` | ต้องอยู่ในช่วงสนามแม่เหล็กโลก |
| 12 | `ROTATION_VECTOR` | fusion 9 แกน, \|q\| ต้องเท่ากับ 1 |
| 13 | `GAME_RV` | fusion 6 แกน ไม่ใช้แม่เหล็ก |
| 14 | `GEOMAGNETIC_RV` | accel + mag ประหยัดพลังงาน |
| 15 | `GRAVITY_LINACC` | แยกแรงโน้มถ่วงออกจากความเร่งจริง |
| 16 | `GYRO_INTEGRATED_RV` | quaternion อัตราสูงบนช่อง SHTP 5 |
| 17 | `RAW_REPORTS` | ค่า ADC ดิบ + อุณหภูมิไดของไจโร |
| 18 | `UNCALIBRATED` | gyro/mag ที่ยังไม่หัก bias พร้อมค่า bias |
| 19 | `MULTI_REPORT_RATE` | เปิด 3 sensor พร้อมกันแล้ววัดอัตราจริง |
| 20 | `EVENT_ENGINES` | step / stability / activity / tap / shake / gesture 13 ตัว |
| 21 | `CALIBRATION_CMD` | คำสั่ง ME calibrate และ stop |
| 22 | `TARE_CMD` | tare แกน Z แล้วล้างค่ากลับ |
| 23 | `SLEEP_WAKE` | sleep แล้วปลุกกลับผ่าน executable channel |
| 24 | `STREAM_INTEGRITY` | timestamp และ sequence number ไม่ตกหล่น |

ปิดท้ายด้วย **รายงานสรุป** ที่อ่านจบในหน้าจอเดียว ไม่ต้องเลื่อนขึ้นไปอ่านย้อน

```
==============================================================
  รายงานผลการทดสอบ  /  TEST REPORT
==============================================================
  [ อุปกรณ์ที่ตรวจพบ ]
    I2C address   : 0x4B   (ขา DI = HIGH)
    Chip          : BNO085 / BNO086 (SH-2 MotionEngine)
    Firmware      : 3.2.17
    Part number   : 10003606
    Build number  : 370
    Images        : 2 ชุด (ชิปแจ้งเฟิร์มแวร์มาหลายอิมเมจ ปกติ)
    Serial number : 0x0000000012AB34CD
    Reset cause   : Power on reset
    Authentic     : [✓] ของแท้ -- part number ตรงตารางเฟิร์มแวร์โรงงาน
                    (เป็นการตรวจระดับโปรโตคอล ไม่ใช่ลายเซ็นดิจิทัล)
--------------------------------------------------------------
  [ ผลรายฟังก์ชัน 24 หัวข้อ ]
    [✓] 01 DEVICE_DETECT       addr=0x4B ACK, พบทั้งหมด 1 ตัว
    [✓] 02 IDENTITY            BNO08x แท้ fw 3.2.17 part=10003606
    [✓] 03 SERIAL_NUMBER       serial=0x0000000012AB34CD
    ...
    [!] 11 MAGNETOMETER        |B|=112.40 uT อยู่นอกช่วงสนามโลก
    ...
    [✓] 24 STREAM_INTEGRITY    120 report ต่อเนื่อง ไม่มีขาด
--------------------------------------------------------------
  [ สรุป ]
    [✓] ผ่าน      23 หัวข้อ
    [!] เตือน      1 หัวข้อ   (สภาพแวดล้อมตอนทดสอบ ไม่ใช่บอร์ดเสีย)
    [✗] ไม่ผ่าน    0 หัวข้อ
        รวมทั้งหมด 24 หัวข้อ
    หัวข้อที่เตือน   : 11-MAGNETOMETER
==============================================================
   #####    #     #####  #####
   #    #  # #   #      #
   #####  #####   ####   ####
   #      #   #       #      #
   #      #   #  #####  #####

  >>> บอร์ดนี้ผ่านการทดสอบทุกหัวข้อ พร้อมส่งมอบ <<<
==============================================================
```

`[✓]` = ผ่าน · `[!]` = เตือน · `[✗]` = ไม่ผ่าน

พิมพ์ `r` แล้ว Enter เพื่อทดสอบซ้ำโดยไม่ต้องกด reset

---

## บรรทัดสำหรับให้เว็บ parse

ทุกหัวข้อจะพิมพ์บรรทัดที่ขึ้นต้นด้วย `#` ตามหลังผลที่คนอ่าน
เว็บดึงไปทำไฟ PASS/FAIL ได้เลย

```
#RESULT,<ลำดับ>,<ชื่อหัวข้อ>,<PASS|FAIL|WARN>,<รายละเอียด>
#DEVICE,<addr>,<fw version>,<part number>,<build>,<serial>,<GENUINE|GENUINE_UNKNOWN_FW|SUSPECT|NO_RESPONSE>
#VERDICT,<PASS|FAIL>,<จำนวน pass>,<จำนวน fail>,<จำนวน warn>
```

`#RESULT` ออกทันทีที่จบแต่ละหัวข้อ ส่วน `#DEVICE` กับ `#VERDICT` ออกท้ายสุดครั้งเดียว

ตัวอย่างบรรทัดจริง

```
#RESULT,1,DEVICE_DETECT,PASS,addr=0x4B ACK, พบทั้งหมด 1 ตัว
#DEVICE,0x4B,3.2.17,10003606,370,0x0000000012AB34CD,GENUINE
#VERDICT,PASS,23,0,1
```

ตัวอย่างโค้ดฝั่งเว็บ

```js
// line = หนึ่งบรรทัดที่อ่านได้จาก Web Serial
if (line.startsWith('#RESULT,')) {
  const [, idx, name, status, detail] = line.split(',');
  addRow(idx, name, status, detail);          // status = PASS | FAIL | WARN
} else if (line.startsWith('#DEVICE,')) {
  const [, addr, fw, part, build, serial, auth] = line.split(',');
  showDevice({ addr, fw, part, build, serial,
               genuine: auth.startsWith('GENUINE') });
} else if (line.startsWith('#VERDICT,')) {
  const [, verdict, pass, fail, warn] = line.split(',');
  showBadge(verdict === 'PASS');              // ไฟเขียว / ไฟแดง
  showCounts({ pass, fail, warn });
}
```

---

## หมายเหตุเรื่องผล WARN

หัวข้อที่ขึ้น **WARN** ส่วนใหญ่ขึ้นกับสภาพแวดล้อมตอนทดสอบ ไม่ใช่บอร์ดเสีย

| หัวข้อ | สาเหตุที่พบบ่อย |
|---|---|
| `SERIAL_NUMBER` | ชิปไม่ได้ถูกโปรแกรม record `0x4B4B` มาจากโรงงาน พบได้บ่อยและไม่กระทบการใช้งาน |
| `RAW_REPORTS` | เฟิร์มแวร์บางบิลด์ไม่ปล่อย raw report ออกทาง I2C ค่าที่ผ่าน fusion ยังครบ |
| `FEATURE_CONFIG` | ชิปตอบ Get Feature แต่ echo `interval = 0` ทั้งที่ส่งรายงานจริง |
| `IDENTITY` | part number ของอิมเมจใหม่กว่าตารางในไลบรารี |
| `CALIBRATION_CMD` | MotionEngine ตอบ status ไม่เป็นศูนย์ |
| `STREAM_INTEGRITY` | timestamp สลับลำดับเพราะ `loop()` ถูกงานอื่นแย่งไป ไม่ใช่ข้อมูลตกหล่น |
| `ACCELEROMETER` | บอร์ดถูกขยับระหว่างวัด ทำให้ขนาดเวกเตอร์ไม่ใช่ 1 g |
| `GYROSCOPE` | บอร์ดถูกขยับ หรือไจโรยังไม่ผ่าน calibration |
| `MAGNETOMETER` | มีเหล็ก ลำโพง มอเตอร์ หรือสายไฟกระแสสูงอยู่ใกล้ |
| `GEOMAGNETIC_RV` | เฟิร์มแวร์บางรุ่นให้อัตราต่ำมาก ต้องรอนานกว่านี้ |
| `GYRO_INTEGRATED_RV` | I2C ช้าหรือสายยาว ทำให้อัตราไม่ถึงเป้า |
| `EVENT_ENGINES` | ไม่ได้เคาะ ไม่ได้เขย่า เอนจินจึงไม่มีเหตุการณ์ให้รายงาน |
| `SLEEP_WAKE` | ชิปใช้เวลาตื่นนานกว่า 700 ms |

**เกณฑ์ตัดสิน:** ผ่านก็ต่อเมื่อ **ไม่มีหัวข้อไหนขึ้น FAIL** เลย

---

## สร้างไฟล์เหล่านี้ใหม่เอง

```bash
cd PlatformIO
cp examples/16_FactoryTest/main.cpp src/main.cpp
pio run -e esp32dev
# ผลลัพธ์อยู่ที่ .pio/build/esp32dev/
#   firmware.factory.bin  = merged-firmware.bin (แฟลชที่ 0x0)
#   bootloader.bin / partitions.bin / firmware.bin
# ส่วน boot_app0.bin มาจาก
#   ~/.platformio/packages/framework-arduinoespressif32/tools/partitions/boot_app0.bin
```
