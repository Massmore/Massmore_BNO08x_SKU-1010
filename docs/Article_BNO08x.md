# คู่มือ Massmore Halley BNO085/BNO086 9-DOF IMU ฉบับจับมือทำ

อ่านค่า Quaternion, Roll/Pitch/Yaw, acceleration, gyroscope, magnetometer และ motion/activity reports ด้วย ESP32 ผ่าน Arduino IDE และ PlatformIO โดยใช้ library Massmore_BNO08x

> **เหมาะสำหรับ:** ผู้เริ่มต้น, Maker, นักศึกษา, Embedded Engineer, robotics, AR/VR, wearable, motion tracking และระบบที่ต้องการ orientation แบบ real time

> **เวลาทำ Quick Start:** ประมาณ 15–30 นาที หากติดตั้ง Arduino IDE หรือ PlatformIO ไว้แล้ว

### ตำแหน่งรูปภาพ 01 — Product overview

ใช้ Block: Upload — ภาพโมดูล Massmore Halley BNO085/BNO086 มุมบน เห็นชื่อขา, Qwiic connector และขนาด board ชัดเจน

## สารบัญ

1. Overview
1. Features
1. Pinout
1. Introduction และ Spec
1. BNO085 vs BNO086
1. Arduino ESP32 Example
1. PlatformIO Example
1. Examples, Calibration และ Tare
1. Troubleshooting
1. Resources: GitHub, Document และ Datasheet

## 1. Overview

BNO085 และ BNO086 เป็น 9-DOF IMU ที่รวม sensor และ processing ส่วนสำคัญไว้ใน package เดียว ได้แก่

- 3-axis accelerometer — วัด acceleration และ gravity direction
- 3-axis gyroscope — วัด angular velocity
- 3-axis magnetometer — วัด magnetic field และช่วยหา heading
- ARM Cortex-M0+ MCU ภายใน chip
- CEVA SH-2 MotionEngine firmware สำหรับ sensor fusion

ความต่างจาก IMU ทั่วไปคือ BNO08x ประมวลผล sensor fusion ภายในตัวเอง แล้วส่งผลลัพธ์ที่พร้อมใช้งาน เช่น quaternion, rotation vector, gravity และ linear acceleration ให้ host MCU โดยตรง จึงลดภาระ ESP32 และลดเวลาที่ต้องใช้ในการออกแบบ/จูนฟิลเตอร์

งานวิจัยด้าน inertial navigation อธิบายตรงกันว่า การอินทิเกรต gyroscope ให้ผลดีในช่วงสั้นแต่เกิด drift เมื่อเวลาผ่านไป จึงต้องอาศัย accelerometer, magnetometer หรือแบบจำลองอื่นช่วยแก้ค่า การใช้ quaternion ยังเหมาะกับการคำนวณ orientation 3 มิติมากกว่า Euler angles เพราะไม่เจอ singularity แบบ gimbal lock ดูพื้นฐานเพิ่มเติมได้จาก Kok, Hol และ Schön (2017) https://arxiv.org/abs/1704.06053 และ Sabatini (2011) https://doi.org/10.3390/s111009182

ไลบรารี Massmore_BNO08x ทำหน้าที่สื่อสาร SH-2/SHTP ให้ทั้งหมด รองรับ I2C, SPI, UART-SHTP และ UART-RVC พร้อมตัวอย่าง 14 ชุดสำหรับ Arduino IDE และ PlatformIO

> **Tip:** ถ้าเพิ่งเริ่ม ให้ใช้ I2C + Rotation Vector ที่ 100 Hz ก่อน อย่าเริ่มด้วย SPI หรือ 1 kHz เพราะการตรวจ wiring และ troubleshooting จะยากกว่า

### 1.1 Features

#### Orientation และ Rotation Vector

| Output | ข้อมูลที่ใช้ | เรตสูงสุด | เหมาะกับงาน |
| --- | --- | --- | --- |
| Rotation Vector | Accel + Gyro + Mag | 400 Hz | เข็มทิศ, หุ่นยนต์ที่ต้องรู้ heading |
| Game Rotation Vector | Accel + Gyro | 400 Hz | VR, gimbal, งานใกล้มอเตอร์ |
| Geomagnetic Rotation Vector | Accel + Mag | 90 Hz | งานประหยัดไฟและเคลื่อนที่ไม่เร็ว |
| AR/VR Stabilized Rotation Vector | 9-axis fusion | 400 Hz | ภาพ 3 มิติที่ต้องการความนิ่ง |
| AR/VR Stabilized Game RV | 6-axis fusion | 400 Hz | AR/VR ในพื้นที่สนามแม่เหล็กรบกวน |
| Gyro-Integrated Rotation Vector | Gyro integration | 1000 Hz | rendering, head tracking, control loop อัตราสูง |

อัตรารายงานสูงสุดมาจาก CEVA BNO08X Datasheet v1.17 https://www.ceva-ip.com/wp-content/uploads/BNO080_085-Datasheet.pdf และเป็นค่าสูงสุดราย report ไม่ได้หมายความว่าจะเปิดทุก report ที่ค่าสูงสุดพร้อมกันได้

#### Motion data

| Output | หน่วยในไลบรารี | เรตสูงสุด |
| --- | --- | --- |
| Accelerometer | m/s² | 500 Hz |
| Gyroscope | rad/s หรือใช้ helper เป็น °/s | 400 Hz |
| Magnetometer | µT | 100 Hz |
| Linear Acceleration | m/s², ตัด gravity แล้ว | 400 Hz |
| Gravity Vector | m/s² | 400 Hz |
| Raw Accelerometer/Gyroscope/Magnetometer | counts พร้อม timestamp | ขึ้นกับ report |

#### Activity และ gesture detection

- Step Counter และ Step Detector
- Personal Activity Classifier
- Stability Classifier และ Stability Detector
- Tap และ Double Tap
- Shake, Flip, Pickup, Tilt, Pocket และ Circle Detector
- Significant Motion Detector
- Sleep Detector

#### System features

- Dynamic calibration และ Save DCD ลง flash
- Tare เพื่อกำหนดทิศ “หน้า” ใหม่ และ persist ได้
- FRS (Flash Record System) read/write
- Product ID และ factory serial number
- Batching และ Report-on-change
- Wake sensor และ sleep/on mode
- Callback และอ่านหลาย report ในรอบเดียว
- ไม่มี external dependency และไม่ใช้ malloc/new

### 1.2 Pinout

### ตำแหน่งรูปภาพ 02 — Pinout

ใช้ Block: Upload — Pinout ของ Massmore Halley BNO08x พร้อมลูกศรชี้ VIN, GND, SDA, SCL, INT, RST, SA0, PS0, PS1, CS และ Qwiic

#### หน้าที่ของขา

ชื่อบนบอร์ดอาจต่างกันเล็กน้อยตาม revision ให้ยึด silkscreen และ schematic ของโมดูลจริงเป็นหลัก

| ชื่อขา | หน้าที่ | หมายเหตุ |
| --- | --- | --- |
| VIN/VCC | ไฟเลี้ยงโมดูล | แนะนำ 3.3V สำหรับเริ่มต้น |
| GND | กราวด์ | ต้องต่อกราวด์ร่วมกับ ESP32 |
| SDA / H_SDA / MISO / TX | I2C data, SPI MISO หรือ UART TX | หน้าที่ขึ้นกับโหมด |
| SCL / H_SCL / SCK / RX | I2C clock, SPI clock หรือ UART RX | หน้าที่ขึ้นกับโหมด |
| SA0 / H_MOSI | เลือก I2C address หรือ SPI MOSI | อ่าน SA0 ตอน reset |
| CS / H_CSN | SPI chip select | active-low |
| INT / H_INTN | แจ้ง host ว่ามีข้อมูล | active-low; แนะนำให้ต่อ |
| RST / NRST | รีเซ็ตเซ็นเซอร์ | active-low; แนะนำให้ต่อ |
| PS0 / WAKE | เลือกโปรโตคอล และใช้ปลุกใน SPI | อ่านโหมดตอน reset |
| PS1 | เลือกโปรโตคอล | อ่านโหมดตอน reset |
| BOOTN | เข้า bootloader เมื่อ LOW ตอน reset | ปกติดึง HIGH ผ่าน 10 kΩ |

#### การเลือกโหมดด้วย PS1/PS0

| PS1 | PS0 | Host interface |
| --- | --- | --- |
| LOW | LOW | I2C |
| LOW | HIGH | UART-RVC |
| HIGH | LOW | UART-SHTP |
| HIGH | HIGH | SPI |

ขา PS1/PS0 ถูกอ่านตอน reset การเปลี่ยนจัมเปอร์ระหว่างทำงานจะไม่เปลี่ยนโหมดจนกว่าจะ reset ใหม่ สำหรับ SPI ขาทั้งสองต้อง HIGH ตั้งแต่ก่อน reset จน H_INTN ทำงานครั้งแรก จากนั้น PS0 กลายเป็นขา WAKE ตามข้อกำหนดใน BNO08X Datasheet https://www.ceva-ip.com/wp-content/uploads/BNO080_085-Datasheet.pdf

#### I2C address

| SA0 ตอน reset | 7-bit address |
| --- | --- |
| LOW | 0x4A |
| HIGH | 0x4B |

> **สำคัญสำหรับ Massmore Halley:** หน้าสินค้าระบุ address เริ่มต้น 0x4A แต่ macro MASSMORE_BNO08X_I2C_ADDR_DEF ในไลบรารีเวอร์ชัน 1.0.1 มีค่า 0x4B คู่มือนี้จึงใช้ 0x4A แบบ explicit หากไม่พบอุปกรณ์ค่อยลอง 0x4B

#### แรงดันไฟและ logic level

หน้าสินค้า Massmore ระบุแรงดันโมดูล 3.3–5V แต่ตัวชิป BNO08x เองกำหนด VDD 2.4–3.6V และ VDDIO 1.7–3.6V ความสามารถรับ 5V จึงต้องมาจาก regulator/level shifter บนบอร์ดโมดูล

แนวทางที่ปลอดภัย:

1. ต่อ VIN/VCC กับ 3V3 ของ ESP32
1. ใช้ pull-up ของ SDA/SCL ไปที่ 3.3V
1. ห้ามป้อน 5V เข้าขา logic
1. หากจะใช้ไฟ 5V ให้ตรวจ schematic/revision ของโมดูลก่อน

> **ทริป:** Qwiic ใช้ไฟและ logic 3.3V เป็นหลัก จึงเหมาะกับ ESP32 โดยตรง

## 2. Introduction

### Sensor fusion ทำงานอย่างไร

เซ็นเซอร์ทั้งสามชนิดมีจุดเด่นและจุดอ่อนต่างกัน:

| เซ็นเซอร์ | จุดเด่น | ข้อจำกัด |
| --- | --- | --- |
| Accelerometer | รู้ทิศ gravity และวัดความเร่ง | ไวต่อแรงสั่น/การเคลื่อนที่ และบอกทิศเหนือไม่ได้ |
| Gyroscope | ตอบสนองเร็วและลื่น | อินทิเกรตแล้วเกิด drift สะสม |
| Magnetometer | อ้างอิงสนามแม่เหล็กสำหรับ heading | ถูกรบกวนจากมอเตอร์ เหล็ก แม่เหล็กและสายกระแสสูง |

MotionEngine รวมข้อมูลเหล่านี้เพื่อให้ได้ orientation ที่เสถียรกว่าการใช้ sensor ใด sensor หนึ่ง เมื่อสภาพแวดล้อมมีสนามแม่เหล็กรบกวน ให้เปลี่ยนจาก Rotation Vector เป็น Game Rotation Vector เพื่อหลีกเลี่ยงการกระโดดของ yaw แลกกับการยอมให้ yaw drift ช้า ๆ

### Quaternion กับ Euler angles

เซ็นเซอร์ส่ง orientation ในรูป quaternion (i, j, k, real) เป็นหลัก ไลบรารีมี helper แปลงเป็น Euler angles:

```cpp
massmore_quat_t q = imu.getQuaternion();
massmore_euler_t e = imu.getEulerDeg();
 
Serial.println(e.roll);   // หมุนรอบแกน X
Serial.println(e.pitch);  // หมุนรอบแกน Y
Serial.println(e.yaw);    // หมุนรอบแกน Z ช่วง -180..180
Serial.println(imu.getHeadingDeg()); // 0..360
```

> **ทริป:** เก็บ/ส่ง/คำนวณภายในด้วย quaternion แล้วแปลงเป็น Euler เฉพาะตอนแสดงผล จะลดปัญหา gimbal lock และการ interpolate มุมข้าม ±180°

### 2.1 Spec

#### ข้อมูลทั่วไปของชิป

| รายการ | ค่า |
| --- | --- |
| ประเภท | 9-DOF Sensor Fusion SiP |
| เซ็นเซอร์ | 3-axis accelerometer + gyroscope + magnetometer |
| Processor ภายใน | 32-bit ARM Cortex-M0+ |
| Firmware | CEVA SH-2 MotionEngine |
| Package ชิป | 28-pin LGA, 5.2 × 3.8 × 1.1 mm |
| VDD | 2.4–3.6 V |
| VDDIO | 1.7–3.6 V |
| Operating temperature | −40 ถึง +85 °C |
| I2C | 100/400 kbit/s, address 0x4A หรือ 0x4B |
| SPI | 4-wire, Mode 3, สูงสุด 3 MHz |
| UART-SHTP | 3 Mbit/s, 8N1 |
| UART-RVC | 115200 8N1, stream 100 Hz |

ค่าด้านบนเป็นสเปกชิป ไม่ใช่ขนาด/แรงดันภายนอกของ breakout board โปรดตรวจหน้าสินค้าและ schematic ของโมดูลประกอบด้วย

#### Performance ของ sensor fusion

| Output | เงื่อนไข | ค่าตาม datasheet |
| --- | --- | --- |
| Rotation Vector | Dynamic rotation error | 3.5° |
| Rotation Vector | Static rotation error | 2.0° |
| Game Rotation Vector | Dynamic non-heading error | 2.5° |
| Game Rotation Vector | Static non-heading error | 1.5° |
| Game Rotation Vector | Dynamic heading drift | 0.5°/min |
| Geomagnetic RV | Dynamic rotation error | 4.5° |
| Gravity | Static angle error | 1.5° |
| Linear Acceleration | Dynamic accuracy | 0.35 m/s² |
| Accelerometer | Dynamic accuracy | 0.3 m/s² |
| Gyroscope | Dynamic accuracy | 3.1°/s |
| Magnetometer | Dynamic accuracy | 1.4 µT |

CEVA ระบุเพิ่มเติมว่าการใช้งานจริง Rotation Vector มักอยู่ราว 5° และ Geomagnetic Rotation Vector ราว 10° เพราะขึ้นกับสภาพสนามแม่เหล็ก จึงควรออกแบบ tolerance จากค่าการใช้งานจริง ไม่ยึดเพียงค่าจำลองในตาราง

#### Latency

| Output | 100 Hz | 200 Hz |
| --- | --- | --- |
| Gyro Rotation Vector | 6.6 ms | 3.7 ms |
| Rotation Vector | 6.6 ms | 3.7 ms |
| Game Rotation Vector | 6.6 ms | 3.7 ms |

#### การใช้พลังงานตัวอย่าง — BNO086

วัดด้วย SPI ที่ VDDIO 3.0V และ VDD 3.3V:

| Configuration | VDDIO current | VDD current | Power |
| --- | --- | --- | --- |
| Idle | 0.047 mA | 0.01 mA | 0.17 mW |
| 9-axis fusion 100 Hz | 3.18 mA | 7.50 mA | 34.29 mW |
| 9-axis fusion 400 Hz | 6.55 mA | 7.50 mA | 44.40 mW |
| Gyro RV 1000 Hz | 6.84 mA | 7.50 mA | 45.27 mW |
| Accelerometer 125 Hz | 0.98 mA | 0.15 mA | 3.44 mW |
| Step counter 31.25 Hz | 0.36 mA | 0.14 mA | 1.54 mW |

#### BNO085 vs BNO086

BNO086 เป็น drop-in replacement ของ BNO085: pinout, host interfaces และ shared SH-2 reports เหมือนกัน จึงใช้ wiring และ Massmore_BNO08x API ชุดเดียวกันได้ BNO086 เพิ่ม 14-bit accelerometer fusion, ลด idle power และรองรับ Interactive Calibration ตาม CEVA BNO08X Datasheet https://www.ceva-ip.com/wp-content/uploads/BNO080_085-Datasheet.pdf

| หัวข้อ | BNO085 | BNO086 | ใช้งานต่างกันอย่างไร |
| --- | --- | --- | --- |
| Pinout และ shared reports | รองรับ | รองรับ | ใช้ wiring, begin() และ report API เดียวกัน |
| Accelerometer fusion | standard resolution | 14-bit accelerometer fusion | BNO086 ได้ resolution ใน fusion path เพิ่มขึ้นโดยไม่ต้องเปลี่ยน code |
| Idle power ตาม CEVA test condition | ประมาณ 0.39 mW | ประมาณ 0.17 mW | BNO086 เหมาะกับระบบที่ idle นาน แต่ต้องวัด power ของ module จริงอีกครั้ง |
| Dynamic Calibration | รองรับ | รองรับ | ใช้ example 05_Calibration, calibrateAll() และ saveCalibration() เหมือนกัน |
| Interactive Calibration | ไม่มี | เพิ่มเข้ามา | Library v1.0.1 ยังไม่มี dedicated method สำหรับ feature นี้ |
| UART-RVC Motion Intent/Request | reserved | รองรับ | ใช้ example 12_UART_RVC แล้วอ่าน motionIntent และ motionRequest |

สำหรับ Rotation Vector, Game Rotation Vector, acceleration, gyroscope, magnetometer, Step Counter, Calibration และ Tare ไม่ต้องเขียน code แยกตามรุ่น หาก application ต้องใช้ BNO086-only reports ให้ตรวจ API ของ library ก่อน เพราะ version 1.0.1 มี protocol IDs สำหรับ Motion Request, Optical Flow และ Dead Reckoning Pose แต่ยังไม่มี typed high-level enable/get API สำหรับ Optical Flow และ Dead Reckoning Pose

### ตำแหน่งรูปภาพ 03 — BNO085/BNO086 comparison

ใช้ Block: Upload — ภาพ BNO085 และ BNO086 วางคู่กัน พร้อม callout “Same wiring/API” และ “BNO086: 14-bit fusion, lower idle power, Interactive Calibration”

## 3. User Guides Step by step

### อุปกรณ์ที่ต้องเตรียม

| รายการ | จำนวน | หมายเหตุ |
| --- | --- | --- |
| Massmore Halley BNO085/BNO086 | 1 | เลือกรุ่นตามงาน |
| ESP32 development board | 1 | ตัวอย่างใช้ ESP32 Dev Module |
| สาย USB data | 1 | สายชาร์จอย่างเดียวอัปโหลดไม่ได้ |
| สายจัมเปอร์ | 6 | VIN, GND, SDA, SCL, INT, RST |
| Breadboard | 1 | ไม่บังคับ |

### ตำแหน่งรูปภาพ 04 — Hardware preparation

ใช้ Block: Upload — ภาพ Massmore Halley, ESP32, USB cable และ jumper/Qwiic cable ก่อนเริ่ม wiring

### ต่อวงจร I2C

### ตำแหน่งรูปภาพ 05 — I2C wiring

ใช้ Block: Upload — Wiring diagram: Massmore Halley BNO08x กับ ESP32 Dev Module พร้อม label GPIO 21, 22, 4 และ 5

| BNO08x | ESP32 | จำเป็น |
| --- | --- | --- |
| VIN/VCC | 3V3 | ✓ |
| GND | GND | ✓ |
| SDA | GPIO 21 | ✓ |
| SCL | GPIO 22 | ✓ |
| INT | GPIO 4 | แนะนำ |
| RST | GPIO 5 | แนะนำ |

ตรวจอีกครั้งก่อนเสียบ USB:

- ☐ VIN ต่อ 3V3 ไม่ใช่ 5V
- ☐ GND ต่อถึงกัน
- ☐ SDA/SCL ไม่สลับ
- ☐ เลขพินในโค้ดตรงกับ GPIO จริง
- ☐ ไม่มีสายหลวม/เส้นทองแดงแตะกัน

> **ทริป:** ถ้าใช้ Qwiic ตรวจลำดับสีสายจาก pinout ของสาย/บอร์ด อย่าเชื่อสีอย่างเดียว เพราะสาย third-party บางชุดจัดสีไม่เหมือนมาตรฐาน

### 3.1 Arduino ESP32 — Example

#### Step 1 — ติดตั้ง Arduino IDE

ดาวน์โหลด Arduino IDE 2 จาก Arduino Software https://www.arduino.cc/en/software แล้วติดตั้งตามระบบปฏิบัติการ หากต้องการรายละเอียดแบบเป็นทางการดู Download and install Arduino IDE https://support.arduino.cc/hc/en-us/articles/360019833020-Download-and-install-Arduino-IDE

#### Step 2 — ติดตั้ง ESP32 board package

1. เปิด File > Preferences บน Windows/Linux หรือ Arduino IDE > Settings บน macOS
1. ใส่ URL ต่อไปนี้ใน Additional Boards Manager URLs

```text
https://espressif.github.io/arduino-esp32/package_esp32_index.json
```

1. เปิด Tools > Board > Boards Manager...
1. ค้นหา esp32
1. ติดตั้ง esp32 by Espressif Systems
1. ปิด/เปิด Arduino IDE ใหม่

อ้างอิงขั้นตอนปัจจุบันจาก Espressif Arduino-ESP32 installation guide https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html

#### Step 3 — ติดตั้งไลบรารี Massmore_BNO08x

วิธี Add ZIP:

1. ดาวน์โหลดรีโพซิทอรีเป็น ZIP
1. แตก ZIP หลัก
1. บีบอัดเฉพาะโฟลเดอร์ ArduinoIDE/ เป็น ZIP ใหม่
1. ตรวจใน ZIP ว่ามี library.properties, src/ และ examples/ ที่ระดับบน
1. เปิด Sketch > Include Library > Add .ZIP Library...
1. เลือก ZIP ที่สร้างในข้อ 3
1. รอข้อความติดตั้งสำเร็จ

Arduino รองรับทั้ง Add ZIP และการวางไลบรารีใน sketchbook ตาม Arduino Help Center https://support.arduino.cc/hc/en-us/articles/5145457742236-Install-libraries-in-the-Arduino-IDE

### ตำแหน่งรูปภาพ 06 — Arduino IDE library installation

ใช้ Block: Upload — Screenshot เมนู Add .ZIP Library และตำแหน่ง File > Examples > Massmore_BNO08x

#### Step 4 — เปิดตัวอย่างแรก

เปิด File > Examples > Massmore_BNO08x > 01_BasicRotationVector

แก้ส่วนกำหนดพิน:

```cpp
#define SDA_PIN 21
#define SCL_PIN 22
#define INT_PIN 4
#define RST_PIN 5
```

จากนั้นแก้ address สำหรับ Massmore Halley เป็น 0x4A:

```cpp
if (!imu.begin(0x4A, Wire, INT_PIN, RST_PIN)) {
```

และเริ่ม I2C ที่ 100 kHz:

```cpp
Wire.begin(SDA_PIN, SCL_PIN);
Wire.setClock(100000);
```

#### Step 5 — เลือก board และ port

1. Tools > Board > esp32 > ESP32 Dev Module หรือรุ่นที่ตรงกับบอร์ด
1. Tools > Port > เลือกพอร์ตที่เพิ่มขึ้นหลังเสียบ USB
1. ตั้ง Upload Speed ตามที่บอร์ดรองรับ เริ่มที่ 460800 หรือ 921600 หากอัปโหลดไม่เสถียรให้ลดเป็น 115200

#### Step 6 — Verify, Upload และเปิด Serial Monitor

1. กด Verify เพื่อตรวจ compile
1. กด Upload
1. หากบอร์ดค้างที่ Connecting... ให้กดปุ่ม BOOT ค้าง แล้วปล่อยเมื่อเริ่มเขียน flash
1. เปิด Serial Monitor
1. ตั้ง baud rate 115200

ผลที่ใช้ยืนยันการทำงานคือ terminal แสดง BNO08x connected และค่า quaternion เปลี่ยนต่อเนื่องเมื่อหมุน board

### ตำแหน่งรูปภาพ 07 — ผลทดสอบ Arduino IDE

ใช้ Block: Upload — Screenshot Serial Monitor 115200 baud หลัง Upload สำเร็จ โดยให้เห็นข้อความเชื่อมต่อและค่า quaternion หรือ Roll/Pitch/Yaw

### พื้นที่สำหรับผลทดสอบ Arduino IDE

> **เว้นพื้นที่สำหรับผลทดสอบจริง:** เพิ่ม screenshot, Serial Monitor หรือ terminal output และข้อสรุปจาก hardware จริงในตำแหน่งนี้

#### Step 7 — อ่านเป็น Roll/Pitch/Yaw

เปิดตัวอย่าง 02_EulerAngles หรือใช้โค้ดฉบับสมบูรณ์นี้:

```cpp
#include <Wire.h>
#include <Massmore_BNO08x.h>
 
constexpr int SDA_PIN = 21;
constexpr int SCL_PIN = 22;
constexpr int INT_PIN = 4;
constexpr int RST_PIN = 5;
constexpr uint8_t BNO_ADDR = 0x4A;
 
MassmoreBNO08x imu;
uint32_t lastPrintMs = 0;
 
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}
 
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
 
  imu.enableDebug(Serial); // ลบบรรทัดนี้เมื่อใช้งานจริงเพื่อลดข้อความ debug
 
  if (!imu.begin(BNO_ADDR, Wire, INT_PIN, RST_PIN)) {
 Serial.print("Init failed: ");
 Serial.println(MassmoreBNO08x::statusToString(imu.getLastError()));
 while (true) delay(100);
  }
 
  if (imu.enableRotationVector(10000) != MASSMORE_OK) {
 Serial.println("Cannot enable Rotation Vector");
 while (true) delay(100);
  }
 
  Serial.println("roll,pitch,yaw,heading,accuracy");
}
 
void loop() {
  imu.updateAll(16);
 
  if (!imu.hasNewReport(MASSMORE_SENSOR_ROTATION_VECTOR)) return;
  if (millis() - lastPrintMs < 100) return;
  lastPrintMs = millis();
 
  massmore_euler_t e = imu.getEulerDeg();
  massmore_accuracy_t a = imu.getAccuracy(MASSMORE_SENSOR_ROTATION_VECTOR);
 
  Serial.print(e.roll, 2); Serial.print(',');
  Serial.print(e.pitch, 2); Serial.print(',');
  Serial.print(e.yaw, 2); Serial.print(',');
  Serial.print(imu.getHeadingDeg(), 2); Serial.print(',');
  Serial.println(MassmoreBNO08x::accuracyToString(a));
}
```

> **Tip:** report อาจวิ่ง 100 Hz แต่ไม่จำเป็นต้องพิมพ์ Serial 100 ครั้ง/วินาที อ่านและ process ทุก packet ได้ แต่ลด print rate เหลือ 10 Hz เพื่อไม่ให้ terminal เป็น bottleneck

### 3.2 PlatformIO — Example

#### Step 1 — ติดตั้ง VS Code และ PlatformIO IDE

1. ติดตั้ง Visual Studio Code https://code.visualstudio.com/
1. เปิด Extensions
1. ค้นหา PlatformIO IDE
1. กด Install และรอ initialization เสร็จ
1. Reload VS Code หากมีคำแนะนำ

คู่มือ official: PlatformIO IDE for VS Code https://docs.platformio.org/en/latest/integration/ide/vscode.html

#### Step 2 — เปิดโปรเจกต์ที่แนบมา

เลือก File > Open Folder... แล้วเปิดโฟลเดอร์:

```text
Massmore_BNO08x/PlatformIO/
```

ต้องเปิด PlatformIO/ เป็น root เพื่อให้ VS Code พบ platformio.ini และ lib/Massmore_BNO08x/

#### Step 3 — เลือก environment

เปิด platformio.ini แล้วตั้งค่า:

```ini
[platformio]
default_envs = esp32dev
```

ตัวอย่าง environment ที่เตรียมไว้:

| บอร์ด | environment |
| --- | --- |
| ESP32 DevKit/WROOM-32 | esp32dev |
| ESP32-S3 DevKitC-1 | esp32-s3-devkitc-1 |
| ESP32-S2 Saola | esp32-s2-saola-1 |
| ESP32-C3 DevKitM-1 | esp32-c3-devkitm-1 |
| ESP32-C6 DevKitC-1 | esp32-c6-devkitc-1 |

#### Step 4 — ตรวจ src/main.cpp

ตั้งพินและเปลี่ยน address เป็น 0x4A สำหรับ Massmore Halley จากนั้นแนะนำให้ใช้:

```cpp
Wire.setClock(100000);
```

เมื่อทำงานเสถียรแล้วค่อยทดลอง 400 kHz หากต้องการ bandwidth เพิ่ม

#### Step 5 — Build, Upload, Monitor

กดไอคอน PlatformIO ตามลำดับ:

1. Build — ต้องขึ้น SUCCESS
1. Upload — ส่ง firmware เข้า ESP32
1. Monitor — เปิด serial monitor 115200 baud

หรือใช้ terminal ในโฟลเดอร์ PlatformIO/:

```bash
pio run
pio run -t upload
pio device monitor -b 115200
```

#### Step 6 — เปลี่ยนตัวอย่าง

ตัวอย่าง PlatformIO อยู่ใน:

```text
PlatformIO/examples/<example-name>/main.cpp
```

คัดลอกตัวอย่างที่ต้องการไปเป็น PlatformIO/src/main.cpp ในสำเนาโปรเจกต์ทำงานของคุณ แล้ว Build ใหม่ ควรมี setup() และ loop() เพียงชุดเดียวใน src/

> **Tip:** โฟลเดอร์ lib/Massmore_BNO08x เป็น project-local library จึงย้าย project ไปอีกเครื่องได้โดยไม่ต้องติดตั้ง library แยกใน global environment

### ตำแหน่งรูปภาพ 08 — PlatformIO Build และ Upload

ใช้ Block: Upload — Screenshot PlatformIO แสดง environment ที่เลือก และ terminal ขึ้น SUCCESS หลัง Build/Upload

### ตำแหน่งรูปภาพ 09 — ผลทดสอบ PlatformIO

ใช้ Block: Upload — Screenshot PlatformIO Monitor 115200 baud โดยใช้ firmware function เดียวกับ Arduino IDE เพื่อให้เปรียบเทียบผลได้ตรงกัน

### พื้นที่สำหรับผลทดสอบ PlatformIO

> **เว้นพื้นที่สำหรับผลทดสอบจริง:** เพิ่ม screenshot, Serial Monitor หรือ terminal output และข้อสรุปจาก hardware จริงในตำแหน่งนี้

### 3.3 Examples และการนำไปใช้งานจริง

| # | ตัวอย่าง | สิ่งที่จะได้ |
| --- | --- | --- |
| 01 | BasicRotationVector | quaternion 9-axis 100 Hz |
| 02 | EulerAngles | roll/pitch/yaw/heading |
| 03 | AccelGyroMag | motion vectors หลายชนิด |
| 04 | ChipIDVerify | Product ID, serial, FRS metadata |
| 05 | Calibration | calibration + Save DCD |
| 06 | Tare | recenter/persist orientation |
| 07 | StepCounterActivity | นับก้าวและจำแนกกิจกรรม |
| 08 | TapShakeDetector | gesture reports |
| 09 | RawSensorData | raw data สำหรับ logging/custom filter |
| 10 | HighRateGyroRV | quaternion อัตราสูง |
| 11 | SPI_Interface | SPI Mode 3 |
| 12 | UART_RVC | heading/acceleration 100 Hz |
| 13 | MultiReportAdvanced | callback, batching, sleep/wake |
| 14 | FRS_Records | อ่าน/เขียน flash records |

#### เลือก report ให้ตรงงาน

หุ่นยนต์เคลื่อนที่ที่ต้องรู้ทิศ: ใช้ Rotation Vector และ calibrate magnetometer หลังติดตั้งเซ็นเซอร์ในตัวหุ่นยนต์จริง วางให้ห่างมอเตอร์/สายแบตเตอรี่

แขนกลหรือ gimbal ใกล้มอเตอร์: ใช้ Game Rotation Vector เพราะไม่ใช้ magnetometer ค่า yaw จะ drift แต่ไม่กระโดดตามสนามแม่เหล็ก

AR/VR: เริ่มจาก AR/VR Stabilized Game RV หากต้องการ smooth orientation โดยไม่อ้างทิศเหนือ และใช้ Gyro-Integrated RV เมื่อระบบ rendering ต้องการข้อมูลถี่มาก

อุปกรณ์นับก้าว: เปิด Step Counter ที่เรตต่ำ ไม่จำเป็นต้องเปิด fusion 400 Hz ทำให้ประหยัดพลังงานกว่า

Robot Vacuum: UART-RVC ใช้สาย data ทางเดียวและ stream 100 Hz เหมาะกับ heading + acceleration แบบง่าย แต่ใช้ report อื่นและคำสั่ง calibration/tare ผ่าน stream ไม่ได้

### ตำแหน่งรูปภาพ 10 — Application example

ใช้ Block: Upload — ภาพตัวอย่างการติดตั้งใน robot, gimbal, wearable หรือ motion controller พร้อมระบุตำแหน่ง sensor และแกน X/Y/Z

### 3.4 Calibration และ Tare

#### Calibration ที่ถูกต้อง

ใช้ตัวอย่าง 05_Calibration และทำตาม CEVA Sensor Calibration Procedure https://www.ceva-ip.com/wp-content/uploads/2019/09/BNO080-BNO085-Sesnor-Calibration-Procedure.pdf

### ตำแหน่งรูปภาพ 11 — Calibration movement

ใช้ Block: Upload — ภาพลำดับการหมุน board รอบ Roll, Pitch และ Yaw รวมถึงท่าวางนิ่งสำหรับ Gyroscope calibration

1. เปิด Serial Monitor 115200 baud
1. พิมพ์ c เพื่อเปิด calibration ของ accel + gyro + mag
1. Magnetometer: หมุนบอร์ดประมาณ 180° แล้วกลับ รอบแกน roll, pitch และ yaw ใช้ประมาณ 2 วินาทีต่อแกน
1. Accelerometer: ถือบอร์ดในทิศที่ต่างกัน 4–6 ทิศ ค้างประมาณ 1 วินาทีต่อทิศ
1. Gyroscope: วางบอร์ดนิ่งบนพื้นมั่นคง 2–3 วินาที
1. ดู heading error/accuracy จนดีขึ้น
1. พิมพ์ s เพื่อ Save DCD ลง flash
1. พิมพ์ e หากต้องการปิด dynamic calibration หลังบันทึก

> **ข้อควรระวัง:** อย่าใช้ท่าเลข 8 เป็นขั้นตอนหลักโดยอัตโนมัติ: แนวทาง “แกว่งเป็นเลข 8” พบได้บ่อยในคู่มือ sensor อื่น แต่คู่มือ BNO08x ของ CEVA ระบุการหมุนไป-กลับรอบแต่ละแกนชัดเจน

#### สภาพแวดล้อมของ magnetometer

- ทำ calibration หลังติดตั้งเซ็นเซอร์ใน enclosure จริง
- ปิดหรือจัดตำแหน่งมอเตอร์/สายกระแสสูงให้เหมือนสภาวะใช้งาน
- หลีกเลี่ยงโต๊ะเหล็ก แม่เหล็ก ลำโพง และไขควงแม่เหล็ก
- ถ้า heading ยังไม่นิ่ง ให้เปรียบเทียบ Game Rotation Vector เพื่อแยกปัญหาแม่เหล็กออกจากปัญหาการสื่อสาร

งานวิจัยด้าน inertial/magnetic orientation แสดงว่าความผิดปกติของสนามแม่เหล็กมีผลโดยตรงต่อความน่าเชื่อถือของ heading และต้องอาศัยการชดเชย/weighting ที่เหมาะสม ดู Sabatini 2006 https://doi.org/10.1109/TBME.2006.875664 และ Magnetometer calibration using inertial sensors https://arxiv.org/abs/1601.05257

#### Tare — กำหนดทิศหน้าใหม่

ทำ calibration ให้เสร็จก่อน จากนั้นเปิด 06_Tare:

- z — ตั้ง yaw ปัจจุบันเป็นศูนย์ เหมาะกับปุ่ม Recenter
- a — ตั้งศูนย์ทุกแกน ต้องวางอุปกรณ์ระดับและหันไปทิศอ้างอิง
- p — บันทึก tare ให้คงอยู่หลังปิดเปิดเครื่อง
- c — ล้าง tare ที่บันทึกไว้

อ่านข้อจำกัดและ basis ของ tare เพิ่มเติมใน CEVA Tare Function Usage Guide https://www.ceva-ip.com/wp-content/uploads/BNO080-BNO085-Tare-Function-Usage-Guide.pdf

### 3.5 Troubleshooting

#### อาการ 1 — BNO08x not found

ทำตามลำดับนี้:

1. ถอด USB แล้วตรวจ VIN=3.3V, GND, SDA, SCL
1. ตรวจพิน ESP32 ในโค้ด
1. ใช้ address 0x4A สำหรับ Massmore Halley
1. หากยังไม่พบ ลอง 0x4B
1. ลด I2C เป็น 100 kHz
1. ต่อ INT และ RST
1. reset ใหม่หลังเปลี่ยน SA0/PS0/PS1
1. ตรวจ BOOTN ต้องไม่ LOW ตอน reset

I2C scanner แบบสั้น:

```cpp
#include <Wire.h>
 
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  Wire.setClock(100000);
 
  for (uint8_t addr = 1; addr < 127; ++addr) {
 Wire.beginTransmission(addr);
 if (Wire.endTransmission() == 0) {
   Serial.printf("Found I2C device at 0x%02X\n", addr);
 }
  }
}
 
void loop() {}
```

#### อาการ 2 — สแกนเจอแต่ begin() ไม่ผ่าน

- ตรวจว่าเป็น BNO08x จริง ไม่ใช่ BNO055 ซึ่งใช้โปรโตคอลคนละแบบ
- เปิด imu.enableDebug(Serial); ก่อน begin()
- ต่อ INT/RST และรอ reset ให้ครบ
- หลีกเลี่ยงการ poll เร็วด้วย loop ที่บล็อก bus
- ตรวจ pull-up SDA/SCL ไป 3.3V; datasheet แนะนำค่าทั่วไป 2–4 kΩ โดยขึ้นกับ bus capacitance

#### อาการ 3 — ข้อมูลหยุดเมื่อเพิ่ม report

- ลดจำนวน report
- ลด report rate
- เรียก updateAll() ให้ถี่
- อย่าใช้ delay() ยาว
- ลดการพิมพ์ Serial
- ใช้ SPI สำหรับ bandwidth สูง

#### อาการ 4 — yaw กระโดดเมื่อมอเตอร์ทำงาน

นี่มักเป็น magnetic interference ไม่ใช่ bug ของ I2C:

1. เปลี่ยนเป็น Game Rotation Vector เพื่อทดสอบ
1. ย้ายเซ็นเซอร์ให้ไกลมอเตอร์ สายเฟส สายแบตเตอรี่ และโครงเหล็ก
1. บิดคู่สายกระแสสูงและลด loop area
1. calibrate ใหม่หลังประกอบจริง
1. หากงานต้องการทิศเหนือมาก ให้ประเมินการใช้อ้างอิงภายนอกเพิ่มเติม

#### อาการ 5 — มุมกลับแกนหรือหมุนผิดทิศ

- ตรวจ orientation ของ PCB และแกน X/Y/Z บนรูป pinout
- อย่าสลับ roll/pitch/yaw ด้วยการเดา
- ถ้าติดตั้งหมุนถาวร ให้ใช้ System Orientation FRS หรือแปลง quaternion อย่างเป็นระบบ
- ทดสอบทีละแกน: วางนิ่ง แล้วหมุน +90° รอบแกนที่รู้แน่นอน

#### อาการ 6 — PlatformIO compile ไม่พบ header

ตรวจโครงสร้าง:

```text
PlatformIO/
├── platformio.ini
├── src/main.cpp
└── lib/Massmore_BNO08x/src/Massmore_BNO08x.h
```

จากนั้นกด PlatformIO: Rebuild IntelliSense Index และ Build ใหม่

## 4. Resources

### 4.1 GitHub

- **GitHub Repository:** Massmore_BNO08x GitHub repository https://github.com/massmore/Massmore_BNO08x
- **GitHub Repository:** Massmore legacy BNO0xx examples https://github.com/Massmore/BNO0xx_SKU-1010
- **GitHub Repository:** Arduino IDE examples https://github.com/massmore/Massmore_BNO08x/tree/main/ArduinoIDE/examples
- **GitHub Repository:** PlatformIO project https://github.com/massmore/Massmore_BNO08x/tree/main/PlatformIO

หากลิงก์รีโพซิทอรีใหม่ยังไม่เผยแพร่ ให้ใช้ไฟล์ในแพ็กเกจ Massmore_BNO08x ที่ได้รับมาจนกว่าหน้า GitHub จะเปิดเป็น public

### 4.2 Document & Datasheet

#### เอกสารจากผู้ผลิต

- **CEVA Document:** CEVA BNO08X Datasheet — 1000-3927 v1.17 https://www.ceva-ip.com/wp-content/uploads/BNO080_085-Datasheet.pdf
- **CEVA Document:** CEVA SH-2 Reference Manual — 1000-3625 https://www.ceva-ip.com/wp-content/uploads/SH-2-Reference-Manual.pdf
- **CEVA Document:** CEVA BNO08X Sensor Calibration Procedure — 1000-4044 https://www.ceva-ip.com/wp-content/uploads/2019/09/BNO080-BNO085-Sesnor-Calibration-Procedure.pdf
- **CEVA Document:** CEVA BNO08X Tare Function Usage Guide — 1000-4045 https://www.ceva-ip.com/wp-content/uploads/BNO080-BNO085-Tare-Function-Usage-Guide.pdf
- **CEVA Document:** CEVA BNO085 Development Kit Quick Start Guide https://www.ceva-ip.com/wp-content/uploads/BNO080-Development-Kit-for-Nucleo-Quick-Start-Guide.pdf

#### เครื่องมือพัฒนา

- **Product Page:** Massmore Halley BNO086/BNO085 product page https://www.massmore.shop/products/2141d3bf-9d0f-4837-badf-a36bcda61638
- **Arduino Download:** Arduino IDE download https://www.arduino.cc/en/software
- **Arduino Guide:** Arduino: Install libraries https://support.arduino.cc/hc/en-us/articles/5145457742236-Install-libraries-in-the-Arduino-IDE
- **Espressif Guide:** Espressif Arduino-ESP32 installation https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html
- **VS Code / PlatformIO Guide:** PlatformIO IDE for VS Code https://docs.platformio.org/en/latest/integration/ide/vscode.html
- **PlatformIO Guide:** PlatformIO project configuration https://docs.platformio.org/en/latest/projectconf/index.html

#### งานวิจัยและบทความวิชาการพื้นฐาน

- **Research:** Kok, Hol และ Schön, Using Inertial Sensors for Position and Orientation Estimation — อธิบาย drift, sensor models และวิธี fusion https://arxiv.org/abs/1704.06053
- **Research:** Sabatini, Kalman-Filter-Based Orientation Determination Using Inertial/Magnetic Sensors — quaternion EKF และผลของ magnetic disturbance https://doi.org/10.3390/s111009182
- **Research:** Sabatini, Quaternion-based extended Kalman filter for determining orientation by inertial and magnetic sensing — การรวม gyro, accelerometer และ magnetometer https://doi.org/10.1109/TBME.2006.875664
- **Research:** Schön et al., Magnetometer calibration using inertial sensors — calibration และผลต่อ heading https://arxiv.org/abs/1601.05257
- **Research:** Madgwick, An efficient orientation filter for inertial and inertial/magnetic sensor arrays — พื้นฐาน quaternion gradient-descent filter สำหรับเปรียบเทียบแนวคิด https://courses.cs.washington.edu/courses/cse466/14au/labs/l4/madgwick_internal_report.pdf

## Checklist ก่อนนำไปใช้ในผลิตภัณฑ์จริง

- ☐ ยืนยัน supply และ logic voltage จาก schematic ของ module revision จริง
- ☐ ยืนยัน I2C address หลังประกอบ
- ☐ ต่อ INT/RST เพื่อ recovery ที่เชื่อถือได้
- ☐ ทดสอบที่ 100 kHz ก่อนเพิ่มเป็น 400 kHz
- ☐ เลือก Rotation Vector ให้ตรงกับสภาพสนามแม่เหล็ก
- ☐ ทำ calibration หลังติดตั้งใน enclosure
- ☐ ทดสอบเมื่อมอเตอร์/โหลดกระแสสูงทำงานจริง
- ☐ จำกัดอัตราพิมพ์ log ไม่ให้รบกวนการอ่าน packet
- ☐ ตรวจ behavior หลัง brownout, reset และถอดเสียบไฟซ้ำ
- ☐ บันทึก firmware version, address, pin map และ calibration procedure ในเอกสารผลิต

บทความโดย **Massmore Biz Co., Ltd.** — **Massmore Website:** Massmore website https://www.massmore.shop — อัปเดตเนื้อหา 1 กันยายน 2569
