/*!
 * @file  Massmore_BNO08x.h
 * @brief Driver สำหรับ BNO085 / BNO086 9-axis sensor-fusion IMU (CEVA SH-2 MotionEngine)
 *        ใช้ได้ทั้ง Arduino IDE และ PlatformIO
 *
 * จุดเด่น
 *  - I2C, SPI และ SHTP-over-UART ในคลาสเดียว (UART-RVC อยู่ใน Massmore_BNO08x_RVC.h)
 *  - Dual API: Simple Blocking API (readAll) และ Advanced Non-blocking FSM (update / isDataReady)
 *  - รองรับ SH-2 sensor report ทุกชนิดตั้งแต่ quaternion พื้นฐานถึง activity classifier
 *  - Chip identity + authenticity: verifyChipID() / getSerialNumber() / isGenuine()
 *  - Calibration, tare, Save DCD, FRS read/write, sleep/wake, soft & hard reset
 *  - ไม่มี dynamic allocation, ไม่ hardcode GPIO, ไม่เรียก Wire/SPI/Serial.begin() ในไลบรารี
 *
 * Bus ownership: sketch เป็นเจ้าของ peripheral ทั้งหมด — ต้องเรียก Wire.begin(sda, scl) /
 * SPI.begin(...) / SerialX.begin(...) เองก่อน แล้วส่ง reference เข้ามาที่ begin*()
 *
 * Supported MCUs: ESP32 (Classic), ESP32-S3 (Arduino-ESP32 Core 3.x+), AVR ATmega328P (Arduino Nano)
 *
 * Massmore_BNO08x — Designed and Manufactured by Massmore (https://www.massmore.shop)
 * Product: https://www.massmore.shop/products/2141d3bf-9d0f-4837-badf-a36bcda61638
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef MASSMORE_BNO08X_H
#define MASSMORE_BNO08X_H

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include "Massmore_BNO08x_Defs.h"

/*!
 * @class Massmore_BNO08x
 * @brief Driver object — สร้างหนึ่งตัวต่อเซ็นเซอร์หนึ่งตัว
 *
 * ตัวอย่างสั้นที่สุด (Simple Blocking API):
 * @code
 *   Massmore_BNO08x imu;
 *   void setup() {
 *     Serial.begin(115200);
 *     Wire.begin(21, 22);                     // ESP32: sketch เป็นคนกำหนดขา
 *     if (!imu.begin(0x4A, Wire)) { ... }     // Massmore board default = 0x4A
 *   }
 *   void loop() {
 *     Massmore_BNO08x_reading_t r;
 *     if (imu.readAll(r)) Serial.println(r.headingDeg);
 *   }
 * @endcode
 *
 * ตัวอย่าง Non-blocking FSM:
 * @code
 *   imu.enableRotationVector(10000);          // 100 Hz
 *   void loop() {
 *     imu.update();                            // ไม่ block
 *     if (imu.hasNewReport(MASSMORE_BNO08X_SENSOR_ROTATION_VECTOR)) {
 *       Serial.println(imu.getYawDeg());
 *     }
 *     // งานอื่นทำต่อได้ทันที
 *   }
 * @endcode
 */
class Massmore_BNO08x {
public:
    Massmore_BNO08x();

    /* ===================================================================
     * SECTION 1 — Start-up (bus reference injection)
     * =================================================================== */

    /*!
     * @brief  เริ่มต้นเซ็นเซอร์บน I2C bus (sketch ต้องเรียก Wire.begin() มาก่อน)
     * @param  address  0x4A (DI/SA0 = LOW, ค่า default ของบอร์ด Massmore) หรือ 0x4B
     *                  ถ้าไม่พบที่ address ที่ระบุ driver จะลอง address อีกตัวให้อัตโนมัติ
     *                  (ดู getI2CAddress() ว่าเจอที่ไหน)
     * @param  wirePort TwoWire instance ที่ใช้ (Wire หรือ Wire1)
     * @param  intPin   ขา H_INTN (pad INT) หรือ -1 ถ้าไม่ได้ต่อ — แนะนำให้ต่อ
     *                  เพราะ driver จะไม่ต้อง poll bus เปล่า ๆ
     * @param  rstPin   ขา NRST (pad RST) หรือ -1 — ต่อแล้วใช้ hardwareReset() ได้
     * @return true เมื่อสำเร็จ; false ให้ดู lastError()
     * @note   หลัง power-on ชิปต้องการเวลา boot; begin() รอให้เองแล้ว
     */
    bool begin(uint8_t address = MASSMORE_BNO08X_I2C_ADDR_DEF,
               TwoWire &wirePort = Wire,
               int8_t intPin = -1,
               int8_t rstPin = -1);

    /*!
     * @brief  เริ่มต้นเซ็นเซอร์บน SPI bus (SPI_MODE3: CPOL=1, CPHA=1, สูงสุด 3 MHz)
     * @param  csPin    Chip select (H_CSN — pad CS)
     * @param  intPin   H_INTN (pad INT) — จำเป็นสำหรับ SPI เพราะ SHTP over SPI ไม่มีวิธี poll
     * @param  rstPin   NRST (pad RST) — จำเป็น เพราะต้อง reset ขณะ PS0/PS1 = HIGH เพื่อ latch โหมด SPI
     * @param  wakePin  PS0/WAKE (pad P0) หรือ -1 — ใช้ปลุกชิปจาก sleep
     * @param  spiPort  SPIClass ที่ใช้ (sketch ต้องเรียก SPI.begin() มาก่อน)
     * @param  speedHz  SPI clock (datasheet ระบุสูงสุด 3 MHz)
     */
    bool beginSPI(int8_t csPin, int8_t intPin, int8_t rstPin,
                  int8_t wakePin = -1,
                  SPIClass &spiPort = SPI,
                  uint32_t speedHz = 3000000UL);

    /*!
     * @brief  เริ่มต้นเซ็นเซอร์แบบ SHTP-over-UART ที่ 3 Mbit/s (strap PS1=1, PS0=0)
     * @param  serialPort Stream ที่ begin() แล้ว (HardwareSerial ฯลฯ)
     * @param  intPin     H_INTN (pad INT) หรือ -1
     * @param  rstPin     NRST (pad RST) หรือ -1
     * @note   ไม่ใช่ UART-RVC — โหมด RVC 100 Hz แบบง่ายใช้คลาส Massmore_BNO08x_RVC
     */
    bool beginUART(Stream &serialPort, int8_t intPin = -1, int8_t rstPin = -1);

    /*! @brief true เมื่อ begin*() สำเร็จแล้ว */
    bool isConnected() const { return _busType != MASSMORE_BNO08X_BUS_NONE; }

    /*! @brief I2C address ที่ begin() พบอุปกรณ์จริง (0x4A หรือ 0x4B) */
    uint8_t getI2CAddress() const { return _i2cAddr; }

    /*! @brief ส่ง diagnostic ของไลบรารีออกทาง Stream (ปกติคือ Serial) */
    void enableDebug(Stream &dbg) { _dbg = &dbg; }
    /*! @brief ปิด diagnostic */
    void disableDebug() { _dbg = nullptr; }

    /* ===================================================================
     * SECTION 2 — Simple Blocking API (สำหรับผู้เริ่มต้น / AVR)
     * =================================================================== */

    /*!
     * @brief  อ่านค่าครบชุดแบบ Blocking (quaternion, Euler, accel, gyro, mag)
     *         ถ้ายังไม่ได้ enable report ที่จำเป็น driver จะ enable ให้เองที่ 50 Hz
     * @param  out        struct รับค่า
     * @param  timeoutMs  เวลารอสูงสุด (ms) — ครั้งแรกหลัง begin() อาจใช้ ~100 ms
     * @return true เมื่อได้ report ครบทั้ง 4 ชนิด; false ให้ดู lastError()
     */
    bool readAll(Massmore_BNO08x_reading_t &out, uint32_t timeoutMs = 300);

    /*!
     * @brief  อ่าน Euler angles (องศา) แบบ Blocking จาก Rotation Vector
     * @return true เมื่อสำเร็จ
     */
    bool readEulerDeg(Massmore_BNO08x_euler_t &outDeg, uint32_t timeoutMs = 300);

    /*!
     * @brief  อ่าน heading แบบเข็มทิศ 0..360 องศา แบบ Blocking
     * @return heading เป็นองศา หรือ NAN หากอ่านไม่สำเร็จ (ดู lastError())
     */
    float readHeadingDeg(uint32_t timeoutMs = 300);

    /*!
     * @brief  รอ report ชนิดที่ระบุหนึ่งครั้งแบบ Blocking (rollover-safe millis())
     * @param  sensorId   MASSMORE_BNO08X_SENSOR_*
     * @param  timeoutMs  เวลารอสูงสุด
     * @return true เมื่อได้ report ใหม่แล้ว (อ่านค่าได้จาก get*())
     */
    bool waitForReport(uint8_t sensorId, uint32_t timeoutMs = 300);

    /* ===================================================================
     * SECTION 3 — Advanced Non-blocking FSM API (multitask / RTOS)
     *   enable*() → update() → isDataReady() → getReadings()
     * =================================================================== */

    /*!
     * @brief  ดึง SHTP packet หนึ่งชุดจากเซ็นเซอร์แล้วถอดรหัส — Non-blocking
     * @return true ถ้าได้ packet และถอดรหัสแล้ว
     * @note   เรียกให้บ่อยที่สุด ถ้าต่อ INT pin ไว้ update() จะ return false ทันที
     *         เมื่อไม่มีข้อมูล จึงเรียกทุก loop() ได้โดยแทบไม่มี cost
     */
    bool update();

    /*!
     * @brief  ดึง packet ทั้งหมดที่ค้างอยู่ (มี budget กันไม่ให้ loop() โดน starve)
     * @param  maxPackets จำนวน packet สูงสุดต่อการเรียก
     * @return จำนวน packet ที่ถอดรหัสได้
     */
    uint8_t updateAll(uint8_t maxPackets = 16);

    /*! @brief true ถ้าเซ็นเซอร์ assert H_INTN (หรือ true เสมอถ้าไม่มี INT pin) */
    bool dataAvailable();

    /*!
     * @brief  ตรวจว่ามี report ชนิดนี้มาใหม่หรือไม่ โดยไม่ล้าง flag (Non-blocking)
     * @param  sensorId MASSMORE_BNO08X_SENSOR_*
     */
    bool isDataReady(uint8_t sensorId) const { return peekNewReport(sensorId); }

    /*! @brief true ถ้า Rotation Vector มาใหม่ (shortcut ของ isDataReady(RV)) */
    bool isDataReady() const { return peekNewReport(MASSMORE_BNO08X_SENSOR_ROTATION_VECTOR); }

    /*!
     * @brief  คัดลอกค่าล่าสุดทั้งหมดออกมา (ไม่ block, ไม่แตะ bus)
     * @param  out struct รับค่า
     */
    void getReadings(Massmore_BNO08x_reading_t &out) const;

    /*! @brief Report ID ของ sensor report ล่าสุดที่ถอดรหัส (0 = ยังไม่มี) */
    uint8_t getLastReportID() const { return _lastReportId; }

    /*!
     * @brief  Test-and-clear: report `id` มาใหม่ตั้งแต่ถามครั้งก่อนหรือไม่
     *         เหมาะกับโค้ดแบบ "รอบนี้ได้ quaternion ใหม่ไหม"
     */
    bool hasNewReport(uint8_t id);

    /*! @brief เหมือน hasNewReport() แต่ไม่ล้าง flag */
    bool peekNewReport(uint8_t id) const;

    /*! @brief ล้าง flag "มาใหม่" ทุกชนิด */
    void clearNewFlags();

    /*!
     * @brief  Callback ที่ถูกเรียกหนึ่งครั้งต่อ sensor report ที่ถอดรหัสได้
     * @param  cb  void f(uint8_t reportId, void *ctx)
     */
    void setReportCallback(void (*cb)(uint8_t reportId, void *ctx), void *ctx = nullptr);

    /* ===================================================================
     * SECTION 4 — Identity and authenticity (ตรวจของแท้)
     * =================================================================== */

    /*!
     * @brief  ตรวจ "CHIP_ID" ของ BNO08x — ชิปไม่มี WHO_AM_I register แต่ใช้
     *         SH-2 Product ID Response (report 0xF8) แทน: firmware part number
     *         ต้องตรงกับ SH-2 application build ที่รู้จัก (10003606 / 10004095)
     * @return true ถ้า Product ID ตรง; false → lastError() = ERR_WRONG_ID / ERR_TIMEOUT
     */
    bool verifyChipID();

    /*!
     * @brief  อ่าน serial number จาก FRS record 0x4B4B (32 bit ล่าง)
     * @return serial number หรือ 0 ถ้าอ่านไม่ได้ / ชิปไม่ได้ถูก program ค่านี้จากโรงงาน
     * @note   ต้องการ 64 bit เต็มให้ใช้ readSerialNumber()
     */
    uint32_t getSerialNumber();

    /*!
     * @brief  Heuristic รวมสำหรับตรวจของแท้:
     *         (1) Product ID Response ถูกต้อง + version สมเหตุสมผล
     *         (2) firmware part number ตรงตารางโรงงาน หรือ
     *         (3) ถ้า part number ไม่รู้จัก → Rotation Vector metadata (FRS 0xE30B)
     *             ต้องมี Q point 14/12 ซึ่งพิสูจน์ว่า SH-2 MotionEngine ทำงานจริง
     * @return true = GENUINE, false = SUSPECT (ดู getLastAuthResult())
     */
    bool isGenuine();

    /*!
     * @brief  ตรวจว่า SH-2 MotionEngine ทำงานจริง โดยอ่าน Rotation Vector metadata
     *         และเทียบ Q point กับค่าจาก SH-2 Reference Manual (14 / 12)
     * @return true ถ้า metadata อ่านได้และ Q point ตรง
     */
    bool verifyMotionEngine();

    /*! @brief ผลล่าสุดของ verifyChip() / isGenuine() */
    Massmore_BNO08x_auth_t getLastAuthResult() const { return _lastAuth; }

    /*!
     * @brief  ขอ Product ID จากชิป (report 0xF9 → 0xF8) แล้ว cache ไว้
     * @return MASSMORE_BNO08X_OK หรือ error code
     */
    Massmore_BNO08x_status_t requestProductID(uint32_t timeoutMs = 300);

    /*!
     * @brief  Product ID ของ SH-2 application (ชุดที่ part number ตรงตาราง
     *         หรือชุดแรกที่ได้รับถ้าไม่มีชุดไหนตรง) — ถูกเติมโดย begin() และ requestProductID()
     */
    const Massmore_BNO08x_product_id_t &getProductID() const { return _productId; }

    /*! @brief จำนวน Product ID Response ที่เก็บได้จากการขอครั้งล่าสุด */
    uint8_t getProductIDCount() const { return _productIdCount; }

    /*!
     * @brief  Product ID Response ชุดที่ index (ตามลำดับที่มาถึง)
     * @param  index 0 .. getProductIDCount()-1; เกินช่วงจะคืนชุดหลัก
     */
    const Massmore_BNO08x_product_id_t &getProductID(uint8_t index) const {
        return (index < _productIdCount) ? _productIds[index] : _productId;
    }

    /*!
     * @brief  ตรวจว่าชิปทำงานเหมือน BNO08x ของแท้ในระดับ Protocol (3 ข้อ):
     *         Product ID Response ถูกต้อง, version สมเหตุสมผล, part number ตรงตารางโรงงาน
     * @return Massmore_BNO08x_auth_t — UNKNOWN_FW ไม่ใช่ความล้มเหลว หมายถึง
     *         CEVA ออก firmware build ใหม่กว่าตารางในไลบรารี
     */
    Massmore_BNO08x_auth_t verifyChip();

    /*! @brief ข้อความอ่านง่ายของผล verifyChip() (บน AVR ค่าที่คืนใช้ได้จนกว่าจะเรียก *ToString() ครั้งถัดไป) */
    static const char *authToString(Massmore_BNO08x_auth_t a);

    /*! @brief ข้อความอ่านง่ายของ Massmore_BNO08x_status_t */
    static const char *statusToString(Massmore_BNO08x_status_t s);

    /*!
     * @brief  อ่าน serial number 64 bit จาก FRS record 0x4B4B
     * @param  serialOut รับค่า serial
     * @return MASSMORE_BNO08X_OK เมื่อสำเร็จ
     */
    Massmore_BNO08x_status_t readSerialNumber(uint64_t &serialOut, uint32_t timeoutMs = 500);

    /*! @brief สาเหตุ reset ล่าสุด (จาก Product ID Response) เป็นข้อความ */
    const char *getResetReasonString() const;

    /* ===================================================================
     * SECTION 5 — Enabling sensors (Set Feature)
     * =================================================================== */

    /*!
     * @brief  Set Feature command แบบครบทุก field — Datasheet Figure 1-33
     * @param  sensorId          report ที่ต้องการ (interval 0 = disable)
     * @param  reportIntervalUs  คาบเวลาเป็น microseconds; 0 = ปิด sensor
     * @param  batchIntervalUs   คาบ batching (0 = ไม่ batch)
     * @param  flags             MASSMORE_BNO08X_FEATURE_FLAG_* bitmap
     * @param  changeSensitivity threshold สำหรับ report-on-change
     * @param  sensorSpecific    configuration word เฉพาะ sensor (32 bit)
     */
    Massmore_BNO08x_status_t setFeature(uint8_t sensorId,
                                 uint32_t reportIntervalUs,
                                 uint32_t batchIntervalUs = 0,
                                 uint8_t  flags = MASSMORE_BNO08X_FEATURE_FLAG_NONE,
                                 uint16_t changeSensitivity = 0,
                                 uint32_t sensorSpecific = 0);

    /*! @brief ทางลัดของ setFeature(id, intervalUs) */
    Massmore_BNO08x_status_t enableReport(uint8_t sensorId, uint32_t reportIntervalUs);

    /*! @brief ปิด sensor (interval 0) */
    Massmore_BNO08x_status_t disableReport(uint8_t sensorId);

    /*! @brief ปิดทุก sensor ที่ object นี้เคย enable */
    void disableAllReports();

    /*! @brief ขอ configuration ปัจจุบันของ sensor (0xFE → 0xFC) */
    Massmore_BNO08x_status_t requestFeature(uint8_t sensorId);

    /*! @brief Report interval (us) ที่ชิปแจ้งล่าสุดว่ากำลังใช้ */
    uint32_t getReportInterval(uint8_t sensorId) const;

    /* ---- Motion / orientation ---------------------------------------- */
    Massmore_BNO08x_status_t enableAccelerometer(uint32_t us = MASSMORE_BNO08X_INTERVAL_100HZ);
    Massmore_BNO08x_status_t enableGyroscope(uint32_t us = MASSMORE_BNO08X_INTERVAL_100HZ);
    Massmore_BNO08x_status_t enableMagnetometer(uint32_t us = MASSMORE_BNO08X_INTERVAL_100HZ);
    Massmore_BNO08x_status_t enableLinearAcceleration(uint32_t us = MASSMORE_BNO08X_INTERVAL_100HZ);
    Massmore_BNO08x_status_t enableGravity(uint32_t us = MASSMORE_BNO08X_INTERVAL_100HZ);
    Massmore_BNO08x_status_t enableGyroscopeUncalibrated(uint32_t us = MASSMORE_BNO08X_INTERVAL_100HZ);
    Massmore_BNO08x_status_t enableMagnetometerUncalibrated(uint32_t us = MASSMORE_BNO08X_INTERVAL_100HZ);

    /*! 9-axis fusion — heading สัมบูรณ์ ต้องมี magnetometer ที่ calibrate แล้ว */
    Massmore_BNO08x_status_t enableRotationVector(uint32_t us = MASSMORE_BNO08X_INTERVAL_100HZ);
    /*! 6-axis fusion — ไม่ใช้ magnetometer yaw drift ได้แต่ทนสนามแม่เหล็กรบกวน */
    Massmore_BNO08x_status_t enableGameRotationVector(uint32_t us = MASSMORE_BNO08X_INTERVAL_100HZ);
    /*! Accel + mag เท่านั้น — กินไฟต่ำ อัตราต่ำ */
    Massmore_BNO08x_status_t enableGeomagneticRotationVector(uint32_t us = MASSMORE_BNO08X_INTERVAL_100HZ);
    /*! Rotation vector ที่ smooth discontinuity — สำหรับ AR/VR headset */
    Massmore_BNO08x_status_t enableARVRStabilizedRotationVector(uint32_t us = MASSMORE_BNO08X_INTERVAL_100HZ);
    /*! Game rotation vector ที่ smooth discontinuity */
    Massmore_BNO08x_status_t enableARVRStabilizedGameRotationVector(uint32_t us = MASSMORE_BNO08X_INTERVAL_100HZ);
    /*! Quaternion อัตราสูงสุด 1 kHz ส่งบน SHTP channel 5 */
    Massmore_BNO08x_status_t enableGyroIntegratedRotationVector(uint32_t us = MASSMORE_BNO08X_INTERVAL_400HZ);

    /* ---- Raw (uncalibrated ADC counts) -------------------------------- */
    Massmore_BNO08x_status_t enableRawAccelerometer(uint32_t us = MASSMORE_BNO08X_INTERVAL_100HZ);
    Massmore_BNO08x_status_t enableRawGyroscope(uint32_t us = MASSMORE_BNO08X_INTERVAL_100HZ);
    Massmore_BNO08x_status_t enableRawMagnetometer(uint32_t us = MASSMORE_BNO08X_INTERVAL_100HZ);

    /* ---- Activity / gesture engines ----------------------------------- */
    Massmore_BNO08x_status_t enableTapDetector(uint32_t us = MASSMORE_BNO08X_INTERVAL_100HZ);
    Massmore_BNO08x_status_t enableStepCounter(uint32_t us = MASSMORE_BNO08X_INTERVAL_10HZ);
    Massmore_BNO08x_status_t enableStepDetector(uint32_t us = MASSMORE_BNO08X_INTERVAL_10HZ);
    Massmore_BNO08x_status_t enableSignificantMotion(uint32_t us = MASSMORE_BNO08X_INTERVAL_10HZ);
    Massmore_BNO08x_status_t enableStabilityClassifier(uint32_t us = MASSMORE_BNO08X_INTERVAL_10HZ);
    Massmore_BNO08x_status_t enableStabilityDetector(uint32_t us = MASSMORE_BNO08X_INTERVAL_10HZ);
    Massmore_BNO08x_status_t enableShakeDetector(uint32_t us = MASSMORE_BNO08X_INTERVAL_10HZ);
    Massmore_BNO08x_status_t enableFlipDetector(uint32_t us = MASSMORE_BNO08X_INTERVAL_10HZ);
    Massmore_BNO08x_status_t enablePickupDetector(uint32_t us = MASSMORE_BNO08X_INTERVAL_10HZ);
    Massmore_BNO08x_status_t enableSleepDetector(uint32_t us = MASSMORE_BNO08X_INTERVAL_10HZ);
    Massmore_BNO08x_status_t enableTiltDetector(uint32_t us = MASSMORE_BNO08X_INTERVAL_10HZ);
    Massmore_BNO08x_status_t enablePocketDetector(uint32_t us = MASSMORE_BNO08X_INTERVAL_10HZ);
    Massmore_BNO08x_status_t enableCircleDetector(uint32_t us = MASSMORE_BNO08X_INTERVAL_10HZ);
    Massmore_BNO08x_status_t enableHeartRateMonitor(uint32_t us = MASSMORE_BNO08X_INTERVAL_10HZ);

    /*!
     * @brief  Personal activity classifier
     * @param  us                report interval
     * @param  enabledActivities bitmap ของกิจกรรมที่ติดตาม (bit n = Massmore_BNO08x_activity_t n)
     *                           0x1F = unknown/vehicle/bicycle/foot/still
     */
    Massmore_BNO08x_status_t enableActivityClassifier(uint32_t us = MASSMORE_BNO08X_INTERVAL_10HZ,
                                               uint32_t enabledActivities = 0x1F);

    /* ---- External environmental sensors on the secondary I2C bus ------ */
    Massmore_BNO08x_status_t enablePressure(uint32_t us = MASSMORE_BNO08X_INTERVAL_10HZ);
    Massmore_BNO08x_status_t enableAmbientLight(uint32_t us = MASSMORE_BNO08X_INTERVAL_10HZ);
    Massmore_BNO08x_status_t enableHumidity(uint32_t us = MASSMORE_BNO08X_INTERVAL_10HZ);
    Massmore_BNO08x_status_t enableProximity(uint32_t us = MASSMORE_BNO08X_INTERVAL_10HZ);
    Massmore_BNO08x_status_t enableTemperature(uint32_t us = MASSMORE_BNO08X_INTERVAL_10HZ);

    /* ===================================================================
     * SECTION 6 — Reading cached data (ไม่แตะ bus)
     * =================================================================== */

    /* ---- Quaternion (rotation vector ชนิดล่าสุดที่มาถึง) ---------------- */
    float getQuatI()    const { return _quat.i; }
    float getQuatJ()    const { return _quat.j; }
    float getQuatK()    const { return _quat.k; }
    float getQuatReal() const { return _quat.real; }
    /*! @brief ค่าประมาณ heading accuracy เป็น radians (rotation vector เท่านั้น) */
    float getQuatAccuracy() const { return _quat.accuracy; }
    /*! @brief คัดลอก quaternion ทั้งชุด */
    Massmore_BNO08x_quat_t getQuaternion() const { return _quat; }

    /* ---- Euler angles คำนวณจาก quaternion ที่ cache ไว้ ------------------ */
    /*! @brief Roll (หมุนรอบ X) radians, -pi..pi */
    float getRoll();
    /*! @brief Pitch (หมุนรอบ Y) radians, -pi/2..pi/2 */
    float getPitch();
    /*! @brief Yaw / heading (หมุนรอบ Z) radians, -pi..pi */
    float getYaw();
    float getRollDeg();
    float getPitchDeg();
    /*! @brief Yaw เป็นองศา -180..180 */
    float getYawDeg();
    /*! @brief Yaw แบบเข็มทิศ 0..360 องศา */
    float getHeadingDeg();
    /*! @brief Euler ทั้งสามแกน (radians) */
    Massmore_BNO08x_euler_t getEuler();
    /*! @brief Euler ทั้งสามแกน (องศา) */
    Massmore_BNO08x_euler_t getEulerDeg();

    /*! @brief แปลง quaternion ใด ๆ เป็น Euler angles (radians) */
    static Massmore_BNO08x_euler_t quaternionToEuler(const Massmore_BNO08x_quat_t &q);

    /* ---- Vectors ------------------------------------------------------ */
    Massmore_BNO08x_vec3_t getAccel()       const { return _accel; }       //!< m/s^2, with gravity
    Massmore_BNO08x_vec3_t getGyro()        const { return _gyro; }        //!< rad/s
    Massmore_BNO08x_vec3_t getMag()         const { return _mag; }         //!< uT
    Massmore_BNO08x_vec3_t getLinearAccel() const { return _linAccel; }    //!< m/s^2, gravity removed
    Massmore_BNO08x_vec3_t getGravity()     const { return _gravity; }     //!< m/s^2
    Massmore_BNO08x_vec3_t getGyroBias()    const { return _gyroBias; }    //!< rad/s
    Massmore_BNO08x_vec3_t getMagBias()     const { return _magBias; }     //!< uT
    Massmore_BNO08x_vec3_t getAngularVelocity() const { return _angVel; }  //!< rad/s, gyro-integrated RV

    float getAccelX() const { return _accel.x; }
    float getAccelY() const { return _accel.y; }
    float getAccelZ() const { return _accel.z; }
    float getGyroX()  const { return _gyro.x; }
    float getGyroY()  const { return _gyro.y; }
    float getGyroZ()  const { return _gyro.z; }
    float getMagX()   const { return _mag.x; }
    float getMagY()   const { return _mag.y; }
    float getMagZ()   const { return _mag.z; }
    float getLinAccelX() const { return _linAccel.x; }
    float getLinAccelY() const { return _linAccel.y; }
    float getLinAccelZ() const { return _linAccel.z; }

    /*! @brief Gyroscope เป็น degrees/second */
    Massmore_BNO08x_vec3_t getGyroDeg() const;

    /* ---- Raw ADC counts ----------------------------------------------- */
    Massmore_BNO08x_vec3i_t getRawAccel() const { return _rawAccel; }
    Massmore_BNO08x_vec3i_t getRawGyro()  const { return _rawGyro; }
    Massmore_BNO08x_vec3i_t getRawMag()   const { return _rawMag; }
    /*! @brief อุณหภูมิ die ของ gyroscope เป็น raw ADC counts (raw gyro report) */
    int16_t getRawGyroTemperature() const { return _rawGyroTemp; }

    /* ---- Environmental ------------------------------------------------ */
    float getPressure()     const { return _pressure; }      //!< hPa
    float getAmbientLight() const { return _ambientLight; }  //!< lux
    float getHumidity()     const { return _humidity; }      //!< %RH
    float getProximity()    const { return _proximity; }     //!< cm
    float getTemperature()  const { return _temperature; }   //!< degC

    /* ---- Event / classifier outputs ----------------------------------- */
    /*! @brief จำนวนก้าวสะสมตั้งแต่ power-on (หรือ reset ล่าสุด) */
    uint32_t getStepCount()   const { return _stepCount; }
    /*! @brief Tap flags — เทียบกับ MASSMORE_BNO08X_TAP_* (ล้างเมื่ออ่าน) */
    uint8_t  getTapDetector();
    /*! @brief Shake flags — เทียบกับ MASSMORE_BNO08X_SHAKE_* (ล้างเมื่ออ่าน) */
    uint16_t getShakeDetector();
    bool     getSignificantMotion();
    bool     getFlipDetected();
    bool     getPickupDetected();
    bool     getTiltDetected();
    bool     getPocketDetected();
    bool     getCircleDetected();
    bool     getStepDetected();
    bool     getStabilityChanged();
    uint16_t getHeartRate()   const { return _heartRate; }
    uint8_t  getSleepState()  const { return _sleepState; }

    Massmore_BNO08x_stability_t getStabilityClassification() const { return _stability; }
    const char          *getStabilityString() const;

    Massmore_BNO08x_activity_t  getActivity() const { return (Massmore_BNO08x_activity_t)_activityMostLikely; }
    const char          *getActivityString() const;
    /*! @brief Confidence 0..100 ของกิจกรรมหนึ่งจาก classifier */
    uint8_t              getActivityConfidence(Massmore_BNO08x_activity_t a) const;

    /* ---- Report metadata ---------------------------------------------- */
    /*!
     * @brief  Accuracy (0..3) ของ sensor ที่ผลิต report `id`
     *         BNO08x ส่งค่านี้ใน status byte ของทุก sensor report
     */
    Massmore_BNO08x_accuracy_t getAccuracy(uint8_t sensorId) const;
    /*! @brief ข้อความ: "Unreliable" / "Low" / "Medium" / "High" */
    static const char *accuracyToString(Massmore_BNO08x_accuracy_t a);

    /*!
     * @brief  Timestamp ของ report ล่าสุด เป็น microseconds บน micros() ของ host
     *
     * BNO08x ไม่ส่งเวลาสัมบูรณ์ ทุก packet มี base-timestamp delta (signed) และทุก
     * report มี delay ทั้งคู่หน่วย 100 us และอ้างอิงจากจังหวะที่ packet ถูกส่ง driver
     * จึง anchor ค่าเหล่านี้กับ micros() ณ ตอนที่รับ packet ทำให้เทียบกับ millis()/micros()
     * ได้และเพิ่มขึ้นแบบ monotonic ความแม่นขึ้นกับความถี่ที่ loop() เรียก update()
     */
    uint64_t getTimestampUs() const { return _timestampUs; }
    /*! @brief Sequence number ของ report ล่าสุด — ใช้ตรวจ report ตกหล่น */
    uint8_t  getSequenceNumber() const { return _lastReportSeq; }

    /* ===================================================================
     * SECTION 7 — Calibration
     * =================================================================== */

    /*! @brief เปิด dynamic calibration ให้ subsystem ที่ระบุ */
    Massmore_BNO08x_status_t calibrate(Massmore_BNO08x_calibrate_target_t target);
    Massmore_BNO08x_status_t calibrateAccelerometer() { return calibrate(MASSMORE_BNO08X_CAL_ACCEL); }
    Massmore_BNO08x_status_t calibrateGyroscope()     { return calibrate(MASSMORE_BNO08X_CAL_GYRO); }
    Massmore_BNO08x_status_t calibrateMagnetometer()  { return calibrate(MASSMORE_BNO08X_CAL_MAG); }
    Massmore_BNO08x_status_t calibratePlanarAccel()   { return calibrate(MASSMORE_BNO08X_CAL_PLANAR_ACCEL); }
    Massmore_BNO08x_status_t calibrateAll()           { return calibrate(MASSMORE_BNO08X_CAL_ACCEL_GYRO_MAG); }
    /*! @brief ปิด dynamic calibration ทุก subsystem */
    Massmore_BNO08x_status_t endCalibration()         { return calibrate(MASSMORE_BNO08X_CAL_STOP); }

    /*! @brief ขอสถานะ calibration enable จาก MotionEngine */
    Massmore_BNO08x_status_t requestCalibrationStatus();
    /*! @brief true เมื่อ ME calibration command ล่าสุดตอบสำเร็จ */
    bool calibrationComplete() const { return _calibrationStatus == 0; }
    /*! @brief Status byte ดิบจาก ME calibration command response ล่าสุด */
    uint8_t getCalibrationStatus() const { return _calibrationStatus; }

    /*! @brief เขียน Dynamic Calibration Data ลง flash ให้อยู่ข้าม reboot */
    Massmore_BNO08x_status_t saveCalibration();
    /*! @brief ให้ชิป auto-save DCD เป็นระยะ (true = เปิด) */
    Massmore_BNO08x_status_t setPeriodicCalibrationSave(bool enable);
    /*! @brief ลบ calibration ที่เก็บไว้แล้ว reset (ชิป reboot เอง) */
    Massmore_BNO08x_status_t clearCalibrationAndReset();

    /* ===================================================================
     * SECTION 8 — Tare (กำหนดทิศ "ข้างหน้า")
     * =================================================================== */

    /*!
     * @brief  ตั้ง orientation ปัจจุบันเป็นศูนย์
     * @param  axes  bitmap ของ Massmore_BNO08x_tare_axis_t — MASSMORE_BNO08X_TARE_AXIS_Z สำหรับ
     *               tare เฉพาะ heading (ปุ่ม recenter) หรือ _ALL สำหรับ alignment เต็มรูปแบบ
     * @param  basis rotation vector ที่ใช้คำนวณ tare
     */
    Massmore_BNO08x_status_t tareNow(uint8_t axes = MASSMORE_BNO08X_TARE_AXIS_ALL,
                              Massmore_BNO08x_tare_basis_t basis = MASSMORE_BNO08X_TARE_BASIS_ROTATION_VECTOR);

    /*! @brief บันทึก tare ปัจจุบันลง System Orientation FRS record */
    Massmore_BNO08x_status_t persistTare();

    /*! @brief ล้าง tare ที่บันทึกไว้ (reorientation = identity) */
    Massmore_BNO08x_status_t clearTare();

    /* ===================================================================
     * SECTION 9 — Power, reset and low level access
     * =================================================================== */

    /*! @brief Soft reset ผ่าน SHTP executable channel (block ~100 ms) */
    Massmore_BNO08x_status_t softReset();

    /*! @brief Pulse ขา NRST — ใช้ได้เมื่อส่ง rstPin มาตอน begin() */
    Massmore_BNO08x_status_t hardwareReset();

    /*! @brief Executable "on": เปิด sensor ทุกตัวที่ config ไว้กลับมา */
    Massmore_BNO08x_status_t modeOn();
    /*! @brief Executable "sleep": เหลือเฉพาะ wake/always-on sensor */
    Massmore_BNO08x_status_t modeSleep();

    /*! @brief Pulse ขา PS0/WAKE (pad P0) — SPI เท่านั้น — เพื่อปลุกชิป */
    void wake();

    /*! @brief ขอชนิด oscillator (command 10) — ผลอยู่ใน getOscillatorType() */
    Massmore_BNO08x_status_t requestOscillatorType();
    uint8_t getOscillatorType() const { return _oscillatorType; }

    /*! @brief ขอ error queue จากชิป (command 1) */
    Massmore_BNO08x_status_t requestErrorList();
    /*! @brief จำนวน error จาก requestErrorList() ล่าสุด */
    uint8_t getErrorCount() const { return _errorCount; }

    /* ---- FRS: flash record system ของชิป -------------------------------- */
    /*!
     * @brief  อ่าน FRS record
     * @param  recordId   MASSMORE_BNO08X_FRS_*
     * @param  dataOut    buffer รับ 32-bit words
     * @param  maxWords   ความจุของ dataOut (words)
     * @param  wordsRead  จำนวน words ที่อ่านได้จริง
     */
    Massmore_BNO08x_status_t readFrsRecord(uint16_t recordId, uint32_t *dataOut,
                                    uint16_t maxWords, uint16_t &wordsRead,
                                    uint32_t timeoutMs = 500);

    /*!
     * @brief  เขียน FRS record (ลบแล้วเขียนทั้ง record)
     * @warning เขียน calibration / orientation record ผิดอาจทำให้ fusion output เพี้ยน
     *          จนกว่าจะกู้คืน — อ่านเก็บไว้ก่อนเสมอ
     */
    Massmore_BNO08x_status_t writeFrsRecord(uint16_t recordId, const uint32_t *data,
                                     uint16_t words, uint32_t timeoutMs = 2000);

    /*! @brief อ่าน metadata record ของ sensor (range, resolution, Q points…) */
    Massmore_BNO08x_status_t readSensorMetadata(uint16_t metadataRecordId,
                                         uint32_t *dataOut, uint16_t maxWords,
                                         uint16_t &wordsRead);

    /* ---- Raw SHTP escape hatch ---------------------------------------- */
    /*! @brief ส่ง cargo ใด ๆ บน channel ใด ๆ */
    Massmore_BNO08x_status_t sendPacket(uint8_t channel, const uint8_t *data, uint16_t len);
    /*! @brief Pointer ไปยัง payload ของ cargo ล่าสุดที่รับได้ */
    const uint8_t *getRawPacket(uint16_t &len, uint8_t &channel) const;

    /*! @brief Error code ล่าสุดที่ driver บันทึกไว้ */
    Massmore_BNO08x_status_t lastError() const { return _lastError; }
    /*! @brief เหมือน lastError() (คงไว้เพื่อความเข้ากันได้) */
    Massmore_BNO08x_status_t getLastError() const { return _lastError; }

private:
    /* ---- transport ---------------------------------------------------- */
    Massmore_BNO08x_bus_t _busType;
    TwoWire  *_i2c;
    SPIClass *_spi;
    Stream   *_uart;
    Stream   *_dbg;

    uint8_t  _i2cAddr;
    uint16_t _i2cChunk;        //!< largest safe I2C payload chunk on this core
    uint32_t _spiSpeed;
    int8_t   _csPin, _intPin, _rstPin, _wakePin;

    /* ---- SHTP state --------------------------------------------------- */
    uint8_t  _rxBuf[MASSMORE_BNO08X_MAX_PACKET];
    uint16_t _rxLen;           //!< payload length (header excluded)
    uint8_t  _rxChannel;
    uint8_t  _rxSeq;
    uint8_t  _txBuf[MASSMORE_BNO08X_MAX_PACKET];
    uint8_t  _seqNum[6];       //!< one outgoing sequence number per channel
    uint8_t  _cmdSeqNum;       //!< sequence number inside 0xF2 command requests

    Massmore_BNO08x_status_t _lastError;
    Massmore_BNO08x_auth_t   _lastAuth;

    /* ---- decoded data ------------------------------------------------- */
    Massmore_BNO08x_quat_t  _quat;
    Massmore_BNO08x_vec3_t  _accel, _gyro, _mag, _linAccel, _gravity;
    Massmore_BNO08x_vec3_t  _gyroBias, _magBias, _angVel;
    Massmore_BNO08x_vec3i_t _rawAccel, _rawGyro, _rawMag;
    int16_t          _rawGyroTemp;
    uint32_t         _rawAccelTimestamp, _rawGyroTimestamp, _rawMagTimestamp;

    float    _pressure, _ambientLight, _humidity, _proximity, _temperature;

    uint32_t _stepCount;
    uint8_t  _tapFlags;
    uint16_t _shakeFlags;
    uint16_t _heartRate;
    uint8_t  _sleepState;
    bool     _sigMotion, _flip, _pickup, _tilt, _pocket, _circle, _stepDetected, _stabilityChanged;
    Massmore_BNO08x_stability_t _stability;
    uint8_t  _activityMostLikely;
    uint8_t  _activityConfidence[MASSMORE_BNO08X_ACTIVITY_COUNT];

    uint64_t _timestampUs;
    int32_t  _timebaseDelta100us;   //!< signed, 100 us ticks — [1] Figure 1-35
    uint32_t _rxHostMicros;         //!< micros() when the packet was received
    uint8_t  _lastReportId;
    uint8_t  _lastReportSeq;

    /* one accuracy nibble and one "new" bit per possible report ID (0x00-0x3F) */
    uint8_t  _accuracyTable[0x40];
    uint8_t  _newFlags[8];        //!< bitmap, 64 report IDs
    uint32_t _intervals[0x40];    //!< last known report interval per sensor
    /* Report lengths this device published in its SHTP advertisement.
     * Per instance, so two sensors in one sketch cannot corrupt each
     * other's table. 0 = not learned, use the fallback. */
    uint8_t  _advertReportLen[0x40];

    /* ---- command / query results -------------------------------------- */
    Massmore_BNO08x_product_id_t _productId;                                 //!< SH-2 application entry
    Massmore_BNO08x_product_id_t _productIds[MASSMORE_BNO08X_MAX_PRODUCT_IDS];
    uint8_t               _productIdCount;
    uint8_t  _calibrationStatus;
    uint8_t  _oscillatorType;
    uint8_t  _errorCount;
    bool     _frsReadDone;
    bool     _frsReadError;
    uint16_t _frsWordsRead;
    uint32_t *_frsTarget;
    uint16_t _frsTargetMax;
    bool     _frsWriteDone;
    bool     _frsWriteWantMore;
    uint8_t  _frsWriteStatus;
    bool     _resetComplete;
    bool     _getFeatureResponse;

    void (*_reportCb)(uint8_t, void *);
    void  *_reportCbCtx;

    /* ---- internals ---------------------------------------------------- */
    bool  i2cProbe(uint8_t address);
    bool  waitForInt(uint32_t timeoutMs);
    bool  receivePacket();
    bool  i2cReceivePacket();
    bool  spiReceivePacket();
    bool  uartReceivePacket();
    bool  i2cSendPacket(uint8_t channel, uint16_t payloadLen);
    bool  spiSendPacket(uint8_t channel, uint16_t payloadLen);
    bool  uartSendPacket(uint8_t channel, uint16_t payloadLen);
    bool  txPacket(uint8_t channel, uint16_t payloadLen);
    bool  ensureEnabled(uint8_t sensorId);
    Massmore_BNO08x_auth_t verifyChipInternal();

    void  parsePacket();
    void  parseInputReports(bool wakeChannel);
    void  parseControlReport();
    void  parseCommandResponse();
    void  parseProductIdResponse();
    void  parseFrsReadResponse();
    void  parseGyroRvPacket();
    uint16_t parseOneSensorReport(uint16_t offset);

    Massmore_BNO08x_status_t sendCommand(uint8_t command, const uint8_t *p, uint8_t pLen);

    void  markNew(uint8_t id);
    void  setAccuracy(uint8_t id, uint8_t acc);
    void  resetState();
    void  applyResetSettleDelay();
    void  dbgPrintf(const char *fmt, ...);

    static inline uint16_t rd16(const uint8_t *p) {
        return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
    }
    static inline int16_t rds16(const uint8_t *p) {
        return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
    }
    static inline uint32_t rd32(const uint8_t *p) {
        return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
               ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
    }
};

#endif /* MASSMORE_BNO08X_H */
