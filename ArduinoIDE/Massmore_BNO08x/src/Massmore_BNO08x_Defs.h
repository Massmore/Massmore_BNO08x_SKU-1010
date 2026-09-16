/*!
 * @file  Massmore_BNO08x_Defs.h
 * @brief ค่าคงที่ของ Protocol, enum และ struct สำหรับ BNO085 / BNO086
 *
 * ทุกค่าในไฟล์นี้อ้างอิงจากเอกสารทางการของผู้ผลิตชิป:
 *   [1] BNO08X Datasheet, CEVA doc 1000-3927 rev 1.16
 *   [2] SH-2 Reference Manual, CEVA doc 1000-3625
 *   [3] Sensor Hub Transport Protocol (SHTP), CEVA doc 1000-3535 rev 1.10
 *   [4] BNO080/BNO085 Tare Function Usage Guide, CEVA doc 1000-4045 rev 1.3
 *   [5] BNO080/BNO085 Sensor Calibration Procedure, CEVA doc 1000-4044
 *
 * Naming: ทุก macro / enum / typedef ขึ้นต้นด้วย MASSMORE_BNO08X_ หรือ
 * Massmore_BNO08x_ เพื่อไม่ชนกับไลบรารีอื่นของ Massmore และของเจ้าอื่น
 *
 * Massmore_BNO08x — Designed and Manufactured by Massmore (https://www.massmore.shop)
 * SPDX-License-Identifier: MIT
 */

#ifndef MASSMORE_BNO08X_DEFS_H
#define MASSMORE_BNO08X_DEFS_H

#include <Arduino.h>
#include <stdint.h>

/* ===========================================================================
 * Library identification
 * ========================================================================= */
#define MASSMORE_BNO08X_VERSION_MAJOR 2
#define MASSMORE_BNO08X_VERSION_MINOR 0
#define MASSMORE_BNO08X_VERSION_PATCH 0
#define MASSMORE_BNO08X_VERSION_STR   "2.0.0"

/* ===========================================================================
 * I2C addresses — Datasheet [1] §1.2.3, Figure 1-12
 * 7-bit address คือ 100101<SA0> → 0x4A เมื่อ SA0 = 0, 0x4B เมื่อ SA0 = 1
 * บนบอร์ด Massmore Halley V2 ขา SA0 คือ pad ที่พิมพ์ว่า DI
 * ค่า default ของบอร์ด Massmore คือ 0x4A (ดู Massmore_Library_Request)
 * ========================================================================= */
#define MASSMORE_BNO08X_I2C_ADDR_LOW   0x4A  //!< DI (SA0) = LOW  — Massmore board default
#define MASSMORE_BNO08X_I2C_ADDR_HIGH  0x4B  //!< DI (SA0) = HIGH — alternate address
#define MASSMORE_BNO08X_I2C_ADDR_DEF   MASSMORE_BNO08X_I2C_ADDR_LOW
#define MASSMORE_BNO08X_I2C_ADDR_ALT   MASSMORE_BNO08X_I2C_ADDR_HIGH

/* Bootloader (DFU) addresses เมื่อขา BOOTN (pad BT) ถูกดึง LOW ตอน reset — [1] §1.4 */
#define MASSMORE_BNO08X_BOOTLOADER_ADDR_LOW   0x28
#define MASSMORE_BNO08X_BOOTLOADER_ADDR_HIGH  0x29

/* ===========================================================================
 * Buffer sizing
 * BNO08x ไม่ส่ง cargo ใหญ่กว่าไม่กี่ร้อย byte ในการใช้งานปกติ
 *   - ESP32 / ESP32-S3 : 300 byte ครอบคลุม sensor report, SHTP advertisement
 *                        และ FRS read response ทั้งหมดอย่างสบาย
 *   - AVR ATmega328P   : SRAM 2 KB จึงลดเหลือ 128 byte — เพียงพอสำหรับ sensor
 *                        report ทุกชนิด (ยาวสุด 16 byte) และ Product ID /
 *                        FRS response ส่วน SHTP advertisement จะถูกตัดท้าย
 *                        ซึ่ง driver จัดการได้ (ใช้ fallback report-length table)
 * Override ได้ด้วย -D MASSMORE_BNO08X_MAX_PACKET=<n>
 * ========================================================================= */
#ifndef MASSMORE_BNO08X_MAX_PACKET
  #if defined(__AVR__)
    #define MASSMORE_BNO08X_MAX_PACKET 128
  #else
    #define MASSMORE_BNO08X_MAX_PACKET 300
  #endif
#endif

/* ===========================================================================
 * Product ID responses
 * Product ID Request หนึ่งครั้งจะได้ Product ID Response กลับมา "หลายชุด"
 * (หนึ่งชุดต่อ firmware image ที่ชิปมี) SH-2 application เป็นเพียงชุดหนึ่ง
 * และไม่ได้มาเป็นชุดแรกเสมอ driver จึงเก็บไว้ทุกชุด
 * ========================================================================= */
#ifndef MASSMORE_BNO08X_MAX_PRODUCT_IDS
  #if defined(__AVR__)
    #define MASSMORE_BNO08X_MAX_PRODUCT_IDS 3
  #else
    #define MASSMORE_BNO08X_MAX_PRODUCT_IDS 5
  #endif
#endif

/* Report interval ที่ Simple Blocking API ใช้เปิด sensor ให้อัตโนมัติ (50 Hz) */
#ifndef MASSMORE_BNO08X_SIMPLE_INTERVAL_US
#define MASSMORE_BNO08X_SIMPLE_INTERVAL_US 20000UL
#endif

/* ===========================================================================
 * SHTP channels — Datasheet [1] §1.3.1
 * ========================================================================= */
typedef enum {
    MASSMORE_BNO08X_CH_COMMAND      = 0,  //!< SHTP command channel (advertisement, errors)
    MASSMORE_BNO08X_CH_EXECUTABLE   = 1,  //!< reset / on / sleep
    MASSMORE_BNO08X_CH_CONTROL      = 2,  //!< SH-2 control: set feature, commands, FRS
    MASSMORE_BNO08X_CH_INPUT_REPORT = 3,  //!< normal (non-wake) sensor reports
    MASSMORE_BNO08X_CH_WAKE_REPORT  = 4,  //!< wake sensor reports
    MASSMORE_BNO08X_CH_GYRO_RV      = 5   //!< gyro-integrated rotation vector (low latency)
} Massmore_BNO08x_channel_t;

/* ===========================================================================
 * Executable channel commands / responses — Datasheet [1] Figure 1-27
 * ========================================================================= */
#define MASSMORE_BNO08X_EXEC_RESET           1
#define MASSMORE_BNO08X_EXEC_ON              2
#define MASSMORE_BNO08X_EXEC_SLEEP           3
#define MASSMORE_BNO08X_EXEC_RESET_COMPLETE  1  //!< read back on channel 1 after a reset

/* ===========================================================================
 * SH-2 control report IDs — Datasheet [1] Figure 1-30
 * ========================================================================= */
#define MASSMORE_BNO08X_REPORT_COMMAND_RESPONSE   0xF1
#define MASSMORE_BNO08X_REPORT_COMMAND_REQUEST    0xF2
#define MASSMORE_BNO08X_REPORT_FRS_READ_RESPONSE  0xF3
#define MASSMORE_BNO08X_REPORT_FRS_READ_REQUEST   0xF4
#define MASSMORE_BNO08X_REPORT_FRS_WRITE_RESPONSE 0xF5
#define MASSMORE_BNO08X_REPORT_FRS_WRITE_DATA     0xF6
#define MASSMORE_BNO08X_REPORT_FRS_WRITE_REQUEST  0xF7
#define MASSMORE_BNO08X_REPORT_PRODUCT_ID_RESP    0xF8
#define MASSMORE_BNO08X_REPORT_PRODUCT_ID_REQ     0xF9
#define MASSMORE_BNO08X_REPORT_TIMESTAMP_REBASE   0xFA
#define MASSMORE_BNO08X_REPORT_BASE_TIMESTAMP     0xFB
#define MASSMORE_BNO08X_REPORT_GET_FEATURE_RESP   0xFC
#define MASSMORE_BNO08X_REPORT_SET_FEATURE_CMD    0xFD
#define MASSMORE_BNO08X_REPORT_GET_FEATURE_REQ    0xFE

/* ===========================================================================
 * Sensor (feature) report IDs — SH-2 Reference Manual [2] §6.5
 * ใช้ทั้งตอน enable (Set Feature) และตอนระบุชนิดของ input report ที่รับมา
 * ========================================================================= */
typedef enum {
    MASSMORE_BNO08X_SENSOR_ACCELEROMETER            = 0x01, //!< m/s^2, includes gravity
    MASSMORE_BNO08X_SENSOR_GYROSCOPE                = 0x02, //!< rad/s, calibrated
    MASSMORE_BNO08X_SENSOR_MAGNETIC_FIELD           = 0x03, //!< uT, calibrated
    MASSMORE_BNO08X_SENSOR_LINEAR_ACCELERATION      = 0x04, //!< m/s^2, gravity removed
    MASSMORE_BNO08X_SENSOR_ROTATION_VECTOR          = 0x05, //!< 9-axis fused quaternion
    MASSMORE_BNO08X_SENSOR_GRAVITY                  = 0x06, //!< m/s^2
    MASSMORE_BNO08X_SENSOR_GYROSCOPE_UNCAL          = 0x07, //!< rad/s + bias
    MASSMORE_BNO08X_SENSOR_GAME_ROTATION_VECTOR     = 0x08, //!< 6-axis, no magnetometer
    MASSMORE_BNO08X_SENSOR_GEOMAGNETIC_RV           = 0x09, //!< accel + mag, low power
    MASSMORE_BNO08X_SENSOR_PRESSURE                 = 0x0A, //!< hPa (external sensor)
    MASSMORE_BNO08X_SENSOR_AMBIENT_LIGHT            = 0x0B, //!< lux (external sensor)
    MASSMORE_BNO08X_SENSOR_HUMIDITY                 = 0x0C, //!< %RH (external sensor)
    MASSMORE_BNO08X_SENSOR_PROXIMITY                = 0x0D, //!< cm  (external sensor)
    MASSMORE_BNO08X_SENSOR_TEMPERATURE              = 0x0E, //!< degC (external sensor)
    MASSMORE_BNO08X_SENSOR_MAGNETIC_FIELD_UNCAL     = 0x0F, //!< uT + hard-iron bias
    MASSMORE_BNO08X_SENSOR_TAP_DETECTOR             = 0x10,
    MASSMORE_BNO08X_SENSOR_STEP_COUNTER             = 0x11,
    MASSMORE_BNO08X_SENSOR_SIGNIFICANT_MOTION       = 0x12,
    MASSMORE_BNO08X_SENSOR_STABILITY_CLASSIFIER     = 0x13,
    MASSMORE_BNO08X_SENSOR_RAW_ACCELEROMETER        = 0x14, //!< ADC counts
    MASSMORE_BNO08X_SENSOR_RAW_GYROSCOPE            = 0x15, //!< ADC counts
    MASSMORE_BNO08X_SENSOR_RAW_MAGNETOMETER         = 0x16, //!< ADC counts
    MASSMORE_BNO08X_SENSOR_STEP_DETECTOR            = 0x18,
    MASSMORE_BNO08X_SENSOR_SHAKE_DETECTOR           = 0x19,
    MASSMORE_BNO08X_SENSOR_FLIP_DETECTOR            = 0x1A,
    MASSMORE_BNO08X_SENSOR_PICKUP_DETECTOR          = 0x1B,
    MASSMORE_BNO08X_SENSOR_STABILITY_DETECTOR       = 0x1C,
    MASSMORE_BNO08X_SENSOR_ACTIVITY_CLASSIFIER      = 0x1E, //!< personal activity classifier
    MASSMORE_BNO08X_SENSOR_SLEEP_DETECTOR           = 0x1F,
    MASSMORE_BNO08X_SENSOR_TILT_DETECTOR            = 0x20,
    MASSMORE_BNO08X_SENSOR_POCKET_DETECTOR          = 0x21,
    MASSMORE_BNO08X_SENSOR_CIRCLE_DETECTOR          = 0x22,
    MASSMORE_BNO08X_SENSOR_HEART_RATE_MONITOR       = 0x23,
    MASSMORE_BNO08X_SENSOR_ARVR_STABILIZED_RV       = 0x28, //!< jump-free rotation vector
    MASSMORE_BNO08X_SENSOR_ARVR_STABILIZED_GRV      = 0x29, //!< jump-free game RV
    MASSMORE_BNO08X_SENSOR_GYRO_INTEGRATED_RV       = 0x2A, //!< up to 1 kHz, channel 5
    MASSMORE_BNO08X_SENSOR_MOTION_REQUEST           = 0x2B, //!< BNO086 only
    MASSMORE_BNO08X_SENSOR_OPTICAL_FLOW             = 0x2C, //!< BNO086 only
    MASSMORE_BNO08X_SENSOR_DEAD_RECKONING_POSE      = 0x2D  //!< BNO086 only
} Massmore_BNO08x_sensor_id_t;

/* ===========================================================================
 * Command IDs ที่ส่งใน 0xF2 Command Request — SH-2 Ref Manual [2] §6.4
 * ========================================================================= */
#define MASSMORE_BNO08X_CMD_ERRORS             1
#define MASSMORE_BNO08X_CMD_COUNTER            2
#define MASSMORE_BNO08X_CMD_TARE               3
#define MASSMORE_BNO08X_CMD_INITIALIZE         4
#define MASSMORE_BNO08X_CMD_SAVE_DCD           6
#define MASSMORE_BNO08X_CMD_ME_CALIBRATE       7
#define MASSMORE_BNO08X_CMD_DCD_PERIOD_SAVE    9
#define MASSMORE_BNO08X_CMD_OSCILLATOR        10
#define MASSMORE_BNO08X_CMD_CLEAR_DCD         11

/* Tare sub-commands — Tare Usage Guide [4] */
#define MASSMORE_BNO08X_TARE_NOW               0
#define MASSMORE_BNO08X_TARE_PERSIST           1
#define MASSMORE_BNO08X_TARE_SET_REORIENTATION 2

/*! Axis bitmap สำหรับ tareNow() — OR รวมกันได้ */
typedef enum {
    MASSMORE_BNO08X_TARE_AXIS_X   = 0x01,
    MASSMORE_BNO08X_TARE_AXIS_Y   = 0x02,
    MASSMORE_BNO08X_TARE_AXIS_Z   = 0x04,
    MASSMORE_BNO08X_TARE_AXIS_ALL = 0x07
} Massmore_BNO08x_tare_axis_t;

/*! Rotation vector ที่ใช้เป็นฐานคำนวณ tare — Tare Usage Guide [4] */
typedef enum {
    MASSMORE_BNO08X_TARE_BASIS_ROTATION_VECTOR       = 0,
    MASSMORE_BNO08X_TARE_BASIS_GAMING_RV             = 1,
    MASSMORE_BNO08X_TARE_BASIS_GEOMAGNETIC_RV        = 2,
    MASSMORE_BNO08X_TARE_BASIS_GYRO_INTEGRATED_RV    = 3,
    MASSMORE_BNO08X_TARE_BASIS_ARVR_STABILIZED_RV    = 4,
    MASSMORE_BNO08X_TARE_BASIS_ARVR_STABILIZED_GRV   = 5
} Massmore_BNO08x_tare_basis_t;

/*! เป้าหมายของ ME (MotionEngine) calibration command */
typedef enum {
    MASSMORE_BNO08X_CAL_ACCEL         = 0,
    MASSMORE_BNO08X_CAL_GYRO          = 1,
    MASSMORE_BNO08X_CAL_MAG           = 2,
    MASSMORE_BNO08X_CAL_PLANAR_ACCEL  = 3,
    MASSMORE_BNO08X_CAL_ACCEL_GYRO_MAG= 4,
    MASSMORE_BNO08X_CAL_STOP          = 5   //!< disable all dynamic calibration
} Massmore_BNO08x_calibrate_target_t;

/* ===========================================================================
 * FRS (Flash Record System) record IDs — Datasheet [1] Figure 1-31
 * ========================================================================= */
#define MASSMORE_BNO08X_FRS_STATIC_CAL_AGM        0x7979
#define MASSMORE_BNO08X_FRS_NOMINAL_CAL_AGM       0x4D4D
#define MASSMORE_BNO08X_FRS_STATIC_CAL_SRA        0x8A8A
#define MASSMORE_BNO08X_FRS_NOMINAL_CAL_SRA       0x4E4E
#define MASSMORE_BNO08X_FRS_DYNAMIC_CAL           0x1F1F
#define MASSMORE_BNO08X_FRS_ME_POWER_MGMT         0xD3E2
#define MASSMORE_BNO08X_FRS_SYSTEM_ORIENTATION    0x2D3E
#define MASSMORE_BNO08X_FRS_ACCEL_ORIENTATION     0x2D41
#define MASSMORE_BNO08X_FRS_GYRO_ORIENTATION      0x2D46
#define MASSMORE_BNO08X_FRS_MAG_ORIENTATION       0x2D4C
#define MASSMORE_BNO08X_FRS_ARVR_STAB_RV          0x3E2D
#define MASSMORE_BNO08X_FRS_ARVR_STAB_GRV         0x3E2E
#define MASSMORE_BNO08X_FRS_SIG_MOTION_CONFIG     0xC274
#define MASSMORE_BNO08X_FRS_SHAKE_DETECT_CONFIG   0x7D7D
#define MASSMORE_BNO08X_FRS_MAX_FUSION_PERIOD     0xD7D7
#define MASSMORE_BNO08X_FRS_SERIAL_NUMBER         0x4B4B
#define MASSMORE_BNO08X_FRS_ES_PRESSURE_CAL       0x39AF
#define MASSMORE_BNO08X_FRS_ES_TEMPERATURE_CAL    0x4D20
#define MASSMORE_BNO08X_FRS_ES_HUMIDITY_CAL       0x1AC9
#define MASSMORE_BNO08X_FRS_ES_AMBIENT_LIGHT_CAL  0x39B1
#define MASSMORE_BNO08X_FRS_ES_PROXIMITY_CAL      0x4DA2
#define MASSMORE_BNO08X_FRS_ALS_CAL               0xD401
#define MASSMORE_BNO08X_FRS_PROXIMITY_CAL         0xD402
#define MASSMORE_BNO08X_FRS_STABILITY_DET_CONFIG  0xED85
#define MASSMORE_BNO08X_FRS_USER_RECORD           0x74B4
#define MASSMORE_BNO08X_FRS_ME_TIME_SOURCE        0xD403
#define MASSMORE_BNO08X_FRS_GYRO_INTEGRATED_RV    0xA1A2

/* Sensor metadata records (ใช้กับ readSensorMetadata) */
#define MASSMORE_BNO08X_FRS_META_ACCELEROMETER    0xE302
#define MASSMORE_BNO08X_FRS_META_GYRO_CALIBRATED  0xE306
#define MASSMORE_BNO08X_FRS_META_MAG_CALIBRATED   0xE309
#define MASSMORE_BNO08X_FRS_META_ROTATION_VECTOR  0xE30B

/* Q point ที่คาดหวังใน Rotation Vector metadata (word 7) — [2] §4.3 / §6.5.18
 * ใช้เป็น "signature" ยืนยันว่า SH-2 MotionEngine ทำงานจริงบนชิป */
#define MASSMORE_BNO08X_META_RV_QPOINT1  14
#define MASSMORE_BNO08X_META_RV_QPOINT2  12

/* ===========================================================================
 * Set Feature flags — SH-2 Reference Manual [2] §6.5.4
 * ========================================================================= */
#define MASSMORE_BNO08X_FEATURE_FLAG_NONE              0x00
#define MASSMORE_BNO08X_FEATURE_FLAG_CHANGE_SENS_ENA   0x01 //!< report on change only
#define MASSMORE_BNO08X_FEATURE_FLAG_CHANGE_SENS_REL   0x02 //!< 0 = absolute, 1 = relative
#define MASSMORE_BNO08X_FEATURE_FLAG_WAKE_ENABLED      0x04 //!< route to wake channel (4)
#define MASSMORE_BNO08X_FEATURE_FLAG_ALWAYS_ON         0x08
#define MASSMORE_BNO08X_FEATURE_FLAG_SNIFF_ENABLED     0x10

/* ===========================================================================
 * Result / error codes (ErrorCode) — ดูข้อความอ่านง่ายได้จาก statusToString()
 * ========================================================================= */
typedef enum {
    MASSMORE_BNO08X_OK               =  0,  //!< success
    MASSMORE_BNO08X_ERR_IO           = -1,  //!< bus level failure (NACK, SPI timeout…)
    MASSMORE_BNO08X_ERR_TIMEOUT      = -2,  //!< the device did not answer in time
    MASSMORE_BNO08X_ERR_BAD_PARAM    = -3,  //!< caller passed an out-of-range argument
    MASSMORE_BNO08X_ERR_NO_DEVICE    = -4,  //!< nothing answered on the bus (NOT_FOUND)
    MASSMORE_BNO08X_ERR_BAD_RESPONSE = -5,  //!< malformed / unexpected packet
    MASSMORE_BNO08X_ERR_NOT_READY    = -6,  //!< begin() has not been called
    MASSMORE_BNO08X_ERR_UNSUPPORTED  = -7,  //!< operation not valid for this transport
    MASSMORE_BNO08X_ERR_WRONG_ID     = -8   //!< Product ID ไม่ตรงกับ BNO08x (WRONG_ID)
} Massmore_BNO08x_status_t;

/* ===========================================================================
 * Chip identity / authenticity
 * ========================================================================= */

/*! Product ID Response ที่ถอดรหัสแล้ว — Datasheet [1] Figure 1-29 */
typedef struct {
    bool     valid;           //!< true once a 0xF8 response has been decoded
    uint8_t  resetCause;      //!< 0=n/a 1=POR 2=internal 3=watchdog 4=external 5=other
    uint8_t  swVersionMajor;
    uint8_t  swVersionMinor;
    uint16_t swVersionPatch;
    uint32_t swPartNumber;    //!< firmware part number, e.g. 10003606
    uint32_t swBuildNumber;
} Massmore_BNO08x_product_id_t;

/*!
 * @brief ผลของ verifyChip()
 *
 * สิ่งที่การตรวจนี้ "พิสูจน์ได้จริง": BNO08x ไม่มี cryptographic attestation
 * จึงไม่มีไลบรารีใดพิสูจน์ของแท้ทางคณิตศาสตร์ได้ สิ่งที่ตรวจได้คือชิปบนบอร์ด
 * ทำงานเหมือน BNO08x ของแท้ในระดับ Protocol: ส่ง SHTP advertisement ถูกต้อง,
 * ตอบ Product ID Request ด้วย firmware part number / version ที่สมเหตุสมผล
 * และมี serial number / metadata อ่านได้จาก FRS ชิปปลอมหรือชิปติดฉลากผิด
 * (BNO055 ที่แปะเป็น BNO086, die เปล่า, บอร์ดไม่มีเซ็นเซอร์) จะตกอย่างน้อยหนึ่งข้อ
 */
typedef enum {
    MASSMORE_BNO08X_AUTH_OK          = 0, //!< valid response AND a known-good firmware part number
    MASSMORE_BNO08X_AUTH_UNKNOWN_FW  = 1, //!< valid BNO08x response, firmware part number not in our table
    MASSMORE_BNO08X_AUTH_BAD_VERSION = 2, //!< responded, but the version fields are implausible
    MASSMORE_BNO08X_AUTH_NO_RESPONSE = 3, //!< no Product ID response — not a BNO08x, or wiring/address wrong
    MASSMORE_BNO08X_AUTH_BAD_RESPONSE= 4  //!< a response arrived but it is malformed
} Massmore_BNO08x_auth_t;

/* ===========================================================================
 * Sensor data containers
 * ========================================================================= */

/*! Accuracy field ใน status byte ของทุก sensor report (bits 1:0) */
typedef enum {
    MASSMORE_BNO08X_ACCURACY_UNRELIABLE = 0,
    MASSMORE_BNO08X_ACCURACY_LOW        = 1,
    MASSMORE_BNO08X_ACCURACY_MEDIUM     = 2,
    MASSMORE_BNO08X_ACCURACY_HIGH       = 3
} Massmore_BNO08x_accuracy_t;

typedef struct { float x, y, z; }                    Massmore_BNO08x_vec3_t;
typedef struct { int16_t x, y, z; }                  Massmore_BNO08x_vec3i_t;
typedef struct { float i, j, k, real, accuracy; }    Massmore_BNO08x_quat_t;
typedef struct { float roll, pitch, yaw; }           Massmore_BNO08x_euler_t;

/*!
 * @brief ชุดข้อมูลรวมสำหรับ Simple Blocking API (readAll) และ getReadings()
 *        ค่าทุกตัวเป็นค่าล่าสุดที่ driver ถอดรหัสไว้
 */
typedef struct {
    Massmore_BNO08x_quat_t  quat;        //!< unit quaternion จาก rotation vector ล่าสุด
    Massmore_BNO08x_euler_t eulerDeg;    //!< roll / pitch / yaw เป็นองศา (-180..180)
    float                   headingDeg;  //!< yaw แบบเข็มทิศ 0..360 องศา
    Massmore_BNO08x_vec3_t  accel;       //!< m/s^2 (รวม gravity)
    Massmore_BNO08x_vec3_t  gyro;        //!< rad/s
    Massmore_BNO08x_vec3_t  mag;         //!< uT
    Massmore_BNO08x_vec3_t  linearAccel; //!< m/s^2 (ตัด gravity ออก) — ถ้า enable ไว้
    Massmore_BNO08x_vec3_t  gravity;     //!< m/s^2 — ถ้า enable ไว้
    Massmore_BNO08x_accuracy_t accuracyRV;   //!< accuracy ของ rotation vector
    Massmore_BNO08x_accuracy_t accuracyMag;  //!< accuracy ของ magnetometer
    uint64_t                timestampUs; //!< host micros() ของ report ล่าสุด
} Massmore_BNO08x_reading_t;

/*! Stability classifier output — SH-2 Ref Manual [2] §6.5.20 */
typedef enum {
    MASSMORE_BNO08X_STABILITY_UNKNOWN     = 0,
    MASSMORE_BNO08X_STABILITY_ON_TABLE    = 1,
    MASSMORE_BNO08X_STABILITY_STATIONARY  = 2,
    MASSMORE_BNO08X_STABILITY_STABLE      = 3,
    MASSMORE_BNO08X_STABILITY_MOTION      = 4,
    MASSMORE_BNO08X_STABILITY_RESERVED    = 5
} Massmore_BNO08x_stability_t;

/*! Personal activity classifier states — SH-2 Ref Manual [2] §6.5.36 */
typedef enum {
    MASSMORE_BNO08X_ACTIVITY_UNKNOWN   = 0,
    MASSMORE_BNO08X_ACTIVITY_IN_VEHICLE= 1,
    MASSMORE_BNO08X_ACTIVITY_ON_BICYCLE= 2,
    MASSMORE_BNO08X_ACTIVITY_ON_FOOT   = 3,
    MASSMORE_BNO08X_ACTIVITY_STILL     = 4,
    MASSMORE_BNO08X_ACTIVITY_TILTING   = 5,
    MASSMORE_BNO08X_ACTIVITY_WALKING   = 6,
    MASSMORE_BNO08X_ACTIVITY_RUNNING   = 7,
    MASSMORE_BNO08X_ACTIVITY_ON_STAIRS = 8,
    MASSMORE_BNO08X_ACTIVITY_COUNT     = 9
} Massmore_BNO08x_activity_t;

/*! Tap detector flag bits — SH-2 Ref Manual [2] §6.5.17 */
#define MASSMORE_BNO08X_TAP_X_POS   0x01
#define MASSMORE_BNO08X_TAP_X_NEG   0x02
#define MASSMORE_BNO08X_TAP_Y_POS   0x04
#define MASSMORE_BNO08X_TAP_Y_NEG   0x08
#define MASSMORE_BNO08X_TAP_Z_POS   0x10
#define MASSMORE_BNO08X_TAP_Z_NEG   0x20
#define MASSMORE_BNO08X_TAP_DOUBLE  0x40

/*! Shake detector flag bits — SH-2 Ref Manual [2] §6.5.32 */
#define MASSMORE_BNO08X_SHAKE_X     0x01
#define MASSMORE_BNO08X_SHAKE_Y     0x02
#define MASSMORE_BNO08X_SHAKE_Z     0x04

/*! Physical transport ที่ driver กำลังใช้ */
typedef enum {
    MASSMORE_BNO08X_BUS_NONE = 0,
    MASSMORE_BNO08X_BUS_I2C  = 1,
    MASSMORE_BNO08X_BUS_SPI  = 2,
    MASSMORE_BNO08X_BUS_UART = 3   //!< SHTP over UART (ไม่ใช่ UART-RVC — ดู Massmore_BNO08x_RVC.h)
} Massmore_BNO08x_bus_t;

/* ===========================================================================
 * Fixed point helpers — Q point scaling, SH-2 Ref Manual [2] §6.5
 * value = raw * 2^-Q
 * ========================================================================= */
#define MASSMORE_BNO08X_Q_TO_FLOAT(raw, q) ((float)(raw) * (1.0f / (float)(1UL << (q))))

/* Q points ที่ BNO08x sensor reports ใช้ */
#define MASSMORE_BNO08X_Q_ACCEL        8   //!< m/s^2  (accel, linear accel, gravity)
#define MASSMORE_BNO08X_Q_GYRO         9   //!< rad/s
#define MASSMORE_BNO08X_Q_MAG          4   //!< uT
#define MASSMORE_BNO08X_Q_QUAT        14   //!< unit quaternion components
#define MASSMORE_BNO08X_Q_QUAT_ACC    12   //!< rotation vector accuracy, radians
#define MASSMORE_BNO08X_Q_ANG_VEL     10   //!< gyro-integrated RV angular velocity, rad/s
#define MASSMORE_BNO08X_Q_PRESSURE    20   //!< hPa
#define MASSMORE_BNO08X_Q_AMBIENT      8   //!< lux
#define MASSMORE_BNO08X_Q_HUMIDITY     8   //!< %RH
#define MASSMORE_BNO08X_Q_PROXIMITY    4   //!< cm
#define MASSMORE_BNO08X_Q_TEMPERATURE  7   //!< degC

/* Report interval helpers (microseconds) */
#define MASSMORE_BNO08X_HZ_TO_US(hz)   ((uint32_t)(1000000UL / (uint32_t)(hz)))
#define MASSMORE_BNO08X_INTERVAL_1HZ    1000000UL
#define MASSMORE_BNO08X_INTERVAL_10HZ    100000UL
#define MASSMORE_BNO08X_INTERVAL_50HZ     20000UL
#define MASSMORE_BNO08X_INTERVAL_100HZ    10000UL
#define MASSMORE_BNO08X_INTERVAL_200HZ     5000UL
#define MASSMORE_BNO08X_INTERVAL_400HZ     2500UL
#define MASSMORE_BNO08X_INTERVAL_1000HZ    1000UL

/* ===========================================================================
 * Physical full-scale ranges — Datasheet [1] §6 (ใช้โดย Factory Test range check)
 * ========================================================================= */
#define MASSMORE_BNO08X_RANGE_ACCEL_MS2     78.4f    //!< ±8 g full scale
#define MASSMORE_BNO08X_RANGE_GYRO_RADS     34.9f    //!< ±2000 dps full scale
#define MASSMORE_BNO08X_RANGE_MAG_UT_XY    1300.0f   //!< ±1300 uT (X, Y)
#define MASSMORE_BNO08X_RANGE_MAG_UT_Z     2500.0f   //!< ±2500 uT (Z)

#endif /* MASSMORE_BNO08X_DEFS_H */
