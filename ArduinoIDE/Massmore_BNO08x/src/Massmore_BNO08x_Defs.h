/*!
 * @file  Massmore_BNO08x_Defs.h
 * @brief Protocol constants, enums and data structures for the BNO085 / BNO086.
 *
 * Every value in this file is taken directly from the official documents:
 *   [1] BNO08X Datasheet, CEVA doc 1000-3927 rev 1.16
 *   [2] SH-2 Reference Manual, CEVA doc 1000-3625
 *   [3] Sensor Hub Transport Protocol (SHTP), CEVA doc 1000-3535 rev 1.10
 *   [4] BNO080/BNO085 Tare Function Usage Guide, CEVA doc 1000-4045 rev 1.3
 *   [5] BNO080/BNO085 Sensor Calibration Procedure, CEVA doc 1000-4044
 *
 * Massmore BNO08x Library — assembled by Massmore (https://www.massmore.shop)
 * SPDX-License-Identifier: MIT
 */

#ifndef MASSMORE_BNO08X_DEFS_H
#define MASSMORE_BNO08X_DEFS_H

#include <Arduino.h>
#include <stdint.h>

/* ===========================================================================
 * Library identification
 * ========================================================================= */
#define MASSMORE_BNO08X_VERSION_MAJOR 1
#define MASSMORE_BNO08X_VERSION_MINOR 0
#define MASSMORE_BNO08X_VERSION_PATCH 2
#define MASSMORE_BNO08X_VERSION_STR   "1.0.2"

/* ===========================================================================
 * I2C addresses — Datasheet [1] §1.2.3, Figure 1-12
 * 7-bit address is 100101<SA0>, i.e. 0x4A when SA0 = 0, 0x4B when SA0 = 1.
 * On the Massmore Halley V2 board that pin is the pad marked DI.
 * ========================================================================= */
#define MASSMORE_BNO08X_I2C_ADDR_LOW   0x4A  //!< DI (SA0) tied low
#define MASSMORE_BNO08X_I2C_ADDR_HIGH  0x4B  //!< DI (SA0) tied high (default)
#define MASSMORE_BNO08X_I2C_ADDR_DEF   MASSMORE_BNO08X_I2C_ADDR_HIGH

/* Bootloader (DFU) addresses when BOOTN (the BT pad) is pulled low at reset
   — [1] §1.4 */
#define MASSMORE_BNO08X_BOOTLOADER_ADDR_LOW   0x28
#define MASSMORE_BNO08X_BOOTLOADER_ADDR_HIGH  0x29

/* ===========================================================================
 * Buffer sizing
 * The BNO08x never sends a cargo larger than a few hundred bytes in normal
 * operation. 300 bytes covers every sensor report, the SHTP advertisement and
 * FRS read responses with plenty of margin, while staying friendly to small
 * MCUs. Override with -D MASSMORE_BNO08X_MAX_PACKET=<n> if you need more.
 * ========================================================================= */
#ifndef MASSMORE_BNO08X_MAX_PACKET
#define MASSMORE_BNO08X_MAX_PACKET 300
#endif

/* ===========================================================================
 * Product ID responses
 * A Product ID Request is answered with SEVERAL Product ID Responses, one per
 * firmware image the part carries — the SH-2 application is only one of them,
 * and it is not always the first to arrive. Keeping just one response is how
 * a genuine part ends up reported as "unknown firmware": you may have kept the
 * bootloader's entry instead of the application's. The driver stores them all.
 * ========================================================================= */
#ifndef MASSMORE_BNO08X_MAX_PRODUCT_IDS
#define MASSMORE_BNO08X_MAX_PRODUCT_IDS 5
#endif

/* ===========================================================================
 * SHTP channels — Datasheet [1] §1.3.1
 * ========================================================================= */
typedef enum {
    MASSMORE_CH_COMMAND      = 0,  //!< SHTP command channel (advertisement, errors)
    MASSMORE_CH_EXECUTABLE   = 1,  //!< reset / on / sleep
    MASSMORE_CH_CONTROL      = 2,  //!< SH-2 control: set feature, commands, FRS
    MASSMORE_CH_INPUT_REPORT = 3,  //!< normal (non-wake) sensor reports
    MASSMORE_CH_WAKE_REPORT  = 4,  //!< wake sensor reports
    MASSMORE_CH_GYRO_RV      = 5   //!< gyro-integrated rotation vector (low latency)
} massmore_channel_t;

/* ===========================================================================
 * Executable channel commands / responses — Datasheet [1] Figure 1-27
 * ========================================================================= */
#define MASSMORE_EXEC_RESET           1
#define MASSMORE_EXEC_ON              2
#define MASSMORE_EXEC_SLEEP           3
#define MASSMORE_EXEC_RESET_COMPLETE  1  //!< read back on channel 1 after a reset

/* ===========================================================================
 * SH-2 control report IDs — Datasheet [1] Figure 1-30
 * ========================================================================= */
#define MASSMORE_REPORT_COMMAND_RESPONSE   0xF1
#define MASSMORE_REPORT_COMMAND_REQUEST    0xF2
#define MASSMORE_REPORT_FRS_READ_RESPONSE  0xF3
#define MASSMORE_REPORT_FRS_READ_REQUEST   0xF4
#define MASSMORE_REPORT_FRS_WRITE_RESPONSE 0xF5
#define MASSMORE_REPORT_FRS_WRITE_DATA     0xF6
#define MASSMORE_REPORT_FRS_WRITE_REQUEST  0xF7
#define MASSMORE_REPORT_PRODUCT_ID_RESP    0xF8
#define MASSMORE_REPORT_PRODUCT_ID_REQ     0xF9
#define MASSMORE_REPORT_TIMESTAMP_REBASE   0xFA
#define MASSMORE_REPORT_BASE_TIMESTAMP     0xFB
#define MASSMORE_REPORT_GET_FEATURE_RESP   0xFC
#define MASSMORE_REPORT_SET_FEATURE_CMD    0xFD
#define MASSMORE_REPORT_GET_FEATURE_REQ    0xFE

/* ===========================================================================
 * Sensor (feature) report IDs — SH-2 Reference Manual [2] §6.5
 * These IDs are used both to *enable* a sensor (Set Feature) and to identify
 * the incoming input reports.
 * ========================================================================= */
typedef enum {
    MASSMORE_SENSOR_ACCELEROMETER            = 0x01, //!< m/s^2, includes gravity
    MASSMORE_SENSOR_GYROSCOPE                = 0x02, //!< rad/s, calibrated
    MASSMORE_SENSOR_MAGNETIC_FIELD           = 0x03, //!< uT, calibrated
    MASSMORE_SENSOR_LINEAR_ACCELERATION      = 0x04, //!< m/s^2, gravity removed
    MASSMORE_SENSOR_ROTATION_VECTOR          = 0x05, //!< 9-axis fused quaternion
    MASSMORE_SENSOR_GRAVITY                  = 0x06, //!< m/s^2
    MASSMORE_SENSOR_GYROSCOPE_UNCAL          = 0x07, //!< rad/s + bias
    MASSMORE_SENSOR_GAME_ROTATION_VECTOR     = 0x08, //!< 6-axis, no magnetometer
    MASSMORE_SENSOR_GEOMAGNETIC_RV           = 0x09, //!< accel + mag, low power
    MASSMORE_SENSOR_PRESSURE                 = 0x0A, //!< hPa (external sensor)
    MASSMORE_SENSOR_AMBIENT_LIGHT            = 0x0B, //!< lux (external sensor)
    MASSMORE_SENSOR_HUMIDITY                 = 0x0C, //!< %RH (external sensor)
    MASSMORE_SENSOR_PROXIMITY                = 0x0D, //!< cm  (external sensor)
    MASSMORE_SENSOR_TEMPERATURE              = 0x0E, //!< degC (external sensor)
    MASSMORE_SENSOR_MAGNETIC_FIELD_UNCAL     = 0x0F, //!< uT + hard-iron bias
    MASSMORE_SENSOR_TAP_DETECTOR             = 0x10,
    MASSMORE_SENSOR_STEP_COUNTER             = 0x11,
    MASSMORE_SENSOR_SIGNIFICANT_MOTION       = 0x12,
    MASSMORE_SENSOR_STABILITY_CLASSIFIER     = 0x13,
    MASSMORE_SENSOR_RAW_ACCELEROMETER        = 0x14, //!< ADC counts
    MASSMORE_SENSOR_RAW_GYROSCOPE            = 0x15, //!< ADC counts
    MASSMORE_SENSOR_RAW_MAGNETOMETER         = 0x16, //!< ADC counts
    MASSMORE_SENSOR_STEP_DETECTOR            = 0x18,
    MASSMORE_SENSOR_SHAKE_DETECTOR           = 0x19,
    MASSMORE_SENSOR_FLIP_DETECTOR            = 0x1A,
    MASSMORE_SENSOR_PICKUP_DETECTOR          = 0x1B,
    MASSMORE_SENSOR_STABILITY_DETECTOR       = 0x1C,
    MASSMORE_SENSOR_ACTIVITY_CLASSIFIER      = 0x1E, //!< personal activity classifier
    MASSMORE_SENSOR_SLEEP_DETECTOR           = 0x1F,
    MASSMORE_SENSOR_TILT_DETECTOR            = 0x20,
    MASSMORE_SENSOR_POCKET_DETECTOR          = 0x21,
    MASSMORE_SENSOR_CIRCLE_DETECTOR          = 0x22,
    MASSMORE_SENSOR_HEART_RATE_MONITOR       = 0x23,
    MASSMORE_SENSOR_ARVR_STABILIZED_RV       = 0x28, //!< jump-free rotation vector
    MASSMORE_SENSOR_ARVR_STABILIZED_GRV      = 0x29, //!< jump-free game RV
    MASSMORE_SENSOR_GYRO_INTEGRATED_RV       = 0x2A, //!< up to 1 kHz, channel 5
    MASSMORE_SENSOR_MOTION_REQUEST           = 0x2B, //!< BNO086 only
    MASSMORE_SENSOR_OPTICAL_FLOW             = 0x2C, //!< BNO086 only
    MASSMORE_SENSOR_DEAD_RECKONING_POSE      = 0x2D  //!< BNO086 only
} massmore_sensor_id_t;

/* ===========================================================================
 * Command IDs, sent inside a 0xF2 Command Request — SH-2 Ref Manual [2] §6.4
 * ========================================================================= */
#define MASSMORE_CMD_ERRORS             1
#define MASSMORE_CMD_COUNTER            2
#define MASSMORE_CMD_TARE               3
#define MASSMORE_CMD_INITIALIZE         4
#define MASSMORE_CMD_SAVE_DCD           6
#define MASSMORE_CMD_ME_CALIBRATE       7
#define MASSMORE_CMD_DCD_PERIOD_SAVE    9
#define MASSMORE_CMD_OSCILLATOR        10
#define MASSMORE_CMD_CLEAR_DCD         11

/* Tare sub-commands — Tare Usage Guide [4] */
#define MASSMORE_TARE_NOW               0
#define MASSMORE_TARE_PERSIST           1
#define MASSMORE_TARE_SET_REORIENTATION 2

/*! Axis bitmap for tareNow(). OR the members together. */
typedef enum {
    MASSMORE_TARE_AXIS_X   = 0x01,
    MASSMORE_TARE_AXIS_Y   = 0x02,
    MASSMORE_TARE_AXIS_Z   = 0x04,
    MASSMORE_TARE_AXIS_ALL = 0x07
} massmore_tare_axis_t;

/*! Which rotation vector the tare is calculated from — Tare Usage Guide [4] */
typedef enum {
    MASSMORE_TARE_BASIS_ROTATION_VECTOR       = 0,
    MASSMORE_TARE_BASIS_GAMING_RV             = 1,
    MASSMORE_TARE_BASIS_GEOMAGNETIC_RV        = 2,
    MASSMORE_TARE_BASIS_GYRO_INTEGRATED_RV    = 3,
    MASSMORE_TARE_BASIS_ARVR_STABILIZED_RV    = 4,
    MASSMORE_TARE_BASIS_ARVR_STABILIZED_GRV   = 5
} massmore_tare_basis_t;

/*! Selector for the ME (MotionEngine) calibration command. */
typedef enum {
    MASSMORE_CAL_ACCEL         = 0,
    MASSMORE_CAL_GYRO          = 1,
    MASSMORE_CAL_MAG           = 2,
    MASSMORE_CAL_PLANAR_ACCEL  = 3,
    MASSMORE_CAL_ACCEL_GYRO_MAG= 4,
    MASSMORE_CAL_STOP          = 5   //!< disable all dynamic calibration
} massmore_calibrate_target_t;

/* ===========================================================================
 * FRS (Flash Record System) record IDs — Datasheet [1] Figure 1-31
 * ========================================================================= */
#define MASSMORE_FRS_STATIC_CAL_AGM        0x7979
#define MASSMORE_FRS_NOMINAL_CAL_AGM       0x4D4D
#define MASSMORE_FRS_STATIC_CAL_SRA        0x8A8A
#define MASSMORE_FRS_NOMINAL_CAL_SRA       0x4E4E
#define MASSMORE_FRS_DYNAMIC_CAL           0x1F1F
#define MASSMORE_FRS_ME_POWER_MGMT         0xD3E2
#define MASSMORE_FRS_SYSTEM_ORIENTATION    0x2D3E
#define MASSMORE_FRS_ACCEL_ORIENTATION     0x2D41
#define MASSMORE_FRS_GYRO_ORIENTATION      0x2D46
#define MASSMORE_FRS_MAG_ORIENTATION       0x2D4C
#define MASSMORE_FRS_ARVR_STAB_RV          0x3E2D
#define MASSMORE_FRS_ARVR_STAB_GRV         0x3E2E
#define MASSMORE_FRS_SIG_MOTION_CONFIG     0xC274
#define MASSMORE_FRS_SHAKE_DETECT_CONFIG   0x7D7D
#define MASSMORE_FRS_MAX_FUSION_PERIOD     0xD7D7
#define MASSMORE_FRS_SERIAL_NUMBER         0x4B4B
#define MASSMORE_FRS_ES_PRESSURE_CAL       0x39AF
#define MASSMORE_FRS_ES_TEMPERATURE_CAL    0x4D20
#define MASSMORE_FRS_ES_HUMIDITY_CAL       0x1AC9
#define MASSMORE_FRS_ES_AMBIENT_LIGHT_CAL  0x39B1
#define MASSMORE_FRS_ES_PROXIMITY_CAL      0x4DA2
#define MASSMORE_FRS_ALS_CAL               0xD401
#define MASSMORE_FRS_PROXIMITY_CAL         0xD402
#define MASSMORE_FRS_STABILITY_DET_CONFIG  0xED85
#define MASSMORE_FRS_USER_RECORD           0x74B4
#define MASSMORE_FRS_ME_TIME_SOURCE        0xD403
#define MASSMORE_FRS_GYRO_INTEGRATED_RV    0xA1A2

/* Sensor metadata records (used by readSensorMetadata) */
#define MASSMORE_FRS_META_ACCELEROMETER    0xE302
#define MASSMORE_FRS_META_GYRO_CALIBRATED  0xE306
#define MASSMORE_FRS_META_MAG_CALIBRATED   0xE309
#define MASSMORE_FRS_META_ROTATION_VECTOR  0xE30B

/* ===========================================================================
 * Set Feature flags — SH-2 Reference Manual [2] §6.5.4
 * ========================================================================= */
#define MASSMORE_FEATURE_FLAG_NONE              0x00
#define MASSMORE_FEATURE_FLAG_CHANGE_SENS_ENA   0x01 //!< report on change only
#define MASSMORE_FEATURE_FLAG_CHANGE_SENS_REL   0x02 //!< 0 = absolute, 1 = relative
#define MASSMORE_FEATURE_FLAG_WAKE_ENABLED      0x04 //!< route to wake channel (4)
#define MASSMORE_FEATURE_FLAG_ALWAYS_ON         0x08
#define MASSMORE_FEATURE_FLAG_SNIFF_ENABLED     0x10

/* ===========================================================================
 * Result / error codes
 * ========================================================================= */
typedef enum {
    MASSMORE_OK               =  0,  //!< success
    MASSMORE_ERR_IO           = -1,  //!< bus level failure (NACK, SPI timeout…)
    MASSMORE_ERR_TIMEOUT      = -2,  //!< the device did not answer in time
    MASSMORE_ERR_BAD_PARAM    = -3,  //!< caller passed an out-of-range argument
    MASSMORE_ERR_NO_DEVICE    = -4,  //!< nothing answered on the bus
    MASSMORE_ERR_BAD_RESPONSE = -5,  //!< malformed / unexpected packet
    MASSMORE_ERR_NOT_READY    = -6,  //!< begin() has not been called
    MASSMORE_ERR_UNSUPPORTED  = -7   //!< operation not valid for this transport
} massmore_status_t;

/* ===========================================================================
 * Chip identity / authenticity
 * ========================================================================= */

/*! Decoded Product ID Response — Datasheet [1] Figure 1-29 */
typedef struct {
    bool     valid;           //!< true once a 0xF8 response has been decoded
    uint8_t  resetCause;      //!< 0=n/a 1=POR 2=internal 3=watchdog 4=external 5=other
    uint8_t  swVersionMajor;
    uint8_t  swVersionMinor;
    uint16_t swVersionPatch;
    uint32_t swPartNumber;    //!< firmware part number, e.g. 10003606
    uint32_t swBuildNumber;
} massmore_product_id_t;

/*!
 * @brief Result of verifyChip().
 *
 * NOTE ON WHAT THIS ACTUALLY PROVES.
 * The BNO08x has no cryptographic attestation, so no library can give you a
 * mathematical proof of authenticity. What this check *does* prove is that the
 * part on your board behaves exactly like a genuine CEVA/Bosch BNO08x at the
 * protocol level: it emits a well formed SHTP advertisement, it answers a
 * Product ID Request with a plausible firmware part number and version, and it
 * holds a readable serial number in its flash record system. Clones and
 * mislabelled parts (a BNO055 relabelled as a BNO086, a dead/blank die, or a
 * board with no sensor at all) fail one of these steps. Treat OK as
 * "this really is BNO08x silicon running factory firmware".
 */
typedef enum {
    MASSMORE_AUTH_OK          = 0, //!< valid response AND a known-good firmware part number
    MASSMORE_AUTH_UNKNOWN_FW  = 1, //!< valid BNO08x response, firmware part number not in our table
    MASSMORE_AUTH_BAD_VERSION = 2, //!< responded, but the version fields are implausible
    MASSMORE_AUTH_NO_RESPONSE = 3, //!< no Product ID response — not a BNO08x, or wiring/address wrong
    MASSMORE_AUTH_BAD_RESPONSE= 4  //!< a response arrived but it is malformed
} massmore_auth_t;

/* ===========================================================================
 * Sensor data containers
 * ========================================================================= */

/*! Accuracy field reported in every sensor report status byte, bits 1:0. */
typedef enum {
    MASSMORE_ACCURACY_UNRELIABLE = 0,
    MASSMORE_ACCURACY_LOW        = 1,
    MASSMORE_ACCURACY_MEDIUM     = 2,
    MASSMORE_ACCURACY_HIGH       = 3
} massmore_accuracy_t;

typedef struct { float x, y, z; }                    massmore_vec3_t;
typedef struct { int16_t x, y, z; }                  massmore_vec3i_t;
typedef struct { float i, j, k, real, accuracy; }    massmore_quat_t;
typedef struct { float roll, pitch, yaw; }           massmore_euler_t;

/*! Stability classifier output — SH-2 Ref Manual [2] §6.5.20 */
typedef enum {
    MASSMORE_STABILITY_UNKNOWN     = 0,
    MASSMORE_STABILITY_ON_TABLE    = 1,
    MASSMORE_STABILITY_STATIONARY  = 2,
    MASSMORE_STABILITY_STABLE      = 3,
    MASSMORE_STABILITY_MOTION      = 4,
    MASSMORE_STABILITY_RESERVED    = 5
} massmore_stability_t;

/*! Personal activity classifier states — SH-2 Ref Manual [2] §6.5.36 */
typedef enum {
    MASSMORE_ACTIVITY_UNKNOWN   = 0,
    MASSMORE_ACTIVITY_IN_VEHICLE= 1,
    MASSMORE_ACTIVITY_ON_BICYCLE= 2,
    MASSMORE_ACTIVITY_ON_FOOT   = 3,
    MASSMORE_ACTIVITY_STILL     = 4,
    MASSMORE_ACTIVITY_TILTING   = 5,
    MASSMORE_ACTIVITY_WALKING   = 6,
    MASSMORE_ACTIVITY_RUNNING   = 7,
    MASSMORE_ACTIVITY_ON_STAIRS = 8,
    MASSMORE_ACTIVITY_COUNT     = 9
} massmore_activity_t;

/*! Tap detector flag bits — SH-2 Ref Manual [2] §6.5.17 */
#define MASSMORE_TAP_X_POS   0x01
#define MASSMORE_TAP_X_NEG   0x02
#define MASSMORE_TAP_Y_POS   0x04
#define MASSMORE_TAP_Y_NEG   0x08
#define MASSMORE_TAP_Z_POS   0x10
#define MASSMORE_TAP_Z_NEG   0x20
#define MASSMORE_TAP_DOUBLE  0x40

/*! Shake detector flag bits — SH-2 Ref Manual [2] §6.5.32 */
#define MASSMORE_SHAKE_X     0x01
#define MASSMORE_SHAKE_Y     0x02
#define MASSMORE_SHAKE_Z     0x04

/*! Which physical transport the driver is talking over. */
typedef enum {
    MASSMORE_BUS_NONE = 0,
    MASSMORE_BUS_I2C  = 1,
    MASSMORE_BUS_SPI  = 2,
    MASSMORE_BUS_UART = 3   //!< SHTP over UART (not UART-RVC — see Massmore_BNO08x_RVC.h)
} massmore_bus_t;

/* ===========================================================================
 * Fixed point helpers — Q point scaling, SH-2 Ref Manual [2] §6.5
 * value = raw * 2^-Q
 * ========================================================================= */
#define MASSMORE_Q_TO_FLOAT(raw, q) ((float)(raw) * (1.0f / (float)(1UL << (q))))

/* Q points used by the BNO08x sensor reports. */
#define MASSMORE_Q_ACCEL        8   //!< m/s^2  (accel, linear accel, gravity)
#define MASSMORE_Q_GYRO         9   //!< rad/s
#define MASSMORE_Q_MAG          4   //!< uT
#define MASSMORE_Q_QUAT        14   //!< unit quaternion components
#define MASSMORE_Q_QUAT_ACC    12   //!< rotation vector accuracy, radians
#define MASSMORE_Q_ANG_VEL     10   //!< gyro-integrated RV angular velocity, rad/s
#define MASSMORE_Q_PRESSURE    20   //!< hPa
#define MASSMORE_Q_AMBIENT      8   //!< lux
#define MASSMORE_Q_HUMIDITY     8   //!< %RH
#define MASSMORE_Q_PROXIMITY    4   //!< cm
#define MASSMORE_Q_TEMPERATURE  7   //!< degC

/* Convenient report interval helpers (microseconds). */
#define MASSMORE_HZ_TO_US(hz)   ((uint32_t)(1000000UL / (uint32_t)(hz)))
#define MASSMORE_INTERVAL_1HZ    1000000UL
#define MASSMORE_INTERVAL_10HZ    100000UL
#define MASSMORE_INTERVAL_50HZ     20000UL
#define MASSMORE_INTERVAL_100HZ    10000UL
#define MASSMORE_INTERVAL_200HZ     5000UL
#define MASSMORE_INTERVAL_400HZ     2500UL
#define MASSMORE_INTERVAL_1000HZ    1000UL

#endif /* MASSMORE_BNO08X_DEFS_H */
