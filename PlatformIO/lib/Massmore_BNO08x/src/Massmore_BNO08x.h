/*!
 * @file  Massmore_BNO08x.h
 * @brief Full featured Arduino / PlatformIO driver for the BNO085 and BNO086
 *        9-axis sensor-fusion IMU (CEVA / Bosch SH-2 MotionEngine).
 *
 * Highlights
 *  - I2C, SPI and SHTP-over-UART transports in one class
 *  - Every SH-2 sensor report the BNO08x can produce, basic to advanced
 *  - Chip identity + authenticity verification (Product ID + FRS serial number)
 *  - Calibration, tare, DCD save, FRS read/write, sleep/wake, soft & hard reset
 *  - Non blocking: nothing in the hot path blocks, optional INT pin support
 *  - No dynamic memory allocation
 *
 * Tested with:
 *  - Arduino IDE + esp32 core 3.x  (ESP32, ESP32-S2/S3, ESP32-C3/C6)
 *  - PlatformIO + platform-espressif32 6.x/3.x board definitions
 *  - AVR, SAMD, RP2040, STM32 Arduino cores
 *
 * Massmore BNO08x Library — assembled by Massmore (https://www.massmore.shop)
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
 * @class MassmoreBNO08x
 * @brief Driver object. Create one per sensor.
 *
 * Minimal use:
 * @code
 *   MassmoreBNO08x imu;
 *   void setup() {
 *     Serial.begin(115200);
 *     Wire.begin();                        // ESP32: Wire.begin(SDA, SCL);
 *     if (!imu.begin()) { ... }            // I2C @ 0x4B
 *     imu.enableRotationVector(10000);     // 100 Hz
 *   }
 *   void loop() {
 *     if (imu.update()) {
 *       if (imu.hasNewReport(MASSMORE_SENSOR_ROTATION_VECTOR)) {
 *         Serial.println(imu.getYawDeg());
 *       }
 *     }
 *   }
 * @endcode
 */
class MassmoreBNO08x {
public:
    MassmoreBNO08x();

    /* ===================================================================
     * SECTION 1 — Start-up
     * =================================================================== */

    /*!
     * @brief Start the sensor on an I2C bus.
     * @param address  0x4A (DI/SA0 low) or 0x4B (DI/SA0 high, the default).
     * @param wirePort Which TwoWire instance to use. Defaults to Wire.
     * @param intPin   H_INTN pin (the INT pad), or -1 if not connected.
     *                 Strongly recommended:
     *                 with an INT pin the driver never polls a silent bus.
     * @param rstPin   NRST pin (the RST pad), or -1. Enables hardwareReset().
     * @return true on success. Call getLastError() for the reason on failure.
     *
     * @note The BNO08x needs ~90 ms after power-on before it answers. begin()
     *       handles that wait for you.
     */
    bool begin(uint8_t address = MASSMORE_BNO08X_I2C_ADDR_DEF,
               TwoWire &wirePort = Wire,
               int8_t intPin = -1,
               int8_t rstPin = -1);

    /*!
     * @brief Start the sensor on an SPI bus (CPOL=1, CPHA=1 → SPI_MODE3).
     * @param csPin    Chip select (H_CSN — the CS pad).
     * @param intPin   H_INTN (the INT pad). REQUIRED for SPI — SHTP over SPI
     *                 has no other way
     *                 to know when a cargo is waiting.
     * @param rstPin   NRST (the RST pad). REQUIRED for SPI — the part must be
     *                 reset while PS0/PS1 (the P0/P1 pads) are high to latch
     *                 SPI mode.
     * @param wakePin  PS0/WAKE (the P0 pad), or -1. Needed to wake the part
     *                 from sleep.
     * @param spiPort  Which SPIClass to use. Defaults to SPI.
     * @param speedHz  SPI clock. The BNO08x is specified to 3 MHz.
     */
    bool beginSPI(int8_t csPin, int8_t intPin, int8_t rstPin,
                  int8_t wakePin = -1,
                  SPIClass &spiPort = SPI,
                  uint32_t speedHz = 3000000UL);

    /*!
     * @brief Start the sensor on SHTP-over-UART, 3 Mbit/s.
     *        Strap PS1=1, PS0=0 — the P1 and P0 pads on the Massmore board.
     *        The sensor's TX is the SDA pad, its RX is the SCL pad.
     * @param serialPort An already-begun Stream (HardwareSerial…).
     * @param intPin     H_INTN (the INT pad), or -1.
     * @param rstPin     NRST (the RST pad), or -1.
     * @note This is *not* UART-RVC. For the simple 100 Hz RVC output stream use
     *       the separate MassmoreBNO08x_RVC class in Massmore_BNO08x_RVC.h.
     */
    bool beginUART(Stream &serialPort, int8_t intPin = -1, int8_t rstPin = -1);

    /*! @brief true once begin*() has completed successfully. */
    bool isConnected() const { return _busType != MASSMORE_BUS_NONE; }

    /*! @brief Route library diagnostics to a Stream (usually Serial). */
    void enableDebug(Stream &dbg) { _dbg = &dbg; }
    /*! @brief Turn diagnostics back off. */
    void disableDebug() { _dbg = nullptr; }

    /* ===================================================================
     * SECTION 2 — Identity and authenticity
     * =================================================================== */

    /*!
     * @brief Ask the device for its Product ID (report 0xF9 → 0xF8) and cache it.
     * @return MASSMORE_OK, or an error code.
     */
    massmore_status_t requestProductID(uint32_t timeoutMs = 300);

    /*!
     * @brief The Product ID of the SH-2 application.
     *
     * The part answers a Product ID Request with one response per firmware
     * image it carries. This returns the entry whose part number matches a
     * known SH-2 application build, or the first entry received if none does.
     * Populated by begin() and requestProductID().
     */
    const massmore_product_id_t &getProductID() const { return _productId; }

    /*! @brief How many Product ID Responses the last request collected. */
    uint8_t getProductIDCount() const { return _productIdCount; }

    /*!
     * @brief One of the collected Product ID Responses, in arrival order.
     * @param index 0 .. getProductIDCount()-1. Out of range returns the
     *        primary entry, so the return value is always readable.
     */
    const massmore_product_id_t &getProductID(uint8_t index) const {
        return (index < _productIdCount) ? _productIds[index] : _productId;
    }

    /*!
     * @brief Verify that the attached part behaves like genuine BNO08x silicon.
     *
     * Runs three independent checks:
     *   1. a well formed Product ID Response arrives,
     *   2. the firmware version fields are plausible (major 1..9, non-zero build),
     *   3. the firmware part number matches a known factory build.
     *
     * See the comment on massmore_auth_t for exactly what this does and does
     * not prove. A result of MASSMORE_AUTH_UNKNOWN_FW is not a failure — it
     * means CEVA shipped a firmware build this library's table predates.
     */
    massmore_auth_t verifyChip();

    /*! @brief Human readable text for a verifyChip() result. */
    static const char *authToString(massmore_auth_t a);

    /*! @brief Human readable text for a massmore_status_t. */
    static const char *statusToString(massmore_status_t s);

    /*!
     * @brief Read the 64-bit factory serial number from FRS record 0x4B4B.
     * @param serialOut Receives the serial number.
     * @return MASSMORE_OK on success.
     */
    massmore_status_t readSerialNumber(uint64_t &serialOut, uint32_t timeoutMs = 500);

    /*! @brief Human readable reset cause from the last Product ID response. */
    const char *getResetReasonString() const;

    /* ===================================================================
     * SECTION 3 — The main loop
     * =================================================================== */

    /*!
     * @brief Pull one SHTP packet from the sensor and decode it. Non blocking.
     * @return true if a packet was received and decoded.
     *
     * Call this as often as you can. If an INT pin was supplied, update()
     * returns immediately (false) whenever the pin is idle, so it costs
     * essentially nothing to call it every loop().
     */
    bool update();

    /*!
     * @brief Drain everything the sensor has queued, up to a budget.
     * @param maxPackets Safety limit so a fast sensor cannot starve loop().
     * @return number of packets decoded.
     */
    uint8_t updateAll(uint8_t maxPackets = 16);

    /*! @brief true if the sensor is asserting H_INTN (or, with no INT pin, always true). */
    bool dataAvailable();

    /*!
     * @brief Report ID of the most recently decoded sensor report, 0 if none.
     *        Cleared by every update() before a new packet is parsed.
     */
    uint8_t getLastReportID() const { return _lastReportId; }

    /*!
     * @brief Test-and-clear: did report `id` arrive since you last asked?
     * Ideal for "did I get a fresh quaternion this loop?" style code.
     */
    bool hasNewReport(uint8_t id);

    /*! @brief Non destructive version of hasNewReport(). */
    bool peekNewReport(uint8_t id) const;

    /*! @brief Clear every "new report" flag. */
    void clearNewFlags();

    /*!
     * @brief Optional callback fired once per decoded sensor report.
     * @param cb  void f(uint8_t reportId, void *ctx)
     */
    void setReportCallback(void (*cb)(uint8_t reportId, void *ctx), void *ctx = nullptr);

    /* ===================================================================
     * SECTION 4 — Enabling sensors
     * =================================================================== */

    /*!
     * @brief The complete Set Feature command — every field of Figure 1-33.
     * @param sensorId          Which report to enable (0 interval = disable).
     * @param reportIntervalUs  Period in microseconds. 0 disables the sensor.
     * @param batchIntervalUs   Batch period in microseconds, 0 for no batching.
     * @param flags             MASSMORE_FEATURE_FLAG_* bitmap.
     * @param changeSensitivity Report-on-change threshold (see flags).
     * @param sensorSpecific    32-bit sensor specific configuration word.
     */
    massmore_status_t setFeature(uint8_t sensorId,
                                 uint32_t reportIntervalUs,
                                 uint32_t batchIntervalUs = 0,
                                 uint8_t  flags = MASSMORE_FEATURE_FLAG_NONE,
                                 uint16_t changeSensitivity = 0,
                                 uint32_t sensorSpecific = 0);

    /*! @brief Shorthand for setFeature(id, intervalUs). */
    massmore_status_t enableReport(uint8_t sensorId, uint32_t reportIntervalUs);

    /*! @brief Stop a sensor (interval 0). */
    massmore_status_t disableReport(uint8_t sensorId);

    /*! @brief Stop every sensor this object has enabled. */
    void disableAllReports();

    /*! @brief Ask for the current configuration of a sensor (0xFE → 0xFC). */
    massmore_status_t requestFeature(uint8_t sensorId);

    /*! @brief The report interval the device last told us it is using, in us. */
    uint32_t getReportInterval(uint8_t sensorId) const;

    /* ---- Motion / orientation ---------------------------------------- */
    massmore_status_t enableAccelerometer(uint32_t us = MASSMORE_INTERVAL_100HZ);
    massmore_status_t enableGyroscope(uint32_t us = MASSMORE_INTERVAL_100HZ);
    massmore_status_t enableMagnetometer(uint32_t us = MASSMORE_INTERVAL_100HZ);
    massmore_status_t enableLinearAcceleration(uint32_t us = MASSMORE_INTERVAL_100HZ);
    massmore_status_t enableGravity(uint32_t us = MASSMORE_INTERVAL_100HZ);
    massmore_status_t enableGyroscopeUncalibrated(uint32_t us = MASSMORE_INTERVAL_100HZ);
    massmore_status_t enableMagnetometerUncalibrated(uint32_t us = MASSMORE_INTERVAL_100HZ);

    /*! 9-axis fusion. Absolute heading, needs a calibrated magnetometer. */
    massmore_status_t enableRotationVector(uint32_t us = MASSMORE_INTERVAL_100HZ);
    /*! 6-axis fusion. No magnetometer, so yaw drifts, but immune to magnetic noise. */
    massmore_status_t enableGameRotationVector(uint32_t us = MASSMORE_INTERVAL_100HZ);
    /*! Accel + mag only. Low power, lower rate. */
    massmore_status_t enableGeomagneticRotationVector(uint32_t us = MASSMORE_INTERVAL_100HZ);
    /*! Rotation vector with the discontinuities smoothed out — for AR/VR headsets. */
    massmore_status_t enableARVRStabilizedRotationVector(uint32_t us = MASSMORE_INTERVAL_100HZ);
    /*! Game rotation vector with the discontinuities smoothed out. */
    massmore_status_t enableARVRStabilizedGameRotationVector(uint32_t us = MASSMORE_INTERVAL_100HZ);
    /*! Highest rate quaternion, up to 1 kHz, delivered on its own SHTP channel. */
    massmore_status_t enableGyroIntegratedRotationVector(uint32_t us = MASSMORE_INTERVAL_400HZ);

    /* ---- Raw (uncalibrated ADC counts, for logging / custom filters) --- */
    massmore_status_t enableRawAccelerometer(uint32_t us = MASSMORE_INTERVAL_100HZ);
    massmore_status_t enableRawGyroscope(uint32_t us = MASSMORE_INTERVAL_100HZ);
    massmore_status_t enableRawMagnetometer(uint32_t us = MASSMORE_INTERVAL_100HZ);

    /* ---- Activity / gesture engines ----------------------------------- */
    massmore_status_t enableTapDetector(uint32_t us = MASSMORE_INTERVAL_100HZ);
    massmore_status_t enableStepCounter(uint32_t us = MASSMORE_INTERVAL_10HZ);
    massmore_status_t enableStepDetector(uint32_t us = MASSMORE_INTERVAL_10HZ);
    massmore_status_t enableSignificantMotion(uint32_t us = MASSMORE_INTERVAL_10HZ);
    massmore_status_t enableStabilityClassifier(uint32_t us = MASSMORE_INTERVAL_10HZ);
    massmore_status_t enableStabilityDetector(uint32_t us = MASSMORE_INTERVAL_10HZ);
    massmore_status_t enableShakeDetector(uint32_t us = MASSMORE_INTERVAL_10HZ);
    massmore_status_t enableFlipDetector(uint32_t us = MASSMORE_INTERVAL_10HZ);
    massmore_status_t enablePickupDetector(uint32_t us = MASSMORE_INTERVAL_10HZ);
    massmore_status_t enableSleepDetector(uint32_t us = MASSMORE_INTERVAL_10HZ);
    massmore_status_t enableTiltDetector(uint32_t us = MASSMORE_INTERVAL_10HZ);
    massmore_status_t enablePocketDetector(uint32_t us = MASSMORE_INTERVAL_10HZ);
    massmore_status_t enableCircleDetector(uint32_t us = MASSMORE_INTERVAL_10HZ);
    massmore_status_t enableHeartRateMonitor(uint32_t us = MASSMORE_INTERVAL_10HZ);

    /*!
     * @brief Personal activity classifier.
     * @param us              Report interval.
     * @param enabledActivities Bitmap of activities to track; bit n corresponds
     *                        to massmore_activity_t n. 0x1F is a sensible
     *                        default (unknown/vehicle/bicycle/foot/still).
     */
    massmore_status_t enableActivityClassifier(uint32_t us = MASSMORE_INTERVAL_10HZ,
                                               uint32_t enabledActivities = 0x1F);

    /* ---- External environmental sensors on the secondary I2C bus ------ */
    massmore_status_t enablePressure(uint32_t us = MASSMORE_INTERVAL_10HZ);
    massmore_status_t enableAmbientLight(uint32_t us = MASSMORE_INTERVAL_10HZ);
    massmore_status_t enableHumidity(uint32_t us = MASSMORE_INTERVAL_10HZ);
    massmore_status_t enableProximity(uint32_t us = MASSMORE_INTERVAL_10HZ);
    massmore_status_t enableTemperature(uint32_t us = MASSMORE_INTERVAL_10HZ);

    /* ===================================================================
     * SECTION 5 — Reading the data
     * =================================================================== */

    /* ---- Quaternion (whichever rotation vector last arrived) ---------- */
    float getQuatI()    const { return _quat.i; }
    float getQuatJ()    const { return _quat.j; }
    float getQuatK()    const { return _quat.k; }
    float getQuatReal() const { return _quat.real; }
    /*! @brief Heading accuracy estimate in radians (rotation vector only). */
    float getQuatAccuracy() const { return _quat.accuracy; }
    /*! @brief Copy the whole quaternion out at once. */
    massmore_quat_t getQuaternion() const { return _quat; }

    /* ---- Euler angles, derived from the cached quaternion ------------- */
    /*! @brief Roll (rotation about X) in radians, -pi..pi. */
    float getRoll();
    /*! @brief Pitch (rotation about Y) in radians, -pi/2..pi/2. */
    float getPitch();
    /*! @brief Yaw / heading (rotation about Z) in radians, -pi..pi. */
    float getYaw();
    float getRollDeg();
    float getPitchDeg();
    /*! @brief Yaw in degrees, -180..180. */
    float getYawDeg();
    /*! @brief Yaw in compass degrees, 0..360. */
    float getHeadingDeg();
    /*! @brief All three Euler angles in radians in one call. */
    massmore_euler_t getEuler();
    /*! @brief All three Euler angles in degrees in one call. */
    massmore_euler_t getEulerDeg();

    /*! @brief Convert an arbitrary quaternion to Euler angles (radians). */
    static massmore_euler_t quaternionToEuler(const massmore_quat_t &q);

    /* ---- Vectors ------------------------------------------------------ */
    massmore_vec3_t getAccel()       const { return _accel; }       //!< m/s^2, with gravity
    massmore_vec3_t getGyro()        const { return _gyro; }        //!< rad/s
    massmore_vec3_t getMag()         const { return _mag; }         //!< uT
    massmore_vec3_t getLinearAccel() const { return _linAccel; }    //!< m/s^2, gravity removed
    massmore_vec3_t getGravity()     const { return _gravity; }     //!< m/s^2
    massmore_vec3_t getGyroBias()    const { return _gyroBias; }    //!< rad/s
    massmore_vec3_t getMagBias()     const { return _magBias; }     //!< uT
    massmore_vec3_t getAngularVelocity() const { return _angVel; }  //!< rad/s, gyro-integrated RV

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

    /*! @brief Gyroscope in degrees/second, for people who prefer them. */
    massmore_vec3_t getGyroDeg() const;

    /* ---- Raw ADC counts ----------------------------------------------- */
    massmore_vec3i_t getRawAccel() const { return _rawAccel; }
    massmore_vec3i_t getRawGyro()  const { return _rawGyro; }
    massmore_vec3i_t getRawMag()   const { return _rawMag; }
    /*! @brief Gyroscope die temperature in raw ADC counts (raw gyro report). */
    int16_t getRawGyroTemperature() const { return _rawGyroTemp; }

    /* ---- Environmental ------------------------------------------------ */
    float getPressure()     const { return _pressure; }      //!< hPa
    float getAmbientLight() const { return _ambientLight; }  //!< lux
    float getHumidity()     const { return _humidity; }      //!< %RH
    float getProximity()    const { return _proximity; }     //!< cm
    float getTemperature()  const { return _temperature; }   //!< degC

    /* ---- Event / classifier outputs ----------------------------------- */
    /*! @brief Cumulative step count since power-on (or since the last reset). */
    uint32_t getStepCount()   const { return _stepCount; }
    /*! @brief Tap flags — test against MASSMORE_TAP_*. Cleared when read. */
    uint8_t  getTapDetector();
    /*! @brief Shake flags — test against MASSMORE_SHAKE_*. Cleared when read. */
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

    massmore_stability_t getStabilityClassification() const { return _stability; }
    const char          *getStabilityString() const;

    massmore_activity_t  getActivity() const { return (massmore_activity_t)_activityMostLikely; }
    const char          *getActivityString() const;
    /*! @brief Confidence 0..100 for one activity from the classifier. */
    uint8_t              getActivityConfidence(massmore_activity_t a) const;

    /* ---- Report metadata ---------------------------------------------- */
    /*!
     * @brief Accuracy of the sensor that produced report `id` (0..3).
     * The BNO08x reports this in the status byte of every sensor report.
     */
    massmore_accuracy_t getAccuracy(uint8_t sensorId) const;
    /*! @brief Text form: "Unreliable" / "Low" / "Medium" / "High". */
    static const char *accuracyToString(massmore_accuracy_t a);

    /*!
     * @brief Timestamp of the most recent report, in microseconds on the
     *        host's micros() clock.
     *
     * The BNO08x does not send an absolute time. Every packet carries a SIGNED
     * base-timestamp delta and every report a delay, both in 100 us ticks and
     * both relative to the moment the packet was transferred. The driver
     * anchors them to micros() taken as the packet arrived, so this value is
     * comparable with millis()/micros() and advances monotonically. Accuracy is
     * limited by how promptly your loop() calls update().
     */
    uint64_t getTimestampUs() const { return _timestampUs; }
    /*! @brief Sequence number of the most recent report — use it to spot drops. */
    uint8_t  getSequenceNumber() const { return _lastReportSeq; }

    /* ===================================================================
     * SECTION 6 — Calibration
     * =================================================================== */

    /*! @brief Enable dynamic calibration for one subsystem. */
    massmore_status_t calibrate(massmore_calibrate_target_t target);
    massmore_status_t calibrateAccelerometer() { return calibrate(MASSMORE_CAL_ACCEL); }
    massmore_status_t calibrateGyroscope()     { return calibrate(MASSMORE_CAL_GYRO); }
    massmore_status_t calibrateMagnetometer()  { return calibrate(MASSMORE_CAL_MAG); }
    massmore_status_t calibratePlanarAccel()   { return calibrate(MASSMORE_CAL_PLANAR_ACCEL); }
    massmore_status_t calibrateAll()           { return calibrate(MASSMORE_CAL_ACCEL_GYRO_MAG); }
    /*! @brief Turn dynamic calibration off for every subsystem. */
    massmore_status_t endCalibration()         { return calibrate(MASSMORE_CAL_STOP); }

    /*! @brief Ask the MotionEngine for its calibration enable state. */
    massmore_status_t requestCalibrationStatus();
    /*! @brief true when the last ME calibration command returned success. */
    bool calibrationComplete() const { return _calibrationStatus == 0; }
    /*! @brief Raw status byte from the last ME calibration command response. */
    uint8_t getCalibrationStatus() const { return _calibrationStatus; }

    /*! @brief Write the Dynamic Calibration Data to flash so it survives reboot. */
    massmore_status_t saveCalibration();
    /*! @brief Let the device auto-save DCD periodically (1 = on, 0 = off). */
    massmore_status_t setPeriodicCalibrationSave(bool enable);
    /*! @brief Erase stored calibration and reset. The device reboots. */
    massmore_status_t clearCalibrationAndReset();

    /* ===================================================================
     * SECTION 7 — Tare (defining "forward")
     * =================================================================== */

    /*!
     * @brief Re-zero the orientation output to the current pose.
     * @param axes  Bitmap of massmore_tare_axis_t. Use MASSMORE_TARE_AXIS_Z for
     *              a heading-only ("user pressed the recenter button") tare, or
     *              MASSMORE_TARE_AXIS_ALL for a full factory alignment.
     * @param basis Which rotation vector the tare is computed from.
     */
    massmore_status_t tareNow(uint8_t axes = MASSMORE_TARE_AXIS_ALL,
                              massmore_tare_basis_t basis = MASSMORE_TARE_BASIS_ROTATION_VECTOR);

    /*! @brief Store the current tare in the System Orientation FRS record. */
    massmore_status_t persistTare();

    /*! @brief Clear the stored tare (set reorientation to identity). */
    massmore_status_t clearTare();

    /* ===================================================================
     * SECTION 8 — Power, reset and low level access
     * =================================================================== */

    /*! @brief Soft reset over the SHTP executable channel. Blocks ~100 ms. */
    massmore_status_t softReset();

    /*! @brief Pulse the NRST pin. Only available if rstPin was supplied. */
    massmore_status_t hardwareReset();

    /*! @brief Executable "on": re-enable every configured sensor. */
    massmore_status_t modeOn();
    /*! @brief Executable "sleep": only wake/always-on sensors keep running. */
    massmore_status_t modeSleep();

    /*! @brief Pulse the PS0/WAKE line — the P0 pad — (SPI only) to wake the part. */
    void wake();

    /*! @brief Query the oscillator type (command 10). Result in getOscillatorType(). */
    massmore_status_t requestOscillatorType();
    uint8_t getOscillatorType() const { return _oscillatorType; }

    /*! @brief Ask the device for its error queue (command 1). */
    massmore_status_t requestErrorList();
    /*! @brief Number of errors reported by the last requestErrorList(). */
    uint8_t getErrorCount() const { return _errorCount; }

    /* ---- FRS: the device's flash record system ------------------------ */
    /*!
     * @brief Read an FRS record.
     * @param recordId   One of the MASSMORE_FRS_* IDs.
     * @param dataOut    Buffer for the 32-bit words.
     * @param maxWords   Capacity of dataOut, in words.
     * @param wordsRead  Receives the number of words actually read.
     */
    massmore_status_t readFrsRecord(uint16_t recordId, uint32_t *dataOut,
                                    uint16_t maxWords, uint16_t &wordsRead,
                                    uint32_t timeoutMs = 500);

    /*!
     * @brief Write an FRS record. Erases and rewrites the whole record.
     * @warning Writing a bad calibration or orientation record can make the
     *          fusion output nonsense until you restore it. Read first.
     */
    massmore_status_t writeFrsRecord(uint16_t recordId, const uint32_t *data,
                                     uint16_t words, uint32_t timeoutMs = 2000);

    /*! @brief Read one sensor's metadata record (range, resolution, Q points…). */
    massmore_status_t readSensorMetadata(uint16_t metadataRecordId,
                                         uint32_t *dataOut, uint16_t maxWords,
                                         uint16_t &wordsRead);

    /* ---- Raw SHTP escape hatch ---------------------------------------- */
    /*! @brief Send an arbitrary cargo on an arbitrary channel. */
    massmore_status_t sendPacket(uint8_t channel, const uint8_t *data, uint16_t len);
    /*! @brief Pointer to the payload of the most recently received cargo. */
    const uint8_t *getRawPacket(uint16_t &len, uint8_t &channel) const;

    /*! @brief The last error the driver recorded. */
    massmore_status_t getLastError() const { return _lastError; }

private:
    /* ---- transport ---------------------------------------------------- */
    massmore_bus_t _busType;
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

    massmore_status_t _lastError;

    /* ---- decoded data ------------------------------------------------- */
    massmore_quat_t  _quat;
    massmore_vec3_t  _accel, _gyro, _mag, _linAccel, _gravity;
    massmore_vec3_t  _gyroBias, _magBias, _angVel;
    massmore_vec3i_t _rawAccel, _rawGyro, _rawMag;
    int16_t          _rawGyroTemp;
    uint32_t         _rawAccelTimestamp, _rawGyroTimestamp, _rawMagTimestamp;

    float    _pressure, _ambientLight, _humidity, _proximity, _temperature;

    uint32_t _stepCount;
    uint8_t  _tapFlags;
    uint16_t _shakeFlags;
    uint16_t _heartRate;
    uint8_t  _sleepState;
    bool     _sigMotion, _flip, _pickup, _tilt, _pocket, _circle, _stepDetected, _stabilityChanged;
    massmore_stability_t _stability;
    uint8_t  _activityMostLikely;
    uint8_t  _activityConfidence[MASSMORE_ACTIVITY_COUNT];

    uint64_t _timestampUs;
    int32_t  _timebaseDelta100us;   //!< signed, 100 us ticks — [1] Figure 1-35
    uint32_t _rxHostMicros;         //!< micros() when the packet was received
    uint8_t  _lastReportId;
    uint8_t  _lastReportSeq;

    /* one accuracy nibble and one "new" bit per possible report ID (0x00-0x3F) */
    uint8_t  _accuracyTable[0x40];
    uint8_t  _newFlags[8];        //!< bitmap, 64 report IDs
    uint32_t _intervals[0x40];
    /* Report lengths this device published in its SHTP advertisement.
     * Per instance, so two sensors in one sketch cannot corrupt each
     * other's table. 0 = not learned, use the fallback. */
    uint8_t  _advertReportLen[0x40];   //!< v1.0.1: was a file-scope static    //!< last known report interval per sensor

    /* ---- command / query results -------------------------------------- */
    massmore_product_id_t _productId;                                 //!< SH-2 application entry
    massmore_product_id_t _productIds[MASSMORE_BNO08X_MAX_PRODUCT_IDS];
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
    bool  waitForInt(uint32_t timeoutMs);
    bool  receivePacket();
    bool  i2cReceivePacket();
    bool  spiReceivePacket();
    bool  uartReceivePacket();
    bool  i2cSendPacket(uint8_t channel, uint16_t payloadLen);
    bool  spiSendPacket(uint8_t channel, uint16_t payloadLen);
    bool  uartSendPacket(uint8_t channel, uint16_t payloadLen);
    bool  txPacket(uint8_t channel, uint16_t payloadLen);

    void  parsePacket();
    void  parseInputReports(bool wakeChannel);
    void  parseControlReport();
    void  parseCommandResponse();
    void  parseProductIdResponse();
    void  parseFrsReadResponse();
    void  parseGyroRvPacket();
    uint16_t parseOneSensorReport(uint16_t offset);

    massmore_status_t sendCommand(uint8_t command, const uint8_t *p, uint8_t pLen);

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
