# Changelog

All notable changes to the Massmore BNO08x library.
Format follows [Keep a Changelog](https://keepachangelog.com/), versioning
follows [Semantic Versioning](https://semver.org/).

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
