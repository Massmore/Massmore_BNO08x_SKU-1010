/*!
 * @file  Massmore_BNO08x.cpp
 * @brief Implementation ของ driver Massmore BNO085 / BNO086
 *
 * โครงสร้างไฟล์ (SECTION):
 *   1  Start-up             — begin() / beginSPI() / beginUART() + auto address fallback
 *   2  SHTP transport       — รับ/ส่ง packet บน I2C, SPI, UART
 *   3  Main loop            — update() / updateAll() / new-report flags
 *   4  Packet parsing       — ถอดรหัส sensor report ทุกชนิด
 *   5  Identity             — Product ID, verifyChip(), serial number
 *   6  Enabling sensors     — Set Feature
 *   7  Data accessors       — quaternion → Euler และ getter อื่น ๆ
 *   8  Commands             — calibration, tare, reset, sleep
 *   9  FRS                  — flash record system read/write
 *  10  Massmore standard API — Simple Blocking API + verifyChipID/isGenuine
 *
 * Protocol references (อ้างอิงในโค้ดตามหมายเลข):
 *   [1] BNO08X Datasheet, CEVA 1000-3927 v1.16
 *   [2] SH-2 Reference Manual, CEVA 1000-3625
 *   [3] Sensor Hub Transport Protocol, CEVA 1000-3535 v1.10
 *   [4] BNO080/BNO085 Tare Function Usage Guide, CEVA 1000-4045 v1.3
 *   [5] BNO080/BNO085 Sensor Calibration Procedure, CEVA 1000-4044
 *
 * Massmore_BNO08x — Designed and Manufactured by Massmore (https://www.massmore.shop)
 * SPDX-License-Identifier: MIT
 */

#include "Massmore_BNO08x.h"
#include <stdarg.h>
#include <math.h>
#include <string.h>

/* ---------------------------------------------------------------------------
 * AVR (ATmega328P, SRAM 2 KB): เก็บ string literal และตารางค่าคงที่ไว้ใน flash
 * (PROGMEM) แทน SRAM — ฟังก์ชัน *ToString() คัดลอกข้อความลง buffer เล็ก ๆ หนึ่งชุด
 * ก่อนคืน const char* (ค่าที่คืนใช้ได้จนกว่าจะเรียก *ToString() ครั้งถัดไป)
 * บน ESP32 ทุกอย่างอยู่ใน flash อยู่แล้ว macro จึงคืน literal ตรง ๆ
 * ------------------------------------------------------------------------- */
#if defined(__AVR__)
  #include <avr/pgmspace.h>
  static char s_massmoreStrBuf[72];
  static const char *massmoreFromFlash(const char *pgm) {
      strncpy_P(s_massmoreStrBuf, pgm, sizeof(s_massmoreStrBuf) - 1);
      s_massmoreStrBuf[sizeof(s_massmoreStrBuf) - 1] = '\0';
      return s_massmoreStrBuf;
  }
  #define MASSMORE_BNO08X_STR(lit) massmoreFromFlash(PSTR(lit))
  #define MASSMORE_BNO08X_VSNPRINTF vsnprintf_P
#else
  #ifndef PSTR
    #define PSTR(s) (s)
  #endif
  #ifndef PROGMEM
    #define PROGMEM
  #endif
  #ifndef pgm_read_byte
    #define pgm_read_byte(p) (*(const uint8_t *)(p))
  #endif
  #define MASSMORE_BNO08X_STR(lit) (lit)
  #define MASSMORE_BNO08X_VSNPRINTF vsnprintf
#endif

/* ---------------------------------------------------------------------------
 * How many bytes we can move in one I2C transaction on this core.
 * SHTP over I2C forbids repeated starts [3] §3.2, so every chunk we request
 * comes back with its own 4 byte header that we discard.
 * ------------------------------------------------------------------------- */
#if defined(ARDUINO_ARCH_ESP32)
  /* esp32 Arduino core 2.x and 3.x both expose a 128 byte Wire buffer and let
   * us grow it at runtime. */
  #define MASSMORE_BNO08X_I2C_BUF 128
#elif defined(I2C_BUFFER_LENGTH)
  #define MASSMORE_BNO08X_I2C_BUF I2C_BUFFER_LENGTH
#elif defined(BUFFER_LENGTH)
  #define MASSMORE_BNO08X_I2C_BUF BUFFER_LENGTH
#elif defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_SAMD) || defined(ARDUINO_ARCH_STM32)
  #define MASSMORE_BNO08X_I2C_BUF 128
#else
  #define MASSMORE_BNO08X_I2C_BUF 32
#endif

/* SHTP advertisement TLV tags — [3] §5.2 and SH-2 app tags */
#define MASSMORE_BNO08X_TAG_NULL              0
#define MASSMORE_BNO08X_TAG_GUID              1
#define MASSMORE_BNO08X_TAG_APP_NAME          8
#define MASSMORE_BNO08X_TAG_CHANNEL_NAME      9
#define MASSMORE_BNO08X_TAG_SH2_VERSION       0x80
#define MASSMORE_BNO08X_TAG_SH2_REPORT_LENS   0x81

/* ---------------------------------------------------------------------------
 * Fallback report length table.
 * At start-up a genuine BNO08x publishes the exact length of every report it
 * supports in its SHTP advertisement, and we use those numbers. This table is
 * only consulted if the advertisement was missed (for example if the host
 * booted long after the sensor did). Lengths are from [2] §6.5 and include the
 * 4 byte report prefix (id, sequence, status, delay).
 * ------------------------------------------------------------------------- */
static const uint8_t kFallbackReportLen[0x40] PROGMEM = {
    /* 0x00 */ 0,
    /* 0x01 */ 10, /* accelerometer            */
    /* 0x02 */ 10, /* gyroscope calibrated     */
    /* 0x03 */ 10, /* magnetic field           */
    /* 0x04 */ 10, /* linear acceleration      */
    /* 0x05 */ 14, /* rotation vector          */
    /* 0x06 */ 10, /* gravity                  */
    /* 0x07 */ 16, /* gyro uncalibrated        */
    /* 0x08 */ 12, /* game rotation vector     */
    /* 0x09 */ 14, /* geomagnetic RV           */
    /* 0x0A */ 8,  /* pressure                 */
    /* 0x0B */ 8,  /* ambient light            */
    /* 0x0C */ 6,  /* humidity                 */
    /* 0x0D */ 6,  /* proximity                */
    /* 0x0E */ 6,  /* temperature              */
    /* 0x0F */ 16, /* magnetic field uncal     */
    /* 0x10 */ 5,  /* tap detector             */
    /* 0x11 */ 12, /* step counter             */
    /* 0x12 */ 6,  /* significant motion       */
    /* 0x13 */ 6,  /* stability classifier     */
    /* 0x14 */ 16, /* raw accelerometer        */
    /* 0x15 */ 16, /* raw gyroscope            */
    /* 0x16 */ 16, /* raw magnetometer         */
    /* 0x17 */ 0,
    /* 0x18 */ 8,  /* step detector            */
    /* 0x19 */ 6,  /* shake detector           */
    /* 0x1A */ 6,  /* flip detector            */
    /* 0x1B */ 6,  /* pickup detector          */
    /* 0x1C */ 6,  /* stability detector       */
    /* 0x1D */ 0,
    /* 0x1E */ 16, /* personal activity class. */
    /* 0x1F */ 5,  /* sleep detector           */
    /* 0x20 */ 6,  /* tilt detector            */
    /* 0x21 */ 6,  /* pocket detector          */
    /* 0x22 */ 6,  /* circle detector          */
    /* 0x23 */ 6,  /* heart rate monitor       */
    /* 0x24 */ 0, 0, 0, 0,
    /* 0x28 */ 14, /* AR/VR stabilized RV      */
    /* 0x29 */ 12, /* AR/VR stabilized GRV     */
    /* 0x2A */ 14, /* gyro integrated RV       */
    /* 0x2B */ 6,  /* motion request (BNO086)  */
    /* 0x2C */ 0, 0, 0,
    /* 0x30 */ 0, 0, 0, 0, 0, 0, 0, 0,
    /* 0x38 */ 0, 0, 0, 0, 0, 0, 0, 0
};

/* ---------------------------------------------------------------------------
 * Known genuine firmware part numbers.
 * CEVA ships the BNO085 and BNO086 with the same SH-2 application image, part
 * number 10003606. If CEVA publishes a new build with a different part number
 * this table will not know it, which is why an unrecognised number is reported
 * as MASSMORE_BNO08X_AUTH_UNKNOWN_FW rather than as a failure.
 * ------------------------------------------------------------------------- */
static const uint32_t kKnownPartNumbers[] = {
    10003606UL,   /* BNO080 / BNO085 / BNO086 SH-2 application */
    10004095UL    /* seen on later BNO086 production builds     */
};
/* Companion images answer the same request with their own part numbers —
 * 10004563 has been observed alongside 10003606 on genuine BNO086 parts. They
 * are not SH-2 applications, so they are deliberately NOT in the table above;
 * verifyChip() looks at every response, so one match is enough. */
static const uint8_t kKnownPartCount =
    sizeof(kKnownPartNumbers) / sizeof(kKnownPartNumbers[0]);

/* ===========================================================================
 * Construction
 * ========================================================================= */
Massmore_BNO08x::Massmore_BNO08x() {
    _busType  = MASSMORE_BNO08X_BUS_NONE;
    _i2c      = nullptr;
    _spi      = nullptr;
    _uart     = nullptr;
    _dbg      = nullptr;
    _i2cAddr  = MASSMORE_BNO08X_I2C_ADDR_DEF;
    _i2cChunk = MASSMORE_BNO08X_I2C_BUF;
    _spiSpeed = 3000000UL;
    _csPin = _intPin = _rstPin = _wakePin = -1;
    _reportCb    = nullptr;
    _reportCbCtx = nullptr;
    _lastError   = MASSMORE_BNO08X_OK;
    _lastAuth    = MASSMORE_BNO08X_AUTH_NO_RESPONSE;
    memset(_advertReportLen, 0, sizeof(_advertReportLen));
    resetState();
}

void Massmore_BNO08x::resetState() {
    _rxLen = 0; _rxChannel = 0; _rxSeq = 0;
    _rxHostMicros = 0;
    _productIdCount = 0;
    memset(_productIds, 0, sizeof(_productIds));
    memset(_seqNum, 0, sizeof(_seqNum));
    _cmdSeqNum = 0;

    memset(&_quat, 0, sizeof(_quat));
    _quat.real = 1.0f;                       /* identity, so Euler is 0/0/0 */
    memset(&_accel, 0, sizeof(_accel));
    memset(&_gyro, 0, sizeof(_gyro));
    memset(&_mag, 0, sizeof(_mag));
    memset(&_linAccel, 0, sizeof(_linAccel));
    memset(&_gravity, 0, sizeof(_gravity));
    memset(&_gyroBias, 0, sizeof(_gyroBias));
    memset(&_magBias, 0, sizeof(_magBias));
    memset(&_angVel, 0, sizeof(_angVel));
    memset(&_rawAccel, 0, sizeof(_rawAccel));
    memset(&_rawGyro, 0, sizeof(_rawGyro));
    memset(&_rawMag, 0, sizeof(_rawMag));
    _rawGyroTemp = 0;
    _rawAccelTimestamp = _rawGyroTimestamp = _rawMagTimestamp = 0;

    _pressure = _ambientLight = _humidity = _proximity = _temperature = 0.0f;

    _stepCount = 0;
    _tapFlags  = 0;
    _shakeFlags = 0;
    _heartRate = 0;
    _sleepState = 0;
    _sigMotion = _flip = _pickup = _tilt = _pocket = _circle = false;
    _stepDetected = _stabilityChanged = false;
    _stability = MASSMORE_BNO08X_STABILITY_UNKNOWN;
    _activityMostLikely = 0;
    memset(_activityConfidence, 0, sizeof(_activityConfidence));

    _timestampUs = 0;
    _timebaseDelta100us = 0;
    _lastReportId = 0;
    _lastReportSeq = 0;

    memset(_accuracyTable, 0, sizeof(_accuracyTable));
    memset(_newFlags, 0, sizeof(_newFlags));
    memset(_intervals, 0, sizeof(_intervals));

    memset(&_productId, 0, sizeof(_productId));
    _calibrationStatus = 1;                   /* 1 = "not confirmed yet" */
    _oscillatorType = 0xFF;
    _errorCount = 0;
    _frsReadDone = _frsReadError = false;
    _frsWordsRead = 0;
    _frsTarget = nullptr;
    _frsTargetMax = 0;
    _frsWriteDone = false;
    _frsWriteWantMore = false;
    _frsWriteStatus = 0xFF;
    _resetComplete = false;
    _getFeatureResponse = false;
}

void Massmore_BNO08x::dbgPrintf(const char *fmt, ...) {
    if (!_dbg) return;
    char buf[128];
    va_list ap;
    va_start(ap, fmt);
    MASSMORE_BNO08X_VSNPRINTF(buf, sizeof(buf), fmt, ap);   /* fmt อยู่ใน flash (PSTR) */
    va_end(ap);
    _dbg->print(buf);
}

/*
 * Wait until the device is ready to talk after a reset. The datasheet [1]
 * only says the part asserts H_INTN once its reset routine completes, so waiting
 * on that pin is the correct method. No CEVA document specifies a boot-to-ready
 * time, so with no INT pin we fall back to 300 ms, the value both the Adafruit
 * and SparkFun drivers settled on.
 */
void Massmore_BNO08x::applyResetSettleDelay() {
    if (_intPin >= 0) {
        uint32_t start = millis();
        while (digitalRead(_intPin) == HIGH && (millis() - start) < 300) {
            delay(1);
        }
        delay(2);
    } else {
        delay(300);
    }
}

/* ===========================================================================
 * SECTION 1 — Start-up (เริ่มต้น bus ที่ sketch เตรียมไว้แล้ว)
 * ========================================================================= */

bool Massmore_BNO08x::begin(uint8_t address, TwoWire &wirePort,
                           int8_t intPin, int8_t rstPin) {
    _busType = MASSMORE_BNO08X_BUS_I2C;
    _i2c     = &wirePort;
    _i2cAddr = address;
    _intPin  = intPin;
    _rstPin  = rstPin;
    _wakePin = -1;
    resetState();
    memset(_advertReportLen, 0, sizeof(_advertReportLen));

    if (_intPin >= 0) pinMode(_intPin, INPUT_PULLUP);
    if (_rstPin >= 0) {
        pinMode(_rstPin, OUTPUT);
        digitalWrite(_rstPin, HIGH);
    }

#if defined(ARDUINO_ARCH_ESP32)
    /* esp32 core >= 2.0.5 (and all of 3.x) can grow the Wire buffer, which
     * lets us pull a whole cargo in fewer transactions. Harmless if it fails. */
    _i2c->setBufferSize(MASSMORE_BNO08X_I2C_BUF);
    _i2c->setTimeOut(50);
#endif
    _i2cChunk = MASSMORE_BNO08X_I2C_BUF;
    if (_i2cChunk < 8) _i2cChunk = 8;          /* sanity floor */

    /* Hardware reset if we can, otherwise ask for a soft one. */
    if (_rstPin >= 0) {
        digitalWrite(_rstPin, LOW);
        delay(10);
        digitalWrite(_rstPin, HIGH);
    }
    applyResetSettleDelay();

    /* มีอุปกรณ์ ACK ที่ address นี้ไหม? ถ้าไม่ ให้ลอง address อีกตัว (0x4A <-> 0x4B)
     * เพราะทั้งสองค่าคือ BNO08x ตัวเดียวกันที่ต่างกันแค่ระดับขา DI (SA0) */
    if (!i2cProbe(_i2cAddr)) {
        uint8_t alt = (_i2cAddr == MASSMORE_BNO08X_I2C_ADDR_LOW)
                          ? MASSMORE_BNO08X_I2C_ADDR_HIGH
                          : MASSMORE_BNO08X_I2C_ADDR_LOW;
        if ((address == MASSMORE_BNO08X_I2C_ADDR_LOW || address == MASSMORE_BNO08X_I2C_ADDR_HIGH)
            && i2cProbe(alt)) {
            dbgPrintf(PSTR("[massmore] no ACK at 0x%02X, found device at 0x%02X\n"), _i2cAddr, alt);
            _i2cAddr = alt;
        } else {
            dbgPrintf(PSTR("[massmore] no ACK at 0x%02X\n"), _i2cAddr);
            _lastError = MASSMORE_BNO08X_ERR_NO_DEVICE;
            _busType   = MASSMORE_BNO08X_BUS_NONE;
            return false;
        }
    }

    /* Drain the start-up chatter: SHTP advertisement, unsolicited reset
     * complete, and the initial product ID responses. */
    uint32_t start = millis();
    while ((millis() - start) < 300) {
        if (!update()) delay(1);
        if (_productId.valid) break;
    }

    if (requestProductID(300) != MASSMORE_BNO08X_OK) {
        dbgPrintf(PSTR("[massmore] no product ID response\n"));
        _lastError = MASSMORE_BNO08X_ERR_NO_DEVICE;
        _busType   = MASSMORE_BNO08X_BUS_NONE;
        return false;
    }

    _lastError = MASSMORE_BNO08X_OK;
    return true;
}

bool Massmore_BNO08x::beginSPI(int8_t csPin, int8_t intPin, int8_t rstPin,
                              int8_t wakePin, SPIClass &spiPort,
                              uint32_t speedHz) {
    if (csPin < 0 || intPin < 0 || rstPin < 0) {
        /* SPI mode is latched by the state of PS0/PS1 at the release of NRST,
         * and SHTP over SPI has no way to poll, so both pins are mandatory. */
        _lastError = MASSMORE_BNO08X_ERR_BAD_PARAM;
        return false;
    }

    _busType  = MASSMORE_BNO08X_BUS_SPI;
    _spi      = &spiPort;
    _csPin    = csPin;
    _intPin   = intPin;
    _rstPin   = rstPin;
    _wakePin  = wakePin;
    _spiSpeed = speedHz;
    resetState();
    memset(_advertReportLen, 0, sizeof(_advertReportLen));

    pinMode(_csPin, OUTPUT);
    digitalWrite(_csPin, HIGH);
    pinMode(_intPin, INPUT_PULLUP);
    pinMode(_rstPin, OUTPUT);
    if (_wakePin >= 0) {
        pinMode(_wakePin, OUTPUT);
        digitalWrite(_wakePin, HIGH);          /* PS0/WAKE is active low */
    }

    /* Reset with PS0/PS1 high so the part comes up in SPI mode — [1] §1.2.4 */
    digitalWrite(_rstPin, LOW);
    delay(10);
    digitalWrite(_rstPin, HIGH);
    applyResetSettleDelay();

    uint32_t start = millis();
    while ((millis() - start) < 400) {
        if (!update()) delay(1);
        if (_productId.valid) break;
    }

    if (requestProductID(400) != MASSMORE_BNO08X_OK) {
        _lastError = MASSMORE_BNO08X_ERR_NO_DEVICE;
        _busType   = MASSMORE_BNO08X_BUS_NONE;
        return false;
    }

    _lastError = MASSMORE_BNO08X_OK;
    return true;
}

bool Massmore_BNO08x::beginUART(Stream &serialPort, int8_t intPin, int8_t rstPin) {
    _busType = MASSMORE_BNO08X_BUS_UART;
    _uart    = &serialPort;
    _intPin  = intPin;
    _rstPin  = rstPin;
    resetState();
    memset(_advertReportLen, 0, sizeof(_advertReportLen));

    if (_intPin >= 0) pinMode(_intPin, INPUT_PULLUP);
    if (_rstPin >= 0) {
        pinMode(_rstPin, OUTPUT);
        digitalWrite(_rstPin, LOW);
        delay(10);
        digitalWrite(_rstPin, HIGH);
    }
    applyResetSettleDelay();

    uint32_t start = millis();
    while ((millis() - start) < 400) {
        if (!update()) delay(1);
        if (_productId.valid) break;
    }

    if (requestProductID(400) != MASSMORE_BNO08X_OK) {
        _lastError = MASSMORE_BNO08X_ERR_NO_DEVICE;
        _busType   = MASSMORE_BNO08X_BUS_NONE;
        return false;
    }
    _lastError = MASSMORE_BNO08X_OK;
    return true;
}

/* ===========================================================================
 * SECTION 2 — SHTP transport (รับ/ส่ง packet ระดับ bus)
 * ========================================================================= */

bool Massmore_BNO08x::dataAvailable() {
    if (_busType == MASSMORE_BNO08X_BUS_NONE) return false;
    if (_intPin >= 0) return digitalRead(_intPin) == LOW;  /* H_INTN active low */
    if (_busType == MASSMORE_BNO08X_BUS_UART) return _uart->available() > 0;
    return true;   /* no INT pin on I2C: we have to try a read to find out */
}

bool Massmore_BNO08x::waitForInt(uint32_t timeoutMs) {
    if (_intPin < 0) { delay(1); return true; }
    uint32_t start = millis();
    while (digitalRead(_intPin) == HIGH) {
        if ((millis() - start) >= timeoutMs) return false;
        delayMicroseconds(50);
    }
    return true;
}

/*
 * SHTP over I2C — [3] §3.2. Every transfer must end with a STOP, so we read
 * the 4 byte header first to learn the cargo length, then pull the payload in
 * chunks. The hub repeats a header at the start of every chunk; we discard it.
 */
bool Massmore_BNO08x::i2cReceivePacket() {
    uint8_t got = _i2c->requestFrom((uint8_t)_i2cAddr, (uint8_t)4);
    if (got != 4) return false;

    uint8_t lsb = _i2c->read();
    uint8_t msb = _i2c->read();
    uint8_t ch  = _i2c->read();
    uint8_t seq = _i2c->read();

    uint16_t raw = ((uint16_t)msb << 8) | lsb;
    if (raw == 0xFFFF) return false;            /* peripheral failure — [3] §2.3.1 */
    uint16_t len = raw & 0x7FFF;                /* bit 15 = continuation flag  */
    if (len <= 4) return false;                 /* 0 = no cargo, null header   */
    len -= 4;

    _rxChannel = ch;
    _rxSeq     = seq;

    uint16_t remaining = len;
    uint16_t idx = 0;
    while (remaining > 0) {
        uint16_t chunk = remaining;
        if (chunk > (uint16_t)(_i2cChunk - 4)) chunk = _i2cChunk - 4;

        uint8_t want = (uint8_t)(chunk + 4);
        if (_i2c->requestFrom((uint8_t)_i2cAddr, want) != want) return false;

        /* discard the repeated header */
        for (uint8_t i = 0; i < 4; i++) (void)_i2c->read();

        for (uint16_t i = 0; i < chunk; i++) {
            uint8_t b = _i2c->read();
            if (idx < MASSMORE_BNO08X_MAX_PACKET) _rxBuf[idx++] = b;
        }
        remaining -= chunk;
    }

    _rxLen = idx;
    return true;
}

/*
 * SHTP over SPI — [1] §1.2.4.2, CPOL=1/CPHA=1 (SPI_MODE3), MSB first.
 * Unlike I2C, one chip select assertion can carry the whole cargo.
 */
bool Massmore_BNO08x::spiReceivePacket() {
    if (_intPin >= 0 && digitalRead(_intPin) == HIGH) return false;

    _spi->beginTransaction(SPISettings(_spiSpeed, MSBFIRST, SPI_MODE3));
    digitalWrite(_csPin, LOW);

    uint8_t hdr[4];
    for (uint8_t i = 0; i < 4; i++) hdr[i] = _spi->transfer(0x00);

    uint16_t raw = ((uint16_t)hdr[1] << 8) | hdr[0];
    uint16_t len = raw & 0x7FFF;

    if (raw == 0xFFFF || len <= 4) {
        digitalWrite(_csPin, HIGH);
        _spi->endTransaction();
        return false;
    }
    len -= 4;

    _rxChannel = hdr[2];
    _rxSeq     = hdr[3];

    uint16_t idx = 0;
    for (uint16_t i = 0; i < len; i++) {
        uint8_t b = _spi->transfer(0x00);
        if (idx < MASSMORE_BNO08X_MAX_PACKET) _rxBuf[idx++] = b;
    }

    digitalWrite(_csPin, HIGH);
    _spi->endTransaction();

    _rxLen = idx;
    return true;
}

/*
 * SHTP over UART — [3] §4. Frames are 0x7E <protocol id> <escaped data> 0x7E,
 * with 0x7D as the control escape (next byte XOR 0x20). Protocol ID 1 = SHTP.
 */
bool Massmore_BNO08x::uartReceivePacket() {
    if (!_uart->available()) return false;

    uint32_t deadline = millis() + 20;
    /* hunt for a start flag */
    int b = -1;
    while (millis() < deadline) {
        if (!_uart->available()) { delayMicroseconds(200); continue; }
        b = _uart->read();
        if (b == 0x7E) break;
    }
    if (b != 0x7E) return false;

    uint8_t  frame[MASSMORE_BNO08X_MAX_PACKET + 8];
    uint16_t n = 0;
    bool escaped = false;
    bool complete = false;

    while (millis() < deadline) {
        if (!_uart->available()) { delayMicroseconds(200); continue; }
        int c = _uart->read();
        if (c < 0) continue;

        if (c == 0x7E) {
            if (n == 0) continue;             /* back to back flags — [3] §4.2 */
            complete = true;
            break;
        }
        if (c == 0x7D) { escaped = true; continue; }
        if (escaped) { c ^= 0x20; escaped = false; }
        if (n < sizeof(frame)) frame[n++] = (uint8_t)c;
    }
    if (!complete || n < 5) return false;

    if (frame[0] != 0x01) return false;        /* not an SHTP payload */

    uint16_t raw = ((uint16_t)frame[2] << 8) | frame[1];
    uint16_t len = raw & 0x7FFF;
    if (raw == 0xFFFF || len <= 4) return false;
    len -= 4;

    _rxChannel = frame[3];
    _rxSeq     = frame[4];

    uint16_t avail = n - 5;
    if (len > avail) len = avail;
    if (len > MASSMORE_BNO08X_MAX_PACKET) len = MASSMORE_BNO08X_MAX_PACKET;
    memcpy(_rxBuf, &frame[5], len);
    _rxLen = len;
    return true;
}

bool Massmore_BNO08x::receivePacket() {
    switch (_busType) {
        case MASSMORE_BNO08X_BUS_I2C:  return i2cReceivePacket();
        case MASSMORE_BNO08X_BUS_SPI:  return spiReceivePacket();
        case MASSMORE_BNO08X_BUS_UART: return uartReceivePacket();
        default:                return false;
    }
}

/* --- transmit -------------------------------------------------------------
 * The caller has already placed the payload at _txBuf[4..]; we fill in the
 * 4 byte SHTP header — [1] Figure 1-26 — and push it out.
 * ------------------------------------------------------------------------ */
bool Massmore_BNO08x::txPacket(uint8_t channel, uint16_t payloadLen) {
    if (channel > 5) { _lastError = MASSMORE_BNO08X_ERR_BAD_PARAM; return false; }
    if (payloadLen + 4 > MASSMORE_BNO08X_MAX_PACKET) {
        _lastError = MASSMORE_BNO08X_ERR_BAD_PARAM;
        return false;
    }

    uint16_t total = payloadLen + 4;
    _txBuf[0] = (uint8_t)(total & 0xFF);
    _txBuf[1] = (uint8_t)(total >> 8);         /* bit 15 clear: not a continuation */
    _txBuf[2] = channel;
    _txBuf[3] = _seqNum[channel]++;

    switch (_busType) {
        case MASSMORE_BNO08X_BUS_I2C:  return i2cSendPacket(channel, payloadLen);
        case MASSMORE_BNO08X_BUS_SPI:  return spiSendPacket(channel, payloadLen);
        case MASSMORE_BNO08X_BUS_UART: return uartSendPacket(channel, payloadLen);
        default: _lastError = MASSMORE_BNO08X_ERR_NOT_READY; return false;
    }
}

bool Massmore_BNO08x::i2cSendPacket(uint8_t channel, uint16_t payloadLen) {
    (void)channel;
    uint16_t total = payloadLen + 4;
    _i2c->beginTransmission(_i2cAddr);
    _i2c->write(_txBuf, total);
    if (_i2c->endTransmission() != 0) {
        _lastError = MASSMORE_BNO08X_ERR_IO;
        return false;
    }
    return true;
}

bool Massmore_BNO08x::spiSendPacket(uint8_t channel, uint16_t payloadLen) {
    (void)channel;
    uint16_t total = payloadLen + 4;

    /* SPI is duplex: anything the hub wanted to send while we write would be
     * clobbered. If it is asserting HINT, take that cargo first. */
    for (uint8_t guard = 0; guard < 4; guard++) {
        if (_intPin < 0 || digitalRead(_intPin) == HIGH) break;
        if (!receivePacket()) break;
        parsePacket();
    }

    if (_wakePin >= 0) {
        digitalWrite(_wakePin, LOW);           /* PS0/WAKE active low — [1] §1.2.4.3 */
        waitForInt(10);
    }

    _spi->beginTransaction(SPISettings(_spiSpeed, MSBFIRST, SPI_MODE3));
    digitalWrite(_csPin, LOW);
    for (uint16_t i = 0; i < total; i++) _spi->transfer(_txBuf[i]);
    digitalWrite(_csPin, HIGH);
    _spi->endTransaction();

    if (_wakePin >= 0) digitalWrite(_wakePin, HIGH);
    return true;
}

bool Massmore_BNO08x::uartSendPacket(uint8_t channel, uint16_t payloadLen) {
    (void)channel;
    uint16_t total = payloadLen + 4;

    _uart->write((uint8_t)0x7E);
    delayMicroseconds(100);                    /* host bytes need >=100us gaps [1] §1.2.3.1 */
    _uart->write((uint8_t)0x01);               /* protocol ID 1 = SHTP */
    delayMicroseconds(100);

    for (uint16_t i = 0; i < total; i++) {
        uint8_t b = _txBuf[i];
        if (b == 0x7E || b == 0x7D) {
            _uart->write((uint8_t)0x7D);
            delayMicroseconds(100);
            _uart->write((uint8_t)(b ^ 0x20));
        } else {
            _uart->write(b);
        }
        delayMicroseconds(100);
    }
    _uart->write((uint8_t)0x7E);
    return true;
}

Massmore_BNO08x_status_t Massmore_BNO08x::sendPacket(uint8_t channel,
                                             const uint8_t *data, uint16_t len) {
    if (_busType == MASSMORE_BNO08X_BUS_NONE) return (_lastError = MASSMORE_BNO08X_ERR_NOT_READY);
    if (!data || len == 0 || len + 4 > MASSMORE_BNO08X_MAX_PACKET)
        return (_lastError = MASSMORE_BNO08X_ERR_BAD_PARAM);
    memcpy(&_txBuf[4], data, len);
    return txPacket(channel, len) ? MASSMORE_BNO08X_OK : _lastError;
}

const uint8_t *Massmore_BNO08x::getRawPacket(uint16_t &len, uint8_t &channel) const {
    len = _rxLen;
    channel = _rxChannel;
    return _rxBuf;
}

/* ===========================================================================
 * SECTION 3 — Main loop (Non-blocking update)
 * ========================================================================= */

bool Massmore_BNO08x::update() {
    if (_busType == MASSMORE_BNO08X_BUS_NONE) return false;
    if (_intPin >= 0 && digitalRead(_intPin) == HIGH) return false;
    if (!receivePacket()) return false;
    /* Anchor for the report timestamps in this packet. Taken before parsing so
     * the value is as close as possible to the moment the bytes arrived. */
    _rxHostMicros = micros();
    parsePacket();
    return true;
}

uint8_t Massmore_BNO08x::updateAll(uint8_t maxPackets) {
    uint8_t n = 0;
    while (n < maxPackets && update()) n++;
    return n;
}

void Massmore_BNO08x::setReportCallback(void (*cb)(uint8_t, void *), void *ctx) {
    _reportCb = cb;
    _reportCbCtx = ctx;
}

void Massmore_BNO08x::markNew(uint8_t id) {
    if (id < 0x40) _newFlags[id >> 3] |= (uint8_t)(1u << (id & 7));
    _lastReportId = id;
    if (_reportCb) _reportCb(id, _reportCbCtx);
}

bool Massmore_BNO08x::hasNewReport(uint8_t id) {
    if (id >= 0x40) return false;
    uint8_t mask = (uint8_t)(1u << (id & 7));
    bool set = (_newFlags[id >> 3] & mask) != 0;
    _newFlags[id >> 3] &= (uint8_t)~mask;
    return set;
}

bool Massmore_BNO08x::peekNewReport(uint8_t id) const {
    if (id >= 0x40) return false;
    return (_newFlags[id >> 3] & (uint8_t)(1u << (id & 7))) != 0;
}

void Massmore_BNO08x::clearNewFlags() {
    memset(_newFlags, 0, sizeof(_newFlags));
}

void Massmore_BNO08x::setAccuracy(uint8_t id, uint8_t acc) {
    if (id < 0x40) _accuracyTable[id] = acc & 0x03;
}

Massmore_BNO08x_accuracy_t Massmore_BNO08x::getAccuracy(uint8_t sensorId) const {
    if (sensorId >= 0x40) return MASSMORE_BNO08X_ACCURACY_UNRELIABLE;
    return (Massmore_BNO08x_accuracy_t)_accuracyTable[sensorId];
}

/* ===========================================================================
 * SECTION 4 — Packet parsing (ถอดรหัส report)
 * ========================================================================= */

void Massmore_BNO08x::parsePacket() {
    if (_rxLen == 0) return;

    switch (_rxChannel) {
    case MASSMORE_BNO08X_CH_COMMAND:
        /* SHTP advertisement (response to Get Advertisement, command 0).
         * A genuine BNO08x publishes the exact length of every report it
         * supports here — [3] §5.2 — which is far better than guessing. */
        if (_rxBuf[0] == 0x00 && _rxLen > 2) {
            uint16_t i = 1;
            while (i + 1 < _rxLen) {
                uint8_t tag = _rxBuf[i];
                uint8_t len = _rxBuf[i + 1];
                if (tag == MASSMORE_BNO08X_TAG_NULL && len == 0) break;
                uint16_t val = i + 2;
                if (val + len > _rxLen) break;

                if (tag == MASSMORE_BNO08X_TAG_SH2_REPORT_LENS) {
                    for (uint8_t n = 0; n + 1 < len; n += 2) {
                        uint8_t rid = _rxBuf[val + n];
                        uint8_t rlen = _rxBuf[val + n + 1];
                        if (rid < 0x40) _advertReportLen[rid] = rlen;
                    }
                }
                i = val + len;
            }
        } else if (_rxBuf[0] == 0x01) {
            /* Error list response — [3] §5.1.2: one byte per error after the
             * report ID and a severity byte. */
            _errorCount = (_rxLen > 1) ? (uint8_t)(_rxLen - 1) : 0;
        }
        break;

    case MASSMORE_BNO08X_CH_EXECUTABLE:
        if (_rxBuf[0] == MASSMORE_BNO08X_EXEC_RESET_COMPLETE) {
            _resetComplete = true;
            dbgPrintf(PSTR("[massmore] reset complete\n"));
        }
        break;

    case MASSMORE_BNO08X_CH_CONTROL:
        parseControlReport();
        break;

    case MASSMORE_BNO08X_CH_INPUT_REPORT:
        parseInputReports(false);
        break;

    case MASSMORE_BNO08X_CH_WAKE_REPORT:
        parseInputReports(true);
        break;

    case MASSMORE_BNO08X_CH_GYRO_RV:
        parseGyroRvPacket();
        break;

    default:
        break;
    }
}

/*
 * Channel 5 carries a bare gyro-integrated rotation vector with no report ID
 * and no timestamp prefix: 4 quaternion words then 3 angular velocity words.
 * SH-2 Reference Manual [2] §6.5.44.
 */
void Massmore_BNO08x::parseGyroRvPacket() {
    if (_rxLen < 14) return;
    _quat.i        = MASSMORE_BNO08X_Q_TO_FLOAT(rds16(&_rxBuf[0]),  MASSMORE_BNO08X_Q_QUAT);
    _quat.j        = MASSMORE_BNO08X_Q_TO_FLOAT(rds16(&_rxBuf[2]),  MASSMORE_BNO08X_Q_QUAT);
    _quat.k        = MASSMORE_BNO08X_Q_TO_FLOAT(rds16(&_rxBuf[4]),  MASSMORE_BNO08X_Q_QUAT);
    _quat.real     = MASSMORE_BNO08X_Q_TO_FLOAT(rds16(&_rxBuf[6]),  MASSMORE_BNO08X_Q_QUAT);
    _angVel.x      = MASSMORE_BNO08X_Q_TO_FLOAT(rds16(&_rxBuf[8]),  MASSMORE_BNO08X_Q_ANG_VEL);
    _angVel.y      = MASSMORE_BNO08X_Q_TO_FLOAT(rds16(&_rxBuf[10]), MASSMORE_BNO08X_Q_ANG_VEL);
    _angVel.z      = MASSMORE_BNO08X_Q_TO_FLOAT(rds16(&_rxBuf[12]), MASSMORE_BNO08X_Q_ANG_VEL);
    markNew(MASSMORE_BNO08X_SENSOR_GYRO_INTEGRATED_RV);
}

void Massmore_BNO08x::parseInputReports(bool wakeChannel) {
    (void)wakeChannel;
    uint16_t off = 0;

    while (off < _rxLen) {
        uint8_t id = _rxBuf[off];

        if (id == MASSMORE_BNO08X_REPORT_BASE_TIMESTAMP) {
            /* Base delta, signed, in 100 us ticks — [1] Figure 1-35 */
            if (off + 5 > _rxLen) break;
            _timebaseDelta100us = (int32_t)rd32(&_rxBuf[off + 1]);
            off += 5;
            continue;
        }
        if (id == MASSMORE_BNO08X_REPORT_TIMESTAMP_REBASE) {
            if (off + 5 > _rxLen) break;
            off += 5;
            continue;
        }

        uint16_t used = parseOneSensorReport(off);
        if (used == 0) break;
        off += used;
    }
}

/*!
 * Decode one sensor input report starting at `offset`.
 * Every report shares the same 4 byte prefix — [1] §1.3.5.2:
 *   [0] report ID   [1] sequence number   [2] status   [3] delay LSBs
 * status bits 1:0 are the accuracy, bits 7:2 the upper delay bits.
 * @return the number of bytes consumed, or 0 if the report is unknown.
 */
uint16_t Massmore_BNO08x::parseOneSensorReport(uint16_t offset) {
    uint8_t id = _rxBuf[offset];
    if (id >= 0x40) return 0;

    uint8_t len = _advertReportLen[id];
    if (len == 0) len = pgm_read_byte(&kFallbackReportLen[id]);
    if (len == 0) return 0;                        /* unknown report: resync */
    if (offset + len > _rxLen) return 0;

    const uint8_t *r = &_rxBuf[offset];

    _lastReportSeq = r[1];
    uint8_t status = r[2];
    setAccuracy(id, status & 0x03);

    /* Reconstruct the report timestamp — [1] §1.3.5.3.
     * delay = (status[7:2] << 8 | delayLSB) ticks of 100 us. */
    uint32_t delay100us = (((uint32_t)(status >> 2)) << 8) | r[3];

    /* Both the base delta and the delay are SIGNED offsets in 100 us ticks
     * from the instant the packet was transferred — reading the base as an
     * unsigned value turns every negative delta into ~4.29e9 and makes the
     * timestamp jump backwards. Anchor them to the host clock instead. */
    int64_t rel100us = (int64_t)_timebaseDelta100us + (int64_t)delay100us;
    int64_t ts       = (int64_t)_rxHostMicros + rel100us * 100;
    _timestampUs     = (ts < 0) ? 0 : (uint64_t)ts;

    switch (id) {
    case MASSMORE_BNO08X_SENSOR_ACCELEROMETER:
        _accel.x = MASSMORE_BNO08X_Q_TO_FLOAT(rds16(&r[4]), MASSMORE_BNO08X_Q_ACCEL);
        _accel.y = MASSMORE_BNO08X_Q_TO_FLOAT(rds16(&r[6]), MASSMORE_BNO08X_Q_ACCEL);
        _accel.z = MASSMORE_BNO08X_Q_TO_FLOAT(rds16(&r[8]), MASSMORE_BNO08X_Q_ACCEL);
        break;

    case MASSMORE_BNO08X_SENSOR_LINEAR_ACCELERATION:
        _linAccel.x = MASSMORE_BNO08X_Q_TO_FLOAT(rds16(&r[4]), MASSMORE_BNO08X_Q_ACCEL);
        _linAccel.y = MASSMORE_BNO08X_Q_TO_FLOAT(rds16(&r[6]), MASSMORE_BNO08X_Q_ACCEL);
        _linAccel.z = MASSMORE_BNO08X_Q_TO_FLOAT(rds16(&r[8]), MASSMORE_BNO08X_Q_ACCEL);
        break;

    case MASSMORE_BNO08X_SENSOR_GRAVITY:
        _gravity.x = MASSMORE_BNO08X_Q_TO_FLOAT(rds16(&r[4]), MASSMORE_BNO08X_Q_ACCEL);
        _gravity.y = MASSMORE_BNO08X_Q_TO_FLOAT(rds16(&r[6]), MASSMORE_BNO08X_Q_ACCEL);
        _gravity.z = MASSMORE_BNO08X_Q_TO_FLOAT(rds16(&r[8]), MASSMORE_BNO08X_Q_ACCEL);
        break;

    case MASSMORE_BNO08X_SENSOR_GYROSCOPE:
        _gyro.x = MASSMORE_BNO08X_Q_TO_FLOAT(rds16(&r[4]), MASSMORE_BNO08X_Q_GYRO);
        _gyro.y = MASSMORE_BNO08X_Q_TO_FLOAT(rds16(&r[6]), MASSMORE_BNO08X_Q_GYRO);
        _gyro.z = MASSMORE_BNO08X_Q_TO_FLOAT(rds16(&r[8]), MASSMORE_BNO08X_Q_GYRO);
        break;

    case MASSMORE_BNO08X_SENSOR_GYROSCOPE_UNCAL:
        _gyro.x     = MASSMORE_BNO08X_Q_TO_FLOAT(rds16(&r[4]),  MASSMORE_BNO08X_Q_GYRO);
        _gyro.y     = MASSMORE_BNO08X_Q_TO_FLOAT(rds16(&r[6]),  MASSMORE_BNO08X_Q_GYRO);
        _gyro.z     = MASSMORE_BNO08X_Q_TO_FLOAT(rds16(&r[8]),  MASSMORE_BNO08X_Q_GYRO);
        _gyroBias.x = MASSMORE_BNO08X_Q_TO_FLOAT(rds16(&r[10]), MASSMORE_BNO08X_Q_GYRO);
        _gyroBias.y = MASSMORE_BNO08X_Q_TO_FLOAT(rds16(&r[12]), MASSMORE_BNO08X_Q_GYRO);
        _gyroBias.z = MASSMORE_BNO08X_Q_TO_FLOAT(rds16(&r[14]), MASSMORE_BNO08X_Q_GYRO);
        break;

    case MASSMORE_BNO08X_SENSOR_MAGNETIC_FIELD:
        _mag.x = MASSMORE_BNO08X_Q_TO_FLOAT(rds16(&r[4]), MASSMORE_BNO08X_Q_MAG);
        _mag.y = MASSMORE_BNO08X_Q_TO_FLOAT(rds16(&r[6]), MASSMORE_BNO08X_Q_MAG);
        _mag.z = MASSMORE_BNO08X_Q_TO_FLOAT(rds16(&r[8]), MASSMORE_BNO08X_Q_MAG);
        break;

    case MASSMORE_BNO08X_SENSOR_MAGNETIC_FIELD_UNCAL:
        _mag.x     = MASSMORE_BNO08X_Q_TO_FLOAT(rds16(&r[4]),  MASSMORE_BNO08X_Q_MAG);
        _mag.y     = MASSMORE_BNO08X_Q_TO_FLOAT(rds16(&r[6]),  MASSMORE_BNO08X_Q_MAG);
        _mag.z     = MASSMORE_BNO08X_Q_TO_FLOAT(rds16(&r[8]),  MASSMORE_BNO08X_Q_MAG);
        _magBias.x = MASSMORE_BNO08X_Q_TO_FLOAT(rds16(&r[10]), MASSMORE_BNO08X_Q_MAG);
        _magBias.y = MASSMORE_BNO08X_Q_TO_FLOAT(rds16(&r[12]), MASSMORE_BNO08X_Q_MAG);
        _magBias.z = MASSMORE_BNO08X_Q_TO_FLOAT(rds16(&r[14]), MASSMORE_BNO08X_Q_MAG);
        break;

    case MASSMORE_BNO08X_SENSOR_ROTATION_VECTOR:
    case MASSMORE_BNO08X_SENSOR_GEOMAGNETIC_RV:
    case MASSMORE_BNO08X_SENSOR_ARVR_STABILIZED_RV:
        _quat.i        = MASSMORE_BNO08X_Q_TO_FLOAT(rds16(&r[4]),  MASSMORE_BNO08X_Q_QUAT);
        _quat.j        = MASSMORE_BNO08X_Q_TO_FLOAT(rds16(&r[6]),  MASSMORE_BNO08X_Q_QUAT);
        _quat.k        = MASSMORE_BNO08X_Q_TO_FLOAT(rds16(&r[8]),  MASSMORE_BNO08X_Q_QUAT);
        _quat.real     = MASSMORE_BNO08X_Q_TO_FLOAT(rds16(&r[10]), MASSMORE_BNO08X_Q_QUAT);
        _quat.accuracy = MASSMORE_BNO08X_Q_TO_FLOAT(rds16(&r[12]), MASSMORE_BNO08X_Q_QUAT_ACC);
        break;

    case MASSMORE_BNO08X_SENSOR_GAME_ROTATION_VECTOR:
    case MASSMORE_BNO08X_SENSOR_ARVR_STABILIZED_GRV:
        _quat.i        = MASSMORE_BNO08X_Q_TO_FLOAT(rds16(&r[4]),  MASSMORE_BNO08X_Q_QUAT);
        _quat.j        = MASSMORE_BNO08X_Q_TO_FLOAT(rds16(&r[6]),  MASSMORE_BNO08X_Q_QUAT);
        _quat.k        = MASSMORE_BNO08X_Q_TO_FLOAT(rds16(&r[8]),  MASSMORE_BNO08X_Q_QUAT);
        _quat.real     = MASSMORE_BNO08X_Q_TO_FLOAT(rds16(&r[10]), MASSMORE_BNO08X_Q_QUAT);
        _quat.accuracy = 0.0f;                 /* no accuracy field in this report */
        break;

    case MASSMORE_BNO08X_SENSOR_GYRO_INTEGRATED_RV:
        /* Also reachable if the device routes it to channel 3. */
        _quat.i   = MASSMORE_BNO08X_Q_TO_FLOAT(rds16(&r[4]),  MASSMORE_BNO08X_Q_QUAT);
        _quat.j   = MASSMORE_BNO08X_Q_TO_FLOAT(rds16(&r[6]),  MASSMORE_BNO08X_Q_QUAT);
        _quat.k   = MASSMORE_BNO08X_Q_TO_FLOAT(rds16(&r[8]),  MASSMORE_BNO08X_Q_QUAT);
        _quat.real= MASSMORE_BNO08X_Q_TO_FLOAT(rds16(&r[10]), MASSMORE_BNO08X_Q_QUAT);
        break;

    case MASSMORE_BNO08X_SENSOR_RAW_ACCELEROMETER:
        _rawAccel.x = rds16(&r[4]);
        _rawAccel.y = rds16(&r[6]);
        _rawAccel.z = rds16(&r[8]);
        _rawAccelTimestamp = rd32(&r[12]);
        break;

    case MASSMORE_BNO08X_SENSOR_RAW_GYROSCOPE:
        _rawGyro.x   = rds16(&r[4]);
        _rawGyro.y   = rds16(&r[6]);
        _rawGyro.z   = rds16(&r[8]);
        _rawGyroTemp = rds16(&r[10]);
        _rawGyroTimestamp = rd32(&r[12]);
        break;

    case MASSMORE_BNO08X_SENSOR_RAW_MAGNETOMETER:
        _rawMag.x = rds16(&r[4]);
        _rawMag.y = rds16(&r[6]);
        _rawMag.z = rds16(&r[8]);
        _rawMagTimestamp = rd32(&r[12]);
        break;

    case MASSMORE_BNO08X_SENSOR_PRESSURE:
        _pressure = MASSMORE_BNO08X_Q_TO_FLOAT((int32_t)rd32(&r[4]), MASSMORE_BNO08X_Q_PRESSURE);
        break;
    case MASSMORE_BNO08X_SENSOR_AMBIENT_LIGHT:
        _ambientLight = MASSMORE_BNO08X_Q_TO_FLOAT((int32_t)rd32(&r[4]), MASSMORE_BNO08X_Q_AMBIENT);
        break;
    case MASSMORE_BNO08X_SENSOR_HUMIDITY:
        _humidity = MASSMORE_BNO08X_Q_TO_FLOAT(rds16(&r[4]), MASSMORE_BNO08X_Q_HUMIDITY);
        break;
    case MASSMORE_BNO08X_SENSOR_PROXIMITY:
        _proximity = MASSMORE_BNO08X_Q_TO_FLOAT(rds16(&r[4]), MASSMORE_BNO08X_Q_PROXIMITY);
        break;
    case MASSMORE_BNO08X_SENSOR_TEMPERATURE:
        _temperature = MASSMORE_BNO08X_Q_TO_FLOAT(rds16(&r[4]), MASSMORE_BNO08X_Q_TEMPERATURE);
        break;

    case MASSMORE_BNO08X_SENSOR_TAP_DETECTOR:
        _tapFlags = r[4];
        break;

    case MASSMORE_BNO08X_SENSOR_STEP_COUNTER:
        /* r[4..7] = latency, r[8..11] = cumulative step count */
        _stepCount = rd32(&r[8]);
        break;

    case MASSMORE_BNO08X_SENSOR_STEP_DETECTOR:
        _stepDetected = true;
        break;

    case MASSMORE_BNO08X_SENSOR_SIGNIFICANT_MOTION:
        _sigMotion = (rd16(&r[4]) != 0);
        break;

    case MASSMORE_BNO08X_SENSOR_STABILITY_CLASSIFIER: {
        Massmore_BNO08x_stability_t s = (Massmore_BNO08x_stability_t)r[4];
        if (s != _stability) _stabilityChanged = true;
        _stability = s;
        break;
    }

    case MASSMORE_BNO08X_SENSOR_STABILITY_DETECTOR:
        _stabilityChanged = (rd16(&r[4]) != 0);
        break;

    case MASSMORE_BNO08X_SENSOR_SHAKE_DETECTOR:
        _shakeFlags = rd16(&r[4]);
        break;
    case MASSMORE_BNO08X_SENSOR_FLIP_DETECTOR:
        _flip = (rd16(&r[4]) != 0);
        break;
    case MASSMORE_BNO08X_SENSOR_PICKUP_DETECTOR:
        _pickup = (rd16(&r[4]) != 0);
        break;
    case MASSMORE_BNO08X_SENSOR_TILT_DETECTOR:
        _tilt = (rd16(&r[4]) != 0);
        break;
    case MASSMORE_BNO08X_SENSOR_POCKET_DETECTOR:
        _pocket = (rd16(&r[4]) != 0);
        break;
    case MASSMORE_BNO08X_SENSOR_CIRCLE_DETECTOR:
        _circle = (rd16(&r[4]) != 0);
        break;
    case MASSMORE_BNO08X_SENSOR_SLEEP_DETECTOR:
        _sleepState = r[4];
        break;
    case MASSMORE_BNO08X_SENSOR_HEART_RATE_MONITOR:
        _heartRate = rd16(&r[4]);
        break;

    case MASSMORE_BNO08X_SENSOR_ACTIVITY_CLASSIFIER:
        /* r[4] bit 7 = last page, bits 6:0 = page number.
         * r[5] = most likely state, r[6..15] = confidence 0..100 per state. */
        _activityMostLikely = r[5];
        for (uint8_t n = 0; n < MASSMORE_BNO08X_ACTIVITY_COUNT; n++) {
            _activityConfidence[n] = r[6 + n];
        }
        break;

    default:
        /* Known length, but we do not decode it. Still advance correctly. */
        break;
    }

    markNew(id);
    return len;
}

void Massmore_BNO08x::parseControlReport() {
    switch (_rxBuf[0]) {
    case MASSMORE_BNO08X_REPORT_PRODUCT_ID_RESP:
        parseProductIdResponse();
        break;
    case MASSMORE_BNO08X_REPORT_COMMAND_RESPONSE:
        parseCommandResponse();
        break;
    case MASSMORE_BNO08X_REPORT_FRS_READ_RESPONSE:
        parseFrsReadResponse();
        break;
    case MASSMORE_BNO08X_REPORT_FRS_WRITE_RESPONSE:
        /* FRS Write Response — [2] §6.3.4. _rxBuf[1] is the status:
         *   0  word received, send the next one
         *   3  write completed (success)
         *   4  ready, send data
         *   8  record valid, keep waiting
         *   1,2,5,6,7,9,10,11 are all failures */
        if (_rxLen >= 2) {
            _frsWriteStatus = _rxBuf[1];
            switch (_frsWriteStatus) {
            case 0:  /* word received */
            case 4:  /* ready         */
                _frsWriteWantMore = true;
                break;
            case 8:  /* record valid, not finished yet */
                break;
            case 3:  /* completed     */
            default: /* every other code is an error */
                _frsWriteDone = true;
                break;
            }
        }
        break;
    case MASSMORE_BNO08X_REPORT_GET_FEATURE_RESP:
        /* Same layout as Set Feature — [1] Figure 1-33. */
        if (_rxLen >= 9) {
            uint8_t sid = _rxBuf[1];
            if (sid < 0x40) _intervals[sid] = rd32(&_rxBuf[5]);
            _getFeatureResponse = true;
        }
        break;
    default:
        break;
    }
}

void Massmore_BNO08x::parseProductIdResponse() {
    if (_rxLen < 14) return;

    Massmore_BNO08x_product_id_t p;
    p.resetCause     = _rxBuf[1];
    p.swVersionMajor = _rxBuf[2];
    p.swVersionMinor = _rxBuf[3];
    p.swPartNumber   = rd32(&_rxBuf[4]);
    p.swBuildNumber  = rd32(&_rxBuf[8]);
    p.swVersionPatch = rd16(&_rxBuf[12]);
    p.valid          = true;

    /* Is this the SH-2 application, or one of the companion images? */
    bool isApp = false;
    for (uint8_t i = 0; i < kKnownPartCount; i++) {
        if (p.swPartNumber == kKnownPartNumbers[i]) { isApp = true; break; }
    }

    /* Keep every distinct response. After a reset the part re-announces itself
     * unprompted, so the same entry can arrive more than once. */
    bool duplicate = false;
    for (uint8_t i = 0; i < _productIdCount; i++) {
        if (_productIds[i].swPartNumber  == p.swPartNumber &&
            _productIds[i].swBuildNumber == p.swBuildNumber) {
            duplicate = true;
            break;
        }
    }
    if (!duplicate && _productIdCount < MASSMORE_BNO08X_MAX_PRODUCT_IDS) {
        _productIds[_productIdCount++] = p;
    }

    /* Primary entry: the SH-2 application when we can recognise it, otherwise
     * whichever response arrived first. */
    if (isApp || !_productId.valid) _productId = p;

    dbgPrintf(PSTR("[massmore] SW %u.%u.%u part %lu build %lu\n"),
              _productId.swVersionMajor, _productId.swVersionMinor,
              _productId.swVersionPatch,
              (unsigned long)_productId.swPartNumber,
              (unsigned long)_productId.swBuildNumber);
}

/*
 * Command Response — [2] §6.3.9:
 *   [0] 0xF1  [1] sequence  [2] command  [3] command sequence
 *   [4] response sequence   [5..15] R0..R10
 */
void Massmore_BNO08x::parseCommandResponse() {
    if (_rxLen < 6) return;
    uint8_t command = _rxBuf[2];

    switch (command) {
    case MASSMORE_BNO08X_CMD_ME_CALIBRATE:
        /* R0 = status, 0 on success — [5] */
        _calibrationStatus = _rxBuf[5];
        break;
    case MASSMORE_BNO08X_CMD_OSCILLATOR:
        _oscillatorType = _rxBuf[5];
        break;
    case MASSMORE_BNO08X_CMD_ERRORS:
        _errorCount = _rxBuf[5];
        break;
    default:
        break;
    }
}

/*
 * FRS Read Response — [2] §6.3.6:
 *   [0] 0xF3  [1] dataLength(7:4) | status(3:0)  [2..3] word offset
 *   [4..7] data0  [8..11] data1  [12..13] FRS type
 *
 * Status codes:
 *   0 no error (more to come)          5 record empty
 *   1 unrecognised FRS type            6 read block completed
 *   2 busy                             7 read block and record completed
 *   3 read record completed            8 device error
 *   4 offset out of range
 */
void Massmore_BNO08x::parseFrsReadResponse() {
    if (_rxLen < 14 || _frsTarget == nullptr) return;

    uint8_t  status     = _rxBuf[1] & 0x0F;
    uint8_t  dataLength = (uint8_t)((_rxBuf[1] >> 4) & 0x0F);
    uint16_t offset     = rd16(&_rxBuf[2]);

    if (status == 1 || status == 2 || status == 4 || status == 8) {
        _frsReadError = true;
        _frsReadDone  = true;
        return;
    }

    if (dataLength >= 1 && offset < _frsTargetMax) {
        _frsTarget[offset] = rd32(&_rxBuf[4]);
        if ((uint16_t)(offset + 1) > _frsWordsRead) _frsWordsRead = offset + 1;
    }
    if (dataLength >= 2 && (uint16_t)(offset + 1) < _frsTargetMax) {
        _frsTarget[offset + 1] = rd32(&_rxBuf[8]);
        if ((uint16_t)(offset + 2) > _frsWordsRead) _frsWordsRead = offset + 2;
    }

    if (status == 3 || status == 6 || status == 7) {
        _frsReadDone = true;                     /* finished, data is good */
    } else if (status == 5) {
        _frsReadError = true;                    /* record exists but is empty */
        _frsReadDone  = true;
    }
}

/* ===========================================================================
 * SECTION 5 — Identity and authenticity (ตรวจของแท้)
 * ========================================================================= */

Massmore_BNO08x_status_t Massmore_BNO08x::requestProductID(uint32_t timeoutMs) {
    if (_busType == MASSMORE_BNO08X_BUS_NONE) return (_lastError = MASSMORE_BNO08X_ERR_NOT_READY);

    _productId.valid = false;
    _productIdCount  = 0;
    memset(_productIds, 0, sizeof(_productIds));

    _txBuf[4] = MASSMORE_BNO08X_REPORT_PRODUCT_ID_REQ;
    _txBuf[5] = 0;                                  /* reserved — [1] Fig 1-28 */
    if (!txPacket(MASSMORE_BNO08X_CH_CONTROL, 2)) return _lastError;

    /* The part answers with one response per firmware image, back to back.
     * Returning on the first one is how you end up holding the bootloader's
     * entry instead of the application's, so collect the whole burst: keep
     * reading until it has been quiet for a moment, or the timeout expires. */
    const uint32_t quietMs = 80;
    uint32_t start   = millis();
    uint32_t lastNew = start;
    uint8_t  seen    = 0;

    while ((millis() - start) < timeoutMs) {
        update();
        if (_productIdCount != seen) {
            seen    = _productIdCount;
            lastNew = millis();
        } else if (seen && (millis() - lastNew) >= quietMs) {
            break;                                  /* burst finished */
        }
        delayMicroseconds(200);
    }

    return _productId.valid ? (_lastError = MASSMORE_BNO08X_OK)
                            : (_lastError = MASSMORE_BNO08X_ERR_TIMEOUT);
}

Massmore_BNO08x_auth_t Massmore_BNO08x::verifyChip() {
    _lastAuth = verifyChipInternal();
    return _lastAuth;
}

Massmore_BNO08x_auth_t Massmore_BNO08x::verifyChipInternal() {
    if (!_productId.valid) {
        if (requestProductID(300) != MASSMORE_BNO08X_OK) return MASSMORE_BNO08X_AUTH_NO_RESPONSE;
    }
    if (!_productId.valid) return MASSMORE_BNO08X_AUTH_NO_RESPONSE;

    /* A blank or non-BNO part that happens to ACK will not produce a coherent
     * version triple; real firmware is 1.x through 9.x with a non-zero build. */
    if (_productId.swVersionMajor == 0 || _productId.swVersionMajor > 9) {
        return MASSMORE_BNO08X_AUTH_BAD_VERSION;
    }
    if (_productId.swBuildNumber == 0 || _productId.swPartNumber == 0) {
        return MASSMORE_BNO08X_AUTH_BAD_RESPONSE;
    }

    /* Any one of the collected responses matching a known SH-2 application
     * build is enough. The companion images carry their own part numbers and
     * would otherwise drag a genuine part down to UNKNOWN_FW. */
    for (uint8_t e = 0; e < _productIdCount; e++) {
        for (uint8_t i = 0; i < kKnownPartCount; i++) {
            if (_productIds[e].swPartNumber == kKnownPartNumbers[i]) {
                return MASSMORE_BNO08X_AUTH_OK;
            }
        }
    }
    for (uint8_t i = 0; i < kKnownPartCount; i++) {
        if (_productId.swPartNumber == kKnownPartNumbers[i]) {
            return MASSMORE_BNO08X_AUTH_OK;
        }
    }
    return MASSMORE_BNO08X_AUTH_UNKNOWN_FW;
}

const char *Massmore_BNO08x::authToString(Massmore_BNO08x_auth_t a) {
    switch (a) {
    case MASSMORE_BNO08X_AUTH_OK:           return MASSMORE_BNO08X_STR("OK - genuine BNO08x factory firmware");
    case MASSMORE_BNO08X_AUTH_UNKNOWN_FW:   return MASSMORE_BNO08X_STR("Valid BNO08x, unrecognised firmware part number");
    case MASSMORE_BNO08X_AUTH_BAD_VERSION:  return MASSMORE_BNO08X_STR("Responded, but version fields are implausible");
    case MASSMORE_BNO08X_AUTH_NO_RESPONSE:  return MASSMORE_BNO08X_STR("No Product ID response - not a BNO08x or wiring/address wrong");
    case MASSMORE_BNO08X_AUTH_BAD_RESPONSE: return MASSMORE_BNO08X_STR("Malformed Product ID response");
    }
    return MASSMORE_BNO08X_STR("Unknown");
}

const char *Massmore_BNO08x::statusToString(Massmore_BNO08x_status_t s) {
    switch (s) {
    case MASSMORE_BNO08X_OK:               return MASSMORE_BNO08X_STR("OK");
    case MASSMORE_BNO08X_ERR_IO:           return MASSMORE_BNO08X_STR("Bus I/O error");
    case MASSMORE_BNO08X_ERR_TIMEOUT:      return MASSMORE_BNO08X_STR("Timeout");
    case MASSMORE_BNO08X_ERR_BAD_PARAM:    return MASSMORE_BNO08X_STR("Bad parameter");
    case MASSMORE_BNO08X_ERR_NO_DEVICE:    return MASSMORE_BNO08X_STR("No device found");
    case MASSMORE_BNO08X_ERR_BAD_RESPONSE: return MASSMORE_BNO08X_STR("Bad response");
    case MASSMORE_BNO08X_ERR_NOT_READY:    return MASSMORE_BNO08X_STR("Not initialised - call begin() first");
    case MASSMORE_BNO08X_ERR_UNSUPPORTED:  return MASSMORE_BNO08X_STR("Unsupported on this transport");
    case MASSMORE_BNO08X_ERR_WRONG_ID:     return MASSMORE_BNO08X_STR("Product ID does not match a BNO08x");
    }
    return MASSMORE_BNO08X_STR("Unknown");
}

const char *Massmore_BNO08x::getResetReasonString() const {
    /* Reset cause codes — [2] §6.3.2 */
    switch (_productId.resetCause) {
    case 0: return MASSMORE_BNO08X_STR("Not applicable");
    case 1: return MASSMORE_BNO08X_STR("Power on reset");
    case 2: return MASSMORE_BNO08X_STR("Internal system reset");
    case 3: return MASSMORE_BNO08X_STR("Watchdog timeout");
    case 4: return MASSMORE_BNO08X_STR("External reset (NRST)");
    case 5: return MASSMORE_BNO08X_STR("Other");
    }
    return MASSMORE_BNO08X_STR("Unknown");
}

Massmore_BNO08x_status_t Massmore_BNO08x::readSerialNumber(uint64_t &serialOut,
                                                   uint32_t timeoutMs) {
    uint32_t words[4] = {0, 0, 0, 0};
    uint16_t read = 0;
    Massmore_BNO08x_status_t rc = readFrsRecord(MASSMORE_BNO08X_FRS_SERIAL_NUMBER, words,
                                         4, read, timeoutMs);
    if (rc != MASSMORE_BNO08X_OK || read == 0) return rc == MASSMORE_BNO08X_OK
                                              ? (_lastError = MASSMORE_BNO08X_ERR_BAD_RESPONSE)
                                              : rc;
    serialOut = (read >= 2)
        ? (((uint64_t)words[1] << 32) | words[0])
        : (uint64_t)words[0];
    return (_lastError = MASSMORE_BNO08X_OK);
}

/* ===========================================================================
 * SECTION 6 — Enabling sensors (Set Feature)
 * ========================================================================= */

Massmore_BNO08x_status_t Massmore_BNO08x::setFeature(uint8_t sensorId,
                                             uint32_t reportIntervalUs,
                                             uint32_t batchIntervalUs,
                                             uint8_t flags,
                                             uint16_t changeSensitivity,
                                             uint32_t sensorSpecific) {
    if (_busType == MASSMORE_BNO08X_BUS_NONE) return (_lastError = MASSMORE_BNO08X_ERR_NOT_READY);

    /* Set Feature Command — [1] Figure 1-33, 17 bytes. */
    uint8_t *p = &_txBuf[4];
    p[0]  = MASSMORE_BNO08X_REPORT_SET_FEATURE_CMD;
    p[1]  = sensorId;
    p[2]  = flags;
    p[3]  = (uint8_t)(changeSensitivity & 0xFF);
    p[4]  = (uint8_t)(changeSensitivity >> 8);
    p[5]  = (uint8_t)(reportIntervalUs & 0xFF);
    p[6]  = (uint8_t)((reportIntervalUs >> 8) & 0xFF);
    p[7]  = (uint8_t)((reportIntervalUs >> 16) & 0xFF);
    p[8]  = (uint8_t)((reportIntervalUs >> 24) & 0xFF);
    p[9]  = (uint8_t)(batchIntervalUs & 0xFF);
    p[10] = (uint8_t)((batchIntervalUs >> 8) & 0xFF);
    p[11] = (uint8_t)((batchIntervalUs >> 16) & 0xFF);
    p[12] = (uint8_t)((batchIntervalUs >> 24) & 0xFF);
    p[13] = (uint8_t)(sensorSpecific & 0xFF);
    p[14] = (uint8_t)((sensorSpecific >> 8) & 0xFF);
    p[15] = (uint8_t)((sensorSpecific >> 16) & 0xFF);
    p[16] = (uint8_t)((sensorSpecific >> 24) & 0xFF);

    if (!txPacket(MASSMORE_BNO08X_CH_CONTROL, 17)) return _lastError;

    if (sensorId < 0x40) _intervals[sensorId] = reportIntervalUs;
    return (_lastError = MASSMORE_BNO08X_OK);
}

Massmore_BNO08x_status_t Massmore_BNO08x::enableReport(uint8_t sensorId, uint32_t us) {
    return setFeature(sensorId, us);
}

Massmore_BNO08x_status_t Massmore_BNO08x::disableReport(uint8_t sensorId) {
    return setFeature(sensorId, 0);
}

void Massmore_BNO08x::disableAllReports() {
    for (uint8_t id = 1; id < 0x40; id++) {
        if (_intervals[id] != 0) {
            setFeature(id, 0);
            delay(2);
        }
    }
}

Massmore_BNO08x_status_t Massmore_BNO08x::requestFeature(uint8_t sensorId) {
    if (_busType == MASSMORE_BNO08X_BUS_NONE) return (_lastError = MASSMORE_BNO08X_ERR_NOT_READY);
    _getFeatureResponse = false;
    _txBuf[4] = MASSMORE_BNO08X_REPORT_GET_FEATURE_REQ;
    _txBuf[5] = sensorId;
    if (!txPacket(MASSMORE_BNO08X_CH_CONTROL, 2)) return _lastError;

    uint32_t start = millis();
    while ((millis() - start) < 200) {
        if (update() && _getFeatureResponse) return (_lastError = MASSMORE_BNO08X_OK);
        delayMicroseconds(200);
    }
    return (_lastError = MASSMORE_BNO08X_ERR_TIMEOUT);
}

uint32_t Massmore_BNO08x::getReportInterval(uint8_t sensorId) const {
    return (sensorId < 0x40) ? _intervals[sensorId] : 0;
}

/* --- convenience wrappers ------------------------------------------------ */
Massmore_BNO08x_status_t Massmore_BNO08x::enableAccelerometer(uint32_t us)
    { return setFeature(MASSMORE_BNO08X_SENSOR_ACCELEROMETER, us); }
Massmore_BNO08x_status_t Massmore_BNO08x::enableGyroscope(uint32_t us)
    { return setFeature(MASSMORE_BNO08X_SENSOR_GYROSCOPE, us); }
Massmore_BNO08x_status_t Massmore_BNO08x::enableMagnetometer(uint32_t us)
    { return setFeature(MASSMORE_BNO08X_SENSOR_MAGNETIC_FIELD, us); }
Massmore_BNO08x_status_t Massmore_BNO08x::enableLinearAcceleration(uint32_t us)
    { return setFeature(MASSMORE_BNO08X_SENSOR_LINEAR_ACCELERATION, us); }
Massmore_BNO08x_status_t Massmore_BNO08x::enableGravity(uint32_t us)
    { return setFeature(MASSMORE_BNO08X_SENSOR_GRAVITY, us); }
Massmore_BNO08x_status_t Massmore_BNO08x::enableGyroscopeUncalibrated(uint32_t us)
    { return setFeature(MASSMORE_BNO08X_SENSOR_GYROSCOPE_UNCAL, us); }
Massmore_BNO08x_status_t Massmore_BNO08x::enableMagnetometerUncalibrated(uint32_t us)
    { return setFeature(MASSMORE_BNO08X_SENSOR_MAGNETIC_FIELD_UNCAL, us); }

Massmore_BNO08x_status_t Massmore_BNO08x::enableRotationVector(uint32_t us)
    { return setFeature(MASSMORE_BNO08X_SENSOR_ROTATION_VECTOR, us); }
Massmore_BNO08x_status_t Massmore_BNO08x::enableGameRotationVector(uint32_t us)
    { return setFeature(MASSMORE_BNO08X_SENSOR_GAME_ROTATION_VECTOR, us); }
Massmore_BNO08x_status_t Massmore_BNO08x::enableGeomagneticRotationVector(uint32_t us)
    { return setFeature(MASSMORE_BNO08X_SENSOR_GEOMAGNETIC_RV, us); }
Massmore_BNO08x_status_t Massmore_BNO08x::enableARVRStabilizedRotationVector(uint32_t us)
    { return setFeature(MASSMORE_BNO08X_SENSOR_ARVR_STABILIZED_RV, us); }
Massmore_BNO08x_status_t Massmore_BNO08x::enableARVRStabilizedGameRotationVector(uint32_t us)
    { return setFeature(MASSMORE_BNO08X_SENSOR_ARVR_STABILIZED_GRV, us); }
Massmore_BNO08x_status_t Massmore_BNO08x::enableGyroIntegratedRotationVector(uint32_t us)
    { return setFeature(MASSMORE_BNO08X_SENSOR_GYRO_INTEGRATED_RV, us); }

Massmore_BNO08x_status_t Massmore_BNO08x::enableRawAccelerometer(uint32_t us)
    { return setFeature(MASSMORE_BNO08X_SENSOR_RAW_ACCELEROMETER, us); }
Massmore_BNO08x_status_t Massmore_BNO08x::enableRawGyroscope(uint32_t us)
    { return setFeature(MASSMORE_BNO08X_SENSOR_RAW_GYROSCOPE, us); }
Massmore_BNO08x_status_t Massmore_BNO08x::enableRawMagnetometer(uint32_t us)
    { return setFeature(MASSMORE_BNO08X_SENSOR_RAW_MAGNETOMETER, us); }

Massmore_BNO08x_status_t Massmore_BNO08x::enableTapDetector(uint32_t us)
    { return setFeature(MASSMORE_BNO08X_SENSOR_TAP_DETECTOR, us); }
Massmore_BNO08x_status_t Massmore_BNO08x::enableStepCounter(uint32_t us)
    { return setFeature(MASSMORE_BNO08X_SENSOR_STEP_COUNTER, us); }
Massmore_BNO08x_status_t Massmore_BNO08x::enableStepDetector(uint32_t us)
    { return setFeature(MASSMORE_BNO08X_SENSOR_STEP_DETECTOR, us); }
Massmore_BNO08x_status_t Massmore_BNO08x::enableSignificantMotion(uint32_t us)
    { return setFeature(MASSMORE_BNO08X_SENSOR_SIGNIFICANT_MOTION, us); }
Massmore_BNO08x_status_t Massmore_BNO08x::enableStabilityClassifier(uint32_t us)
    { return setFeature(MASSMORE_BNO08X_SENSOR_STABILITY_CLASSIFIER, us); }
Massmore_BNO08x_status_t Massmore_BNO08x::enableStabilityDetector(uint32_t us)
    { return setFeature(MASSMORE_BNO08X_SENSOR_STABILITY_DETECTOR, us); }
Massmore_BNO08x_status_t Massmore_BNO08x::enableShakeDetector(uint32_t us)
    { return setFeature(MASSMORE_BNO08X_SENSOR_SHAKE_DETECTOR, us); }
Massmore_BNO08x_status_t Massmore_BNO08x::enableFlipDetector(uint32_t us)
    { return setFeature(MASSMORE_BNO08X_SENSOR_FLIP_DETECTOR, us); }
Massmore_BNO08x_status_t Massmore_BNO08x::enablePickupDetector(uint32_t us)
    { return setFeature(MASSMORE_BNO08X_SENSOR_PICKUP_DETECTOR, us); }
Massmore_BNO08x_status_t Massmore_BNO08x::enableSleepDetector(uint32_t us)
    { return setFeature(MASSMORE_BNO08X_SENSOR_SLEEP_DETECTOR, us); }
Massmore_BNO08x_status_t Massmore_BNO08x::enableTiltDetector(uint32_t us)
    { return setFeature(MASSMORE_BNO08X_SENSOR_TILT_DETECTOR, us); }
Massmore_BNO08x_status_t Massmore_BNO08x::enablePocketDetector(uint32_t us)
    { return setFeature(MASSMORE_BNO08X_SENSOR_POCKET_DETECTOR, us); }
Massmore_BNO08x_status_t Massmore_BNO08x::enableCircleDetector(uint32_t us)
    { return setFeature(MASSMORE_BNO08X_SENSOR_CIRCLE_DETECTOR, us); }
Massmore_BNO08x_status_t Massmore_BNO08x::enableHeartRateMonitor(uint32_t us)
    { return setFeature(MASSMORE_BNO08X_SENSOR_HEART_RATE_MONITOR, us); }

Massmore_BNO08x_status_t Massmore_BNO08x::enableActivityClassifier(uint32_t us,
                                                           uint32_t enabledActivities) {
    /* The activity bitmap rides in the sensor specific configuration word —
     * [2] §6.5.36. */
    return setFeature(MASSMORE_BNO08X_SENSOR_ACTIVITY_CLASSIFIER, us, 0,
                      MASSMORE_BNO08X_FEATURE_FLAG_NONE, 0, enabledActivities);
}

Massmore_BNO08x_status_t Massmore_BNO08x::enablePressure(uint32_t us)
    { return setFeature(MASSMORE_BNO08X_SENSOR_PRESSURE, us); }
Massmore_BNO08x_status_t Massmore_BNO08x::enableAmbientLight(uint32_t us)
    { return setFeature(MASSMORE_BNO08X_SENSOR_AMBIENT_LIGHT, us); }
Massmore_BNO08x_status_t Massmore_BNO08x::enableHumidity(uint32_t us)
    { return setFeature(MASSMORE_BNO08X_SENSOR_HUMIDITY, us); }
Massmore_BNO08x_status_t Massmore_BNO08x::enableProximity(uint32_t us)
    { return setFeature(MASSMORE_BNO08X_SENSOR_PROXIMITY, us); }
Massmore_BNO08x_status_t Massmore_BNO08x::enableTemperature(uint32_t us)
    { return setFeature(MASSMORE_BNO08X_SENSOR_TEMPERATURE, us); }

/* ===========================================================================
 * SECTION 7 — Data accessors and maths (quaternion → Euler)
 * ========================================================================= */

Massmore_BNO08x_euler_t Massmore_BNO08x::quaternionToEuler(const Massmore_BNO08x_quat_t &q) {
    Massmore_BNO08x_euler_t e;
    const float w = q.real, x = q.i, y = q.j, z = q.k;

    /* Standard ZYX (yaw-pitch-roll) extraction. */
    const float sinr_cosp = 2.0f * (w * x + y * z);
    const float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
    e.roll = atan2f(sinr_cosp, cosr_cosp);

    /* Clamp before asinf so numerical drift past +-1 cannot produce NaN. */
    float sinp = 2.0f * (w * y - z * x);
    if (sinp > 1.0f)  sinp = 1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    e.pitch = asinf(sinp);

    const float siny_cosp = 2.0f * (w * z + x * y);
    const float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
    e.yaw = atan2f(siny_cosp, cosy_cosp);

    return e;
}

Massmore_BNO08x_euler_t Massmore_BNO08x::getEuler() { return quaternionToEuler(_quat); }

Massmore_BNO08x_euler_t Massmore_BNO08x::getEulerDeg() {
    Massmore_BNO08x_euler_t e = quaternionToEuler(_quat);
    e.roll  *= 57.2957795131f;
    e.pitch *= 57.2957795131f;
    e.yaw   *= 57.2957795131f;
    return e;
}

float Massmore_BNO08x::getRoll()  { return quaternionToEuler(_quat).roll; }
float Massmore_BNO08x::getPitch() { return quaternionToEuler(_quat).pitch; }
float Massmore_BNO08x::getYaw()   { return quaternionToEuler(_quat).yaw; }
float Massmore_BNO08x::getRollDeg()  { return getRoll()  * 57.2957795131f; }
float Massmore_BNO08x::getPitchDeg() { return getPitch() * 57.2957795131f; }
float Massmore_BNO08x::getYawDeg()   { return getYaw()   * 57.2957795131f; }

float Massmore_BNO08x::getHeadingDeg() {
    float h = getYawDeg();
    if (h < 0.0f) h += 360.0f;
    return h;
}

Massmore_BNO08x_vec3_t Massmore_BNO08x::getGyroDeg() const {
    Massmore_BNO08x_vec3_t v;
    v.x = _gyro.x * 57.2957795131f;
    v.y = _gyro.y * 57.2957795131f;
    v.z = _gyro.z * 57.2957795131f;
    return v;
}

uint8_t Massmore_BNO08x::getTapDetector() {
    uint8_t f = _tapFlags;
    _tapFlags = 0;
    return f;
}

uint16_t Massmore_BNO08x::getShakeDetector() {
    uint16_t f = _shakeFlags;
    _shakeFlags = 0;
    return f;
}

bool Massmore_BNO08x::getSignificantMotion() { bool v = _sigMotion; _sigMotion = false; return v; }
bool Massmore_BNO08x::getFlipDetected()      { bool v = _flip;      _flip = false;      return v; }
bool Massmore_BNO08x::getPickupDetected()    { bool v = _pickup;    _pickup = false;    return v; }
bool Massmore_BNO08x::getTiltDetected()      { bool v = _tilt;      _tilt = false;      return v; }
bool Massmore_BNO08x::getPocketDetected()    { bool v = _pocket;    _pocket = false;    return v; }
bool Massmore_BNO08x::getCircleDetected()    { bool v = _circle;    _circle = false;    return v; }
bool Massmore_BNO08x::getStepDetected()      { bool v = _stepDetected; _stepDetected = false; return v; }
bool Massmore_BNO08x::getStabilityChanged()  { bool v = _stabilityChanged; _stabilityChanged = false; return v; }

const char *Massmore_BNO08x::getStabilityString() const {
    switch (_stability) {
    case MASSMORE_BNO08X_STABILITY_UNKNOWN:    return MASSMORE_BNO08X_STR("Unknown");
    case MASSMORE_BNO08X_STABILITY_ON_TABLE:   return MASSMORE_BNO08X_STR("On table");
    case MASSMORE_BNO08X_STABILITY_STATIONARY: return MASSMORE_BNO08X_STR("Stationary");
    case MASSMORE_BNO08X_STABILITY_STABLE:     return MASSMORE_BNO08X_STR("Stable");
    case MASSMORE_BNO08X_STABILITY_MOTION:     return MASSMORE_BNO08X_STR("In motion");
    default:                            return MASSMORE_BNO08X_STR("Reserved");
    }
}

const char *Massmore_BNO08x::getActivityString() const {
    switch (_activityMostLikely) {
    case MASSMORE_BNO08X_ACTIVITY_UNKNOWN:    return MASSMORE_BNO08X_STR("Unknown");
    case MASSMORE_BNO08X_ACTIVITY_IN_VEHICLE: return MASSMORE_BNO08X_STR("In vehicle");
    case MASSMORE_BNO08X_ACTIVITY_ON_BICYCLE: return MASSMORE_BNO08X_STR("On bicycle");
    case MASSMORE_BNO08X_ACTIVITY_ON_FOOT:    return MASSMORE_BNO08X_STR("On foot");
    case MASSMORE_BNO08X_ACTIVITY_STILL:      return MASSMORE_BNO08X_STR("Still");
    case MASSMORE_BNO08X_ACTIVITY_TILTING:    return MASSMORE_BNO08X_STR("Tilting");
    case MASSMORE_BNO08X_ACTIVITY_WALKING:    return MASSMORE_BNO08X_STR("Walking");
    case MASSMORE_BNO08X_ACTIVITY_RUNNING:    return MASSMORE_BNO08X_STR("Running");
    case MASSMORE_BNO08X_ACTIVITY_ON_STAIRS:  return MASSMORE_BNO08X_STR("On stairs");
    default:                           return MASSMORE_BNO08X_STR("Unknown");
    }
}

uint8_t Massmore_BNO08x::getActivityConfidence(Massmore_BNO08x_activity_t a) const {
    if ((uint8_t)a >= MASSMORE_BNO08X_ACTIVITY_COUNT) return 0;
    return _activityConfidence[(uint8_t)a];
}

const char *Massmore_BNO08x::accuracyToString(Massmore_BNO08x_accuracy_t a) {
    switch (a) {
    case MASSMORE_BNO08X_ACCURACY_UNRELIABLE: return MASSMORE_BNO08X_STR("Unreliable");
    case MASSMORE_BNO08X_ACCURACY_LOW:        return MASSMORE_BNO08X_STR("Low");
    case MASSMORE_BNO08X_ACCURACY_MEDIUM:     return MASSMORE_BNO08X_STR("Medium");
    case MASSMORE_BNO08X_ACCURACY_HIGH:       return MASSMORE_BNO08X_STR("High");
    }
    return MASSMORE_BNO08X_STR("Unknown");
}

/* ===========================================================================
 * SECTION 8 — Commands 0xF2 (calibration / tare / reset / sleep)
 * ========================================================================= */

/*
 * Command Request — [2] §6.3.8:
 *   [0] 0xF2  [1] sequence number  [2] command  [3..11] P0..P8
 */
Massmore_BNO08x_status_t Massmore_BNO08x::sendCommand(uint8_t command,
                                              const uint8_t *p, uint8_t pLen) {
    if (_busType == MASSMORE_BNO08X_BUS_NONE) return (_lastError = MASSMORE_BNO08X_ERR_NOT_READY);
    if (pLen > 9) return (_lastError = MASSMORE_BNO08X_ERR_BAD_PARAM);

    uint8_t *d = &_txBuf[4];
    memset(d, 0, 12);
    d[0] = MASSMORE_BNO08X_REPORT_COMMAND_REQUEST;
    d[1] = _cmdSeqNum++;
    d[2] = command;
    if (p && pLen) memcpy(&d[3], p, pLen);

    return txPacket(MASSMORE_BNO08X_CH_CONTROL, 12) ? (_lastError = MASSMORE_BNO08X_OK) : _lastError;
}

/* --- calibration --------------------------------------------------------- */
Massmore_BNO08x_status_t Massmore_BNO08x::calibrate(Massmore_BNO08x_calibrate_target_t target) {
    /* ME Calibration command — [5].
     * Motions the device expects, datasheet [1] Figure 3-2 — NOT a figure of
     * eight, that is BNO055 folklore:
     *   accelerometer  4 to 6 unique orientations, held still ~1 s in each
     *   gyroscope      set down on a stationary surface for 2 to 3 s
     *   magnetometer   rotate ~180 deg and back about EACH of roll, pitch and
     *                  yaw, about 2 s per axis
     * Stop once accuracy reads Medium or High, then saveCalibration().
     * Calibration ENABLE settings do not survive a reset — [1] §3.1.1.
     * Parameters:
     * P0 accel enable, P1 gyro enable, P2 mag enable, P3 subcommand,
     * P4 planar accel enable. */
    uint8_t p[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    switch (target) {
    case MASSMORE_BNO08X_CAL_ACCEL:          p[0] = 1; break;
    case MASSMORE_BNO08X_CAL_GYRO:           p[1] = 1; break;
    case MASSMORE_BNO08X_CAL_MAG:            p[2] = 1; break;
    case MASSMORE_BNO08X_CAL_PLANAR_ACCEL:   p[4] = 1; break;
    case MASSMORE_BNO08X_CAL_ACCEL_GYRO_MAG: p[0] = p[1] = p[2] = 1; break;
    case MASSMORE_BNO08X_CAL_STOP:           break;    /* all zero = disable */
    default: return (_lastError = MASSMORE_BNO08X_ERR_BAD_PARAM);
    }
    _calibrationStatus = 1;                     /* until the device confirms */
    return sendCommand(MASSMORE_BNO08X_CMD_ME_CALIBRATE, p, 9);
}

Massmore_BNO08x_status_t Massmore_BNO08x::requestCalibrationStatus() {
    uint8_t p[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    p[3] = 0x01;                                /* subcommand: get ME calibration */
    return sendCommand(MASSMORE_BNO08X_CMD_ME_CALIBRATE, p, 9);
}

Massmore_BNO08x_status_t Massmore_BNO08x::saveCalibration() {
    return sendCommand(MASSMORE_BNO08X_CMD_SAVE_DCD, nullptr, 0);
}

Massmore_BNO08x_status_t Massmore_BNO08x::setPeriodicCalibrationSave(bool enable) {
    uint8_t p[9] = {0};
    p[0] = enable ? 0 : 1;                      /* P0: 0 = enable, 1 = disable */
    return sendCommand(MASSMORE_BNO08X_CMD_DCD_PERIOD_SAVE, p, 9);
}

Massmore_BNO08x_status_t Massmore_BNO08x::clearCalibrationAndReset() {
    Massmore_BNO08x_status_t rc = sendCommand(MASSMORE_BNO08X_CMD_CLEAR_DCD, nullptr, 0);
    if (rc != MASSMORE_BNO08X_OK) return rc;
    delay(200);                                 /* the device reboots itself */
    applyResetSettleDelay();
    updateAll(16);
    return MASSMORE_BNO08X_OK;
}

/* --- tare ---------------------------------------------------------------- */
Massmore_BNO08x_status_t Massmore_BNO08x::tareNow(uint8_t axes, Massmore_BNO08x_tare_basis_t basis) {
    if ((axes & MASSMORE_BNO08X_TARE_AXIS_ALL) == 0) return (_lastError = MASSMORE_BNO08X_ERR_BAD_PARAM);
    uint8_t p[9] = {0};
    p[0] = MASSMORE_BNO08X_TARE_NOW;                   /* P0 subcommand */
    p[1] = (uint8_t)(axes & 0x07);              /* P1 axis bitmap */
    p[2] = (uint8_t)basis;                      /* P2 rotation vector basis */
    return sendCommand(MASSMORE_BNO08X_CMD_TARE, p, 9);
}

Massmore_BNO08x_status_t Massmore_BNO08x::persistTare() {
    uint8_t p[9] = {0};
    p[0] = MASSMORE_BNO08X_TARE_PERSIST;
    return sendCommand(MASSMORE_BNO08X_CMD_TARE, p, 9);
}

Massmore_BNO08x_status_t Massmore_BNO08x::clearTare() {
    /* Set Reorientation with an identity quaternion clears the stored tare. */
    uint8_t p[9] = {0};
    p[0] = MASSMORE_BNO08X_TARE_SET_REORIENTATION;
    /* P1..P8 = X, Y, Z, W as int16 Q14. Identity is (0,0,0,1.0) => W = 0x4000 */
    p[7] = 0x00;
    p[8] = 0x40;
    return sendCommand(MASSMORE_BNO08X_CMD_TARE, p, 9);
}

/* --- power / reset -------------------------------------------------------- */
Massmore_BNO08x_status_t Massmore_BNO08x::softReset() {
    if (_busType == MASSMORE_BNO08X_BUS_NONE) return (_lastError = MASSMORE_BNO08X_ERR_NOT_READY);
    _resetComplete = false;
    _txBuf[4] = MASSMORE_BNO08X_EXEC_RESET;
    if (!txPacket(MASSMORE_BNO08X_CH_EXECUTABLE, 1)) return _lastError;

    delay(100);
    memset(_advertReportLen, 0, sizeof(_advertReportLen));
    applyResetSettleDelay();
    updateAll(24);                               /* consume advert + reset complete */
    return (_lastError = MASSMORE_BNO08X_OK);
}

Massmore_BNO08x_status_t Massmore_BNO08x::hardwareReset() {
    if (_rstPin < 0) return (_lastError = MASSMORE_BNO08X_ERR_UNSUPPORTED);
    _resetComplete = false;
    digitalWrite(_rstPin, LOW);
    delay(10);
    digitalWrite(_rstPin, HIGH);
    memset(_advertReportLen, 0, sizeof(_advertReportLen));
    applyResetSettleDelay();
    updateAll(24);
    return (_lastError = MASSMORE_BNO08X_OK);
}

Massmore_BNO08x_status_t Massmore_BNO08x::modeOn() {
    _txBuf[4] = MASSMORE_BNO08X_EXEC_ON;
    return txPacket(MASSMORE_BNO08X_CH_EXECUTABLE, 1) ? (_lastError = MASSMORE_BNO08X_OK) : _lastError;
}

Massmore_BNO08x_status_t Massmore_BNO08x::modeSleep() {
    _txBuf[4] = MASSMORE_BNO08X_EXEC_SLEEP;
    return txPacket(MASSMORE_BNO08X_CH_EXECUTABLE, 1) ? (_lastError = MASSMORE_BNO08X_OK) : _lastError;
}

void Massmore_BNO08x::wake() {
    if (_wakePin < 0) return;
    digitalWrite(_wakePin, LOW);
    delayMicroseconds(50);
    waitForInt(10);
    digitalWrite(_wakePin, HIGH);
}

Massmore_BNO08x_status_t Massmore_BNO08x::requestOscillatorType() {
    _oscillatorType = 0xFF;
    Massmore_BNO08x_status_t rc = sendCommand(MASSMORE_BNO08X_CMD_OSCILLATOR, nullptr, 0);
    if (rc != MASSMORE_BNO08X_OK) return rc;
    uint32_t start = millis();
    while ((millis() - start) < 200) {
        if (update() && _oscillatorType != 0xFF) return (_lastError = MASSMORE_BNO08X_OK);
        delayMicroseconds(200);
    }
    return (_lastError = MASSMORE_BNO08X_ERR_TIMEOUT);
}

Massmore_BNO08x_status_t Massmore_BNO08x::requestErrorList() {
    uint8_t p[9] = {0};
    p[0] = 0;                                    /* severity 0 = all errors */
    _errorCount = 0;
    Massmore_BNO08x_status_t rc = sendCommand(MASSMORE_BNO08X_CMD_ERRORS, p, 9);
    if (rc != MASSMORE_BNO08X_OK) return rc;
    uint32_t start = millis();
    while ((millis() - start) < 200) { update(); delayMicroseconds(200); }
    return (_lastError = MASSMORE_BNO08X_OK);
}

/* ===========================================================================
 * SECTION 9 — FRS (flash record system)
 * ========================================================================= */

Massmore_BNO08x_status_t Massmore_BNO08x::readFrsRecord(uint16_t recordId,
                                                uint32_t *dataOut,
                                                uint16_t maxWords,
                                                uint16_t &wordsRead,
                                                uint32_t timeoutMs) {
    if (_busType == MASSMORE_BNO08X_BUS_NONE) return (_lastError = MASSMORE_BNO08X_ERR_NOT_READY);
    if (!dataOut || maxWords == 0) return (_lastError = MASSMORE_BNO08X_ERR_BAD_PARAM);

    memset(dataOut, 0, (size_t)maxWords * sizeof(uint32_t));
    _frsTarget    = dataOut;
    _frsTargetMax = maxWords;
    _frsWordsRead = 0;
    _frsReadDone  = false;
    _frsReadError = false;

    /* FRS Read Request — [2] §6.3.5:
     * [0] 0xF4 [1] reserved [2..3] read offset [4..5] record ID [6..7] block size
     * A block size of 0 means "the whole record". */
    uint8_t *p = &_txBuf[4];
    p[0] = MASSMORE_BNO08X_REPORT_FRS_READ_REQUEST;
    p[1] = 0;
    p[2] = 0; p[3] = 0;
    p[4] = (uint8_t)(recordId & 0xFF);
    p[5] = (uint8_t)(recordId >> 8);
    p[6] = 0; p[7] = 0;
    if (!txPacket(MASSMORE_BNO08X_CH_CONTROL, 8)) { _frsTarget = nullptr; return _lastError; }

    uint32_t start = millis();
    while ((millis() - start) < timeoutMs && !_frsReadDone) {
        if (!update()) delayMicroseconds(200);
    }

    wordsRead  = _frsWordsRead;
    bool err   = _frsReadError;
    bool done  = _frsReadDone;
    _frsTarget = nullptr;

    if (!done) return (_lastError = MASSMORE_BNO08X_ERR_TIMEOUT);
    if (err && wordsRead == 0) return (_lastError = MASSMORE_BNO08X_ERR_BAD_RESPONSE);
    return (_lastError = MASSMORE_BNO08X_OK);
}

Massmore_BNO08x_status_t Massmore_BNO08x::writeFrsRecord(uint16_t recordId,
                                                 const uint32_t *data,
                                                 uint16_t words,
                                                 uint32_t timeoutMs) {
    if (_busType == MASSMORE_BNO08X_BUS_NONE) return (_lastError = MASSMORE_BNO08X_ERR_NOT_READY);
    if (!data || words == 0) return (_lastError = MASSMORE_BNO08X_ERR_BAD_PARAM);

    _frsWriteDone     = false;
    _frsWriteWantMore = false;
    _frsWriteStatus   = 0xFF;

    /* FRS Write Request — [2] §6.3.3:
     * [0] 0xF7 [1] reserved [2..3] length in words [4..5] record ID */
    uint8_t *p = &_txBuf[4];
    p[0] = MASSMORE_BNO08X_REPORT_FRS_WRITE_REQUEST;
    p[1] = 0;
    p[2] = (uint8_t)(words & 0xFF);
    p[3] = (uint8_t)(words >> 8);
    p[4] = (uint8_t)(recordId & 0xFF);
    p[5] = (uint8_t)(recordId >> 8);
    if (!txPacket(MASSMORE_BNO08X_CH_CONTROL, 6)) return _lastError;

    /* The device drives the pace: it answers every data packet with a write
     * response, and only when that response says "received" or "ready" do we
     * send the next pair of words. */
    uint16_t offset = 0;
    uint32_t start  = millis();

    while ((millis() - start) < timeoutMs && !_frsWriteDone) {
        if (!update()) { delayMicroseconds(200); continue; }

        if (_frsWriteWantMore && offset < words) {
            _frsWriteWantMore = false;

            uint8_t *d = &_txBuf[4];
            memset(d, 0, 12);
            d[0] = MASSMORE_BNO08X_REPORT_FRS_WRITE_DATA;
            d[1] = 0;
            d[2] = (uint8_t)(offset & 0xFF);
            d[3] = (uint8_t)(offset >> 8);

            uint32_t w0 = data[offset++];
            d[4] = (uint8_t)(w0);       d[5] = (uint8_t)(w0 >> 8);
            d[6] = (uint8_t)(w0 >> 16); d[7] = (uint8_t)(w0 >> 24);
            if (offset < words) {
                uint32_t w1 = data[offset++];
                d[8]  = (uint8_t)(w1);       d[9]  = (uint8_t)(w1 >> 8);
                d[10] = (uint8_t)(w1 >> 16); d[11] = (uint8_t)(w1 >> 24);
            }
            if (!txPacket(MASSMORE_BNO08X_CH_CONTROL, 12)) return _lastError;
        }
    }

    if (!_frsWriteDone) return (_lastError = MASSMORE_BNO08X_ERR_TIMEOUT);
    if (_frsWriteStatus != 3)                    /* 3 = write completed */
        return (_lastError = MASSMORE_BNO08X_ERR_BAD_RESPONSE);
    return (_lastError = MASSMORE_BNO08X_OK);
}

Massmore_BNO08x_status_t Massmore_BNO08x::readSensorMetadata(uint16_t metadataRecordId,
                                                     uint32_t *dataOut,
                                                     uint16_t maxWords,
                                                     uint16_t &wordsRead) {
    return readFrsRecord(metadataRecordId, dataOut, maxWords, wordsRead, 600);
}

/* ===========================================================================
 * SECTION 10 — Massmore standard API
 *   Simple Blocking API (readAll / readEulerDeg / readHeadingDeg) และ
 *   identity helpers (verifyChipID / getSerialNumber / isGenuine)
 * ========================================================================= */

/* ตรวจว่ามีอุปกรณ์ ACK ที่ address นี้ (ใช้เฉพาะตอน begin) */
bool Massmore_BNO08x::i2cProbe(uint8_t address) {
    _i2c->beginTransmission(address);
    return _i2c->endTransmission() == 0;
}

/* ถ้า sensor นี้ยังไม่ถูก enable ให้เปิดที่ SIMPLE_INTERVAL (50 Hz) */
bool Massmore_BNO08x::ensureEnabled(uint8_t sensorId) {
    if (sensorId >= 0x40) { _lastError = MASSMORE_BNO08X_ERR_BAD_PARAM; return false; }
    if (_intervals[sensorId] != 0) return true;
    return setFeature(sensorId, MASSMORE_BNO08X_SIMPLE_INTERVAL_US) == MASSMORE_BNO08X_OK;
}

bool Massmore_BNO08x::waitForReport(uint8_t sensorId, uint32_t timeoutMs) {
    if (_busType == MASSMORE_BNO08X_BUS_NONE) { _lastError = MASSMORE_BNO08X_ERR_NOT_READY; return false; }
    if (!ensureEnabled(sensorId)) return false;
    hasNewReport(sensorId);                      /* ล้าง flag เก่าก่อนรอของใหม่ */

    uint32_t t0 = millis();
    while ((uint32_t)(millis() - t0) < timeoutMs) {      /* rollover-safe */
        if (update()) {
            if (peekNewReport(sensorId)) {
                hasNewReport(sensorId);
                _lastError = MASSMORE_BNO08X_OK;
                return true;
            }
        } else {
            delayMicroseconds(200);
        }
    }
    _lastError = MASSMORE_BNO08X_ERR_TIMEOUT;
    return false;
}

bool Massmore_BNO08x::readAll(Massmore_BNO08x_reading_t &out, uint32_t timeoutMs) {
    static const uint8_t kIds[4] = {
        MASSMORE_BNO08X_SENSOR_ROTATION_VECTOR,
        MASSMORE_BNO08X_SENSOR_ACCELEROMETER,
        MASSMORE_BNO08X_SENSOR_GYROSCOPE,
        MASSMORE_BNO08X_SENSOR_MAGNETIC_FIELD
    };
    if (_busType == MASSMORE_BNO08X_BUS_NONE) { _lastError = MASSMORE_BNO08X_ERR_NOT_READY; return false; }

    for (uint8_t i = 0; i < 4; i++) {
        if (!ensureEnabled(kIds[i])) return false;
        hasNewReport(kIds[i]);
    }

    uint32_t t0  = millis();
    uint8_t  got = 0;
    while ((uint32_t)(millis() - t0) < timeoutMs) {      /* rollover-safe */
        if (!update()) { delayMicroseconds(200); continue; }
        got = 0;
        for (uint8_t i = 0; i < 4; i++) if (peekNewReport(kIds[i])) got++;
        if (got == 4) break;
    }
    if (got < 4) { _lastError = MASSMORE_BNO08X_ERR_TIMEOUT; return false; }

    for (uint8_t i = 0; i < 4; i++) hasNewReport(kIds[i]);
    getReadings(out);
    _lastError = MASSMORE_BNO08X_OK;
    return true;
}

bool Massmore_BNO08x::readEulerDeg(Massmore_BNO08x_euler_t &outDeg, uint32_t timeoutMs) {
    if (!waitForReport(MASSMORE_BNO08X_SENSOR_ROTATION_VECTOR, timeoutMs)) return false;
    outDeg = getEulerDeg();
    return true;
}

float Massmore_BNO08x::readHeadingDeg(uint32_t timeoutMs) {
    if (!waitForReport(MASSMORE_BNO08X_SENSOR_ROTATION_VECTOR, timeoutMs)) return NAN;
    return getHeadingDeg();
}

void Massmore_BNO08x::getReadings(Massmore_BNO08x_reading_t &out) const {
    out.quat        = _quat;
    out.eulerDeg    = quaternionToEuler(_quat);
    out.eulerDeg.roll  *= 57.2957795131f;
    out.eulerDeg.pitch *= 57.2957795131f;
    out.eulerDeg.yaw   *= 57.2957795131f;
    out.headingDeg  = (out.eulerDeg.yaw < 0.0f) ? out.eulerDeg.yaw + 360.0f : out.eulerDeg.yaw;
    out.accel       = _accel;
    out.gyro        = _gyro;
    out.mag         = _mag;
    out.linearAccel = _linAccel;
    out.gravity     = _gravity;
    out.accuracyRV  = getAccuracy(MASSMORE_BNO08X_SENSOR_ROTATION_VECTOR);
    out.accuracyMag = getAccuracy(MASSMORE_BNO08X_SENSOR_MAGNETIC_FIELD);
    out.timestampUs = _timestampUs;
}

bool Massmore_BNO08x::verifyChipID() {
    Massmore_BNO08x_auth_t a = verifyChip();
    switch (a) {
    case MASSMORE_BNO08X_AUTH_OK:
        _lastError = MASSMORE_BNO08X_OK;
        return true;
    case MASSMORE_BNO08X_AUTH_NO_RESPONSE:
        _lastError = MASSMORE_BNO08X_ERR_TIMEOUT;
        return false;
    default:
        _lastError = MASSMORE_BNO08X_ERR_WRONG_ID;
        return false;
    }
}

uint32_t Massmore_BNO08x::getSerialNumber() {
    uint64_t s = 0;
    if (readSerialNumber(s) != MASSMORE_BNO08X_OK) return 0;
    return (uint32_t)(s & 0xFFFFFFFFUL);
}

bool Massmore_BNO08x::verifyMotionEngine() {
    /* Rotation Vector metadata — [2] §4.3: word 7 = Q point 1 (bits 15:0) และ
     * Q point 2 (bits 31:16) ของแท้ต้องเป็น 14 และ 12 ตามลำดับ */
    uint32_t meta[8];
    uint16_t words = 0;
    if (readSensorMetadata(MASSMORE_BNO08X_FRS_META_ROTATION_VECTOR, meta, 8, words)
            != MASSMORE_BNO08X_OK || words < 8) {
        return false;
    }
    uint16_t q1 = (uint16_t)(meta[7] & 0xFFFF);
    uint16_t q2 = (uint16_t)((meta[7] >> 16) & 0xFFFF);
    return (q1 == MASSMORE_BNO08X_META_RV_QPOINT1) && (q2 == MASSMORE_BNO08X_META_RV_QPOINT2);
}

bool Massmore_BNO08x::isGenuine() {
    Massmore_BNO08x_auth_t a = verifyChip();
    if (a == MASSMORE_BNO08X_AUTH_OK) return true;            /* known factory SH-2 build */
    if (a != MASSMORE_BNO08X_AUTH_UNKNOWN_FW) return false;   /* no / malformed response */
    /* firmware ใหม่กว่าตาราง: ยอมรับเมื่อ MotionEngine metadata ตรง signature */
    return verifyMotionEngine();
}
