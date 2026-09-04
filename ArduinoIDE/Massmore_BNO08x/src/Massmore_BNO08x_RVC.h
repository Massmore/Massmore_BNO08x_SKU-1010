/*!
 * @file  Massmore_BNO08x_RVC.h
 * @brief UART-RVC mode driver for the BNO085 / BNO086.
 *
 * UART-RVC ("Robot Vacuum Cleaner" mode) is the simplest way to use the sensor:
 * strap PS1 low and PS0 high (the P1 and P0 pads on the Massmore board),
 * connect one wire from the sensor's TX to an RX on
 * your MCU, and the part streams heading and acceleration at 100 Hz with no
 * host commands at all. No SHTP, no configuration, no I2C.
 *
 * Trade-offs versus SHTP mode: you get yaw/pitch/roll and 3-axis acceleration
 * only, at a fixed 100 Hz, with no quaternion, no calibration control and no
 * tare. If you need any of those, use the MassmoreBNO08x class instead.
 *
 * Wiring (Datasheet [1] §1.2.5, Figure 1-23):
 *   PS1 -> GND, PS0 -> VDDIO, BOOTN -> 10k to VDDIO, sensor TX -> MCU RX.
 *   On the Massmore Halley V2 board: P1 -> GND, P0 -> 3Vo, BT left open,
 *   and the sensor's TX comes out on the SDA pad.
 *   The part needs its external crystal or clock in this mode; the internal
 *   oscillator is not accurate enough to drive the UART reliably.
 *
 * Packet format (Datasheet [1] Figure 1-25), 19 bytes at 115200 8N1:
 *   0xAA 0xAA index yawL yawH pitchL pitchH rollL rollH
 *   axL axH ayL ayH azL azH  MI MR rsvd csum
 * Angles are 0.01 degree units; acceleration is in milli-g.
 * The checksum is the 8-bit sum of bytes 2..17.
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef MASSMORE_BNO08X_RVC_H
#define MASSMORE_BNO08X_RVC_H

#include <Arduino.h>
#include "Massmore_BNO08x_Defs.h"

/*! One decoded UART-RVC frame. */
typedef struct {
    uint8_t index;      //!< 0..255, increments once per report — use it to spot drops
    float   yaw;        //!< degrees, -180..180
    float   pitch;      //!< degrees, -90..90
    float   roll;       //!< degrees, -180..180
    float   accelX;     //!< m/s^2
    float   accelY;     //!< m/s^2
    float   accelZ;     //!< m/s^2
    uint8_t motionIntent;   //!< BNO086 only, otherwise reserved
    uint8_t motionRequest;  //!< BNO086 only, otherwise reserved
} massmore_rvc_report_t;

/*!
 * @class MassmoreBNO08x_RVC
 * @brief Decoder for the BNO08x UART-RVC output stream.
 *
 * @code
 *   MassmoreBNO08x_RVC rvc;
 *   void setup() {
 *     Serial.begin(115200);
 *     Serial1.begin(115200, SERIAL_8N1, 16, 17);   // ESP32: RX=16, TX=17
 *     rvc.begin(Serial1);
 *   }
 *   void loop() {
 *     massmore_rvc_report_t r;
 *     if (rvc.read(r)) Serial.println(r.yaw);
 *   }
 * @endcode
 */
class MassmoreBNO08x_RVC {
public:
    MassmoreBNO08x_RVC() : _uart(nullptr), _idx(0), _badChecksums(0) {}

    /*!
     * @brief Attach to an already-begun serial port (115200 8N1).
     * @return always true — RVC is a one way stream, so there is nothing to
     *         probe. Call read() and check that frames start arriving.
     */
    bool begin(Stream &serialPort) {
        _uart = &serialPort;
        _idx = 0;
        _badChecksums = 0;
        return true;
    }

    /*!
     * @brief Non-blocking frame decoder. Call it often from loop().
     * @param out Receives the decoded frame.
     * @return true when a complete, checksum-valid frame was decoded.
     */
    bool read(massmore_rvc_report_t &out) {
        if (!_uart) return false;

        while (_uart->available()) {
            int c = _uart->read();
            if (c < 0) break;
            uint8_t b = (uint8_t)c;

            /* Resynchronise on the 0xAA 0xAA header. */
            if (_idx == 0) { if (b == 0xAA) _buf[_idx++] = b; continue; }
            if (_idx == 1) {
                if (b == 0xAA) _buf[_idx++] = b;
                else           _idx = 0;
                continue;
            }

            _buf[_idx++] = b;
            if (_idx < 19) continue;
            _idx = 0;

            /* Checksum: sum of the 16 payload bytes, index through reserved. */
            uint8_t sum = 0;
            for (uint8_t i = 2; i < 18; i++) sum = (uint8_t)(sum + _buf[i]);
            if (sum != _buf[18]) { _badChecksums++; continue; }

            out.index  = _buf[2];
            out.yaw    = s16(&_buf[3])  * 0.01f;
            out.pitch  = s16(&_buf[5])  * 0.01f;
            out.roll   = s16(&_buf[7])  * 0.01f;
            /* milli-g -> m/s^2 : mg * 9.80665 / 1000 */
            out.accelX = s16(&_buf[9])  * 0.0098067f;
            out.accelY = s16(&_buf[11]) * 0.0098067f;
            out.accelZ = s16(&_buf[13]) * 0.0098067f;
            out.motionIntent  = _buf[15];
            out.motionRequest = _buf[16];
            return true;
        }
        return false;
    }

    /*! @brief How many frames have been dropped for a bad checksum. */
    uint32_t getChecksumErrors() const { return _badChecksums; }

private:
    static inline int16_t s16(const uint8_t *p) {
        return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
    }

    Stream  *_uart;
    uint8_t  _buf[19];
    uint8_t  _idx;
    uint32_t _badChecksums;
};

#endif /* MASSMORE_BNO08X_RVC_H */
