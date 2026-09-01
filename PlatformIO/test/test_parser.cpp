/*
 * Host-side functional test for Massmore_BNO08x.
 *
 * Feeds synthetic SHTP cargoes through a mock TwoWire and checks that the
 * driver decodes them into the values the SH-2 Reference Manual says it
 * should. Build and run on a PC; no hardware needed.
 */
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <vector>
#include <deque>
#include <string>

#include "Massmore_BNO08x.h"
#include "Massmore_BNO08x_RVC.h"

/* ------------------------------------------------------------------ */
/* Arduino shims                                                       */
/* ------------------------------------------------------------------ */
static unsigned long g_millis = 0;
unsigned long millis() { return g_millis; }
unsigned long micros() { return g_millis * 1000; }
void delay(unsigned long ms) { g_millis += ms; }
void delayMicroseconds(unsigned int us) { g_millis += (us / 1000) ? (us / 1000) : 0; g_millis++; }
void pinMode(int, int) {}
void digitalWrite(int, int) {}
int  digitalRead(int) { return 0; }          /* INT always asserted (LOW) */
int  digitalPinToInterrupt(int p) { return p; }

size_t Print::write(const uint8_t *, size_t n) { return n; }
size_t Print::print(const char *s) { return fputs(s, stdout), strlen(s); }
size_t Print::print(int) { return 0; }
size_t Print::print(unsigned) { return 0; }
size_t Print::print(float) { return 0; }
size_t Print::print(float, int) { return 0; }
size_t Print::print(double, int) { return 0; }
size_t Print::print(char) { return 0; }
size_t Print::print(unsigned long) { return 0; }
size_t Print::print(long) { return 0; }
size_t Print::print(int, int) { return 0; }
size_t Print::print(unsigned, int) { return 0; }
size_t Print::print(unsigned long, int) { return 0; }
size_t Print::print(uint8_t, int) { return 0; }
size_t Print::println(const char *s) { return printf("%s\n", s); }
size_t Print::println(int) { return 0; }
size_t Print::println(unsigned) { return 0; }
size_t Print::println(float) { return 0; }
size_t Print::println(float, int) { return 0; }
size_t Print::println(double, int) { return 0; }
size_t Print::println(char) { return 0; }
size_t Print::println(unsigned long) { return 0; }
size_t Print::println(long) { return 0; }
size_t Print::println(int, int) { return 0; }
size_t Print::println(unsigned, int) { return 0; }
size_t Print::println(unsigned long, int) { return 0; }
size_t Print::println() { return 0; }
size_t Print::printf(const char *, ...) { return 0; }

/* ------------------------------------------------------------------ */
/* Mock I2C that behaves like a BNO08x SHTP slave                      */
/* ------------------------------------------------------------------ */
struct Frame { uint8_t channel; std::vector<uint8_t> payload; };

static std::deque<Frame>   g_queue;
static size_t              g_offset = 0;      /* payload bytes already served */
static std::vector<uint8_t> g_readBuf;
static size_t              g_readPos = 0;
static uint8_t             g_seq[6] = {0,0,0,0,0,0};
static std::vector<uint8_t> g_lastWrite;

void queueFrame(uint8_t channel, std::vector<uint8_t> payload) {
    g_queue.push_back({channel, std::move(payload)});
}

TwoWire Wire;
SPIClass SPI;
HardwareSerial Serial;
HardwareSerial Serial1;

bool TwoWire::begin() { return true; }
bool TwoWire::begin(int, int) { return true; }
bool TwoWire::setBufferSize(size_t) { return true; }
void TwoWire::setTimeOut(uint16_t) {}
void TwoWire::setClock(uint32_t) {}
void TwoWire::beginTransmission(uint8_t) { g_lastWrite.clear(); }
static uint8_t g_resetCause = 1;   /* what the mock reports next */

/* Real hardware answers a Product ID Request with a Product ID Response, so
 * the mock does too. Everything else the tests queue explicitly. */
uint8_t TwoWire::endTransmission() {
    if (g_lastWrite.size() >= 6 &&
        g_lastWrite[2] == MASSMORE_CH_CONTROL &&
        g_lastWrite[4] == MASSMORE_REPORT_PRODUCT_ID_REQ) {
        std::vector<uint8_t> pid = { 0xF8, g_resetCause, 3, 2 };
        uint32_t part = 10003606UL, build = 370UL; uint16_t patch = 17;
        pid.push_back((uint8_t)part);       pid.push_back((uint8_t)(part >> 8));
        pid.push_back((uint8_t)(part>>16)); pid.push_back((uint8_t)(part >> 24));
        pid.push_back((uint8_t)build);      pid.push_back((uint8_t)(build >> 8));
        pid.push_back((uint8_t)(build>>16));pid.push_back((uint8_t)(build >> 24));
        pid.push_back((uint8_t)patch);      pid.push_back((uint8_t)(patch >> 8));
        pid.push_back(0); pid.push_back(0);
        g_queue.push_back({ MASSMORE_CH_CONTROL, pid });
    }
    return 0;
}
size_t TwoWire::write(uint8_t b) { g_lastWrite.push_back(b); return 1; }
size_t TwoWire::write(const uint8_t *p, size_t n) {
    g_lastWrite.insert(g_lastWrite.end(), p, p + n);
    return n;
}
int TwoWire::available() { return (int)(g_readBuf.size() - g_readPos); }
int TwoWire::read() {
    if (g_readPos >= g_readBuf.size()) return -1;
    return g_readBuf[g_readPos++];
}
int TwoWire::peek() { return g_readPos < g_readBuf.size() ? g_readBuf[g_readPos] : -1; }

size_t TwoWire::requestFrom(uint8_t, uint8_t quantity) {
    g_readBuf.clear();
    g_readPos = 0;

    if (g_queue.empty()) {                    /* null header: no cargo */
        g_readBuf.assign(quantity, 0);
        return quantity;
    }

    Frame &f = g_queue.front();
    size_t remainingPayload = f.payload.size() - g_offset;
    uint16_t hdrLen = (uint16_t)(remainingPayload + 4);
    if (g_offset > 0) hdrLen |= 0x8000;        /* continuation bit */

    g_readBuf.push_back((uint8_t)(hdrLen & 0xFF));
    g_readBuf.push_back((uint8_t)(hdrLen >> 8));
    g_readBuf.push_back(f.channel);
    g_readBuf.push_back(g_seq[f.channel]);

    if (quantity > 4) {
        size_t want = quantity - 4;
        if (want > remainingPayload) want = remainingPayload;
        for (size_t i = 0; i < want; i++) g_readBuf.push_back(f.payload[g_offset + i]);
        while (g_readBuf.size() < quantity) g_readBuf.push_back(0);  /* zero pad */
        g_offset += want;
        if (g_offset >= f.payload.size()) {
            g_seq[f.channel]++;
            g_queue.pop_front();
            g_offset = 0;
        }
    }
    return quantity;
}

void SPIClass::begin() {}
void SPIClass::beginTransaction(SPISettings) {}
void SPIClass::endTransaction() {}
uint8_t SPIClass::transfer(uint8_t) { return 0; }
void HardwareSerial::begin(unsigned long) {}
void HardwareSerial::begin(unsigned long, uint32_t, int, int) {}
int HardwareSerial::available() { return 0; }
int HardwareSerial::read() { return -1; }
int HardwareSerial::peek() { return -1; }

/* ------------------------------------------------------------------ */
/* Test scaffolding                                                    */
/* ------------------------------------------------------------------ */
static int g_pass = 0, g_fail = 0;

static void check(const char *what, bool ok, const std::string &detail = "") {
    if (ok) { printf("  PASS  %s\n", what); g_pass++; }
    else    { printf("  FAIL  %s  %s\n", what, detail.c_str()); g_fail++; }
}
static void checkNear(const char *what, float got, float want, float tol) {
    bool ok = fabsf(got - want) <= tol;
    char buf[128];
    snprintf(buf, sizeof(buf), "got %.6f want %.6f", got, want);
    check(what, ok, ok ? "" : buf);
}

static void push16(std::vector<uint8_t> &v, int16_t x) {
    v.push_back((uint8_t)(x & 0xFF));
    v.push_back((uint8_t)((uint16_t)x >> 8));
}
static void push32(std::vector<uint8_t> &v, uint32_t x) {
    v.push_back((uint8_t)(x));       v.push_back((uint8_t)(x >> 8));
    v.push_back((uint8_t)(x >> 16)); v.push_back((uint8_t)(x >> 24));
}

/* A 4 byte report prefix: id, sequence, status (accuracy in bits 1:0), delay */
static void pushPrefix(std::vector<uint8_t> &v, uint8_t id, uint8_t seq,
                       uint8_t accuracy, uint16_t delay100us) {
    v.push_back(id);
    v.push_back(seq);
    v.push_back((uint8_t)((accuracy & 0x03) | ((delay100us >> 8) << 2)));
    v.push_back((uint8_t)(delay100us & 0xFF));
}

/* ------------------------------------------------------------------ */
int main() {
    MassmoreBNO08x imu;

    printf("\n=== Massmore BNO08x parser tests ===\n\n");

    /* --- 1. start-up: advertisement then product ID -------------------- */
    printf("[1] Start-up, advertisement and Product ID\n");
    {
        /* SHTP advertisement carrying the SH-2 report-length TLV (tag 0x81) */
        std::vector<uint8_t> adv = { 0x00 };
        adv.push_back(0x01); adv.push_back(0x10);          /* TAG_GUID, len 16 */
        for (int i = 0; i < 16; i++) adv.push_back(0);
        const uint8_t lens[] = { 0x01,10, 0x02,10, 0x03,10, 0x05,14,
                                 0x08,12, 0x11,12, 0x14,16, 0x1E,16, 0x10,5 };
        adv.push_back(0x81);
        adv.push_back(sizeof(lens));
        adv.insert(adv.end(), lens, lens + sizeof(lens));
        adv.push_back(0x00); adv.push_back(0x00);           /* end of adverts */
        queueFrame(MASSMORE_CH_COMMAND, adv);

        /* the mock answers the Product ID Request the way the part does:
           v3.2.17, part 10003606, build 370, reset cause 1 (power on) */
        g_resetCause = 1;

        bool ok = imu.begin(0x4B, Wire, 4, 5);
        check("begin() succeeds", ok);

        const massmore_product_id_t &id = imu.getProductID();
        check("product ID decoded", id.valid);
        check("SW version 3.2.17",
              id.swVersionMajor == 3 && id.swVersionMinor == 2 && id.swVersionPatch == 17);
        check("part number 10003606", id.swPartNumber == 10003606UL);
        check("build number 370",     id.swBuildNumber == 370UL);
        check("reset cause = power on reset", id.resetCause == 1);
        check("reset reason string",
              std::string(imu.getResetReasonString()) == "Power on reset");
        check("verifyChip() == OK", imu.verifyChip() == MASSMORE_AUTH_OK);
    }

    /* --- 2. rotation vector, Q14 / Q12 --------------------------------- */
    printf("\n[2] Rotation vector decode (Q14 quaternion, Q12 accuracy)\n");
    {
        /* identity-ish: w = 1.0 -> 16384 in Q14 */
        std::vector<uint8_t> p;
        pushPrefix(p, MASSMORE_SENSOR_ROTATION_VECTOR, 7, MASSMORE_ACCURACY_HIGH, 0);
        push16(p, 0);        /* i */
        push16(p, 0);        /* j */
        push16(p, 0);        /* k */
        push16(p, 16384);    /* real = 1.0 */
        push16(p, 410);      /* accuracy = 410/4096 = 0.1001 rad */
        queueFrame(MASSMORE_CH_INPUT_REPORT, p);

        check("update() returns a packet", imu.update());
        check("hasNewReport(RV)", imu.hasNewReport(MASSMORE_SENSOR_ROTATION_VECTOR));
        check("flag is consumed once",
              !imu.peekNewReport(MASSMORE_SENSOR_ROTATION_VECTOR));

        checkNear("quat.real == 1.0", imu.getQuatReal(), 1.0f, 1e-4f);
        checkNear("quat.i == 0",      imu.getQuatI(),    0.0f, 1e-6f);
        checkNear("accuracy 0.1001 rad", imu.getQuatAccuracy(), 410.0f/4096.0f, 1e-5f);
        check("accuracy field = High",
              imu.getAccuracy(MASSMORE_SENSOR_ROTATION_VECTOR) == MASSMORE_ACCURACY_HIGH);

        massmore_euler_t e = imu.getEulerDeg();
        checkNear("identity -> roll 0",  e.roll,  0.0f, 1e-3f);
        checkNear("identity -> pitch 0", e.pitch, 0.0f, 1e-3f);
        checkNear("identity -> yaw 0",   e.yaw,   0.0f, 1e-3f);
        checkNear("heading 0..360",      imu.getHeadingDeg(), 0.0f, 1e-3f);
    }

    /* --- 3. a real rotation: 90 deg yaw -------------------------------- */
    printf("\n[3] 90 degree yaw quaternion\n");
    {
        /* w = cos(45) = 0.7071 -> 11585, k = sin(45) = 0.7071 -> 11585 */
        std::vector<uint8_t> p;
        pushPrefix(p, MASSMORE_SENSOR_ROTATION_VECTOR, 8, MASSMORE_ACCURACY_HIGH, 0);
        push16(p, 0); push16(p, 0); push16(p, 11585); push16(p, 11585);
        push16(p, 0);
        queueFrame(MASSMORE_CH_INPUT_REPORT, p);
        imu.update();

        massmore_euler_t e = imu.getEulerDeg();
        checkNear("yaw == 90 deg", e.yaw, 90.0f, 0.1f);
        checkNear("roll == 0",     e.roll, 0.0f, 0.1f);
        checkNear("pitch == 0",    e.pitch, 0.0f, 0.1f);
        checkNear("heading == 90", imu.getHeadingDeg(), 90.0f, 0.1f);
    }

    /* --- 4. negative yaw wraps into 0..360 heading --------------------- */
    printf("\n[4] Negative yaw -> compass heading\n");
    {
        /* -90 deg yaw: w = 0.7071, k = -0.7071 */
        std::vector<uint8_t> p;
        pushPrefix(p, MASSMORE_SENSOR_ROTATION_VECTOR, 9, MASSMORE_ACCURACY_HIGH, 0);
        push16(p, 0); push16(p, 0); push16(p, -11585); push16(p, 11585);
        push16(p, 0);
        queueFrame(MASSMORE_CH_INPUT_REPORT, p);
        imu.update();

        checkNear("yaw == -90 deg",   imu.getYawDeg(), -90.0f, 0.1f);
        checkNear("heading == 270",   imu.getHeadingDeg(), 270.0f, 0.1f);
    }

    /* --- 5. accel Q8, gyro Q9, mag Q4 in one batched cargo ------------- */
    printf("\n[5] Batched cargo: base timestamp + accel + gyro + mag\n");
    {
        std::vector<uint8_t> p;
        /* base timestamp report: 120 ticks of 100 us = 12 ms */
        p.push_back(MASSMORE_REPORT_BASE_TIMESTAMP);
        push32(p, 120);

        /* accelerometer: 9.81 m/s^2 on Z -> 9.81 * 256 = 2511 */
        pushPrefix(p, MASSMORE_SENSOR_ACCELEROMETER, 1, MASSMORE_ACCURACY_MEDIUM, 0);
        push16(p, 256);    /* 1.0 m/s^2 */
        push16(p, -512);   /* -2.0     */
        push16(p, 2511);   /* 9.809    */

        /* gyroscope: Q9. 1.0 rad/s -> 512 */
        pushPrefix(p, MASSMORE_SENSOR_GYROSCOPE, 2, MASSMORE_ACCURACY_HIGH, 17);
        push16(p, 512);    /* 1.0 rad/s */
        push16(p, -256);   /* -0.5      */
        push16(p, 0);

        /* magnetometer: Q4. 25.0 uT -> 400 */
        pushPrefix(p, MASSMORE_SENSOR_MAGNETIC_FIELD, 3, MASSMORE_ACCURACY_LOW, 0);
        push16(p, 400);    /* 25.0 uT */
        push16(p, -160);   /* -10.0   */
        push16(p, 800);    /* 50.0    */

        queueFrame(MASSMORE_CH_INPUT_REPORT, p);
        imu.update();

        massmore_vec3_t a = imu.getAccel();
        checkNear("accel.x 1.0 m/s2",  a.x,  1.0f,   1e-3f);
        checkNear("accel.y -2.0 m/s2", a.y, -2.0f,   1e-3f);
        checkNear("accel.z 9.809",     a.z,  9.8086f, 1e-3f);

        massmore_vec3_t g = imu.getGyro();
        checkNear("gyro.x 1.0 rad/s",  g.x,  1.0f, 1e-4f);
        checkNear("gyro.y -0.5 rad/s", g.y, -0.5f, 1e-4f);

        massmore_vec3_t gd = imu.getGyroDeg();
        checkNear("gyro.x 57.30 deg/s", gd.x, 57.2958f, 1e-2f);

        massmore_vec3_t m = imu.getMag();
        checkNear("mag.x 25.0 uT",  m.x,  25.0f, 1e-3f);
        checkNear("mag.y -10.0 uT", m.y, -10.0f, 1e-3f);
        checkNear("mag.z 50.0 uT",  m.z,  50.0f, 1e-3f);

        check("all three reports flagged new",
              imu.hasNewReport(MASSMORE_SENSOR_ACCELEROMETER) &&
              imu.hasNewReport(MASSMORE_SENSOR_GYROSCOPE) &&
              imu.hasNewReport(MASSMORE_SENSOR_MAGNETIC_FIELD));

        check("per-report accuracy tracked separately",
              imu.getAccuracy(MASSMORE_SENSOR_ACCELEROMETER) == MASSMORE_ACCURACY_MEDIUM &&
              imu.getAccuracy(MASSMORE_SENSOR_GYROSCOPE)     == MASSMORE_ACCURACY_HIGH &&
              imu.getAccuracy(MASSMORE_SENSOR_MAGNETIC_FIELD)== MASSMORE_ACCURACY_LOW);

        /* mag was the last report parsed: 120 base + 0 delay = 12000 us */
        check("timestamp = (base + delay) * 100us",
              imu.getTimestampUs() == 12000ULL,
              "got " + std::to_string((unsigned long long)imu.getTimestampUs()));
    }

    /* --- 6. step counter ---------------------------------------------- */
    printf("\n[6] Step counter\n");
    {
        std::vector<uint8_t> p;
        pushPrefix(p, MASSMORE_SENSOR_STEP_COUNTER, 4, 3, 0);
        push32(p, 0);        /* latency */
        push32(p, 12345);    /* steps   */
        queueFrame(MASSMORE_CH_INPUT_REPORT, p);
        imu.update();
        check("step count 12345", imu.getStepCount() == 12345UL,
              "got " + std::to_string(imu.getStepCount()));
    }

    /* --- 7. tap detector latch ---------------------------------------- */
    printf("\n[7] Tap detector flags latch then clear\n");
    {
        std::vector<uint8_t> p;
        pushPrefix(p, MASSMORE_SENSOR_TAP_DETECTOR, 5, 3, 0);
        p.push_back(MASSMORE_TAP_Z_POS | MASSMORE_TAP_DOUBLE);
        queueFrame(MASSMORE_CH_INPUT_REPORT, p);
        imu.update();

        uint8_t t = imu.getTapDetector();
        check("double tap on +Z", (t & MASSMORE_TAP_Z_POS) && (t & MASSMORE_TAP_DOUBLE));
        check("flags cleared after read", imu.getTapDetector() == 0);
    }

    /* --- 8. activity classifier --------------------------------------- */
    printf("\n[8] Personal activity classifier\n");
    {
        std::vector<uint8_t> p;
        pushPrefix(p, MASSMORE_SENSOR_ACTIVITY_CLASSIFIER, 6, 3, 0);
        p.push_back(0x80);                       /* page 0, last page */
        p.push_back(MASSMORE_ACTIVITY_WALKING);  /* most likely state */
        uint8_t conf[10] = { 0, 0, 0, 5, 10, 0, 82, 3, 0, 0 };
        for (int i = 0; i < 10; i++) p.push_back(conf[i]);
        queueFrame(MASSMORE_CH_INPUT_REPORT, p);
        imu.update();

        check("activity = walking", imu.getActivity() == MASSMORE_ACTIVITY_WALKING);
        check("activity string", std::string(imu.getActivityString()) == "Walking");
        check("walking confidence 82",
              imu.getActivityConfidence(MASSMORE_ACTIVITY_WALKING) == 82);
        check("still confidence 10",
              imu.getActivityConfidence(MASSMORE_ACTIVITY_STILL) == 10);
    }

    /* --- 9. gyro-integrated RV on channel 5 --------------------------- */
    printf("\n[9] Gyro-integrated rotation vector on SHTP channel 5\n");
    {
        std::vector<uint8_t> p;              /* no report prefix on this channel */
        push16(p, 0); push16(p, 0); push16(p, 11585); push16(p, 11585);
        push16(p, 1024);   /* angVelX = 1024/1024 = 1.0 rad/s (Q10) */
        push16(p, -512);   /* -0.5 rad/s */
        push16(p, 0);
        queueFrame(MASSMORE_CH_GYRO_RV, p);
        imu.update();

        check("flagged as gyro-integrated RV",
              imu.hasNewReport(MASSMORE_SENSOR_GYRO_INTEGRATED_RV));
        checkNear("yaw 90 deg from channel 5", imu.getYawDeg(), 90.0f, 0.1f);
        massmore_vec3_t w = imu.getAngularVelocity();
        checkNear("angVel.x 1.0 rad/s (Q10)",  w.x,  1.0f, 1e-4f);
        checkNear("angVel.y -0.5 rad/s (Q10)", w.y, -0.5f, 1e-4f);
    }

    /* --- 10. raw sensors ---------------------------------------------- */
    printf("\n[10] Raw sensor reports\n");
    {
        std::vector<uint8_t> p;
        pushPrefix(p, MASSMORE_SENSOR_RAW_ACCELEROMETER, 10, 3, 0);
        push16(p, 100); push16(p, -200); push16(p, 300);
        push16(p, 0);                 /* reserved */
        push32(p, 987654321UL);       /* timestamp */
        queueFrame(MASSMORE_CH_INPUT_REPORT, p);
        imu.update();

        massmore_vec3i_t r = imu.getRawAccel();
        check("raw accel counts", r.x == 100 && r.y == -200 && r.z == 300);
    }

    /* --- 11. Set Feature encoding ------------------------------------- */
    printf("\n[11] Set Feature command encoding\n");
    {
        g_lastWrite.clear();
        imu.setFeature(MASSMORE_SENSOR_ROTATION_VECTOR,
                       10000,                                  /* 100 Hz     */
                       500000,                                 /* batch      */
                       MASSMORE_FEATURE_FLAG_WAKE_ENABLED,
                       0x1234,                                 /* sensitivity*/
                       0xDEADBEEF);                            /* config word*/

        check("packet is 4 header + 17 payload bytes", g_lastWrite.size() == 21,
              "got " + std::to_string(g_lastWrite.size()));
        check("SHTP length field = 21",
              g_lastWrite[0] == 21 && g_lastWrite[1] == 0);
        check("sent on control channel 2", g_lastWrite[2] == MASSMORE_CH_CONTROL);
        check("report ID 0xFD", g_lastWrite[4] == MASSMORE_REPORT_SET_FEATURE_CMD);
        check("feature = rotation vector",
              g_lastWrite[5] == MASSMORE_SENSOR_ROTATION_VECTOR);
        check("wake flag set",
              g_lastWrite[6] == MASSMORE_FEATURE_FLAG_WAKE_ENABLED);
        check("change sensitivity little endian",
              g_lastWrite[7] == 0x34 && g_lastWrite[8] == 0x12);
        check("report interval 10000 us little endian",
              g_lastWrite[9]  == 0x10 && g_lastWrite[10] == 0x27 &&
              g_lastWrite[11] == 0x00 && g_lastWrite[12] == 0x00);
        check("batch interval 500000 us little endian",
              g_lastWrite[13] == 0x20 && g_lastWrite[14] == 0xA1 &&
              g_lastWrite[15] == 0x07 && g_lastWrite[16] == 0x00);
        check("sensor specific word little endian",
              g_lastWrite[17] == 0xEF && g_lastWrite[18] == 0xBE &&
              g_lastWrite[19] == 0xAD && g_lastWrite[20] == 0xDE);
        check("interval cached locally",
              imu.getReportInterval(MASSMORE_SENSOR_ROTATION_VECTOR) == 10000UL);
    }

    /* --- 12. Tare command encoding ------------------------------------ */
    printf("\n[12] Tare command encoding (CEVA 1000-4045)\n");
    {
        g_lastWrite.clear();
        imu.tareNow(MASSMORE_TARE_AXIS_ALL, MASSMORE_TARE_BASIS_ROTATION_VECTOR);
        check("12 byte command payload", g_lastWrite.size() == 16,
              "got " + std::to_string(g_lastWrite.size()));
        check("report ID 0xF2", g_lastWrite[4] == MASSMORE_REPORT_COMMAND_REQUEST);
        check("command 3 = tare",       g_lastWrite[6] == MASSMORE_CMD_TARE);
        check("P0 subcommand 0 = tare now", g_lastWrite[7] == 0x00);
        check("P1 axes 0x07 = all three",   g_lastWrite[8] == 0x07);
        check("P2 basis 0 = rotation vector", g_lastWrite[9] == 0x00);

        g_lastWrite.clear();
        imu.tareNow(MASSMORE_TARE_AXIS_Z, MASSMORE_TARE_BASIS_GAMING_RV);
        check("Z-only tare: P1 = 0x04", g_lastWrite[8] == 0x04);
        check("game RV basis: P2 = 1",  g_lastWrite[9] == 0x01);

        g_lastWrite.clear();
        imu.persistTare();
        check("persist tare: P0 = 1", g_lastWrite[7] == 0x01);
    }

    /* --- 13. ME calibration command encoding -------------------------- */
    printf("\n[13] ME calibration command encoding (CEVA 1000-4044)\n");
    {
        g_lastWrite.clear();
        imu.calibrateAll();
        check("command 7 = ME calibrate", g_lastWrite[6] == MASSMORE_CMD_ME_CALIBRATE);
        check("P0 accel enable",  g_lastWrite[7]  == 1);
        check("P1 gyro enable",   g_lastWrite[8]  == 1);
        check("P2 mag enable",    g_lastWrite[9]  == 1);
        check("P4 planar off",    g_lastWrite[11] == 0);

        g_lastWrite.clear();
        imu.calibrateMagnetometer();
        check("mag only: P0/P1 clear, P2 set",
              g_lastWrite[7] == 0 && g_lastWrite[8] == 0 && g_lastWrite[9] == 1);

        g_lastWrite.clear();
        imu.endCalibration();
        check("stop: all enables clear",
              g_lastWrite[7] == 0 && g_lastWrite[8] == 0 && g_lastWrite[9] == 0);

        g_lastWrite.clear();
        imu.saveCalibration();
        check("command 6 = save DCD", g_lastWrite[6] == MASSMORE_CMD_SAVE_DCD);

        /* command response says success */
        std::vector<uint8_t> resp = { 0xF1, 0, MASSMORE_CMD_ME_CALIBRATE, 0, 0, 0,
                                      0,0,0,0,0,0,0,0,0,0 };
        queueFrame(MASSMORE_CH_CONTROL, resp);
        imu.update();
        check("calibrationComplete() after status 0", imu.calibrationComplete());
    }

    /* --- 14. Product ID request encoding ------------------------------ */
    printf("\n[14] Product ID request encoding\n");
    {
        g_lastWrite.clear();
        g_resetCause = 4;                       /* external reset (NRST) */
        massmore_status_t rc = imu.requestProductID(200);
        check("requestProductID OK", rc == MASSMORE_OK,
              MassmoreBNO08x::statusToString(rc));
        check("external reset cause decoded",
              std::string(imu.getResetReasonString()) == "External reset (NRST)");
    }

    /* --- 15. FRS read: serial number ---------------------------------- */
    printf("\n[15] FRS read (serial number record 0x4B4B)\n");
    {
        /* One response carrying two words then "read record completed" (3). */
        std::vector<uint8_t> r;
        r.push_back(MASSMORE_REPORT_FRS_READ_RESPONSE);
        r.push_back((uint8_t)((2 << 4) | 3));    /* dataLength 2, status 3 */
        push16(r, 0);                            /* offset 0 */
        push32(r, 0x11223344UL);
        push32(r, 0x55667788UL);
        push16(r, MASSMORE_FRS_SERIAL_NUMBER);
        queueFrame(MASSMORE_CH_CONTROL, r);

        uint64_t serial = 0;
        massmore_status_t rc = imu.readSerialNumber(serial, 300);
        check("readSerialNumber OK", rc == MASSMORE_OK,
              MassmoreBNO08x::statusToString(rc));
        check("serial = 0x5566778811223344",
              serial == 0x5566778811223344ULL);
    }

    /* --- 16. FRS read: empty record is reported as an error ------------ */
    printf("\n[16] FRS read of an empty record\n");
    {
        std::vector<uint8_t> r;
        r.push_back(MASSMORE_REPORT_FRS_READ_RESPONSE);
        r.push_back((uint8_t)((0 << 4) | 5));    /* dataLength 0, status 5 = empty */
        push16(r, 0);
        push32(r, 0); push32(r, 0);
        push16(r, MASSMORE_FRS_USER_RECORD);
        queueFrame(MASSMORE_CH_CONTROL, r);

        uint32_t buf[4]; uint16_t words = 0;
        massmore_status_t rc = imu.readFrsRecord(MASSMORE_FRS_USER_RECORD, buf, 4, words, 300);
        check("empty record reported, not silently OK",
              rc == MASSMORE_ERR_BAD_RESPONSE, MassmoreBNO08x::statusToString(rc));
    }

    /* --- 17. executable channel: reset complete ----------------------- */
    printf("\n[17] Executable channel and unknown report resync\n");
    {
        queueFrame(MASSMORE_CH_EXECUTABLE, { MASSMORE_EXEC_RESET_COMPLETE });
        check("reset complete packet accepted", imu.update());

        g_lastWrite.clear();
        imu.modeSleep();
        check("sleep goes to channel 1", g_lastWrite[2] == MASSMORE_CH_EXECUTABLE);
        check("sleep payload = 3", g_lastWrite[4] == MASSMORE_EXEC_SLEEP);

        g_lastWrite.clear();
        imu.modeOn();
        check("on payload = 2", g_lastWrite[4] == MASSMORE_EXEC_ON);

        /* An unrecognised report ID must not hang or run off the buffer. */
        std::vector<uint8_t> bad;
        pushPrefix(bad, 0x3F, 0, 0, 0);       /* no such report */
        bad.push_back(0xAA); bad.push_back(0xBB);
        queueFrame(MASSMORE_CH_INPUT_REPORT, bad);
        imu.update();
        check("unknown report ID does not crash or flag", true);
    }

    /* --- 18. large cargo spanning many I2C chunks --------------------- */
    printf("\n[18] Cargo larger than one I2C transaction\n");
    {
        /* 20 accelerometer reports back to back = 200 payload bytes, which
           forces the chunked read path on a 128 byte Wire buffer. */
        std::vector<uint8_t> p;
        p.push_back(MASSMORE_REPORT_BASE_TIMESTAMP);
        push32(p, 0);
        for (int n = 0; n < 20; n++) {
            pushPrefix(p, MASSMORE_SENSOR_ACCELEROMETER, (uint8_t)n, 3, 0);
            push16(p, (int16_t)(256 * (n + 1)));    /* x = n+1 m/s^2 */
            push16(p, 0);
            push16(p, 0);
        }
        queueFrame(MASSMORE_CH_INPUT_REPORT, p);
        check("multi-chunk cargo received", imu.update());
        checkNear("last accel sample x = 20.0", imu.getAccel().x, 20.0f, 1e-3f);
    }

    /* --- 19. UART-RVC decoder ----------------------------------------- */
    printf("\n[19] UART-RVC frame decode (datasheet worked example)\n");
    {
        /* The example from the datasheet, Figure 1-25 commentary:
           index 222, yaw 0.01, pitch -1.10, roll 20.85,
           ax -371 mg, ay -20 mg, az 977 mg, checksum 0xE7 */
        const uint8_t frame[19] = {
            0xAA, 0xAA, 0xDE, 0x01, 0x00, 0x92, 0xFF, 0x25, 0x08,
            0x8D, 0xFE, 0xEC, 0xFF, 0xD1, 0x03, 0x00, 0x00, 0x00, 0xE7
        };
        uint8_t sum = 0;
        for (int i = 2; i < 18; i++) sum = (uint8_t)(sum + frame[i]);
        check("datasheet checksum 0xE7 matches our algorithm", sum == 0xE7,
              "computed 0x" + std::to_string((int)sum));

        int16_t yaw   = (int16_t)((uint16_t)frame[3]  | ((uint16_t)frame[4]  << 8));
        int16_t pitch = (int16_t)((uint16_t)frame[5]  | ((uint16_t)frame[6]  << 8));
        int16_t roll  = (int16_t)((uint16_t)frame[7]  | ((uint16_t)frame[8]  << 8));
        int16_t ax    = (int16_t)((uint16_t)frame[9]  | ((uint16_t)frame[10] << 8));
        checkNear("yaw 0.01 deg",   yaw   * 0.01f,  0.01f,  1e-4f);
        checkNear("pitch -1.10 deg",pitch * 0.01f, -1.10f,  1e-4f);
        checkNear("roll 20.85 deg", roll  * 0.01f,  20.85f, 1e-4f);
        checkNear("ax -3.638 m/s2", ax * 0.0098067f, -3.638f, 2e-3f);
    }

    /* --- 20. quaternion to Euler edge case: gimbal lock ---------------- */
    printf("\n[20] Euler conversion at the pitch singularity\n");
    {
        massmore_quat_t q;
        /* pitch = +90 deg: w = cos(45), j = sin(45) */
        q.i = 0.0f; q.j = 0.7071068f; q.k = 0.0f; q.real = 0.7071068f;
        q.accuracy = 0.0f;
        massmore_euler_t e = MassmoreBNO08x::quaternionToEuler(q);
        checkNear("pitch = 90 deg", e.pitch * 57.2957795f, 90.0f, 0.01f);
        check("no NaN at the singularity",
              !std::isnan(e.roll) && !std::isnan(e.pitch) && !std::isnan(e.yaw));

        /* deliberately denormalised, |sin| slightly > 1 */
        q.i = 0.0f; q.j = 0.72f; q.k = 0.0f; q.real = 0.72f;
        e = MassmoreBNO08x::quaternionToEuler(q);
        check("clamped asin, still no NaN",
              !std::isnan(e.pitch) && fabsf(e.pitch) <= 1.5708f + 1e-4f);
    }

    /* ------------------------------------------------------------------ */
    printf("\n=====================================\n");
    printf("  %d passed, %d failed\n", g_pass, g_fail);
    printf("=====================================\n\n");
    return g_fail == 0 ? 0 : 1;
}
