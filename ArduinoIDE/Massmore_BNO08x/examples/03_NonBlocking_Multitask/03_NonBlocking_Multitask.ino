/*
  03_NonBlocking_Multitask — Massmore_BNO08x (Advanced Non-blocking FSM API)
  ---------------------------------------------------------------------------
  แสดงว่า loop() ไม่ถูก block: เปิด Rotation Vector 100 Hz + Accelerometer 100 Hz
  แล้วใช้ update() / isDataReady() / getReadings() ควบคู่กับงานอื่น (LED กะพริบ
  ทุก 500 ms และตัวนับรอบ loop ต่อวินาที)

  ลำดับ FSM:  enable*()  →  update()  →  isDataReady()  →  getReadings()
  ทุกตัวจับเวลาด้วย millis() แบบ rollover-safe: (uint32_t)(millis() - t0) >= period

  Wiring (I2C): เหมือน 01_BasicRead — ต่อ INT เพิ่มจะได้ประสิทธิภาพสูงสุด
    ESP32: SDA 21 / SCL 22 / INT 4      ESP32-S3: SDA 8 / SCL 9 / INT 4
    Nano : SDA A4 / SCL A5 / INT D2

  Designed and Manufactured by Massmore — https://www.massmore.shop
*/

#include <Wire.h>
#include <Massmore_BNO08x.h>

#if defined(CONFIG_IDF_TARGET_ESP32S3)
  #define I2C_SDA_PIN  8
  #define I2C_SCL_PIN  9
  #define INT_PIN      4
#elif defined(ARDUINO_ARCH_ESP32)
  #define I2C_SDA_PIN  21
  #define I2C_SCL_PIN  22
  #define INT_PIN      4
#else
  #define INT_PIN      2
#endif
#define I2C_ADDRESS   MASSMORE_BNO08X_I2C_ADDR_DEF

#ifndef LED_BUILTIN
  #define LED_BUILTIN 2
#endif

Massmore_BNO08x imu;

uint32_t ledTimer   = 0;
uint32_t printTimer = 0;
uint32_t statTimer  = 0;
uint32_t loopCount  = 0;
uint32_t rvCount    = 0;
bool     ledState   = false;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) { }
  Serial.println(F("\nMassmore_BNO08x - 03_NonBlocking_Multitask"));

  pinMode(LED_BUILTIN, OUTPUT);

#if defined(ARDUINO_ARCH_ESP32)
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
#else
  Wire.begin();
#endif
  Wire.setClock(100000);

  if (!imu.begin(I2C_ADDRESS, Wire, INT_PIN)) {
    Serial.print(F("BNO08x not found: "));
    Serial.println(Massmore_BNO08x::statusToString(imu.lastError()));
    while (true) delay(100);
  }

  // Step 1 ของ FSM: เปิด report ที่ต้องการ (interval เป็น microseconds)
  imu.enableRotationVector(MASSMORE_BNO08X_INTERVAL_100HZ);
  imu.enableAccelerometer(MASSMORE_BNO08X_INTERVAL_100HZ);
  Serial.println(F("Rotation Vector + Accelerometer @ 100 Hz. loop() never blocks."));
}

void loop() {
  loopCount++;
  uint32_t now = millis();

  // ---- Task A: IMU (Non-blocking) ----------------------------------------
  imu.update();                                             // Step 2: ดึง packet ถ้ามี (ไม่รอ)
  if (imu.isDataReady(MASSMORE_BNO08X_SENSOR_ROTATION_VECTOR)) {   // Step 3
    imu.hasNewReport(MASSMORE_BNO08X_SENSOR_ROTATION_VECTOR);      // ล้าง flag
    rvCount++;
  }

  // ---- Task B: LED กะพริบทุก 500 ms (ไม่ใช้ delay) ------------------------
  if ((uint32_t)(now - ledTimer) >= 500) {
    ledTimer = now;
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState ? HIGH : LOW);
  }

  // ---- Task C: พิมพ์ค่าล่าสุด 10 Hz --------------------------------------
  if ((uint32_t)(now - printTimer) >= 100) {
    printTimer = now;
    Massmore_BNO08x_reading_t r;
    imu.getReadings(r);                                     // Step 4: copy ค่า cache (ไม่แตะ bus)
    Serial.print(F("yaw="));   Serial.print(r.eulerDeg.yaw, 1);
    Serial.print(F(" pitch=")); Serial.print(r.eulerDeg.pitch, 1);
    Serial.print(F(" roll="));  Serial.print(r.eulerDeg.roll, 1);
    Serial.print(F(" | az="));  Serial.println(r.accel.z, 2);
  }

  // ---- Task D: สถิติทุก 1 s — loop/s สูงแปลว่าไม่มีอะไร block -------------
  if ((uint32_t)(now - statTimer) >= 1000) {
    statTimer = now;
    Serial.print(F("[stats] loop/s=")); Serial.print(loopCount);
    Serial.print(F("  RV reports/s=")); Serial.println(rvCount);
    loopCount = 0;
    rvCount   = 0;
  }
}
