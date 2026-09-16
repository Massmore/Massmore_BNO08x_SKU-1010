/*
  05_SPI_Advance — Massmore_BNO08x (SPI, high-rate)
  ---------------------------------------------------------------------------
  SPI เหมาะกับงานอัตราสูง: ไม่มี address phase, ไม่มี clock stretching และย้าย
  SHTP cargo ทั้งชุดได้ในการ assert CS ครั้งเดียว BNO08x ใช้ SPI Mode 3
  (CPOL = 1, CPHA = 1) สูงสุด 3 MHz

  WIRING — โหมด SPI ถูก latch ตอน reset: P0 และ P1 ต้อง HIGH ก่อนปล่อย RST
    Halley V2     ESP32 (Classic)   ESP32-S3    Arduino Nano
    ---------     ---------------   --------    ------------
    3Vo       ->  3V3               3V3         3V3   (SPI pad เป็น 3.3 V logic ล้วน — Nano ต้องใช้ level shifter)
    GND       ->  GND               GND         GND
    SCL (SCK) ->  GPIO 18           GPIO 12     D13
    SDA (MISO)->  GPIO 19           GPIO 13     D12
    DI  (MOSI)->  GPIO 23           GPIO 11     D11
    CS        ->  GPIO 5            GPIO 10     D10
    INT       ->  GPIO 4            GPIO 4      D2     (จำเป็น)
    RST       ->  GPIO 17           GPIO 5      D3     (จำเป็น)
    P0 (WAKE) ->  GPIO 16           GPIO 6      D4     (หรือต่อ 3Vo แล้วตั้ง WAKE_PIN = -1)
    P1        ->  3Vo
    BT        ->  ปล่อยลอย

  หลัง reset ขา P0 เปลี่ยนหน้าที่เป็น WAKE (active-low) — driver จัดการให้เมื่อส่ง wakePin
  หมายเหตุ: บนบอร์ด Halley V2 มีเฉพาะ SDA/SCL ที่ผ่าน level shifter; DI และ CS เป็น 3.3 V ล้วน

  Designed and Manufactured by Massmore — https://www.massmore.shop
*/

#include <SPI.h>
#include <Massmore_BNO08x.h>

// ---- pin ถูกกำหนดใน sketch เท่านั้น -------------------------------------------------
#if defined(CONFIG_IDF_TARGET_ESP32S3)
  #define SPI_SCK_PIN   12
  #define SPI_MISO_PIN  13
  #define SPI_MOSI_PIN  11
  #define CS_PIN        10
  #define INT_PIN       4
  #define RST_PIN       5
  #define WAKE_PIN      6
#elif defined(ARDUINO_ARCH_ESP32)
  #define SPI_SCK_PIN   18
  #define SPI_MISO_PIN  19
  #define SPI_MOSI_PIN  23
  #define CS_PIN        5
  #define INT_PIN       4
  #define RST_PIN       17
  #define WAKE_PIN      16
#else                       // AVR Nano: hardware SPI ขาตายตัว D13/D12/D11
  #define CS_PIN        10
  #define INT_PIN       2
  #define RST_PIN       3
  #define WAKE_PIN      4
#endif
#define SPI_SPEED_HZ    3000000UL   // ลดเป็น 1000000UL ถ้าสายยาว
// ---------------------------------------------------------------------------------

Massmore_BNO08x imu;
uint32_t printTimer = 0;
uint32_t rateTimer  = 0;
uint32_t rvCount    = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) { }
  Serial.println(F("\nMassmore_BNO08x - 05_SPI_Advance"));

  // sketch เป็นเจ้าของ SPI bus
#if defined(ARDUINO_ARCH_ESP32)
  SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, CS_PIN);   // Core 3.x
#else
  SPI.begin();
#endif

  if (!imu.beginSPI(CS_PIN, INT_PIN, RST_PIN, WAKE_PIN, SPI, SPI_SPEED_HZ)) {
    Serial.print(F("BNO08x not found on SPI: "));
    Serial.println(Massmore_BNO08x::statusToString(imu.lastError()));
    Serial.println(F("Check that P1 and P0 were both HIGH during reset."));
    while (true) delay(100);
  }

  const Massmore_BNO08x_product_id_t &id = imu.getProductID();
  Serial.print(F("Connected. Firmware "));
  Serial.print(id.swVersionMajor); Serial.print('.');
  Serial.print(id.swVersionMinor); Serial.print('.');
  Serial.print(id.swVersionPatch);
  Serial.print(F("  part ")); Serial.println(id.swPartNumber);

  // SPI รับอัตราที่ I2C รับไม่ไหวได้สบาย
  imu.enableRotationVector(MASSMORE_BNO08X_INTERVAL_400HZ);
  imu.enableAccelerometer(MASSMORE_BNO08X_INTERVAL_200HZ);
  imu.enableGyroscope(MASSMORE_BNO08X_INTERVAL_200HZ);
}

void loop() {
  if (imu.updateAll(32)) {
    if (imu.hasNewReport(MASSMORE_BNO08X_SENSOR_ROTATION_VECTOR)) rvCount++;
  }

  uint32_t now = millis();
  if ((uint32_t)(now - printTimer) >= 100) {
    printTimer = now;
    Massmore_BNO08x_euler_t e = imu.getEulerDeg();
    Massmore_BNO08x_vec3_t  g = imu.getGyroDeg();
    Serial.print(F("rpy "));
    Serial.print(e.roll, 1);  Serial.print(' ');
    Serial.print(e.pitch, 1); Serial.print(' ');
    Serial.print(e.yaw, 1);
    Serial.print(F("  gyro(dps) "));
    Serial.print(g.x, 1); Serial.print(' ');
    Serial.print(g.y, 1); Serial.print(' ');
    Serial.println(g.z, 1);
  }
  if ((uint32_t)(now - rateTimer) >= 1000) {
    rateTimer = now;
    Serial.print(F("[rate] Rotation Vector = ")); Serial.print(rvCount); Serial.println(F(" Hz"));
    rvCount = 0;
  }
}
