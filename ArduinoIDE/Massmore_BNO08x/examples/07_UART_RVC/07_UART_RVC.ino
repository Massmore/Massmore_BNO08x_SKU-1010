/*
  07_UART_RVC — Massmore_BNO08x_RVC (UART-RVC mode)
  ---------------------------------------------------------------------------
  โหมดที่ง่ายที่สุดของ BNO08x: strap ขา 2 ขา ต่อสาย 1 เส้น แล้วชิป stream
  yaw / pitch / roll + acceleration ที่ 100 Hz ตลอดเวลา โดย host ไม่ต้องส่งคำสั่ง
  (ไม่มี I2C, ไม่มี SHTP, ไม่ต้อง config) — เหมาะกับหุ่นยนต์ที่ต้องการแค่ heading

  WIRING — โหมดถูก latch ตอน reset
    Halley V2       ESP32 (Classic)   ESP32-S3 (MOMO)  Arduino Nano
    ---------       ---------------   ---------------  ------------
    3Vo         ->  3V3               3V3              3V3 (UART pad เป็น 3.3 V — Nano ต้องใช้ level shifter)
    GND         ->  GND               GND              GND
    SDA (TX)    ->  GPIO 21 (RX)      GPIO 14 (RX)     D2 (SoftwareSerial RX)
    SCL         ->  GPIO 22 (ไม่ใช้)   GPIO 15 (ไม่ใช้)   -   (สายเดิมจาก I2C ต่อทิ้งไว้ได้)
    RST         ->  GPIO 17           -                -   (แนะนำ: sketch reset ชิปให้ latch โหมด RVC)
    P1 (PS1)    ->  GND  (LOW)
    P0 (PS0)    ->  3Vo  (HIGH)
    BT          ->  ปล่อยลอย
  ถ้าไม่ได้ต่อ RST ให้ถอดไฟเซ็นเซอร์หลังเปลี่ยน P0/P1 ไม่อย่างนั้นชิปจะยังอยู่โหมดเดิม

  ในโหมดนี้ pad SDA คือ TX ของเซ็นเซอร์ (115200 8N1) — MCU 3.3 V ต่อตรงได้

  ได้: yaw/pitch/roll (0.01°), accel 3 แกน, 100 Hz, สายเส้นเดียว
  ไม่ได้: quaternion, calibration control, tare, report อื่น ๆ

  Designed and Manufactured by Massmore — https://www.massmore.shop
*/

#include <Massmore_BNO08x_RVC.h>

#if defined(CONFIG_IDF_TARGET_ESP32S3)       // MOMO by Massmore (ESP32-S3 + CH343P)
  #define RVC_RX_PIN  14      // ต่อกับ pad SDA (TX ของเซ็นเซอร์)
  #define RVC_TX_PIN  15      // RVC ไม่ใช้ แต่ HardwareSerial ต้องมีขา
  #define RST_PIN     -1
  HardwareSerial &rvcSerial = Serial1;
#elif defined(ARDUINO_ARCH_ESP32)
  #define RVC_RX_PIN  21      // ต่อกับ pad SDA (TX ของเซ็นเซอร์)
  #define RVC_TX_PIN  22      // RVC ไม่ใช้ แต่ HardwareSerial ต้องมีขา
  #define RST_PIN     17      // -1 = ไม่ต่อ
  HardwareSerial &rvcSerial = Serial1;
#else
  #include <SoftwareSerial.h>   // AVR core built-in (ไม่ใช่ dependency ภายนอก)
  #define RVC_RX_PIN  2
  #define RVC_TX_PIN  3
  SoftwareSerial rvcSerial(RVC_RX_PIN, RVC_TX_PIN);
  #define RST_PIN     -1
#endif

Massmore_BNO08x_RVC rvc;
uint32_t printTimer = 0;
uint8_t  lastIndex  = 0;
uint32_t dropped    = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) { }
  Serial.println(F("\nMassmore_BNO08x - 07_UART_RVC"));

  // โหมดถูก latch ตอน reset: ถ้าต่อ RST ไว้ reset ชิปหนึ่งครั้ง แล้วรอให้เริ่ม stream
  if (RST_PIN >= 0) {
    pinMode(RST_PIN, OUTPUT);
    digitalWrite(RST_PIN, LOW);
    delay(10);
    digitalWrite(RST_PIN, HIGH);
    delay(300);
  }

  // sketch เป็นเจ้าของ UART
#if defined(ARDUINO_ARCH_ESP32)
  rvcSerial.begin(115200, SERIAL_8N1, RVC_RX_PIN, RVC_TX_PIN);   // Core 3.x
#else
  rvcSerial.begin(115200);
#endif
  rvc.begin(rvcSerial);

  Serial.println(F("yaw\tpitch\troll\tax\tay\taz"));
}

void loop() {
  Massmore_BNO08x_rvc_report_t r;

  if (rvc.read(r)) {
    // index เพิ่มทีละ 1 ต่อ frame — ถ้ากระโดดแปลว่า frame หาย
    uint8_t expected = (uint8_t)(lastIndex + 1);
    if (lastIndex != 0 && r.index != expected) dropped++;
    lastIndex = r.index;

    if ((uint32_t)(millis() - printTimer) >= 100) {        // พิมพ์ 10 Hz
      printTimer = millis();
      Serial.print(r.yaw, 2);    Serial.print('\t');
      Serial.print(r.pitch, 2);  Serial.print('\t');
      Serial.print(r.roll, 2);   Serial.print('\t');
      Serial.print(r.accelX, 2); Serial.print('\t');
      Serial.print(r.accelY, 2); Serial.print('\t');
      Serial.print(r.accelZ, 2);
      if (dropped || rvc.getChecksumErrors()) {
        Serial.print(F("\t(dropped ")); Serial.print(dropped);
        Serial.print(F(", bad csum "));  Serial.print(rvc.getChecksumErrors());
        Serial.print(')');
      }
      Serial.println();
    }
  }
}
