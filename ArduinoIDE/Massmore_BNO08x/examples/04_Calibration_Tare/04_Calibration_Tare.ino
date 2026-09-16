/*
  04_Calibration_Tare — Massmore_BNO08x
  ---------------------------------------------------------------------------
  Calibration ตามขั้นตอนของ CEVA (doc 1000-4044) และ Tare (doc 1000-4045)
  ควบคุมผ่าน Serial Monitor (115200, ส่งทีละตัวอักษร)

  CALIBRATION
    c  เปิด dynamic calibration ของ accel + gyro + mag
    m  เปิด calibration เฉพาะ magnetometer
    e  ปิด dynamic calibration
    s  Save DCD (เขียน calibration ลง flash — อยู่ข้าม power cycle)
    x  ลบ calibration ที่เก็บไว้แล้ว reset
    ?  ถามสถานะ ME calibration

    วิธีขยับบอร์ด (ตาม datasheet ไม่ใช่ท่าเลข 8 ของ BNO055):
      magnetometer : หมุน ~180° แล้วกลับ รอบแกน roll, pitch, yaw ทีละแกน ~2 s/แกน
      accelerometer: ถือนิ่ง 4–6 ทิศทางที่ต่างกันชัดเจน ~1 s ต่อทิศ
      gyroscope    : วางนิ่งบนพื้นมั่นคง 2–3 s
    รอจน RV accuracy = Medium/High แล้วกด s

  TARE (กำหนดทิศ "ข้างหน้า") — ทำ calibration ก่อนเสมอ
    z  tare เฉพาะแกน Z = recenter heading (ปุ่ม "ตั้งศูนย์")
    a  tare ทุกแกน (วางระดับและหันไปทิศอ้างอิงก่อน)
    p  persist tare ลง flash (System Orientation FRS record)
    t  clear tare ที่บันทึกไว้
    h  แสดงคำสั่ง

  Wiring (I2C): เหมือน 01_BasicRead

  Designed and Manufactured by Massmore — https://www.massmore.shop
*/

#include <Wire.h>
#include <Massmore_BNO08x.h>

#if defined(CONFIG_IDF_TARGET_ESP32S3)
  #define I2C_SDA_PIN  8
  #define I2C_SCL_PIN  9
#elif defined(ARDUINO_ARCH_ESP32)
  #define I2C_SDA_PIN  21
  #define I2C_SCL_PIN  22
#endif
#define I2C_ADDRESS   MASSMORE_BNO08X_I2C_ADDR_DEF

Massmore_BNO08x imu;
uint32_t printTimer = 0;

static void printHelp() {
  Serial.println(F("cal: c=all m=mag e=end s=save x=erase+reset ?=status | tare: z=Z a=all p=persist t=clear"));
}

static void enableReports() {
  imu.enableRotationVector(MASSMORE_BNO08X_INTERVAL_50HZ);
  imu.enableMagnetometer(MASSMORE_BNO08X_INTERVAL_50HZ);
  imu.enableGameRotationVector(MASSMORE_BNO08X_INTERVAL_50HZ);
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) { }
  Serial.println(F("\nMassmore_BNO08x - 04_Calibration_Tare"));

#if defined(ARDUINO_ARCH_ESP32)
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
#else
  Wire.begin();
#endif
  Wire.setClock(100000);

  if (!imu.begin(I2C_ADDRESS, Wire)) {
    Serial.print(F("BNO08x not found: "));
    Serial.println(Massmore_BNO08x::statusToString(imu.lastError()));
    while (true) delay(100);
  }
  enableReports();
  printHelp();
}

void loop() {
  imu.updateAll();

  if (Serial.available()) {
    char c = (char)Serial.read();
    switch (c) {
      // ---- calibration ----
      case 'c': imu.calibrateAll();          Serial.println(F("> calibration ON (accel+gyro+mag)")); break;
      case 'm': imu.calibrateMagnetometer(); Serial.println(F("> magnetometer calibration ON")); break;
      case 'e': imu.endCalibration();        Serial.println(F("> calibration OFF")); break;
      case 's': imu.saveCalibration();       Serial.println(F("> Save DCD sent (written to flash)")); break;
      case 'x':
        Serial.println(F("> erasing calibration + reset..."));
        imu.clearCalibrationAndReset();
        enableReports();
        Serial.println(F("  done, reports re-enabled"));
        break;
      case '?':
        imu.requestCalibrationStatus();
        delay(50);
        imu.updateAll();
        Serial.print(F("> ME calibration status = "));
        Serial.print(imu.getCalibrationStatus());
        Serial.println(imu.calibrationComplete() ? F(" (OK)") : F(" (pending/failed)"));
        break;
      // ---- tare ----
      case 'z': imu.tareNow(MASSMORE_BNO08X_TARE_AXIS_Z);   Serial.println(F("> heading recentered (Z only)")); break;
      case 'a': imu.tareNow(MASSMORE_BNO08X_TARE_AXIS_ALL); Serial.println(F("> full tare (all axes)")); break;
      case 'p': imu.persistTare();                          Serial.println(F("> tare persisted to FRS")); break;
      case 't': imu.clearTare();                            Serial.println(F("> stored tare cleared")); break;
      case 'h': printHelp(); break;
      default: break;
    }
  }

  if ((uint32_t)(millis() - printTimer) < 500) return;
  printTimer = millis();

  Massmore_BNO08x_euler_t e = imu.getEulerDeg();
  float headingErrDeg = imu.getQuatAccuracy() * 57.2957795f;

  Serial.print(F("roll="));  Serial.print(e.roll, 1);
  Serial.print(F(" pitch=")); Serial.print(e.pitch, 1);
  Serial.print(F(" yaw="));   Serial.print(e.yaw, 1);
  Serial.print(F(" | mag acc: "));
  Serial.print(Massmore_BNO08x::accuracyToString(imu.getAccuracy(MASSMORE_BNO08X_SENSOR_MAGNETIC_FIELD)));
  Serial.print(F("  RV acc: "));
  Serial.print(Massmore_BNO08x::accuracyToString(imu.getAccuracy(MASSMORE_BNO08X_SENSOR_ROTATION_VECTOR)));
  Serial.print(F("  heading err="));
  Serial.print(headingErrDeg, 1);
  Serial.print(F(" deg"));
  if (headingErrDeg > 0.0f && headingErrDeg < 10.0f) Serial.print(F("  <- good, press 's' to save"));
  Serial.println();
}
