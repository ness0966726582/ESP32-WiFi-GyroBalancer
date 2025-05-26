// Phase2__imu_mpu6050_read.ino - 姿態資料讀取（含 TCA9548A 多工器）
// 支援多組 MPU6050，印出每個感測器的 Pitch / Roll / Yaw 資料

#include <Wire.h>
#include <MPU6050.h>

#define TCA_ADDR 0x70  // TCA9548A 的 I2C 位址

MPU6050 mpu;

void tca_select(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  for (uint8_t i = 0; i < 8; i++) {
    tca_select(i);
    mpu.initialize();
    if (mpu.testConnection()) {
      Serial.print("✅ MPU6050 found on channel ");
      Serial.println(i);
    } else {
      Serial.print("❌ No MPU6050 on channel ");
      Serial.println(i);
    }
  }
}

void loop() {
  for (uint8_t i = 0; i < 8; i++) {
    tca_select(i);

    if (!mpu.testConnection()) continue;

    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // 簡易角度估算（未經濾波）
    float pitch = atan2(ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
    float roll  = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI;
    float yaw   = atan2(az, sqrt(ax * ax + ay * ay)) * 180.0 / PI;

    Serial.print("CH"); Serial.print(i);
    Serial.print(" → Pitch: "); Serial.print(pitch, 1);
    Serial.print("°, Roll: "); Serial.print(roll, 1);
    Serial.print("°, Yaw: "); Serial.print(yaw, 1);
    Serial.println("°");

    delay(100); // 依需求可調整頻率
  }

  Serial.println("----");
  delay(500);
}
