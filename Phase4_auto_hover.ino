// Phase4_auto_hover.ino - 自動懸停系統（PID 微調後穩定懸停）

#include <Wire.h>
#include <MPU6050.h>

// 微調後的 PID 參數（依現場測試可再調整）
float kp = 2.2;
float ki = 0.01;
float kd = 0.8;

float errorPitch, lastErrorPitch = 0, integralPitch = 0;
float errorRoll,  lastErrorRoll  = 0, integralRoll  = 0;

// 目標姿態角度
const float targetPitch = 0.0;
const float targetRoll  = 0.0;

MPU6050 mpu;

// PWM 馬達控制腳位
const int motor1Pin = 25;
const int motor2Pin = 26;
const int motor3Pin = 27;
const int motor4Pin = 14;

// PWM 設定
const int freq = 5000;
const int resolution = 8;
const int baseSpeed = 140;  // 調整為接近懸停的基礎油門
const int maxPWM = 255;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();

  // PWM 初始化
  ledcSetup(0, freq, resolution); ledcAttachPin(motor1Pin, 0);
  ledcSetup(1, freq, resolution); ledcAttachPin(motor2Pin, 1);
  ledcSetup(2, freq, resolution); ledcAttachPin(motor3Pin, 2);
  ledcSetup(3, freq, resolution); ledcAttachPin(motor4Pin, 3);

  Serial.println("🛸 Auto Hover Initialized");
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float pitch = atan2(ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  float roll  = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI;

  // === PID for Pitch ===
  errorPitch = targetPitch - pitch;
  integralPitch += errorPitch;
  float derivativePitch = errorPitch - lastErrorPitch;
  float outputPitch = kp * errorPitch + ki * integralPitch + kd * derivativePitch;
  lastErrorPitch = errorPitch;

  // === PID for Roll ===
  errorRoll = targetRoll - roll;
  integralRoll += errorRoll;
  float derivativeRoll = errorRoll - lastErrorRoll;
  float outputRoll = kp * errorRoll + ki * integralRoll + kd * derivativeRoll;
  lastErrorRoll = errorRoll;

  // 馬達補償
  int m1 = constrain(baseSpeed + outputPitch - outputRoll, 0, maxPWM);
  int m2 = constrain(baseSpeed - outputPitch - outputRoll, 0, maxPWM);
  int m3 = constrain(baseSpeed - outputPitch + outputRoll, 0, maxPWM);
  int m4 = constrain(baseSpeed + outputPitch + outputRoll, 0, maxPWM);

  ledcWrite(0, m1);
  ledcWrite(1, m2);
  ledcWrite(2, m3);
  ledcWrite(3, m4);

  Serial.print("P:"); Serial.print(pitch, 1);
  Serial.print(" R:"); Serial.print(roll, 1);
  Serial.print(" | M1:"); Serial.print(m1);
  Serial.print(" M2:"); Serial.print(m2);
  Serial.print(" M3:"); Serial.print(m3);
  Serial.print(" M4:"); Serial.println(m4);

  delay(30); // 控制速度
}
