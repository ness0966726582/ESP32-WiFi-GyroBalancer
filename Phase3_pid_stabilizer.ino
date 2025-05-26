// Phase3_pid_stabilizer.ino - 簡易 PID 姿態控制（Hover 模擬）

#include <Wire.h>
#include <MPU6050.h>

// PID 參數
float kp = 1.5;
float ki = 0.0;
float kd = 0.5;

float errorPitch, lastErrorPitch = 0, integralPitch = 0;
float errorRoll,  lastErrorRoll  = 0, integralRoll  = 0;

// 目標角度（穩定狀態）
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
const int resolution = 8; // 0~255
const int baseSpeed = 130; // 起始油門值
const int maxPWM = 255;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();

  // 初始化 PWM 通道
  ledcSetup(0, freq, resolution); ledcAttachPin(motor1Pin, 0);
  ledcSetup(1, freq, resolution); ledcAttachPin(motor2Pin, 1);
  ledcSetup(2, freq, resolution); ledcAttachPin(motor3Pin, 2);
  ledcSetup(3, freq, resolution); ledcAttachPin(motor4Pin, 3);

  Serial.println("🚁 PID Stabilizer Ready.");
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

  // 馬達補償計算（M1對角M3控制Pitch，M2對角M4控制Roll）
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

  delay(50); // 控制頻率
}
