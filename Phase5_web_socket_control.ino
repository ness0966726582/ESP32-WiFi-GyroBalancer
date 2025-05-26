// Phase5_web_socket_control.ino - WebSocket WiFi 控制飛行器（ESP32）

#include <WiFi.h>
#include <WebSocketsServer.h>
#include <Wire.h>
#include <MPU6050.h>

// WiFi 設定
const char* ssid = "YOUR_SSID";
const char* password = "YOUR_PASSWORD";

WebSocketsServer webSocket = WebSocketsServer(81);

// MPU6050 + PID
MPU6050 mpu;
float kp = 2.2, ki = 0.01, kd = 0.8;
float errorPitch, lastErrorPitch = 0, integralPitch = 0;
float errorRoll,  lastErrorRoll  = 0, integralRoll  = 0;

// 目標姿態
float targetPitch = 0.0;
float targetRoll  = 0.0;
int baseSpeed = 140;
const int maxPWM = 255;

// 馬達腳位
const int motor1Pin = 25;
const int motor2Pin = 26;
const int motor3Pin = 27;
const int motor4Pin = 14;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();

  // PWM 初始化
  ledcSetup(0, 5000, 8); ledcAttachPin(motor1Pin, 0);
  ledcSetup(1, 5000, 8); ledcAttachPin(motor2Pin, 1);
  ledcSetup(2, 5000, 8); ledcAttachPin(motor3Pin, 2);
  ledcSetup(3, 5000, 8); ledcAttachPin(motor4Pin, 3);

  // WiFi & WebSocket 初始化
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }
  Serial.println("\n✅ WiFi connected: " + WiFi.localIP().toString());
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  Serial.println("🌐 WebSocket server started (port 81)");
}

void loop() {
  webSocket.loop();

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float pitch = atan2(ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  float roll  = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI;

  // PID Pitch
  errorPitch = targetPitch - pitch;
  integralPitch += errorPitch;
  float outputPitch = kp * errorPitch + ki * integralPitch + kd * (errorPitch - lastErrorPitch);
  lastErrorPitch = errorPitch;

  // PID Roll
  errorRoll = targetRoll - roll;
  integralRoll += errorRoll;
  float outputRoll = kp * errorRoll + ki * integralRoll + kd * (errorRoll - lastErrorRoll);
  lastErrorRoll = errorRoll;

  int m1 = constrain(baseSpeed + outputPitch - outputRoll, 0, maxPWM);
  int m2 = constrain(baseSpeed - outputPitch - outputRoll, 0, maxPWM);
  int m3 = constrain(baseSpeed - outputPitch + outputRoll, 0, maxPWM);
  int m4 = constrain(baseSpeed + outputPitch + outputRoll, 0, maxPWM);

  ledcWrite(0, m1);
  ledcWrite(1, m2);
  ledcWrite(2, m3);
  ledcWrite(3, m4);
}

// WebSocket 指令處理
void webSocketEvent(uint8_t client_num, WStype_t type, uint8_t * payload, size_t length) {
  if (type == WStype_TEXT) {
    String msg = String((char *)payload);
    Serial.println("📩 Received: " + msg);

    // 預設格式: "P:1.5,R:-2.3,T:150"
    if (msg.startsWith("P:")) {
      int pIndex = msg.indexOf("P:");
      int rIndex = msg.indexOf(",R:");
      int tIndex = msg.indexOf(",T:");

      targetPitch = msg.substring(pIndex + 2, rIndex).toFloat();
      targetRoll  = msg.substring(rIndex + 3, tIndex).toFloat();
      baseSpeed   = msg.substring(tIndex + 3).toInt();

      Serial.printf("🎯 Target Pitch: %.2f, Roll: %.2f, Thrust: %d\n", targetPitch, targetRoll, baseSpeed);
    }
  }
}
