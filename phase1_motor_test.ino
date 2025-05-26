// ESP32 4-Motor PWM Test

// 馬達 PWM 腳位
const int motor1Pin = 25;
const int motor2Pin = 26;
const int motor3Pin = 27;
const int motor4Pin = 14;

// PWM 參數
const int freq = 5000;
const int resolution = 8; // 8-bit: 0~255

// PWM 通道
const int pwmChannel1 = 0;
const int pwmChannel2 = 1;
const int pwmChannel3 = 2;
const int pwmChannel4 = 3;

int speedValue = 128; // 初始速度

void setup() {
  Serial.begin(115200);
  delay(1000);

  // 初始化每個 PWM 通道與腳位
  ledcSetup(pwmChannel1, freq, resolution);
  ledcAttachPin(motor1Pin, pwmChannel1);

  ledcSetup(pwmChannel2, freq, resolution);
  ledcAttachPin(motor2Pin, pwmChannel2);

  ledcSetup(pwmChannel3, freq, resolution);
  ledcAttachPin(motor3Pin, pwmChannel3);

  ledcSetup(pwmChannel4, freq, resolution);
  ledcAttachPin(motor4Pin, pwmChannel4);

  Serial.println("⚙️ Motor PWM Test Ready! Type 0~255 to set speed for all motors.");
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() > 0) {
      int value = input.toInt();

      if (value >= 0 && value <= 255) {
        speedValue = value;
        Serial.print("🚀 Setting all motors to speed: ");
        Serial.println(speedValue);

        ledcWrite(pwmChannel1, speedValue);
        ledcWrite(pwmChannel2, speedValue);
        ledcWrite(pwmChannel3, speedValue);
        ledcWrite(pwmChannel4, speedValue);
      } else {
        Serial.println("❌ Invalid speed! Enter a number between 0 and 255.");
      }
    }
  }
}
