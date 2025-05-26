# 🚁 ESP32 ESP32-WiFi-GyroBalancer Project

用 ESP32 打造一套分階段的四軸飛行器實驗平台，從馬達控制到姿態穩定，逐步實現自製飛控！

---

## 📦 專案目錄結構

```
esp32-drone-stabilization/
├── phase1_motor_test/
│   └── motor_pwm_test.ino        # 四馬達 PWM 測試程式
├── phase2_imu_reading/
│   └── imu_mpu6050_read.ino      # MPU6050 讀取角度與加速度
├── phase3_pid_control/
│   └── pid_stabilizer.ino        # PID 計算 + 輸出馬達補償值
├── phase4_hover_test/
│   └── auto_hover.ino            # 自動懸停實驗程式（進階）
├── phase5_network_control/
│   └── web_socket_control.ino    # 使用 WebSocket 透過 Wi-Fi 控制飛行（待實作）
├── ue5_simulation/
│   └── (預留) unreal_flight_sim.cpp  # Unreal Engine 模擬飛行資料接收器
├── hardware/
│   └── wiring_diagram.png        # 接線圖（建議用 Fritzing 畫）
└── README.md
```

---

## 🧪 階段目標

### Phase 1: 馬達啟動測試
- ✅ 控制四個 ESC + 無刷馬達轉動
- ✅ 確認 ESC 校正無誤
- ✅ PWM 輸出穩定，轉速均勻

### Phase 2: 姿態資料讀取
- ✅ 用 MPU6050 擷取 Pitch / Roll / Yaw
- ✅ 串接 I2C Multiplexer（TCA9548A）
- ✅ 每次 loop 印出每顆感測器數值

### Phase 3: PID 姿態控制
- ✅ 撰寫 PID 控制器
- ✅ 將傾斜補償轉成馬達輸出修正值
- ✅ 練習 Hover 模擬（桌上原地轉）

### Phase 4: 自動懸停
- ✅ 整合感測器 + PID + 輸出
- ✅ 微調 PID 參數
- ✅ 嘗試短時間內懸停 5～10 秒

### Phase 5: WebSocket 控制（Wi-Fi 模式）
- 🌐 建立 ESP32 WebSocket Server
- 🌐 從手機、網頁或 UE5 傳送控制指令（推力、角度）
- 🌐 ESP32 接收指令並轉為姿態目標，交由 PID 控制輸出

### Phase 6 (預留): Unreal Engine 模擬飛行
- 🕹️ 透過 WebSocket 接收 ESP32 傳來的姿態資料
- 🕹️ 使用 UE5 Blueprint 或 C++ 控制虛擬飛行器模型
- 🕹️ 結合實體數據與虛擬模擬，進行飛行訓練與展示

---

## 🔌 硬體需求

| 名稱 | 數量 | 備註 |
|------|------|------|
| ESP32 DevKit | 1 | 控制主機 |
| BLDC 無刷馬達 | 4 | 建議 2300KV 級距 |
| ESC 電調 | 4 | 支援 PWM 控制 |
| MPU6050 | 4 | 建議用 I2C 多工器整合 |
| TCA9548A | 1 | I2C multiplexer 模組 |
| 螺旋槳 | 4 | CW/CCW 各兩片 |
| 鋰電池 3S | 1 | 建議 ≥1500mAh |
| 杜邦線、麵包板 | 多條 | 建議顏色區分清楚 |

---

## 🪛 硬體接線說明

### ESP32 到 TCA9548A 接線

| ESP32 | TCA9548A | 說明 |
|--------|-----------|------|
| 3.3V   | VCC       | I2C 電源 |
| GND    | GND       | 共地 |
| GPIO21 | SDA       | I2C 資料線 |
| GPIO22 | SCL       | I2C 時脈線 |

### TCA9548A 到 MPU6050（通道 0~3）

| TCA9548A Port | MPU6050 | 位址 |
|---------------|----------|------|
| Channel 0     | MPU1     | 0x68（AD0 接 GND）|
| Channel 1     | MPU2     | 0x68（AD0 接 GND）|
| Channel 2     | MPU3     | 0x68（AD0 接 GND）|
| Channel 3     | MPU4     | 0x68（AD0 接 GND）|

### ESP32 PWM 輸出控制 ESC

| ESP32 GPIO | 接 ESC | 功能 |
|------------|--------|------|
| GPIO13     | ESC1   | 前左馬達 PWM |
| GPIO14     | ESC2   | 前右馬達 PWM |
| GPIO25     | ESC3   | 後左馬達 PWM |
| GPIO26     | ESC4   | 後右馬達 PWM |

> ESC 的電源由鋰電池直接供應，請共地至 ESP32。

---

## 📘 如何開始

```bash
git clone https://github.com/your-username/esp32-drone-stabilization.git
cd esp32-drone-stabilization/phase1_motor_test
open motor_pwm_test.ino
```

---

## 📋 授權

MIT License
