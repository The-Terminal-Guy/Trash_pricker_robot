# 🤖 Trash Picker Robot

A WiFi-controlled robot with a 5-DOF robotic arm, live camera stream, and a web-based dashboard — all running on dual ESP32s.

---

## 📸 Features

- **Drive control** — Forward, Backward, Left, Right with acceleration ramping
- **Robotic arm** — Base (continuous rotation), Shoulder, Elbow, Wrist, Gripper
- **Live camera stream** — MJPEG stream via ESP32-CAM with snapshot support
- **Web dashboard** — Served directly from the ESP32, no app needed
- **Speed sliders** — Tune max speed and turn speed on the fly
- **Mode switching** — Safely swap between Drive and Arm mode with hardware timer management

---

## 🗂️ Folder Structure

```
Trash_pricker_robot/
├── main_esp32/
│   └── main_esp32.ino       # Main controller: motors, servos, web server, UART
├── esp32_cam/
│   └── esp32_cam.ino        # ESP32-CAM: camera stream, IP reporting over UART
└── README.md
```

---

## 🔌 Hardware

| Component        | Details                          |
|------------------|----------------------------------|
| Main Controller  | ESP32 DevKit                     |
| Camera Module    | ESP32-CAM (AI-Thinker)           |
| Motor Driver     | L298N Dual H-Bridge              |
| Drive Motors     | 2x DC Motors                     |
| Arm Servos       | MG995 (Base continuous), SG90/MG90s (Shoulder, Elbow, Wrist, Gripper) |
| Power            | External 5V (3-5A) for servos, 7.4–11.1V LiPo for motors |

---

## 📡 Wiring Summary

### Main ESP32 → L298N
| ESP32 Pin | L298N Pin | Role               |
|-----------|-----------|--------------------|
| D5        | ENA       | Left Motor Speed   |
| D4        | IN1       | Left Forward       |
| D2        | IN2       | Left Backward      |
| D18       | ENB       | Right Motor Speed  |
| D19       | IN3       | Right Forward      |
| D21       | IN4       | Right Backward     |

### Main ESP32 → Servos
| ESP32 Pin | Servo     | Range         |
|-----------|-----------|---------------|
| D13       | Base      | Continuous    |
| D32       | Shoulder  | 45–180°       |
| D14       | Elbow     | 0–90°         |
| D33       | Wrist     | 0–180°        |
| D27       | Gripper   | 0–60°         |

### Main ESP32 ↔ ESP32-CAM (UART)
| ESP32 Pin | ESP32-CAM Pin | Role         |
|-----------|---------------|--------------|
| RX2 (D16) | U0TX (GPIO1)  | Receive IP   |
| TX2 (D17) | U0RX (GPIO3)  | Send data    |
| GND       | GND           | Common ground |
| 5V        | 5V            | Power         |

> ⚠️ Disconnect UART wires from ESP32-CAM before uploading firmware to it.

---

## 📦 Libraries Required

Install via Arduino Library Manager:

- `ArduinoJson` by Benoît Blanchon
- `ESP32Servo` by Kevin Harrington
- `WiFi` (built-in with ESP32 board package)
- `WebServer` (built-in with ESP32 board package)

---

## 🚀 Getting Started

1. Flash `main_esp32.ino` to your main ESP32
2. Flash `esp32_cam.ino` to your ESP32-CAM
3. Connect to WiFi AP: **SSID:** `ESP32Panel` **Password:** `eps@32panel`
4. Open browser and navigate to `http://192.168.4.1`
5. Use the dashboard to drive the robot or control the arm

---

## ⚡ Power Notes

- **Servos** must be powered by an external 5V supply (3–5A minimum). Do NOT power from ESP32's 5V pin.
- **Motors** powered via L298N from a 7.4V–11.1V LiPo directly.
- All GND lines must be shared (common ground across ESP32, ESP32-CAM, L298N, servos, and battery).

---

## 🛠️ Built With

- Arduino / ESP-IDF (via Arduino IDE)
- Vanilla HTML/CSS/JS dashboard (served from ESP32 PROGMEM)
