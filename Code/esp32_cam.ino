// ================= ESP32-CAM Firmware =================
// Place your ESP32-CAM code here.
// This module is responsible for:
//   - Connecting to the main ESP32's WiFi AP (SSID: ESP32Panel)
//   - Sending its IP address over UART: Serial.println("IP:" + WiFi.localIP().toString())
//   - Serving MJPEG stream at /stream
//   - Serving snapshot at /capture
