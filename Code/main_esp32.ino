#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>

const char* ssid     = "ESP32Panel";
const char* password = "eps@32panel";

WebServer server(80);

// ================= MOTOR (L298N) =================
#define pwmLeft   5
#define in1Left   4
#define in2Left   2
#define pwmRight  18
#define in3Right  19
#define in4Right  21
#define PWM_FREQ  1000
#define PWM_RES   8

#ifdef USE_LEGACY_LEDC
  #define CH_LEFT  14
  #define CH_RIGHT 15
  void motorSetup() {
    ledcSetup(CH_LEFT,  PWM_FREQ, PWM_RES); ledcAttachPin(pwmLeft,  CH_LEFT);
    ledcSetup(CH_RIGHT, PWM_FREQ, PWM_RES); ledcAttachPin(pwmRight, CH_RIGHT);
  }
  void motorWrite(int l, int r) { ledcWrite(CH_LEFT, l); ledcWrite(CH_RIGHT, r); }
  void motorDetach() {
    ledcDetachPin(pwmLeft);
    ledcDetachPin(pwmRight);
  }
#else
  void motorSetup() {
    pinMode(pwmLeft, OUTPUT);
    pinMode(pwmRight, OUTPUT);
  }
  void motorWrite(int l, int r) {
    analogWrite(pwmLeft, l);
    analogWrite(pwmRight, r);
  }
  void motorDetach() {
    analogWrite(pwmLeft, 0);
    analogWrite(pwmRight, 0);
    pinMode(pwmLeft, INPUT);
    pinMode(pwmRight, INPUT);
  }
#endif

void setLeftDir(bool fwd) {
  digitalWrite(in1Left, fwd ? HIGH : LOW);
  digitalWrite(in2Left, fwd ? LOW  : HIGH);
}
void setRightDir(bool fwd) {
  digitalWrite(in3Right, fwd ? HIGH : LOW);
  digitalWrite(in4Right, fwd ? LOW  : HIGH);
}
void brakeMotors() {
  motorWrite(0, 0);
  digitalWrite(in1Left,  HIGH); digitalWrite(in2Left,  HIGH);
  digitalWrite(in3Right, HIGH); digitalWrite(in4Right, HIGH);
}

// ================= ESP32-CAM UART =================
#define CAM_RX   16
#define CAM_TX   17
#define CAM_BAUD 115200

String camIP     = "";
bool   camOnline = false;

void readCamUart() {
  while (Serial2.available()) {
    String line = Serial2.readStringUntil('\n');
    line.trim();
    if (line.startsWith("IP:")) { camIP = line.substring(3); camOnline = true; }
    if (line == "OFFLINE")      { camOnline = false; }
  }
}

// ================= MOTOR STATE =================
int  maxSpeed     = 255;
int  turnSpeed    = 130;
int  accStep      = 15;
int  minPWM       = 100;
int  currentLeft  = 0;
int  currentRight = 0;
char currentCmd   = 'S';
bool motorsBraked = false;

// ================= SERVOS =================
#define PIN_BASE      13  // MG995 Continuous rotation
#define PIN_SHOULDER  32  // 45-180 degrees
#define PIN_ELBOW     14  // 0-90 degrees
#define PIN_WRIST     33  // 0-180 degrees
#define PIN_GRIPPER   27  // 0-60 degrees

Servo servoBase;
Servo servoShoulder;
Servo servoElbow;
Servo servoWrist;
Servo servoGripper;

int baseSpeed   = 90; // 90=stop, 0=full CCW, 180=full CW
int shoulderPos = 45; // Min of 45-180
int elbowPos    = 45; // Mid of 0-90
int wristPos    = 90; // Mid of 0-180
int gripperPos  = 30; // Mid of 0-60

// ================= MODE SWITCHING =================
bool isArmMode = true; // Set true so switchToDriveMode() fires on boot

void switchToDriveMode() {
  if (!isArmMode) return;
  isArmMode = false;

  if (servoBase.attached())     servoBase.detach();
  if (servoShoulder.attached()) servoShoulder.detach();
  if (servoElbow.attached())    servoElbow.detach();
  if (servoWrist.attached())    servoWrist.detach();
  if (servoGripper.attached())  servoGripper.detach();

  currentCmd   = 'S';
  currentLeft  = 0;
  currentRight = 0;
  motorSetup();
  brakeMotors();
}

void switchToArmMode() {
  if (isArmMode) return;
  isArmMode = true;

  brakeMotors();
  motorDetach();

  // Fixed attach order — deterministic timer assignment, no writes on attach
  servoBase.attach(PIN_BASE,         500, 2400);
  servoShoulder.attach(PIN_SHOULDER, 500, 2400);
  servoElbow.attach(PIN_ELBOW,       500, 2400);
  servoWrist.attach(PIN_WRIST,       500, 2400);
  servoGripper.attach(PIN_GRIPPER,   500, 2400);
}

// ================= FORWARD DECLARATIONS =================
void handleDashboard();
void handleMode();
void handleMove();
void handleServo();
void handleConfig();
void handleCamStatus();
void updateMotors();

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  Serial.println("\nBooting Robot Controller...");

  // Drive servo signal pins LOW immediately — kills floating pin noise on boot
  pinMode(PIN_BASE,     OUTPUT); digitalWrite(PIN_BASE,     LOW);
  pinMode(PIN_SHOULDER, OUTPUT); digitalWrite(PIN_SHOULDER, LOW);
  pinMode(PIN_ELBOW,    OUTPUT); digitalWrite(PIN_ELBOW,    LOW);
  pinMode(PIN_WRIST,    OUTPUT); digitalWrite(PIN_WRIST,    LOW);
  pinMode(PIN_GRIPPER,  OUTPUT); digitalWrite(PIN_GRIPPER,  LOW);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  Serial.println("WiFi AP Started. SSID: " + String(ssid));

  delay(1000);

  Serial2.begin(CAM_BAUD, SERIAL_8N1, CAM_RX, CAM_TX);
  pinMode(in1Left,  OUTPUT);
  pinMode(in2Left,  OUTPUT);
  pinMode(in3Right, OUTPUT);
  pinMode(in4Right, OUTPUT);

  // Only timers 2 & 3 for servos — leaves 0 & 1 free for motor PWM
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  servoBase.setPeriodHertz(50);
  servoShoulder.setPeriodHertz(50);
  servoElbow.setPeriodHertz(50);
  servoWrist.setPeriodHertz(50);
  servoGripper.setPeriodHertz(50);

  // Start in DRIVE MODE by default
  switchToDriveMode();

  // ✅ ALL routes registered BEFORE server.begin()
  server.on("/",          handleDashboard);
  server.on("/mode",      HTTP_POST, handleMode);
  server.on("/move",      HTTP_POST, handleMove);
  server.on("/servo",     HTTP_POST, handleServo);
  server.on("/config",    HTTP_POST, handleConfig);
  server.on("/camstatus", HTTP_GET,  handleCamStatus);

  server.begin();
  Serial.println("System Ready.");
}

// ================= LOOP =================
void loop() {
  server.handleClient();
  if (!isArmMode) updateMotors();
  readCamUart();
  delay(10);
}

// ================= UPDATE MOTORS =================
void updateMotors() {
  int targetL = 0, targetR = 0;
  switch (currentCmd) {
    case 'F': setLeftDir(true);  setRightDir(true);  targetL = maxSpeed;  targetR = maxSpeed;  motorsBraked = false; break;
    case 'B': setLeftDir(false); setRightDir(false); targetL = maxSpeed;  targetR = maxSpeed;  motorsBraked = false; break;
    case 'L': setLeftDir(false); setRightDir(true);  targetL = turnSpeed; targetR = turnSpeed; motorsBraked = false; break;
    case 'R': setLeftDir(true);  setRightDir(false); targetL = turnSpeed; targetR = turnSpeed; motorsBraked = false; break;
    case 'S': default: break;
  }

  if (currentCmd != 'S') {
    currentLeft  += constrain(targetL - currentLeft,  -accStep, accStep);
    currentRight += constrain(targetR - currentRight, -accStep, accStep);
    currentLeft   = constrain(currentLeft,  0, maxSpeed);
    currentRight  = constrain(currentRight, 0, maxSpeed);

    int outL = (currentLeft  > 0) ? map(currentLeft,  1, maxSpeed, minPWM, 255) : 0;
    int outR = (currentRight > 0) ? map(currentRight, 1, maxSpeed, minPWM, 255) : 0;
    motorWrite(outL, outR);
  }
}

// ================= ENDPOINTS =================
void handleMode() {
  StaticJsonDocument<128> doc;
  if (deserializeJson(doc, server.arg("plain"))) { server.send(400, "text/plain", "Bad JSON"); return; }
  String mode = doc["mode"].as<String>();
  if (mode == "drive")    switchToDriveMode();
  else if (mode == "arm") switchToArmMode();
  server.send(200, "text/plain", "OK");
}

void handleMove() {
  if (isArmMode) switchToDriveMode();
  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, server.arg("plain"))) { server.send(400, "text/plain", "Bad JSON"); return; }
  currentCmd = doc["cmd"].as<String>()[0];
  if (currentCmd == 'S') {
    currentLeft = 0; currentRight = 0;
    brakeMotors(); motorsBraked = true;
  }
  server.send(200, "text/plain", "OK");
}

void handleConfig() {
  StaticJsonDocument<128> doc;
  if (deserializeJson(doc, server.arg("plain"))) { server.send(400, "text/plain", "Bad JSON"); return; }
  if (doc.containsKey("maxSpeed"))  maxSpeed  = constrain((int)doc["maxSpeed"],  50, 255);
  if (doc.containsKey("turnSpeed")) turnSpeed = constrain((int)doc["turnSpeed"], 50, 255);
  server.send(200, "text/plain", "OK");
}

void handleServo() {
  if (!isArmMode) switchToArmMode();
  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, server.arg("plain"))) { server.send(400, "text/plain", "Bad JSON"); return; }

  if (doc.containsKey("base")) {
    baseSpeed = constrain((int)doc["base"], 0, 180);
    servoBase.write(baseSpeed);
  }
  if (doc.containsKey("shoulder")) {
    shoulderPos = constrain((int)doc["shoulder"], 45, 180);
    servoShoulder.write(shoulderPos);
  }
  if (doc.containsKey("elbow")) {
    elbowPos = constrain((int)doc["elbow"], 0, 90);
    servoElbow.write(elbowPos);
  }
  if (doc.containsKey("wrist")) {
    wristPos = constrain((int)doc["wrist"], 0, 180);
    servoWrist.write(wristPos);
  }
  if (doc.containsKey("gripper")) {
    gripperPos = constrain((int)doc["gripper"], 0, 60);
    servoGripper.write(gripperPos);
  }
  server.send(200, "text/plain", "OK");
}

void handleCamStatus() {
  String resp = "{\"online\":" + String(camOnline ? "true" : "false") + ",\"ip\":\"" + camIP + "\"}";
  server.send(200, "application/json", resp);
}

// ================= DASHBOARD HTML =================
const char DASHBOARD_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
  <title>Esp32CtrlPanel Dashboard</title>
  <style>
    :root {
      --bg: #121212; --card-bg: #1e1e1e; --primary: #00adb5;
      --text: #eeeeee; --danger: #ff2e63; --success: #00e676;
    }
    * { box-sizing: border-box; margin: 0; padding: 0; font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; }
    body { background: var(--bg); color: var(--text); padding: 15px; display: flex; flex-direction: column; align-items: center; min-height: 100vh; }

    .header { text-align: center; margin-bottom: 15px; }
    .header h2 { color: var(--primary); letter-spacing: 2px; text-transform: uppercase; font-size: 1.5rem; }
    .header p { font-size: 0.8rem; color: #888; }

    .layout { display: flex; flex-direction: row; width: 100%; max-width: 900px; gap: 15px; align-items: flex-start; }

    .console-panel { flex: 1; background: var(--card-bg); border-radius: 12px; padding: 15px; height: 75vh; display: flex; flex-direction: column; box-shadow: 0 4px 10px rgba(0,0,0,0.5); }
    .console-panel h4 { color: var(--primary); margin-bottom: 10px; font-size: 0.9rem; text-transform: uppercase; letter-spacing: 1px; }
    .console-box { flex: 1; background: #000; color: #0f0; font-family: monospace; font-size: 0.75rem; padding: 10px; overflow-y: auto; border-radius: 8px; border: 1px solid #333; line-height: 1.4; }

    .main-panel { flex: 2; display: flex; flex-direction: column; gap: 15px; width: 100%; }
    .card { background: var(--card-bg); border-radius: 12px; padding: 15px; box-shadow: 0 4px 10px rgba(0,0,0,0.5); }

    .stream-box { position: relative; width: 100%; aspect-ratio: 4/3; background: #000; border-radius: 8px; overflow: hidden; display: flex; align-items: center; justify-content: center; }
    .stream-box img { width: 100%; height: 100%; object-fit: cover; display: none; }
    .status-badge { position: absolute; top: 10px; left: 10px; padding: 4px 8px; border-radius: 4px; font-size: 0.75rem; font-weight: bold; background: rgba(0,0,0,0.7); z-index: 10; }
    .online { color: var(--success); } .offline { color: var(--danger); }
    .snap-btn { position: absolute; bottom: 10px; right: 10px; background: rgba(0,0,0,0.7); color: #fff; border: 1px solid var(--primary); border-radius: 4px; padding: 6px 10px; font-size: 0.75rem; cursor: pointer; transition: 0.2s; z-index: 10; }
    .snap-btn:hover { background: var(--primary); }

    .tabs { display: flex; gap: 10px; }
    .tab-btn { flex: 1; padding: 12px; background: #2a2a2a; color: #aaa; border: none; border-radius: 8px; font-size: 0.9rem; font-weight: bold; cursor: pointer; transition: 0.2s; }
    .tab-btn.active { background: var(--primary); color: #fff; }

    .tab-content { display: none; }
    .tab-content.active { display: block; animation: fadeIn 0.3s ease; }
    @keyframes fadeIn { from { opacity: 0; transform: translateY(5px); } to { opacity: 1; transform: translateY(0); } }

    .d-pad { display: grid; grid-template-columns: repeat(3, 1fr); gap: 10px; max-width: 250px; margin: 10px auto; }
    .d-btn { background: #333; border: none; border-radius: 8px; padding: 20px; font-size: 1.5rem; color: #fff; cursor: pointer; transition: 0.1s; user-select: none; display: flex; justify-content: center; align-items: center; }
    .d-btn:active, .d-btn.pressed { background: var(--primary); transform: scale(0.95); }
    .d-btn.stop { background: var(--danger); font-size: 1.2rem; }
    .d-btn.stop:active, .d-btn.stop.pressed { background: #ff003c; }
    .empty { visibility: hidden; }

    .speed-row { display: flex; align-items: center; gap: 12px; margin-top: 12px; padding-top: 12px; border-top: 1px solid #2a2a2a; }
    .speed-row label { font-size: 0.8rem; color: #888; white-space: nowrap; min-width: 90px; }
    .speed-row span { color: var(--primary); font-size: 0.85rem; min-width: 35px; text-align: right; }

    .base-controls { display: flex; gap: 10px; justify-content: center; align-items: center; margin-bottom: 18px; }
    .base-btn { flex: 1; max-width: 110px; padding: 12px 0; background: #2a2a2a; color: #fff; border: none; border-radius: 8px; font-size: 1.1rem; font-weight: bold; cursor: pointer; transition: 0.1s; user-select: none; }
    .base-btn:active, .base-btn.pressed { background: var(--primary); transform: scale(0.95); }
    .base-btn.stop { background: var(--danger); }
    .base-btn.stop:active { background: #ff003c; }
    .base-label { font-size: 0.75rem; color: #888; text-align: center; margin-bottom: 8px; text-transform: uppercase; letter-spacing: 1px; }

    .slider-row { margin-bottom: 15px; }
    .slider-row label { display: flex; justify-content: space-between; margin-bottom: 8px; font-size: 0.85rem; color: #bbb; font-weight: bold; }
    .slider-row span { color: var(--primary); }
    input[type="range"] { width: 100%; -webkit-appearance: none; background: #333; height: 8px; border-radius: 4px; outline: none; }
    input[type="range"]::-webkit-slider-thumb { -webkit-appearance: none; width: 20px; height: 20px; border-radius: 50%; background: var(--primary); cursor: pointer; }

    @media (max-width: 768px) {
      .layout { flex-direction: column-reverse; }
      .console-panel { height: 200px; width: 100%; }
    }
  </style>
</head>
<body>

  <div class="header">
    <h2>Esp32CtrlPanel</h2>
    <p>Central Control Station</p>
  </div>

  <div class="layout">

    <!-- CONSOLE -->
    <div class="console-panel">
      <h4>System Log</h4>
      <div class="console-box" id="consoleBox">
        <div>[SYSTEM] Boot successful.</div>
        <div>[SYSTEM] Awaiting commands...</div>
      </div>
    </div>

    <div class="main-panel">

      <!-- CAMERA -->
      <div class="card">
        <div class="stream-box">
          <div id="camStatus" class="status-badge offline">CONNECTING...</div>
          <img id="streamImg" alt="Camera Stream">
          <button class="snap-btn" onclick="snapshot()">SNAP</button>
        </div>
      </div>

      <!-- TABS -->
      <div class="tabs">
        <button class="tab-btn active" id="btn-drive" onclick="switchTab('drive')">DRIVE</button>
        <button class="tab-btn" id="btn-arm" onclick="switchTab('arm')">ARM CONTROLS</button>
      </div>

      <!-- DRIVE TAB -->
      <div id="tab-drive" class="card tab-content active">
        <div class="d-pad">
          <div class="empty"></div>
          <button class="d-btn" id="bF" onmousedown="press('F','bF',event)" onmouseup="release(event)" onmouseleave="release(event)" ontouchstart="press('F','bF',event)" ontouchend="release(event)">&#9650;</button>
          <div class="empty"></div>

          <button class="d-btn" id="bL" onmousedown="press('L','bL',event)" onmouseup="release(event)" onmouseleave="release(event)" ontouchstart="press('L','bL',event)" ontouchend="release(event)">&#9664;</button>
          <button class="d-btn stop" id="bS" onmousedown="forceStop(event)" ontouchstart="forceStop(event)">STOP</button>
          <button class="d-btn" id="bR" onmousedown="press('R','bR',event)" onmouseup="release(event)" onmouseleave="release(event)" ontouchstart="press('R','bR',event)" ontouchend="release(event)">&#9654;</button>

          <div class="empty"></div>
          <button class="d-btn" id="bB" onmousedown="press('B','bB',event)" onmouseup="release(event)" onmouseleave="release(event)" ontouchstart="press('B','bB',event)" ontouchend="release(event)">&#9660;</button>
          <div class="empty"></div>
        </div>

        <div class="speed-row">
          <label>Max Speed</label>
          <input type="range" id="speedSlider" min="50" max="255" value="255" oninput="updateSpeed(this.value)">
          <span id="speedVal">255</span>
        </div>
        <div class="speed-row">
          <label>Turn Speed</label>
          <input type="range" id="turnSlider" min="50" max="255" value="130" oninput="updateTurn(this.value)">
          <span id="turnVal">130</span>
        </div>
      </div>

      <!-- ARM TAB -->
      <div id="tab-arm" class="card tab-content">
        <div class="base-label">Base Rotation</div>
        <div class="base-controls">
          <button class="base-btn" id="bCCW"
            onmousedown="basePress(0,'bCCW',event)" onmouseup="baseRelease(event)" onmouseleave="baseRelease(event)"
            ontouchstart="basePress(0,'bCCW',event)" ontouchend="baseRelease(event)">&#8634; CCW</button>
          <button class="base-btn stop" id="bBStop" onmousedown="baseStop(event)" ontouchstart="baseStop(event)">STOP</button>
          <button class="base-btn" id="bCW"
            onmousedown="basePress(180,'bCW',event)" onmouseup="baseRelease(event)" onmouseleave="baseRelease(event)"
            ontouchstart="basePress(180,'bCW',event)" ontouchend="baseRelease(event)">CW &#8635;</button>
        </div>

        <div class="slider-row">
          <label>Shoulder <span id="v-sh">45&deg;</span></label>
          <input type="range" id="sh" min="45" max="180" value="45" oninput="updateArm('shoulder', this.value, 'v-sh')">
        </div>
        <div class="slider-row">
          <label>Elbow <span id="v-el">45&deg;</span></label>
          <input type="range" id="el" min="0" max="90" value="45" oninput="updateArm('elbow', this.value, 'v-el')">
        </div>
        <div class="slider-row">
          <label>Wrist <span id="v-wr">90&deg;</span></label>
          <input type="range" id="wr" min="0" max="180" value="90" oninput="updateArm('wrist', this.value, 'v-wr')">
        </div>
        <div class="slider-row">
          <label>Gripper <span id="v-gr">30&deg;</span></label>
          <input type="range" id="gr" min="0" max="60" value="30" oninput="updateArm('gripper', this.value, 'v-gr')">
        </div>
      </div>

    </div>
  </div>

  <script>
    // --- Console Logger ---
    function logCmd(msg) {
      const cb = document.getElementById('consoleBox');
      const time = new Date().toLocaleTimeString('en-US', {hour12: false, hour: '2-digit', minute:'2-digit', second:'2-digit'});
      const line = document.createElement('div');
      line.textContent = `[${time}] > ${msg}`;
      cb.appendChild(line);
      if (cb.childNodes.length > 50) cb.removeChild(cb.firstChild);
      cb.scrollTop = cb.scrollHeight;
    }

    // --- Tabs ---
    function switchTab(tab) {
      document.getElementById('tab-drive').classList.toggle('active', tab === 'drive');
      document.getElementById('tab-arm').classList.toggle('active', tab === 'arm');
      document.getElementById('btn-drive').classList.toggle('active', tab === 'drive');
      document.getElementById('btn-arm').classList.toggle('active', tab === 'arm');
      fetch('/mode', { method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({mode: tab}) });
      logCmd('MODE SWITCHED: ' + tab.toUpperCase() + ' ACTIVE');
    }

    // --- Camera ---
    let camIp = '';
    const imgEl = document.getElementById('streamImg');
    const badge = document.getElementById('camStatus');

    function checkCam() {
      fetch('/camstatus').then(r => r.json()).then(d => {
        if (d.online && d.ip) {
          if (camIp !== d.ip) logCmd('CAMERA: Stream connected at ' + d.ip);
          camIp = d.ip;
          badge.className = 'status-badge online';
          badge.textContent = 'LIVE: ' + d.ip;
          imgEl.onerror = () => {
            imgEl.style.display = 'none';
            imgEl.src = '';
            logCmd('CAMERA: Stream lost, retrying...');
          };
          if (imgEl.style.display !== 'block') {
            imgEl.src = '';
            setTimeout(() => {
              imgEl.src = 'http://' + d.ip + '/stream?t=' + Date.now();
              imgEl.style.display = 'block';
            }, 50);
          }
        } else {
          badge.className = 'status-badge offline';
          badge.textContent = 'CAM OFFLINE';
          imgEl.style.display = 'none';
          imgEl.src = '';
          camIp = '';
        }
      }).catch(console.error);
    }
    checkCam(); setInterval(checkCam, 2000);

    function snapshot() {
      if (!camIp) { alert('Camera Offline'); return; }
      logCmd('CAMERA: Snapshot requested');
      window.open('http://' + camIp + '/capture', '_blank');
    }

    // --- Drive Logic ---
    let activeBtn = null, heldKey = null, holdTimer = null;

    function press(cmd, id, e) {
      if (e && e.cancelable) e.preventDefault();
      if (activeBtn) document.getElementById(activeBtn).classList.remove('pressed');
      activeBtn = id;
      document.getElementById(id).classList.add('pressed');
      fetch('/move', { method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({cmd}) });
      logCmd('DRIVE: ' + cmd);
    }

    function release(e) {
      if (e && e.cancelable) e.preventDefault();
      if (!activeBtn) return;
      document.getElementById(activeBtn).classList.remove('pressed');
      activeBtn = null;
      fetch('/move', { method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({cmd:'S'}) });
      logCmd('DRIVE: BRAKE');
    }

    function forceStop(e) {
      if (e && e.cancelable) e.preventDefault();
      stopHold();
      press('S', 'bS', null);
      setTimeout(() => {
        if (activeBtn === 'bS') { document.getElementById('bS').classList.remove('pressed'); activeBtn = null; }
      }, 200);
    }

    const keyMap = {'w':'F','s':'B','a':'L','d':'R'};
    const btnMap = {'F':'bF','B':'bB','L':'bL','R':'bR'};

    function startHold(c) {
      if (heldKey === c) return;
      stopHold(); heldKey = c;
      press(c, btnMap[c], null);
      holdTimer = setInterval(() => press(c, btnMap[c], null), 150);
    }

    function stopHold() {
      if (holdTimer) { clearInterval(holdTimer); holdTimer = null; }
      heldKey = null;
      release(null);
    }

    document.addEventListener('keydown', e => {
      if (e.key === ' ' && document.getElementById('tab-drive').classList.contains('active')) { e.preventDefault(); forceStop(null); return; }
      const c = keyMap[e.key.toLowerCase()];
      if (c && document.getElementById('tab-drive').classList.contains('active')) { e.preventDefault(); startHold(c); }
    });

    document.addEventListener('keyup', e => {
      if (e.key === ' ' && document.getElementById('tab-drive').classList.contains('active')) { e.preventDefault(); return; }
      const c = keyMap[e.key.toLowerCase()];
      if (c && heldKey === c) stopHold();
    });

    // --- Speed Sliders ---
    let speedTimer = null;
    function updateSpeed(val) {
      document.getElementById('speedVal').textContent = val;
      clearTimeout(speedTimer);
      speedTimer = setTimeout(() => {
        fetch('/config', { method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({maxSpeed: parseInt(val)}) });
        logCmd('CONFIG: Max Speed=' + val);
      }, 150);
    }

    let turnTimer = null;
    function updateTurn(val) {
      document.getElementById('turnVal').textContent = val;
      clearTimeout(turnTimer);
      turnTimer = setTimeout(() => {
        fetch('/config', { method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({turnSpeed: parseInt(val)}) });
        logCmd('CONFIG: Turn Speed=' + val);
      }, 150);
    }

    // --- Base Controls ---
    let activeBaseBtn = null;

    function basePress(speed, id, e) {
      if (e && e.cancelable) e.preventDefault();
      if (activeBaseBtn) document.getElementById(activeBaseBtn).classList.remove('pressed');
      activeBaseBtn = id;
      document.getElementById(id).classList.add('pressed');
      fetch('/servo', { method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({base: speed}) });
      logCmd('BASE: ' + (speed === 0 ? 'CCW' : 'CW'));
    }

    function baseRelease(e) {
      if (e && e.cancelable) e.preventDefault();
      if (!activeBaseBtn) return;
      document.getElementById(activeBaseBtn).classList.remove('pressed');
      activeBaseBtn = null;
      fetch('/servo', { method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({base: 90}) });
      logCmd('BASE: STOP');
    }

    function baseStop(e) {
      if (e && e.cancelable) e.preventDefault();
      if (activeBaseBtn) { document.getElementById(activeBaseBtn).classList.remove('pressed'); activeBaseBtn = null; }
      fetch('/servo', { method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({base: 90}) });
      logCmd('BASE: FORCE STOP');
    }

    // --- Arm Sliders ---
    let armTimers = {};
    function updateArm(joint, value, labelId) {
      document.getElementById(labelId).innerHTML = value + '&deg;';
      clearTimeout(armTimers[joint]);
      armTimers[joint] = setTimeout(() => {
        const payload = {};
        payload[joint] = parseInt(value);
        fetch('/servo', { method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(payload) });
        logCmd('ARM: ' + joint.toUpperCase() + '=' + value + '°');
      }, 100);
    }
  </script>
</body>
</html>
)rawliteral";

void handleDashboard() {
  server.send(200, "text/html", DASHBOARD_HTML);
}
