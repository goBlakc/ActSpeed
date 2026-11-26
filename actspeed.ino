/*
  ActSpeed - ESP32-C3 Super Mini joystick firmware (one file, Arduino IDE)

  The joystick is a dumb sensor: no dead zone, no gestures, just scaled X/Y
  published over MQTT with the time each sample was taken, so the backend can
  undo Wi-Fi delays (docs/firmware.md). It can also be a Bluetooth mouse and
  keyboard that carries out what the backend resolves (the "hid" topic).

  Wiring: VCC -> 3V3 (NOT 5V) | GND -> GND | VRx -> GPIO4 | VRy -> GPIO3 | SW -> GPIO2 (unused)
  Optional PS/2 finger mouse: C = clock -> GPIO5, D = data -> GPIO6. The mouse chip (EM84510)
  needs 4.5-5.5 V: power it from 5V and put a 3.3V/5V level shifter on C and D.

  Arduino IDE:
    Board "ESP32C3 Dev Module", Tools > "USB CDC On Boot: Enabled",
    Tools > Partition Scheme: "Huge APP (3MB No OTA)".
    Libraries (Library Manager): espMqttClient (Bert Melis), ArduinoJson 7,
    NimBLE-Arduino 2.x (only for the Bluetooth options).

  Setup over the Serial Monitor (115200, newline):
    set ssid <name> | set pass <pw> | set host <broker-ip> | reboot | cal | show | help
*/ 

// ---------------- Build options: pick one ----------------
#define HID_BLE        // Wi-Fi/MQTT + Bluetooth mouse/keyboard (default)
// (comment HID_BLE out for Wi-Fi/MQTT only: Wi-Fi power saving off, lowest latency)
// #define USB_BRIDGE  // samples as JSON lines on USB serial (tools/usb_bridge.py); comment HID_BLE out
// #define BLE_BRIDGE  // samples over a custom BLE characteristic (tools/ble_bridge.py); comment HID_BLE out

// Optional defaults for your bench; "set ssid ..." over Serial takes precedence.
#define DEFAULT_SSID ""
#define DEFAULT_PASS ""
#define DEFAULT_HOST ""

#include <Arduino.h>
#include <WiFi.h>
#include <Preferences.h>
#include <esp_task_wdt.h>
#include <espMqttClient.h>
#include <ArduinoJson.h>
#if defined(HID_BLE) || defined(BLE_BRIDGE)
#include <NimBLEDevice.h>
#endif
#if defined(HID_BLE)
#include <NimBLEHIDDevice.h>
#endif
#if defined(HID_BLE) && (defined(USB_BRIDGE) || defined(BLE_BRIDGE))
#error "HID_BLE needs the MQTT build: comment out HID_BLE to use USB_BRIDGE or BLE_BRIDGE"
#endif

#define FW_VERSION "1.1.0"
#if !defined(USB_BRIDGE) && !defined(BLE_BRIDGE)
#define MQTT_BUILD 1
#else
#define MQTT_BUILD 0
#endif

// ---------------- Hardware ----------------
static const uint8_t PIN_X  = 4;   // VRx (ADC1_CH4)
static const uint8_t PIN_Y  = 3;   // VRy (ADC1_CH3)
static const uint8_t PIN_SW = 2;   // SW  (strapping pin; the backend has no buttons, so not read)
static const uint8_t PIN_PS2_CLK = 5;
static const uint8_t PIN_PS2_DAT = 6;

// Flip these if the directions are wrong when checked with mosquitto_sub.
// Requirement: pushing RIGHT gives +x, pushing UP gives +y.
#define INVERT_X 0
#define INVERT_Y 1

// ---------------- Protocol tuning (docs/firmware.md) ----------------
#define SAMPLE_PERIOD_US   10000   // read the stick at 100 Hz
#define CHANGE_THRESHOLD   20      // publish when |dx| or |dy| >= 20
#define MIN_PUBLISH_MS     20      // 50 messages/s, except outward pushes and returns
#define HEARTBEAT_MS       1000    // publish at least once per second
#define NEAR_CENTER        50      // "back at rest" window
#define ADC_AVG            6       // ADC readings averaged per sample
#define KEEPALIVE_S        20
#define WDT_TIMEOUT_S      8

// ---------------- Settings (stored in NVS, set over Serial) ----------------
struct Settings {
  String ssid, pass, host, user, mqttPass;
  uint16_t port = 1883;
  int minX = 0, maxX = 4095, minY = 0, maxY = 4095;
  bool ble = true;        // Bluetooth HID on (HID_BLE builds)
  bool apple = false;     // HID host is an iPhone/iPad/Mac: "primary" = Cmd
  bool ps2 = false;       // PS/2 finger mouse attached
  int scrollDiv = 20;     // backend scroll units per wheel notch
} cfg;

Preferences prefs;

// Declared here, before any function: the Arduino IDE adds function
// prototypes at the top of the sketch, and they must see these types.
#ifdef HID_BLE
struct HidEvent { uint8_t kind, mods, code; uint16_t usage; };   // kind: 0 key, 1 consumer, 2 mouse buttons
// One message from the backend's "hid" topic, handed from the MQTT task to the loop.
struct HidMsg { uint16_t len; char buf[400]; };
#endif

static void loadSettings() {
  prefs.begin("actspeed", true);
  cfg.ssid      = prefs.getString("ssid", DEFAULT_SSID);
  cfg.pass      = prefs.getString("pass", DEFAULT_PASS);
  cfg.host      = prefs.getString("host", DEFAULT_HOST);
  cfg.port      = prefs.getUShort("port", 1883);
  cfg.user      = prefs.getString("user", "");
  cfg.mqttPass  = prefs.getString("mqttpass", "");
  cfg.minX      = prefs.getInt("minX", 0);
  cfg.maxX      = prefs.getInt("maxX", 4095);
  cfg.minY      = prefs.getInt("minY", 0);
  cfg.maxY      = prefs.getInt("maxY", 4095);
  cfg.ble       = prefs.getBool("ble", true);
  cfg.apple     = prefs.getString("hidos", "other") == "apple";
  cfg.ps2       = prefs.getBool("ps2", false);
  cfg.scrollDiv = max(1, (int)prefs.getInt("scrolldiv", 20));
  prefs.end();
#ifndef HID_BLE
  cfg.ble = cfg.ps2 = false;               // both need the Bluetooth HID build
#endif
}

static void saveString(const char* key, const String& v) {
  prefs.begin("actspeed", false);
  prefs.putString(key, v);
  prefs.end();
}

static bool parseOnOff(const String& v, bool& out) {
  if (v == "on" || v == "1" || v == "true")   { out = true;  return true; }
  if (v == "off" || v == "0" || v == "false") { out = false; return true; }
  return false;
}

// ---------------- State ----------------
static char deviceId[24];
static char topicRaw[96];
static char topicStatus[96];
static char topicHid[96];

espMqttClient mqtt;

static int centerX = 2048, centerY = 2048;
static uint32_t seqNo = 0;
static int lastX = 0, lastY = 0;
static uint32_t lastPubMs = 0;
static uint32_t nextSampleUs = 0;

static const uint32_t backoffMs[] = {1000, 2000, 5000, 10000};
static uint8_t backoffIdx = 0;
static uint32_t lastAttemptMs = 0;

static void publishStatus();

// ---------------- Joystick ----------------
static int readAxis(uint8_t pin) {
  long sum = 0;
  for (int i = 0; i < ADC_AVG; i++) sum += analogRead(pin);
  return (int)(sum / ADC_AVG);
}

static int scaleAxis(int raw, int centerRaw, int minRaw, int maxRaw) {
  long v;
  if (raw >= centerRaw) {
    int span = maxRaw - centerRaw;
    if (span <= 0) return 0;
    v = (long)(raw - centerRaw) * 1000 / span;
  } else {
    int span = centerRaw - minRaw;
    if (span <= 0) return 0;
    v = (long)(raw - centerRaw) * 1000 / span;
  }
  if (v < -1000) v = -1000;
  if (v > 1000) v = 1000;
  return (int)v;
}

static void measureCenter() {
  long sx = 0, sy = 0;
  const int n = 20;                       // ~200 ms at rest
  for (int i = 0; i < n; i++) {
    sx += readAxis(PIN_X);
    sy += readAxis(PIN_Y);
    esp_task_wdt_reset();
    delay(10);
  }
  centerX = sx / n;
  centerY = sy / n;
  Serial.printf("Center: x=%d y=%d\n", centerX, centerY);
}

static void runCalibration() {
  Serial.println("Calibration: move the stick to every edge, in full circles, for 6 s...");
  int mnX = 4095, mxX = 0, mnY = 4095, mxY = 0;
  uint32_t end = millis() + 6000;
  while (millis() < end) {
    int x = readAxis(PIN_X), y = readAxis(PIN_Y);
    mnX = min(mnX, x); mxX = max(mxX, x);
    mnY = min(mnY, y); mxY = max(mxY, y);
    esp_task_wdt_reset();
    delay(10);
  }
  if (mxX - mnX < 1500 || mxY - mnY < 1500) {
    Serial.printf("Range too small (x %d..%d, y %d..%d). Not saved.\n", mnX, mxX, mnY, mxY);
    return;
  }
  cfg.minX = mnX; cfg.maxX = mxX; cfg.minY = mnY; cfg.maxY = mxY;
  prefs.begin("actspeed", false);
  prefs.putInt("minX", mnX); prefs.putInt("maxX", mxX);
  prefs.putInt("minY", mnY); prefs.putInt("maxY", mxY);
  prefs.end();
  Serial.printf("Saved: x %d..%d, y %d..%d. Release the stick.\n", mnX, mxX, mnY, mxY);
  delay(800);
  measureCenter();
}

// ---------------- PS/2 finger mouse (optional, "set ps2 on") ----------------
// Device -> host frames: 1 start, 8 data, 1 parity, 1 stop, clocked by the mouse.
static volatile uint8_t ps2Q[64], ps2Head = 0, ps2Tail = 0, ps2Bits = 0, ps2Acc = 0, ps2Par = 0;
static volatile uint32_t ps2LastEdge = 0;
static uint8_t ps2Pkt[3], ps2N = 0;
static uint32_t ps2LastByte = 0, ps2LastTry = 0;
static bool ps2Ok = false;
static int ps2AccX = 0, ps2AccY = 0;
static uint8_t ps2Buttons = 0;

static void IRAM_ATTR ps2Isr() {
  uint32_t t = micros();
  if (t - ps2LastEdge > 1500) { ps2Bits = 0; ps2Acc = 0; }
  ps2LastEdge = t;
  uint8_t bit = digitalRead(PIN_PS2_DAT);
  if (ps2Bits == 0) { if (!bit) ps2Bits = 1; return; }
  if (ps2Bits <= 8) { ps2Acc |= bit << (ps2Bits - 1); ps2Bits++; return; }
  if (ps2Bits == 9) { ps2Par = bit; ps2Bits = 10; return; }
  ps2Bits = 0;
  if (bit && ((__builtin_popcount(ps2Acc) + ps2Par) & 1)) { ps2Q[ps2Head] = ps2Acc; ps2Head = (ps2Head + 1) & 63; }
}

static bool ps2WaitPin(int pin, int lvl) {
  uint32_t t = micros();
  while (digitalRead(pin) != lvl) if (micros() - t > 20000) return false;
  return true;
}

static bool ps2Send(uint8_t b) {           // host -> device
  detachInterrupt(PIN_PS2_CLK);
  pinMode(PIN_PS2_CLK, OUTPUT); digitalWrite(PIN_PS2_CLK, LOW); delayMicroseconds(200);
  pinMode(PIN_PS2_DAT, OUTPUT); digitalWrite(PIN_PS2_DAT, LOW);
  pinMode(PIN_PS2_CLK, INPUT_PULLUP);
  bool ok = ps2WaitPin(PIN_PS2_CLK, LOW);
  uint8_t par = 1;
  for (int i = 0; ok && i < 9; i++) {
    uint8_t bit = i < 8 ? (b >> i) & 1 : par;
    if (i < 8) par ^= bit;
    if (bit) pinMode(PIN_PS2_DAT, INPUT_PULLUP); else { pinMode(PIN_PS2_DAT, OUTPUT); digitalWrite(PIN_PS2_DAT, LOW); }   // open-drain: never drive high
    ok = ps2WaitPin(PIN_PS2_CLK, HIGH) && ps2WaitPin(PIN_PS2_CLK, LOW);
  }
  pinMode(PIN_PS2_DAT, INPUT_PULLUP);
  ok = ok && ps2WaitPin(PIN_PS2_DAT, LOW) && ps2WaitPin(PIN_PS2_CLK, LOW) &&
       ps2WaitPin(PIN_PS2_DAT, HIGH) && ps2WaitPin(PIN_PS2_CLK, HIGH);
  ps2Bits = 0;
  attachInterrupt(PIN_PS2_CLK, ps2Isr, FALLING);
  return ok;
}

static bool ps2WaitAck() {
  uint32_t t = millis();
  while (millis() - t < 100) {
    while (ps2Tail != ps2Head) { uint8_t b = ps2Q[ps2Tail]; ps2Tail = (ps2Tail + 1) & 63; if (b == 0xFA) return true; }
    delay(1);
  }
  return false;
}

static void ps2Enable() { ps2Ok = ps2Send(0xF4) && ps2WaitAck(); }   // F4 = start streaming

static void ps2Begin() {
  pinMode(PIN_PS2_CLK, INPUT_PULLUP);
  pinMode(PIN_PS2_DAT, INPUT_PULLUP);
  attachInterrupt(PIN_PS2_CLK, ps2Isr, FALLING);
  delay(700);                              // let the mouse finish its power-on test
  ps2Tail = ps2Head;
  ps2Enable();
  Serial.printf("PS/2 mouse: %s\n", ps2Ok ? "ok" : "not answering (retrying)");
}

static void ps2Poll() {
  if (!ps2Ok && millis() - ps2LastTry > 3000) { ps2LastTry = millis(); ps2Enable(); }
  while (ps2Tail != ps2Head) {
    uint8_t b = ps2Q[ps2Tail]; ps2Tail = (ps2Tail + 1) & 63;
    uint32_t now = millis();
    if (now - ps2LastByte > 15) ps2N = 0;
    ps2LastByte = now;
    if (ps2N == 0 && (b == 0xFA || !(b & 0x08))) continue;   // resync on the always-1 bit
    ps2Pkt[ps2N++] = b;
    if (ps2N < 3) continue;
    ps2N = 0; ps2Ok = true;
    ps2Buttons = ps2Pkt[0] & 0x07;
    if (ps2Pkt[0] & 0xC0) continue;                           // overflow
    ps2AccX += ps2Pkt[1] - ((ps2Pkt[0] << 4) & 0x100);
    ps2AccY -= ps2Pkt[2] - ((ps2Pkt[0] << 3) & 0x100);        // PS/2 up is positive
  }
}

// ---------------- Bluetooth HID (HID_BLE) ----------------
#ifdef HID_BLE
// Report 1: keyboard (mods, reserved, 6 keys). Report 2: mouse (buttons,
// x, y, wheel). Report 3: consumer control (one 16-bit usage).
static const uint8_t HID_REPORT_MAP[] = {
  0x05, 0x01, 0x09, 0x06, 0xA1, 0x01, 0x85, 0x01,             // Keyboard, report 1
  0x05, 0x07, 0x19, 0xE0, 0x29, 0xE7, 0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x08, 0x81, 0x02,
  0x95, 0x01, 0x75, 0x08, 0x81, 0x01,
  0x95, 0x06, 0x75, 0x08, 0x15, 0x00, 0x25, 0x65, 0x05, 0x07, 0x19, 0x00, 0x29, 0x65, 0x81, 0x00,
  0xC0,
  0x05, 0x01, 0x09, 0x02, 0xA1, 0x01, 0x85, 0x02, 0x09, 0x01, 0xA1, 0x00,   // Mouse, report 2
  0x05, 0x09, 0x19, 0x01, 0x29, 0x03, 0x15, 0x00, 0x25, 0x01, 0x95, 0x03, 0x75, 0x01, 0x81, 0x02,
  0x95, 0x01, 0x75, 0x05, 0x81, 0x03,
  0x05, 0x01, 0x09, 0x30, 0x09, 0x31, 0x09, 0x38, 0x15, 0x81, 0x25, 0x7F, 0x75, 0x08, 0x95, 0x03, 0x81, 0x06,
  0xC0, 0xC0,
  0x05, 0x0C, 0x09, 0x01, 0xA1, 0x01, 0x85, 0x03,             // Consumer control, report 3
  0x15, 0x00, 0x26, 0xFF, 0x03, 0x19, 0x00, 0x2A, 0xFF, 0x03, 0x75, 0x10, 0x95, 0x01, 0x81, 0x00,
  0xC0,
};

enum : uint8_t { MOD_CTRL = 0x01, MOD_SHIFT = 0x02, MOD_ALT = 0x04, MOD_GUI = 0x08 };

static NimBLEServer* hidServer = nullptr;
static NimBLECharacteristic *hidKeys = nullptr, *hidMouse = nullptr, *hidConsumer = nullptr;
static volatile bool hidConnected = false;
static volatile bool hidStateChanged = false;
static uint8_t mouseButtons = 0;   // held buttons (drag lock)
static int scrollRem = 0;

class HidServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer*, NimBLEConnInfo&) override { hidConnected = true; hidStateChanged = true; }
  void onDisconnect(NimBLEServer*, NimBLEConnInfo&, int) override {
    hidConnected = false; hidStateChanged = true;
    NimBLEDevice::startAdvertising();
  }
};

static void setupHid() {
  String name = String("ActSpeed ") + (deviceId + 6);   // e.g. "ActSpeed 4b9978"
  NimBLEDevice::init(name.c_str());
  NimBLEDevice::setSecurityAuth(true, false, true);       // bonding, no MITM, secure connections
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);
  hidServer = NimBLEDevice::createServer();
  hidServer->setCallbacks(new HidServerCallbacks());
  NimBLEHIDDevice* hid = new NimBLEHIDDevice(hidServer);
  hid->setManufacturer("ActSpeed");
  hid->setPnp(0x02, 0xe502, 0xa111, 0x0110);
  hid->setHidInfo(0x00, 0x01);
  hid->setReportMap((uint8_t*)HID_REPORT_MAP, sizeof(HID_REPORT_MAP));
  hidKeys = hid->getInputReport(1);
  hidMouse = hid->getInputReport(2);
  hidConsumer = hid->getInputReport(3);
  hid->startServices();
  hid->setBatteryLevel(100);
  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->setAppearance(0x03C0);                             // generic HID
  adv->addServiceUUID(hid->getHidService()->getUUID());
  adv->enableScanResponse(true);
  adv->setName(name.c_str());
  adv->start();
  Serial.printf("Bluetooth HID advertising as \"%s\"\n", name.c_str());
}

static const char* hidState() {
  if (!cfg.ble || !hidServer) return "off";
  return hidConnected ? "connected" : "advertising";
}

static void maintainHid() {
  static uint32_t last = 0;
  if (!hidServer || millis() - last < 1000) return;
  last = millis();
  if (!hidConnected && !NimBLEDevice::getAdvertising()->isAdvertising()) NimBLEDevice::startAdvertising();
}

static void mouseReport(uint8_t buttons, int8_t dx, int8_t dy, int8_t wheel) {
  if (!hidConnected) return;
  uint8_t r[4] = { buttons, (uint8_t)dx, (uint8_t)dy, (uint8_t)wheel };
  hidMouse->setValue(r, sizeof(r));
  hidMouse->notify();
}

static void mouseMove(int dx, int dy, int wheel = 0) {
  while (dx || dy || wheel) {
    int sx = constrain(dx, -127, 127), sy = constrain(dy, -127, 127), sw = constrain(wheel, -127, 127);
    mouseReport(mouseButtons, sx, sy, sw);
    dx -= sx; dy -= sy; wheel -= sw;
  }
}

// Keystrokes and clicks are queued and sent one report per loop pass, at
// least 8 ms apart, so a long text does not overflow the BLE stack.
static HidEvent hidQ[256];
static uint16_t hidHead = 0, hidTail = 0;
static uint32_t hidLastSend = 0;

static void hidPush(HidEvent e) {
  uint16_t next = (hidHead + 1) % 256;
  if (next == hidTail) return;              // full: drop
  hidQ[hidHead] = e; hidHead = next;
}
static void queueKey(uint8_t mods, uint8_t code) {
  hidPush({0, mods, code, 0});
  hidPush({0, 0, 0, 0});                    // release
}
static void queueConsumer(uint16_t usage) {
  hidPush({1, 0, 0, usage});
  hidPush({1, 0, 0, 0});
}
static void queueClick(uint8_t button) {
  hidPush({2, 0, (uint8_t)(mouseButtons | button), 0});
  hidPush({2, 0, mouseButtons, 0});
}

static void pumpHid() {
  if (hidTail == hidHead || !hidConnected || millis() - hidLastSend < 8) return;
  hidLastSend = millis();
  HidEvent e = hidQ[hidTail]; hidTail = (hidTail + 1) % 256;
  if (e.kind == 0) {
    uint8_t r[8] = { e.mods, 0, e.code, 0, 0, 0, 0, 0 };
    hidKeys->setValue(r, sizeof(r)); hidKeys->notify();
  } else if (e.kind == 1) {
    uint8_t r[2] = { (uint8_t)e.usage, (uint8_t)(e.usage >> 8) };
    hidConsumer->setValue(r, sizeof(r)); hidConsumer->notify();
  } else {
    mouseReport(e.code, 0, 0, 0);
  }
}

// US layout: the key and shift state for a printable ASCII character.
static bool asciiKey(char c, uint8_t& code, uint8_t& mods) {
  mods = 0;
  if (c >= 'a' && c <= 'z') { code = 0x04 + (c - 'a'); return true; }
  if (c >= 'A' && c <= 'Z') { code = 0x04 + (c - 'A'); mods = MOD_SHIFT; return true; }
  if (c >= '1' && c <= '9') { code = 0x1E + (c - '1'); return true; }
  if (c == '0') { code = 0x27; return true; }
  static const char* plain   = " -=[]\\;'`,./\n\t";
  static const uint8_t plainCodes[] = {0x2C, 0x2D, 0x2E, 0x2F, 0x30, 0x31, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x28, 0x2B};
  static const char* shifted = "!@#$%^&*()_+{}|:\"~<>?";
  static const uint8_t shiftedCodes[] = {0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27,
                                         0x2D, 0x2E, 0x2F, 0x30, 0x31, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38};
  if (const char* p = strchr(plain, c))   { code = plainCodes[p - plain]; return true; }
  if (const char* p = strchr(shifted, c)) { code = shiftedCodes[p - shifted]; mods = MOD_SHIFT; return true; }
  return false;
}

// Key names as in the action catalogue (KeyboardEvent.code style).
static uint8_t usageCode(const char* u) {
  struct { const char* name; uint8_t code; } table[] = {
    {"ArrowUp", 0x52}, {"ArrowDown", 0x51}, {"ArrowLeft", 0x50}, {"ArrowRight", 0x4F},
    {"Enter", 0x28}, {"Escape", 0x29}, {"Tab", 0x2B}, {"Backspace", 0x2A}, {"Space", 0x2C},
    {"Delete", 0x4C}, {"Home", 0x4A}, {"End", 0x4D}, {"PageUp", 0x4B}, {"PageDown", 0x4E},
    {"BracketLeft", 0x2F}, {"BracketRight", 0x30},
  };
  for (auto& e : table) if (!strcmp(u, e.name)) return e.code;
  if (!strncmp(u, "Key", 3) && u[3] >= 'A' && u[3] <= 'Z' && !u[4]) return 0x04 + (u[3] - 'A');
  if (!strncmp(u, "Digit", 5) && u[5] >= '0' && u[5] <= '9' && !u[6]) return u[5] == '0' ? 0x27 : 0x1E + (u[5] - '1');
  if (u[0] == 'F' && isdigit(u[1])) { int n = atoi(u + 1); if (n >= 1 && n <= 12) return 0x3A + n - 1; }
  return 0;
}

static uint8_t modBits(JsonVariantConst mods) {
  uint8_t m = 0;
  for (JsonVariantConst v : mods.as<JsonArrayConst>()) {
    const char* s = v | "";
    if (!strcmp(s, "primary")) m |= cfg.apple ? MOD_GUI : MOD_CTRL;
    else if (!strcmp(s, "ctrl") || !strcmp(s, "control")) m |= MOD_CTRL;
    else if (!strcmp(s, "shift")) m |= MOD_SHIFT;
    else if (!strcmp(s, "alt") || !strcmp(s, "option")) m |= MOD_ALT;
    else if (!strcmp(s, "meta") || !strcmp(s, "gui") || !strcmp(s, "cmd")) m |= MOD_GUI;
  }
  return m;
}

// Carries out one resolved action from the backend ({"key","params"}).
static void runHidAction(JsonDocument& doc) {
  const char* key = doc["key"] | "";
  JsonVariantConst p = doc["params"];
  if (!strcmp(key, "pointer.move")) { mouseMove(p["dx"] | 0, p["dy"] | 0); return; }
  if (!strcmp(key, "scroll")) {
    scrollRem -= (int)(p["dy"] | 0);                         // screen down = wheel down
    int notches = scrollRem / cfg.scrollDiv;
    scrollRem -= notches * cfg.scrollDiv;
    if (notches) mouseMove(0, 0, notches);
    return;
  }
  if (!strcmp(key, "pointer.click"))  { queueClick(1); return; }
  if (!strcmp(key, "pointer.double")) { queueClick(1); queueClick(1); return; }
  if (!strcmp(key, "pointer.right"))  { queueClick(2); return; }
  if (!strcmp(key, "pointer.drag_lock")) {
    mouseButtons ^= 1;
    hidPush({2, 0, mouseButtons, 0});
    return;
  }
  if (!strcmp(key, "text.type")) {
    const char* text = p["text"] | "";
    uint8_t code, mods;
    for (const char* c = text; *c; c++) if (asciiKey(*c, code, mods)) queueKey(mods, code);
    return;
  }
  if (!strncmp(key, "key.", 4)) {
    uint8_t code = usageCode(p["usage"] | "");
    uint8_t mods = modBits(p["mods"]);
    if (!strcmp(key, "key.combo.switch_window") && cfg.apple) mods = MOD_GUI;   // Cmd+Tab
    if (code) queueKey(mods, code);
    return;
  }
  if (!strcmp(key, "scan.next"))   { queueKey(0, 0x2B); return; }               // Tab
  if (!strcmp(key, "scan.prev"))   { queueKey(MOD_SHIFT, 0x2B); return; }       // Shift+Tab
  if (!strcmp(key, "scan.select")) { queueKey(0, 0x2C); return; }               // Space
  if (!strcmp(key, "system.home")) { if (cfg.apple) queueKey(MOD_GUI, 0x0B); else queueConsumer(0x223); return; }  // Cmd+H / AC Home
  if (!strcmp(key, "system.back")) { if (cfg.apple) queueKey(MOD_GUI, 0x2F); else queueConsumer(0x224); return; }  // Cmd+[ / AC Back
  if (!strcmp(key, "system.app_switch")) { queueKey(cfg.apple ? MOD_GUI : MOD_ALT, 0x2B); return; }
  struct { const char* name; uint16_t usage; } media[] = {
    {"system.volume_up", 0xE9}, {"system.volume_down", 0xEA}, {"system.mute", 0xE2},
    {"system.media_play_pause", 0xCD}, {"system.media_next", 0xB5}, {"system.media_previous", 0xB6},
  };
  for (auto& m : media) if (!strcmp(key, m.name)) { queueConsumer(m.usage); return; }
  // screen.* and anything else need a client that can see the screen.
}

// MQTT messages arrive on espMqttClient's own task; they are handed to the
// loop through this queue so all BLE calls happen in one place.
static QueueHandle_t hidMsgs = nullptr;

static void onMqttMessage(const espMqttClientTypes::MessageProperties&, const char* topic,
                          const uint8_t* payload, size_t len, size_t index, size_t total) {
  static HidMsg m;
  if (strcmp(topic, topicHid) != 0 || total >= sizeof(m.buf)) return;
  memcpy(m.buf + index, payload, len);
  if (index + len < total) return;           // more chunks to come
  m.len = total;
  m.buf[total] = 0;
  xQueueSend(hidMsgs, &m, 0);                // drop if the loop is far behind
}

static void handleHidMessages() {
  static HidMsg m;
  while (hidMsgs && xQueueReceive(hidMsgs, &m, 0) == pdTRUE) {
    JsonDocument doc;
    if (deserializeJson(doc, m.buf, m.len)) continue;
    runHidAction(doc);
  }
}

// The finger mouse drives the Bluetooth mouse directly: no Wi-Fi round trip.
static void ps2ToHid() {
  static uint8_t lastButtons = 0;
  if (ps2AccX || ps2AccY) { mouseMove(ps2AccX, ps2AccY); ps2AccX = ps2AccY = 0; }
  if (ps2Buttons != lastButtons) {
    lastButtons = ps2Buttons;
    mouseReport(mouseButtons | ps2Buttons, 0, 0, 0);   // PS/2 and HID share the L/R/M bit order
  }
}
#else
static const char* hidState() { return "off"; }
#endif

// ---------------- Serial console ----------------
static void showConfig() {
  Serial.println("---- ActSpeed " FW_VERSION " ----");
  Serial.printf("device id : %s\n", deviceId);
  Serial.printf("wifi ssid : %s (%s)\n", cfg.ssid.c_str(), WiFi.status() == WL_CONNECTED ? "connected" : "not connected");
  Serial.printf("ip        : %s  rssi %d dBm  power save %s\n", WiFi.localIP().toString().c_str(), WiFi.RSSI(),
                WiFi.getSleep() ? "on" : "off");
  Serial.printf("broker    : %s:%u (%s)\n", cfg.host.c_str(), cfg.port, mqtt.connected() ? "connected" : "not connected");
  Serial.printf("mqtt user : %s\n", cfg.user.c_str());
  Serial.printf("cal       : x %d..%d  y %d..%d  center %d,%d\n", cfg.minX, cfg.maxX, cfg.minY, cfg.maxY, centerX, centerY);
#ifdef HID_BLE
  Serial.printf("bluetooth : %s (host %s, scroll %d)\n", hidState(), cfg.apple ? "apple" : "other", cfg.scrollDiv);
#endif
  Serial.printf("ps2 mouse : %s\n", cfg.ps2 ? (ps2Ok ? "on, ok" : "on, not answering") : "off");
}

static void handleCommand(String line) {
  line.trim();
  if (!line.length()) return;
  int sp = line.indexOf(' ');
  String cmd = sp < 0 ? line : line.substring(0, sp);
  String rest = sp < 0 ? "" : line.substring(sp + 1);
  rest.trim();

  if (cmd == "show") { showConfig(); return; }
  if (cmd == "cal")  { runCalibration(); return; }
  if (cmd == "reboot") { ESP.restart(); return; }
  if (cmd == "defaults") {                       // forget saved Wi-Fi/broker settings, use DEFAULT_* above
    prefs.begin("actspeed", false);
    prefs.remove("ssid"); prefs.remove("pass"); prefs.remove("host");
    prefs.remove("port"); prefs.remove("user"); prefs.remove("mqttpass");
    prefs.end();
    Serial.println("Saved Wi-Fi/broker settings cleared. Type 'reboot'.");
    return;
  }
  if (cmd == "set") {
    int sp2 = rest.indexOf(' ');
    if (sp2 < 0) { Serial.println("usage: set <key> <value>   (type 'help')"); return; }
    String key = rest.substring(0, sp2);
    String val = rest.substring(sp2 + 1);
    val.trim();
    bool b;
    prefs.begin("actspeed", false);
    bool ok = true;
    if (key == "ssid" || key == "pass" || key == "host" || key == "user" || key == "mqttpass") prefs.putString(key.c_str(), val);
    else if (key == "port") prefs.putUShort("port", (uint16_t)val.toInt());
    else if (key == "ble" && parseOnOff(val, b)) prefs.putBool("ble", b);
    else if (key == "ps2" && parseOnOff(val, b)) prefs.putBool("ps2", b);
    else if (key == "hidos" && (val == "apple" || val == "other")) prefs.putString("hidos", val);
    else if (key == "scroll" && val.toInt() > 0) prefs.putInt("scrolldiv", val.toInt());
    else ok = false;
    prefs.end();
    Serial.println(ok ? "saved. Type 'reboot' to apply." : "unknown key or bad value (type 'help')");
    return;
  }
  Serial.println("commands: show | cal | defaults | reboot | set <key> <value>");
  Serial.println("  keys: ssid pass host port user mqttpass");
  Serial.println("        ble on|off      Bluetooth mouse/keyboard (off = lowest Wi-Fi latency)");
  Serial.println("        hidos apple|other  Cmd or Ctrl for shortcuts, Home/Back keys");
  Serial.println("        scroll <n>      scroll units per wheel notch (default 20)");
  Serial.println("        ps2 on|off      PS/2 finger mouse on GPIO5 (clock) / GPIO6 (data)");
}

static void pollSerial() {
  static String buf;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (buf.length()) handleCommand(buf);
      buf = "";
    } else if ((c == 8 || c == 127) && buf.length()) {
      buf.remove(buf.length() - 1);
    } else if (c >= 32 && buf.length() < 160) {
      buf += c;
    }
  }
}

static const char* wifiStatusName(wl_status_t st) {
  switch (st) {
    case WL_CONNECTED:       return "connected";
    case WL_NO_SSID_AVAIL:   return "network not found (wrong SSID, or 5 GHz / out of range)";
    case WL_CONNECT_FAILED:  return "connect failed (wrong password?)";
    case WL_DISCONNECTED:    return "disconnected / still trying";
    case WL_IDLE_STATUS:     return "idle";
    default:                 return "other";
  }
}

static void reportWifi() {
  static uint32_t last = 0;
  if (!cfg.ssid.length() || WiFi.status() == WL_CONNECTED) return;
  if (millis() - last < 5000) return;
  last = millis();
  Serial.printf("WiFi: %s (code %d) ssid='%s'\n", wifiStatusName(WiFi.status()), (int)WiFi.status(), cfg.ssid.c_str());
}

// ---------------- MQTT ----------------
static char willPayload[96];

static void publishStatus() {
  if (!mqtt.connected()) return;
  char payload[128];
#ifdef HID_BLE
  snprintf(payload, sizeof(payload), "{\"status\":\"online\",\"fw\":\"" FW_VERSION "\",\"hid\":true,\"ble\":\"%s\"}", hidState());
#else
  snprintf(payload, sizeof(payload), "{\"status\":\"online\",\"fw\":\"" FW_VERSION "\"}");
#endif
  mqtt.publish(topicStatus, 1, true, payload);
}

static void setupMqtt() {
  snprintf(topicRaw,    sizeof(topicRaw),    "actspeed/v1/devices/%s/raw", deviceId);
  snprintf(topicStatus, sizeof(topicStatus), "actspeed/v1/devices/%s/status", deviceId);
  snprintf(topicHid,    sizeof(topicHid),    "actspeed/v1/devices/%s/hid", deviceId);

  mqtt.setServer(cfg.host.c_str(), cfg.port);   // pointers must stay valid: cfg is global
  mqtt.setClientId(deviceId);
  mqtt.setKeepAlive(KEEPALIVE_S);
  mqtt.setCleanSession(true);
  if (cfg.user.length()) mqtt.setCredentials(cfg.user.c_str(), cfg.mqttPass.c_str());
  // Last Will: retained, QoS 1, "offline"
  snprintf(willPayload, sizeof(willPayload), "{\"status\":\"offline\",\"fw\":\"" FW_VERSION "\"}");
  mqtt.setWill(topicStatus, 1, true, willPayload);

  mqtt.onConnect([](bool) {
    Serial.println("MQTT connected");
    backoffIdx = 0;
    publishStatus();
#ifdef HID_BLE
    if (cfg.ble) mqtt.subscribe(topicHid, 1);
#endif
  });
  mqtt.onDisconnect([](espMqttClientTypes::DisconnectReason r) {
    Serial.printf("MQTT disconnected (%d)\n", (int)r);
  });
#ifdef HID_BLE
  mqtt.onMessage(onMqttMessage);
#endif
}

static void maintainConnection() {
  if (WiFi.status() != WL_CONNECTED || !cfg.host.length()) return;
  if (mqtt.connected()) return;
  if (millis() - lastAttemptMs < backoffMs[backoffIdx]) return;
  lastAttemptMs = millis();
  mqtt.connect();
  if (backoffIdx < 3) backoffIdx++;
}

// ---------------- BLE sample bridge (BLE_BRIDGE mode) ----------------
#ifdef BLE_BRIDGE
// Must match tools/ble_bridge.py
#define BLE_SERVICE_UUID "a5a50001-7c3d-4e5b-9b0a-5ac75bee0001"
#define BLE_CHAR_UUID    "a5a50002-7c3d-4e5b-9b0a-5ac75bee0001"

static NimBLEServer* bleServer = nullptr;
static NimBLECharacteristic* bleChar = nullptr;

static void setupBle() {
  String name = String("ActSpeed-") + (deviceId + 6);   // e.g. ActSpeed-4b9978
  NimBLEDevice::init(name.c_str());
  bleServer = NimBLEDevice::createServer();
  NimBLEService* svc = bleServer->createService(BLE_SERVICE_UUID);
  bleChar = svc->createCharacteristic(BLE_CHAR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  svc->start();
  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(BLE_SERVICE_UUID);
  adv->start();
  Serial.printf("BLE advertising as %s\n", name.c_str());
}

static bool bleConnected() { return bleServer && bleServer->getConnectedCount() > 0; }

static void maintainBle() {
  static uint32_t last = 0;
  if (millis() - last < 1000) return;
  last = millis();
  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  if (!bleConnected() && !adv->isAdvertising()) adv->start();
}

// 8 bytes, little endian: seq u32, x i16, y i16
static void bleSend(uint32_t seq, int x, int y) {
  uint8_t b[8];
  b[0] = seq; b[1] = seq >> 8; b[2] = seq >> 16; b[3] = seq >> 24;
  int16_t xi = x, yi = y;
  b[4] = (uint8_t)xi; b[5] = (uint8_t)(xi >> 8);
  b[6] = (uint8_t)yi; b[7] = (uint8_t)(yi >> 8);
  bleChar->setValue(b, sizeof(b));
  bleChar->notify();
}
#endif

// ---------------- Setup / loop ----------------
void setup() {
  Serial.begin(115200);
#if ARDUINO_USB_CDC_ON_BOOT
  Serial.setTxTimeoutMs(0);                // never block when no PC is listening
#endif

  // Watchdog: restart by itself if the loop ever hangs.
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  esp_task_wdt_config_t wdt = { .timeout_ms = WDT_TIMEOUT_S * 1000, .idle_core_mask = 0, .trigger_panic = true };
  esp_task_wdt_reconfigure(&wdt);
#else
  esp_task_wdt_init(WDT_TIMEOUT_S, true);
#endif
  esp_task_wdt_add(NULL);

  loadSettings();

  uint64_t mac = ESP.getEfuseMac();       // last 3 bytes of the base MAC
  snprintf(deviceId, sizeof(deviceId), "esp32-%02x%02x%02x",
           (uint8_t)(mac >> 24), (uint8_t)(mac >> 32), (uint8_t)(mac >> 40));

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_X, ADC_11db);
  analogSetPinAttenuation(PIN_Y, ADC_11db);
  delay(300);                              // let the stick settle
  measureCenter();
  if (cfg.ps2) ps2Begin();

#if MQTT_BUILD
  setupMqtt();

  WiFi.mode(WIFI_STA);
  WiFi.onEvent([](arduino_event_id_t event, arduino_event_info_t info) {
    if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED)
      Serial.printf("WiFi disconnect reason: %d (2/15/202=bad password, 201=SSID not found, 205=join failed)\n",
                    (int)info.wifi_sta_disconnected.reason);
  });
  WiFi.setTxPower(WIFI_POWER_8_5dBm);   // many C3 Super Minis fail to join at full power
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
#ifdef HID_BLE
  bool radioShared = cfg.ble;
#else
  bool radioShared = false;
#endif
  // Power saving holds packets in the radio for 100-300 ms and releases them
  // in bursts. Turn it off unless Bluetooth shares the radio, which needs it;
  // the backend then undoes the bursts using each sample's "t".
  WiFi.setSleep(radioShared);
  if (cfg.ssid.length()) WiFi.begin(cfg.ssid.c_str(), cfg.pass.c_str());
#endif

#ifdef HID_BLE
  if (cfg.ble) {
    hidMsgs = xQueueCreate(16, sizeof(HidMsg));
    setupHid();
  }
#endif
#ifdef BLE_BRIDGE
  setupBle();
#endif

  showConfig();
#if MQTT_BUILD
  if (!cfg.ssid.length() || !cfg.host.length())
    Serial.println("Not configured. Use: set ssid ..., set pass ..., set host <broker-ip>, then reboot.");
#endif

  nextSampleUs = micros();
}

void loop() {
  esp_task_wdt_reset();
  pollSerial();
  if (cfg.ps2) ps2Poll();
#if MQTT_BUILD
  maintainConnection();
  reportWifi();
#endif
#ifdef HID_BLE
  if (cfg.ble) {
    maintainHid();
    handleHidMessages();
    if (cfg.ps2) ps2ToHid();
    pumpHid();
    if (hidStateChanged) { hidStateChanged = false; publishStatus(); }
  }
#endif
#ifdef BLE_BRIDGE
  maintainBle();
#endif

  uint32_t nowUs = micros();
  if ((int32_t)(nowUs - nextSampleUs) < 0) { delay(1); return; }
  nextSampleUs += SAMPLE_PERIOD_US;
  if ((int32_t)(nowUs - nextSampleUs) > 5 * SAMPLE_PERIOD_US) nextSampleUs = nowUs;  // fell behind: resync

  uint32_t now = millis();                 // when this sample was taken: sent as "t"
  int x = scaleAxis(readAxis(PIN_X), centerX, cfg.minX, cfg.maxX);
  int y = scaleAxis(readAxis(PIN_Y), centerY, cfg.minY, cfg.maxY);
  if (INVERT_X) x = -x;
  if (INVERT_Y) y = -y;

#if MQTT_BUILD
  if (!mqtt.connected()) return;           // no buffering while offline: old positions are useless
#endif
#ifdef BLE_BRIDGE
  if (!bleConnected()) return;             // nobody listening: drop the sample
#endif

  auto nearCenter = [](int a, int b) { return abs(a) < NEAR_CENTER && abs(b) < NEAR_CENTER; };
  bool changed   = abs(x - lastX) >= CHANGE_THRESHOLD || abs(y - lastY) >= CHANGE_THRESHOLD;
  bool returned  = nearCenter(x, y) && !nearCenter(lastX, lastY);
  bool heartbeat = now - lastPubMs >= HEARTBEAT_MS;
  // A quick flick lasts 30-40 ms. Rate-limiting it can drop the sample with
  // the deepest push, so pushes further out than the last one sent and
  // returns to centre always go out.
  bool outward   = max(abs(x), abs(y)) >= max(abs(lastX), abs(lastY)) + CHANGE_THRESHOLD;
  bool due       = now - lastPubMs >= MIN_PUBLISH_MS;

  if ((changed && (due || outward)) || returned || heartbeat) {
    uint32_t seq = seqNo++;
#if defined(BLE_BRIDGE)
    bleSend(seq, x, y);                    // tools/ble_bridge.py forwards this to MQTT
#else
    char payload[80];
    snprintf(payload, sizeof(payload), "{\"seq\":%lu,\"t\":%lu,\"x\":%d,\"y\":%d}",
             (unsigned long)seq, (unsigned long)now, x, y);
#if defined(USB_BRIDGE)
    Serial.println(payload);               // tools/usb_bridge.py forwards this line to MQTT
#else
    mqtt.publish(topicRaw, 0, false, payload);
#endif
#endif
    lastX = x; lastY = y; lastPubMs = now;
  }
}
