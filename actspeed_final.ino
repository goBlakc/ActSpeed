/*
  JoyLink - ESP32-C3 Super Mini joystick bridge
  Wiring: SW -> GPIO2 | VRy -> GPIO3 | VRx -> GPIO4 | joystick VCC -> 3V3 (NOT 5V) | GND -> GND

  Streams "J,x,y,sw,mouseOk" and "M,dx,dy" ~50x/second over USB serial (Cable) and Wi-Fi (http://<address>/events).
  Wi-Fi is set from the HTML page:
    - over the cable:  page sends  W,<ssid>,<pass>   (url-encoded)
    - or via the fallback network "JoyLink" (only active when no saved network works)
  The last network is saved in flash and rejoined at boot. Address: joylink.local (or the IP).

  Arduino IDE: board "ESP32C3 Dev Module", Tools > "USB CDC On Boot: Enabled". No extra libraries.
*/
#include <WiFi.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEHIDDevice.h>

#define SW_PIN  2
#define VRY_PIN 3
#define VRX_PIN 4

const char* AP_SSID = "JoyLink";
const char* AP_PASS = "joystick123";

Preferences prefs;
WiFiServer server(80);
WiFiClient sse;
String inBuf;
unsigned long lastSend = 0, lastIp = 0;

// ---------- Finger mouse (PS/2): C = clock -> GPIO5, D = data -> GPIO6. Power it from 3V3! ----------
#define PS2_CLK 5
#define PS2_DAT 6
#define LED_PIN 8                          // on-board LED (lit = laptop/Bluetooth mode)

// Laptop (Bluetooth) mouse tuning. Find the right Rotate/Mirror in the web page first, then copy them here.
#define MOUSE_ROT 0                        // 0..3 = rotate 0/90/180/270 degrees
#define MOUSE_MIRROR 0                     // 1 = mirror left/right
#define MOUSE_GAIN 1.0f                    // cursor speed
#define STICK_INVERT_Y 0                   // 1 if stick Up scrolls the wrong way
#define THR 1100                           // stick push needed (raw counts, bigger = less sensitive)
volatile uint8_t q[64], qh = 0, qt = 0, ps2N = 0, ps2Acc = 0, ps2Par = 0;
volatile uint32_t ps2Last = 0;
uint8_t pk[3], pn = 0;
unsigned long lastB = 0, lastTry = 0;
int accX = 0, accY = 0;
bool ps2Ok = false;

void IRAM_ATTR onClk() {                   // device -> host: 1 start, 8 data, 1 parity, 1 stop
  uint32_t t = micros();
  if (t - ps2Last > 1500) { ps2N = 0; ps2Acc = 0; }
  ps2Last = t;
  uint8_t bit = digitalRead(PS2_DAT);
  if (ps2N == 0) { if (!bit) ps2N = 1; return; }
  if (ps2N <= 8) { ps2Acc |= bit << (ps2N - 1); ps2N++; return; }
  if (ps2N == 9) { ps2Par = bit; ps2N = 10; return; }
  ps2N = 0;
  if (bit && ((__builtin_popcount(ps2Acc) + ps2Par) & 1)) { q[qh] = ps2Acc; qh = (qh + 1) & 63; }
}

bool waitPin(int pin, int lvl) {
  uint32_t t = micros();
  while (digitalRead(pin) != lvl) if (micros() - t > 20000) return false;
  return true;
}

bool ps2Send(uint8_t b) {                  // host -> device
  detachInterrupt(PS2_CLK);
  pinMode(PS2_CLK, OUTPUT); digitalWrite(PS2_CLK, LOW); delayMicroseconds(200);
  pinMode(PS2_DAT, OUTPUT); digitalWrite(PS2_DAT, LOW);
  pinMode(PS2_CLK, INPUT_PULLUP);
  bool ok = waitPin(PS2_CLK, LOW);
  uint8_t par = 1;
  for (int i = 0; ok && i < 9; i++) {
    uint8_t bit = i < 8 ? (b >> i) & 1 : par;
    if (i < 8) par ^= bit;
    digitalWrite(PS2_DAT, bit);
    ok = waitPin(PS2_CLK, HIGH) && waitPin(PS2_CLK, LOW);
  }
  pinMode(PS2_DAT, INPUT_PULLUP);
  ok = ok && waitPin(PS2_DAT, LOW) && waitPin(PS2_CLK, LOW) && waitPin(PS2_DAT, HIGH) && waitPin(PS2_CLK, HIGH);
  pinMode(PS2_CLK, INPUT_PULLUP);
  ps2N = 0;
  attachInterrupt(PS2_CLK, onClk, FALLING);
  return ok;
}

bool ps2WaitAck() {
  unsigned long t = millis();
  while (millis() - t < 100) {
    while (qt != qh) { uint8_t b = q[qt]; qt = (qt + 1) & 63; if (b == 0xFA) return true; }
    delay(1);
  }
  return false;
}

void ps2Enable() { ps2Ok = ps2Send(0xF4) && ps2WaitAck(); }   // F4 = start sending movement

void ps2Begin() {
  pinMode(PS2_CLK, INPUT_PULLUP);
  pinMode(PS2_DAT, INPUT_PULLUP);
  attachInterrupt(PS2_CLK, onClk, FALLING);
  delay(700);                              // let the mouse finish its power-on test
  qt = qh;
  ps2Enable();
}

void ps2Poll() {
  if (!ps2Ok && millis() - lastTry > 3000) { lastTry = millis(); ps2Enable(); }
  while (qt != qh) {
    uint8_t b = q[qt]; qt = (qt + 1) & 63;
    unsigned long now = millis();
    if (now - lastB > 15) pn = 0;
    lastB = now;
    if (pn == 0 && (b == 0xFA || !(b & 0x08))) continue;   // resync
    pk[pn++] = b;
    if (pn == 3) {
      pn = 0; ps2Ok = true;
      if (pk[0] & 0xC0) continue;                          // overflow, drop
      accX += pk[1] - ((pk[0] << 4) & 0x100);
      accY -= pk[2] - ((pk[0] << 3) & 0x100);              // PS/2 up is positive
    }
  }
}

// ---------- Bluetooth mouse for the whole laptop ----------
static const uint8_t REPORT_MAP[] = {
  0x05,0x01,0x09,0x02,0xA1,0x01,0x85,0x01,0x09,0x01,0xA1,0x00,
  0x05,0x09,0x19,0x01,0x29,0x03,0x15,0x00,0x25,0x01,0x95,0x03,0x75,0x01,0x81,0x02,
  0x95,0x01,0x75,0x05,0x81,0x03,
  0x05,0x01,0x09,0x30,0x09,0x31,0x09,0x38,0x15,0x81,0x25,0x7F,0x75,0x08,0x95,0x03,0x81,0x06,
  0xC0,0xC0 };
BLECharacteristic* mouseIn = nullptr;
volatile bool bleConn = false;
bool pcMode = false, btnL = false;
int cx = 2048, cy = 2048;

class SrvCb : public BLEServerCallbacks {
  void onConnect(BLEServer*) { bleConn = true; }
  void onDisconnect(BLEServer*) { bleConn = false; BLEDevice::startAdvertising(); }
};

void bleBegin() {
  BLEDevice::init("JoyLink Mouse");
  BLEServer* s = BLEDevice::createServer();
  s->setCallbacks(new SrvCb());
  BLEHIDDevice* hid = new BLEHIDDevice(s);
  mouseIn = hid->inputReport(1);
  hid->manufacturer()->setValue("JoyLink");
  hid->pnp(0x02, 0xe502, 0xa111, 0x0210);
  hid->hidInfo(0x00, 0x02);
  BLESecurity* sec = new BLESecurity();
  sec->setAuthenticationMode(ESP_LE_AUTH_BOND);
  hid->reportMap((uint8_t*)REPORT_MAP, sizeof(REPORT_MAP));
  hid->startServices();
  hid->setBatteryLevel(100);
  BLEAdvertising* adv = s->getAdvertising();
  adv->setAppearance(0x03C2);              // mouse
  adv->addServiceUUID(hid->hidService()->getUUID());
  adv->start();
}

void sendMouse(uint8_t btn, int8_t dx, int8_t dy, int8_t wh) {
  if (!bleConn || !mouseIn) return;
  uint8_t r[4] = { btn, (uint8_t)dx, (uint8_t)dy, (uint8_t)wh };
  mouseIn->setValue(r, 4);
  mouseIn->notify();
}

// Runs every 20 ms. Stick: UP/DOWN = scroll, LEFT = right-click, hold RIGHT 5 s = switch page/laptop mode.
// Joystick button = left click (hold = drag). Finger mouse = cursor.
void pcTick(int x, int y, bool sw) {
  int dx = x - cx, dy = (y - cy) * (STICK_INVERT_Y ? -1 : 1);
  static int8_t cur = 0; static bool fired = false; static unsigned long t0 = 0, ls = 0;
  int8_t want = 0;
  if (abs(dy) > THR && abs(dy) * 2 > abs(dx) * 3) want = dy < 0 ? 1 : 2;
  else if (abs(dx) > THR && abs(dx) * 2 > abs(dy) * 3) want = dx > 0 ? 4 : 3;
  if (abs(dx) < THR / 2 && abs(dy) < THR / 2) cur = 0;
  else if (want && want != cur) { cur = want; t0 = millis(); fired = false; }
  unsigned long held = millis() - t0;

  if (cur == 4 && held > 5000 && !fired) {
    fired = true; pcMode = !pcMode;
    digitalWrite(LED_PIN, pcMode ? LOW : HIGH);
    if (!pcMode) { btnL = false; sendMouse(0, 0, 0, 0); }
    return;
  }
  if (!pcMode) return;

  if (cur && held > 100) {
    if (cur == 3 && !fired) { fired = true; sendMouse(2, 0, 0, 0); sendMouse(0, 0, 0, 0); }
    else if ((cur == 1 || cur == 2) && millis() - ls > 110) { ls = millis(); sendMouse(btnL ? 1 : 0, 0, 0, cur == 1 ? 1 : -1); }
  }
  if (sw != btnL) { btnL = sw; sendMouse(sw ? 1 : 0, 0, 0, 0); }

  int mx = accX, my = accY; accX = accY = 0;
  if (MOUSE_MIRROR) mx = -mx;
  for (int i = 0; i < MOUSE_ROT; i++) { int t = mx; mx = -my; my = t; }
  mx = (int)(mx * MOUSE_GAIN); my = (int)(my * MOUSE_GAIN);
  while (mx || my) {
    int sx = constrain(mx, -127, 127), sy = constrain(my, -127, 127);
    sendMouse(btnL ? 1 : 0, sx, sy, 0);
    mx -= sx; my -= sy;
  }
}

void emit(const String& l) {
  Serial.println(l);
  if (sse && sse.connected()) sse.print("data: " + l + "\n\n");
}

String urlDecode(const String& s) {
  String o;
  for (unsigned i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '+') o += ' ';
    else if (c == '%' && i + 2 < s.length()) { o += (char)strtol(s.substring(i + 1, i + 3).c_str(), NULL, 16); i += 2; }
    else o += c;
  }
  return o;
}

int readAvg(int pin) {
  long s = 0;
  for (int i = 0; i < 8; i++) s += analogRead(pin);
  return s / 8;
}

void startAP() {
  WiFi.disconnect(true);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);      // lower power = cooler board
}

bool joinWifi(const String& ssid, const String& pass, bool save) {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid.c_str(), pass.c_str());
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  unsigned long t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t < 12000) delay(200);
  if (WiFi.status() == WL_CONNECTED) {
    if (save) { prefs.putString("s", ssid); prefs.putString("p", pass); }
    MDNS.begin("joylink");
    Serial.println("IP," + WiFi.localIP().toString());
    return true;
  }
  Serial.println("ERR,wifi");
  startAP();                               // fall back so you can try again
  return false;
}

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c != '\n') { if (inBuf.length() < 300) inBuf += c; continue; }
    inBuf.trim();
    if (inBuf.startsWith("W,")) {
      int i = inBuf.indexOf(',', 2);
      if (i > 0) joinWifi(urlDecode(inBuf.substring(2, i)), urlDecode(inBuf.substring(i + 1)), true);
    }
    inBuf = "";
  }
}

void handleClient() {
  WiFiClient c = server.available();
  if (!c) return;
  c.setTimeout(200);
  String req = c.readStringUntil('\n');
  while (c.available()) c.read();
  if (req.startsWith("GET /events")) {
    if (sse) sse.stop();
    c.print("HTTP/1.1 200 OK\r\nContent-Type: text/event-stream\r\n"
            "Cache-Control: no-cache\r\nAccess-Control-Allow-Origin: *\r\n"
            "Connection: keep-alive\r\n\r\nretry: 1000\n\n");
    sse = c;
  } else if (req.startsWith("GET /wifi?")) {
    int a = req.indexOf("s="), b = req.indexOf("&p="), e = req.indexOf(" HTTP");
    c.print("HTTP/1.1 200 OK\r\nAccess-Control-Allow-Origin: *\r\n\r\nOK");
    c.stop();
    if (a > 0 && b > a && e > b) joinWifi(urlDecode(req.substring(a + 2, b)), urlDecode(req.substring(b + 3, e)), true);
  } else {
    c.print("HTTP/1.1 200 OK\r\nAccess-Control-Allow-Origin: *\r\n\r\nJoyLink OK");
    c.stop();
  }
}

void setup() {
  Serial.begin(115200);
#if ARDUINO_USB_CDC_ON_BOOT
  Serial.setTxTimeoutMs(0);                // never block when no PC is listening
#endif
  pinMode(SW_PIN, INPUT_PULLUP);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  ps2Begin();
  pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, HIGH);
  { long sx = 0, sy = 0; for (int i = 0; i < 16; i++) { sx += readAvg(VRX_PIN); sy += readAvg(VRY_PIN); delay(5); } cx = sx / 16; cy = sy / 16; }  // stick centre (keep it at rest while booting)
  bleBegin();
  prefs.begin("joylink", false);
  String s = prefs.getString("s", ""), p = prefs.getString("p", "");
  if (s.length() == 0 || !joinWifi(s, p, false)) startAP();
  server.begin();
}

void loop() {
  handleSerial();
  handleClient();
  ps2Poll();
  unsigned long now = millis();
  if (WiFi.status() == WL_CONNECTED && now - lastIp > 3000) {
    lastIp = now;
    Serial.println("IP," + WiFi.localIP().toString());
  }
  if (now - lastSend < 20) return;
  lastSend = now;

  int x = readAvg(VRX_PIN), y = readAvg(VRY_PIN);
  bool sw = digitalRead(SW_PIN) == LOW;
  pcTick(x, y, sw);
  emit("J," + String(x) + "," + String(y) + "," + String(sw ? 1 : 0) + "," + String(ps2Ok ? 1 : 0) + "," + String(pcMode ? 1 : 0));
  if (!pcMode && (accX || accY)) { emit("M," + String(accX) + "," + String(accY)); accX = accY = 0; }
}
