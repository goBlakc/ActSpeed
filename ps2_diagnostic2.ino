// PS/2 diagnostic 2: talks to the mouse and prints its replies. Serial Monitor at 115200.
// Needs Tools > USB CDC On Boot: Enabled. C -> GPIO5, D -> GPIO6, 10k pull-ups to 3V3.
#define CLK 5
#define DAT 6

volatile uint8_t q[128], qh = 0, qt = 0, nb = 0, acc = 0, par = 0;
volatile uint32_t last = 0;

void IRAM_ATTR onClk() {
  uint32_t t = micros();
  if (t - last > 1500) { nb = 0; acc = 0; }
  last = t;
  uint8_t bit = digitalRead(DAT);
  if (nb == 0) { if (!bit) nb = 1; return; }
  if (nb <= 8) { acc |= bit << (nb - 1); nb++; return; }
  if (nb == 9) { par = bit; nb = 10; return; }
  nb = 0;
  if (bit && ((__builtin_popcount(acc) + par) & 1)) { q[qh] = acc; qh = (qh + 1) & 127; }
}

bool waitPin(int pin, int lvl) {
  uint32_t t = micros();
  while (digitalRead(pin) != lvl) if (micros() - t > 20000) return false;
  return true;
}

bool ps2Send(uint8_t b) {
  detachInterrupt(CLK);
  pinMode(CLK, OUTPUT); digitalWrite(CLK, LOW); delayMicroseconds(200);
  pinMode(DAT, OUTPUT); digitalWrite(DAT, LOW);
  pinMode(CLK, INPUT_PULLUP);
  bool ok = waitPin(CLK, LOW);
  uint8_t p = 1;
  for (int i = 0; ok && i < 9; i++) {
    uint8_t bit = i < 8 ? (b >> i) & 1 : p;
    if (i < 8) p ^= bit;
    if (bit) pinMode(DAT, INPUT_PULLUP); else { pinMode(DAT, OUTPUT); digitalWrite(DAT, LOW); }   // open-drain: never drive high
    ok = waitPin(CLK, HIGH) && waitPin(CLK, LOW);
  }
  pinMode(DAT, INPUT_PULLUP);
  ok = ok && waitPin(DAT, LOW) && waitPin(CLK, LOW) && waitPin(DAT, HIGH) && waitPin(CLK, HIGH);
  pinMode(CLK, INPUT_PULLUP);
  nb = 0;
  attachInterrupt(CLK, onClk, FALLING);
  return ok;
}

void drain(const char* label, int ms) {
  Serial.print(label);
  bool any = false;
  unsigned long t = millis();
  while (millis() - t < ms) {
    while (qt != qh) { Serial.printf("%02X ", q[qt]); qt = (qt + 1) & 127; any = true; }
    delay(1);
  }
  if (!any) Serial.print("(nothing)");
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  pinMode(CLK, INPUT_PULLUP);
  pinMode(DAT, INPUT_PULLUP);
  attachInterrupt(CLK, onClk, FALLING);
  delay(2500);
  Serial.printf("Idle levels: CLK=%d DAT=%d (both should be 1)\n", digitalRead(CLK), digitalRead(DAT));
  drain("Bytes seen since power-up (AA 00 if the mouse just powered on): ", 100);
  bool ok = ps2Send(0xFF);
  Serial.printf("Sent RESET (FF): %s\n", ok ? "mouse clocked it in" : "NO RESPONSE from the mouse");
  drain("Reply to reset (expect FA AA 00): ", 1000);
  ok = ps2Send(0xF4);
  Serial.printf("Sent ENABLE (F4): %s\n", ok ? "mouse clocked it in" : "NO RESPONSE from the mouse");
  drain("Reply to enable (expect FA): ", 300);
  Serial.println("Now move the mouse. Each movement prints 3 bytes (status, dx, dy):");
}

void loop() {
  static uint8_t c = 0;
  static unsigned long lastB = 0;
  while (qt != qh) {
    if (c && millis() - lastB > 15) { Serial.println(); c = 0; }
    lastB = millis();
    Serial.printf("%02X ", q[qt]); qt = (qt + 1) & 127;
    if (++c == 3) { Serial.println(); c = 0; }
  }
}
