// PS/2 line diagnostic for the finger mouse. Upload, open Serial Monitor at 115200.
// Needs Tools > USB CDC On Boot: Enabled.
#define CLK 5   // C
#define DAT 6   // D

volatile uint32_t clkEdges = 0, datEdges = 0;
void IRAM_ATTR onClk() { clkEdges++; }
void IRAM_ATTR onDat() { datEdges++; }

void setup() {
  Serial.begin(115200);
  pinMode(CLK, INPUT_PULLUP);
  pinMode(DAT, INPUT_PULLUP);
  attachInterrupt(CLK, onClk, CHANGE);
  attachInterrupt(DAT, onDat, CHANGE);
  delay(2000);
  Serial.println("Edges since boot: CLK=" + String(clkEdges) + " DAT=" + String(datEdges));
  Serial.println("(a mouse that just powered up sends AA 00 = about 40+ clock edges)");
  Serial.println("Now move the mouse over a surface...");
}

void loop() {
  static uint32_t lc = 0, ld = 0;
  Serial.printf("CLK level=%d  DAT level=%d  | clock edges last second=%u  data edges=%u\n",
                digitalRead(CLK), digitalRead(DAT), clkEdges - lc, datEdges - ld);
  lc = clkEdges; ld = datEdges;
  delay(1000);
}
