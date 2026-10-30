// Two-channel optical tachometer for the nested-motor rig.
//
//   Channel 0 -> D2 (INT0): cradle encoder ring   (Phase 3)  /  dyno disc (Phase 1)
//   Channel 1 -> D3 (INT1): output encoder disc   (Phase 3)
//
// Slot-sensor modules: VCC -> 5V, GND -> GND, DO -> D2 / D3.
//
// Serial at 250000 baud (exact on a 16 MHz AVR). Commands, newline-terminated:
//   p <ch> <ppr>   set pulses per revolution for a channel (defaults: ch0 = 2, ch1 = 4)
//   s              summary mode (default): every 200 ms prints  t_ms,rpm0,rpm1,pulses0,pulses1
//   r <ch>         raw mode: prints one line per pulse on that channel:  t_us
//                  (used for the spin-up / inertia test; start it, then switch the motor on)
//
// RPM in summary mode = (pulses in window - 1) / ppr / (last - first pulse time), i.e.
// whole pulse intervals only, so the window boundaries add no quantization error.

const uint8_t PIN[2] = {2, 3};
const unsigned long MIN_INTERVAL_US = 40;   // reject comparator chatter on a single edge
const unsigned long REPORT_MS = 200;
const unsigned long STALL_US = 500000UL;    // no pulse for this long -> report 0 rpm

volatile unsigned long lastUs[2], firstInWindow[2], lastInWindow[2];
volatile unsigned int countInWindow[2];
volatile unsigned long totalPulses[2];

#define RAW_BUF 64
volatile unsigned long rawBuf[RAW_BUF];
volatile uint8_t rawHead = 0, rawTail = 0;
volatile int8_t rawChannel = -1;

uint8_t ppr[2] = {2, 4};

void pulse(uint8_t ch) {
  unsigned long now = micros();
  if (now - lastUs[ch] < MIN_INTERVAL_US) return;
  lastUs[ch] = now;
  totalPulses[ch]++;
  if (countInWindow[ch] == 0) firstInWindow[ch] = now;
  lastInWindow[ch] = now;
  countInWindow[ch]++;
  if (rawChannel == ch) {
    uint8_t next = (rawHead + 1) % RAW_BUF;
    if (next != rawTail) {        // drop on overflow rather than corrupt
      rawBuf[rawHead] = now;
      rawHead = next;
    }
  }
}

void isr0() { pulse(0); }
void isr1() { pulse(1); }

void setup() {
  Serial.begin(250000);
  for (uint8_t ch = 0; ch < 2; ch++) pinMode(PIN[ch], INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN[0]), isr0, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN[1]), isr1, RISING);
  Serial.println(F("# nested-motor tachometer ready; mode=summary"));
  Serial.println(F("t_ms,rpm0,rpm1,pulses0,pulses1"));
}

float windowRpm(uint8_t ch, unsigned int n, unsigned long first, unsigned long last, unsigned long nowUs) {
  if (nowUs - lastUs[ch] > STALL_US || n < 2) return 0.0;
  return (n - 1) * 60.0e6 / ((float)ppr[ch] * (float)(last - first));
}

void handleCommand() {
  static char line[24];
  static uint8_t len = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c != '\n' && c != '\r') {
      if (len < sizeof(line) - 1) line[len++] = c;
      continue;
    }
    line[len] = 0;
    len = 0;
    int a = 0, b = 0;
    if (line[0] == 'p' && sscanf(line + 1, "%d %d", &a, &b) == 2 && a >= 0 && a < 2 && b > 0) {
      ppr[a] = b;
      Serial.print(F("# ppr")); Serial.print(a); Serial.print('='); Serial.println(b);
    } else if (line[0] == 'r' && sscanf(line + 1, "%d", &a) == 1 && a >= 0 && a < 2) {
      noInterrupts(); rawHead = rawTail = 0; rawChannel = a; interrupts();
      Serial.print(F("# mode=raw ch=")); Serial.print(a); Serial.print(F(" ppr=")); Serial.println(ppr[a]);
      Serial.println(F("t_us"));
    } else if (line[0] == 's') {
      rawChannel = -1;
      Serial.println(F("# mode=summary"));
      Serial.println(F("t_ms,rpm0,rpm1,pulses0,pulses1"));
    }
  }
}

void loop() {
  handleCommand();

  if (rawChannel >= 0) {
    while (rawTail != rawHead) {
      noInterrupts(); unsigned long t = rawBuf[rawTail]; interrupts();
      rawTail = (rawTail + 1) % RAW_BUF;
      Serial.println(t);
    }
    return;
  }

  static unsigned long nextReport = 0;
  unsigned long nowMs = millis();
  if (nowMs < nextReport) return;
  nextReport = nowMs + REPORT_MS;

  unsigned int n[2]; unsigned long f[2], l[2], tot[2];
  noInterrupts();
  for (uint8_t ch = 0; ch < 2; ch++) {
    n[ch] = countInWindow[ch]; f[ch] = firstInWindow[ch]; l[ch] = lastInWindow[ch]; tot[ch] = totalPulses[ch];
    // carry the last pulse into the next window so no interval is lost between windows
    countInWindow[ch] = n[ch] > 0 ? 1 : 0;
    firstInWindow[ch] = l[ch];
  }
  interrupts();
  unsigned long nowUs = micros();
  Serial.print(nowMs);
  for (uint8_t ch = 0; ch < 2; ch++) { Serial.print(','); Serial.print(windowRpm(ch, n[ch], f[ch], l[ch], nowUs), 1); }
  for (uint8_t ch = 0; ch < 2; ch++) { Serial.print(','); Serial.print(tot[ch]); }
  Serial.println();
}
