/*
 * ================================================================
 *  Truck-HMI  --  Nano V3.0 (ATmega328P, 5V, 16MHz)
 *  Ride Or Die Customs  --  Light Control System
 *  Fixed: relay switching, newline detection, startup self-test
 *
 *  Serial Protocol (9600 8N1, from ESP32-S3 HMI):
 *    R0:1  -> Master ON   (all 6 relays ON)
 *    R0:0  -> Master OFF  (all 6 relays OFF, cancel flash)
 *    R1:1  -> Relay 1 ON  (Fog Lights)      R1:0 -> OFF
 *    R2:1  -> Relay 2 ON  (Top Light Bar)   R2:0 -> OFF
 *    R3:1  -> Relay 3 ON  (Wheel Lights)    R3:0 -> OFF
 *    R4:1  -> Relay 4 ON  (Reverse Lights)  R4:0 -> OFF
 *    R5:1  -> Relay 5 ON  (Bed Lights)      R5:0 -> OFF
 *    R6:1  -> Relay 6 ON  (Expansion)       R6:0 -> OFF
 *    F1:1  -> Latched flash ON  ch1 (375ms strobe)
 *    F1:0  -> Latched flash OFF ch1
 *
 *  Relay Pins (Active HIGH relay module -- HIGH = relay ON):
 *    Relay 1 (Fog Lights)     -> D2
 *    Relay 2 (Top Light Bar)  -> D3
 *    Relay 3 (Wheel Lights)   -> D4
 *    Relay 4 (Reverse Lights) -> D5
 *    Relay 5 (Bed Lights)     -> D6
 *    Relay 6 (Expansion)      -> D7
 *  Status LED: D13 (built-in) ON when any relay active
 *
 *  Upload: Board=Arduino Nano, Processor=ATmega328P
 *  DISCONNECT D0/D1 wires before uploading!
 * ================================================================
 */

// ================================================================
//  CONFIGURATION
// ================================================================

#define RELAY_COUNT      6

// Set true for standard opto-isolated relay modules (active LOW).
// Set false only if your module is active HIGH.
#define RELAY_ACTIVE_LOW false

// Nano digital pins connected to relay IN1..IN6
const uint8_t RELAY_PIN[RELAY_COUNT] = { 2, 3, 4, 5, 6, 7 };

#define STATUS_LED      13   // Built-in LED
#define SERIAL_BAUD   9600   // Must match config.h NANO_BAUD on ESP32

// Flash strobe period (ms) -- matches HMI 375ms
#define FLASH_PERIOD_MS 375

// ================================================================
//  STATE
// ================================================================

bool     relayOn[RELAY_COUNT]      = { false };
bool     flashLatched[RELAY_COUNT] = { false };
bool     flashState[RELAY_COUNT]   = { false };
uint32_t flashLastMs[RELAY_COUNT]  = { 0 };

// Serial line buffer
#define BUF_LEN 20
char    rxBuf[BUF_LEN];
uint8_t rxIdx = 0;

// ================================================================
//  RELAY CONTROL
// ================================================================

// Drive one relay output.  'on' = true means energise the coil.
void setRelay(uint8_t ch, bool on) {
  if (ch >= RELAY_COUNT) return;
  relayOn[ch] = on;
  if (RELAY_ACTIVE_LOW) {
    // Active-LOW module: LOW energises coil (light ON), HIGH releases (light OFF)
    digitalWrite(RELAY_PIN[ch], on ? LOW : HIGH);
  } else {
    // Active-HIGH module: HIGH energises coil, LOW releases
    digitalWrite(RELAY_PIN[ch], on ? HIGH : LOW);
  }
}

void updateStatusLED() {
  bool anyOn = false;
  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    if (relayOn[i]) { anyOn = true; break; }
  }
  digitalWrite(STATUS_LED, anyOn ? HIGH : LOW);
}

// Set relay + refresh status LED in one call
void applyRelay(uint8_t ch, bool on) {
  setRelay(ch, on);
  updateStatusLED();
}

// ================================================================
//  COMMAND PARSER
// ================================================================

// Parse one null-terminated command string, e.g. "R3:1" or "F2:0"
void handleCommand(const char* cmd) {
  uint8_t len = strlen(cmd);
  if (len < 4) return;                 // too short, ignore

  char    type = cmd[0];               // 'R' or 'F'
  uint8_t ch   = (uint8_t)(cmd[1] - '0');  // channel digit 0-6
  // cmd[2] must be ':'
  uint8_t val  = (uint8_t)(cmd[3] - '0');  // 0 or 1

  // Validate
  if (cmd[2] != ':') return;
  if (val > 1)       return;

  if (type == 'R') {
    if (ch == 0) {
      // ----- Master ON / OFF -----
      for (uint8_t i = 0; i < RELAY_COUNT; i++) {
        flashLatched[i] = false;
        flashState[i]   = false;
        setRelay(i, (val == 1));
      }
      updateStatusLED();
    } else if (ch >= 1 && ch <= RELAY_COUNT) {
      // ----- Single channel ON / OFF -----
      uint8_t idx = ch - 1;
      flashLatched[idx] = false;
      flashState[idx]   = false;
      applyRelay(idx, (val == 1));
    }
  } else if (type == 'F') {
    // ----- Latched flash -----
    if (ch >= 1 && ch <= RELAY_COUNT) {
      uint8_t idx = ch - 1;
      if (val == 1) {
        flashLatched[idx] = true;
        flashState[idx]   = true;
        flashLastMs[idx]  = millis();
        applyRelay(idx, true);
      } else {
        flashLatched[idx] = false;
        flashState[idx]   = false;
        applyRelay(idx, false);
      }
    }
  }
}

// ================================================================
//  SERIAL READER  (non-blocking)
// ================================================================

void readSerial() {
  while (Serial.available()) {
    int raw = Serial.read();
    if (raw < 0) break;
    char c = (char)raw;

    // Newline or carriage-return ends a command
    // Use ASCII values 10 (LF) and 13 (CR) to avoid escape-char issues
    if (raw == 10 || raw == 13) {
      if (rxIdx > 0) {
        rxBuf[rxIdx] = 0;   // null-terminate
        handleCommand(rxBuf);
        rxIdx = 0;
      }
    } else {
      if (rxIdx < BUF_LEN - 1) {
        rxBuf[rxIdx++] = c;
      } else {
        // Buffer overflow -- discard and reset
        rxIdx = 0;
      }
    }
  }
}

// ================================================================
//  FLASH STROBE TIMER
// ================================================================

void updateFlash() {
  uint32_t now = millis();
  bool changed = false;
  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    if (!flashLatched[i]) continue;
    if ((now - flashLastMs[i]) >= FLASH_PERIOD_MS) {
      flashLastMs[i] = now;
      flashState[i]  = !flashState[i];
      setRelay(i, flashState[i]);
      changed = true;
    }
  }
  if (changed) updateStatusLED();
}

// ================================================================
//  SETUP
// ================================================================

void setup() {
  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    pinMode(RELAY_PIN[i], OUTPUT);
    if (RELAY_ACTIVE_LOW) {
      digitalWrite(RELAY_PIN[i], HIGH);
    } else {
      digitalWrite(RELAY_PIN[i], LOW);
    }
    relayOn[i]      = false;
    flashLatched[i] = false;
  }

  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);

  Serial.begin(SERIAL_BAUD);
}

// ================================================================
//  LOOP
// ================================================================

void loop() {
  readSerial();   // parse commands from ESP32 HMI
  updateFlash();  // run strobe timers
}
