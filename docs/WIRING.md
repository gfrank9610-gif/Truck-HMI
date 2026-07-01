# Truck-HMI — Nano V3.0 Wiring Guide
**Ride Or Die Customs Light Control System**

---

## 1. Overview

The system uses an **Elecrow CrowPanel 5.0 (ESP32-S3)** touchscreen HMI to send
serial commands to an **Arduino Nano V3.0 (ATmega328P, 5V, 16MHz)**.
The Nano drives a **6-channel relay module** that switches 12V truck lighting circuits.

---

## 2. Components Required

| Item | Qty | Notes |
|------|-----|-------|
| Arduino Nano V3.0 (ATmega328P) | 1 | 5V, 16MHz |
| 6-Channel Relay Module (5V coil, opto-isolated) | 1 | Active-LOW trigger (e.g. SainSmart 6ch) |
| Elecrow CrowPanel 5.0 V2.0 | 1 | ESP32-S3, running Truck-HMI firmware |
| Jumper wires (M-F or M-M) | ~15 | Nano to relay + Nano to ESP32 |
| 10 kΩ resistor | 1 | Voltage divider — Nano TX -> ESP32 RX |
| 20 kΩ resistor | 1 | Voltage divider lower leg |
| 12V DC power supply / fused bus | 1 | Powers lighting loads — fuse each channel! |
| 5V USB power supply | 1 | Powers Nano + relay logic |
| Inline fuse holders + fuses | 6 | One per lighting circuit (size to load) |

---

## 3. Channel Map

| # | Default Label | Nano Pin | Relay IN |
|---|---------------|----------|----------|
| 0 | MASTER (all) | — | all |
| 1 | Fog Lights | D2 | IN1 |
| 2 | Top Light Bar | D3 | IN2 |
| 3 | Wheel Lights | D4 | IN3 |
| 4 | Reverse Lights | D5 | IN4 |
| 5 | Bed Lights | D6 | IN5 |
| 6 | Expansion | D7 | IN6 |

---

## 4. Serial Command Reference

| Command | Action |
|---------|--------|
| `R0:1\n` | Master ON — all 6 relays ON |
| `R0:0\n` | Master OFF — all 6 relays OFF, cancel all flash |
| `R1:1\n` | Fog Lights ON |
| `R1:0\n` | Fog Lights OFF |
| `R2:1\n` … `R6:1\n` | Channels 2–6 ON (same pattern) |
| `F1:1\n` | Latched flash ON — ch1 strobes at 375 ms |
| `F1:0\n` | Latched flash OFF — ch1 stops strobing |

---

## 5. Wiring Schematics

### 5a. Block Diagram

```
  +---------------------------+   UART 9600    +---------------------------+
  |  ELECROW CROWPANEL 5.0    |   baud         |   ARDUINO NANO V3.0       |
  |  ESP32-S3 HMI             |                |   ATmega328P 5V 16MHz     |
  |                           |                |                           |
  |  GPIO17 (TX) -------------|--------------->| D0 / RX                   |
  |  GPIO18 (RX) <---[VD]-----|----------------| D1 / TX                   |
  |  GND ----------------------|----------------| GND                      |
  +---------------------------+                |                           |
                                               | D2 ----------------------->| IN1 -+
                                               | D3 ----------------------->| IN2  |
                                               | D4 ----------------------->| IN3  | 6-CH
                                               | D5 ----------------------->| IN4  | RELAY
                                               | D6 ----------------------->| IN5  | MODULE
                                               | D7 ----------------------->| IN6 -+
                                               | 5V ------------------------>| VCC
                                               | GND ----------------------->| GND
                                               +---------------------------+

[VD] = Voltage Divider (10k + 20k) on Nano TX -> ESP32 RX
       protects ESP32 3.3V GPIO from Nano 5V output
```

---

### 5b. Nano Pin-Out

```
                       ARDUINO NANO V3.0
                  +-------------------------------+
  ESP32 GPIO17 -->| D0 / RX           D1 / TX    |--[10k]--+-> ESP32 GPIO18
                  |                              |          +--[20k]-- GND
  RELAY IN1   -->| D2                D13 (LED)  | (built-in, no wire)
  RELAY IN2   -->| D3                     3V3   |
  RELAY IN3   -->| D4                      A5   |
  RELAY IN4   -->| D5                      A4   |
  RELAY IN5   -->| D6                      A3   |
  RELAY IN6   -->| D7                      A2   |
                  |                         A1   |
             +-->| GND                     A0   |
             |    |                       AREF   |
          5V-+-->| 5V                   RESET   |
             |    +-------------------------------+
             |
        Common GND bus (also connect relay GND and ESP32 GND here)
```

---

### 5c. Relay Module Wiring

```
  6-CHANNEL RELAY MODULE  (Active-LOW, Opto-Isolated)
  +--------------------------------------------------------------+
  |  VCC  -------- Nano 5V pin                                   |
  |  GND  -------- Nano GND pin                                  |
  |                                                              |
  |  IN1  -------- Nano D2  (Fog Lights)       Active LOW       |
  |  IN2  -------- Nano D3  (Top Light Bar)    Active LOW       |
  |  IN3  -------- Nano D4  (Wheel Lights)     Active LOW       |
  |  IN4  -------- Nano D5  (Reverse Lights)   Active LOW       |
  |  IN5  -------- Nano D6  (Bed Lights)       Active LOW       |
  |  IN6  -------- Nano D7  (Expansion)        Active LOW       |
  |                                                              |
  |  RELAY 1 SCREW TERMINALS:                                    |
  |    COM  ---- 12V+ Bus                                        |
  |    NO   ---- Fog Light positive (+) wire                     |
  |    NC   ---- (unused)                                        |
  |                                                              |
  |  Repeat for Relays 2-6  (same: COM->12V+,  NO->light +)     |
  +--------------------------------------------------------------+
```

---

### 5d. Voltage Divider — Nano TX to ESP32 RX

```
  Nano D1/TX  (5V output)
       |
     [10 kΩ]
       |
       +---------- ESP32 GPIO18 / RX  (3.3V max safe input)
       |
     [20 kΩ]
       |
      GND

  Result: 5V × 20k/(10k+20k) = 3.33V  -- safe for ESP32 GPIO

  NOTE: Nano RX (D0) receives 3.3V from ESP32 TX -- safe without a divider.
```

---

### 5e. Lighting Circuit (per channel)

```
  12V Battery / Fused Supply (+)
       |
     [FUSE]   <-- size to light draw: LED bar = 5A, LED strip = 1-2A
       |
   RELAY COM
   RELAY NO --------- Light positive (+) wire
                       Light negative (-) wire ------- 12V GND / Chassis Ground
```

---

## 6. Quick-Reference Pin Table

| Nano Pin | Connects To | Notes |
|----------|-------------|-------|
| D0 / RX | ESP32 GPIO17 (TX) | Direct wire — no extra parts |
| D1 / TX | ESP32 GPIO18 (RX) via VD | 10kΩ + 20kΩ voltage divider |
| D2 | Relay IN1 | Fog Lights |
| D3 | Relay IN2 | Top Light Bar |
| D4 | Relay IN3 | Wheel Lights |
| D5 | Relay IN4 | Reverse Lights |
| D6 | Relay IN5 | Bed Lights |
| D7 | Relay IN6 | Expansion |
| 5V | Relay VCC | Relay logic power |
| GND | Relay GND + ESP32 GND | Common ground — all three boards |
| D13 | (built-in LED) | Lit when any relay is active |

---

## 7. Important Notes

### Disconnecting D0/D1 Before Upload
Pins D0 and D1 are shared with the Nano's USB serial port. **Disconnect the wires
from D0 and D1 before uploading a sketch.** Reconnect after upload completes.

### Active-LOW Relay Module
Most 5V relay modules are active-LOW: the coil energises when the IN pin is driven LOW.
The sketch has `#define RELAY_ACTIVE_LOW true`. If your module is active-HIGH (uncommon),
change that define to `false`.

### Power Supply Separation
Keep 12V lighting power **completely separate** from 5V logic power. Only the relay
contact terminals (COM / NO / NC) bridge the two voltage domains. Never connect 12V
to Nano pins or relay IN/VCC/GND pins.

### Fusing
Fuse every lighting circuit individually on the positive wire from relay COM to the light.
Size fuses to the actual load current.

### Flash / Strobe Mode
The HMI sends `F<ch>:1` to start a latched 375ms strobe and `F<ch>:0` to stop it.
Each channel is independent. Master OFF (`R0:0`) also cancels all active flashes.

### Arduino IDE Upload Settings
- **Board:** Arduino Nano
- **Processor:** ATmega328P  _(use "Old Bootloader" if upload fails)_
- **Port:** your COM port
- **File:** `src/Truck_HMI_Nano.ino`

---

*Ride Or Die Customs — gfrank9610-gif/Truck-HMI*
