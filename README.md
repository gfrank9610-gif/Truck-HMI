# Truck-HMI — Ride Or Die Customs Light Control System

A 6-channel truck lighting controller using an **Elecrow CrowPanel 5.0 ESP32-S3**
touchscreen HMI and an **Arduino Nano V3.0** relay driver.

---

## Repository Structure

```
Truck-HMI/
  src/
    main.cpp              # ESP32-S3 HMI firmware (LovyanGFX touchscreen UI)
    Truck_HMI_Nano.ino    # Arduino Nano V3.0 relay control sketch  <-- NEW
  include/
    config.h              # HMI pin/layout config (NANO_BAUD, NANO_RX/TX, etc.)
    touch.h               # GT911 touch driver for CrowPanel 5.0
  docs/
    WIRING.md             # Full wiring schematic, pin table, and notes  <-- NEW
  platformio.ini          # PlatformIO config for ESP32-S3 build
```

---

## Hardware

| Board | Role |
|-------|------|
| Elecrow CrowPanel 5.0 V2.0 (ESP32-S3, 800×480) | Touchscreen HMI — sends relay commands |
| Arduino Nano V3.0 (ATmega328P, 5V, 16MHz) | Relay driver — receives commands, switches lights |
| 6-Channel Relay Module (5V, opto-isolated, active-LOW) | Switches 12V lighting circuits |

---

## Channels

| # | Label | Nano Pin | Relay |
|---|-------|----------|-------|
| 0 | MASTER | — | all |
| 1 | Fog Lights | D2 | IN1 |
| 2 | Top Light Bar | D3 | IN2 |
| 3 | Wheel Lights | D4 | IN3 |
| 4 | Reverse Lights | D5 | IN4 |
| 5 | Bed Lights | D6 | IN5 |
| 6 | Expansion | D7 | IN6 |

Channel labels are editable on the HMI touchscreen (hold the header 4 seconds to enter Engineering Mode).

---

## Serial Protocol (ESP32 HMI -> Nano)

9600 baud, 8N1, newline-terminated commands.

| Command | Action |
|---------|--------|
| `R0:1` | Master ON — all relays on |
| `R0:0` | Master OFF — all relays off, cancel all flash |
| `R1:1` | Channel 1 ON (Fog Lights) |
| `R1:0` | Channel 1 OFF |
| `R2:1` … `R6:1` | Channels 2-6 ON |
| `F1:1` | Latched flash ON — ch1 strobes at 375ms |
| `F1:0` | Latched flash OFF — ch1 stops |

---

## HMI Button Behaviour

- **Tap** — toggle relay on/off
- **Hold 2s** — flash while held (375ms period)
- **Hold 4s** — latch flash (continues after release)
- **Hold header 4s** — enter Engineering Mode (PIN required)
- **Engineering Mode** — rename channel labels, stored in ESP32 NVS flash

---

## Wiring

See **[docs/WIRING.md](docs/WIRING.md)** for the full wiring schematic, voltage
divider details, relay module wiring, lighting circuit diagrams, and important safety notes.

**Quick summary:**
- ESP32 GPIO17 (TX) → Nano D0 (RX) — direct
- ESP32 GPIO18 (RX) ← Nano D1 (TX) via 10kΩ/20kΩ voltage divider
- Nano D2–D7 → Relay IN1–IN6
- Nano 5V/GND → Relay VCC/GND
- 12V lighting on relay COM/NO terminals (one fuse per channel)

---

## Building & Uploading

### ESP32-S3 HMI (PlatformIO)
```bash
pio run --target upload
```
Board: `esp32-s3-devkitc-1-myboard` (custom JSON in repo root)

### Arduino Nano V3.0 (Arduino IDE)
1. Open `src/Truck_HMI_Nano.ino`
2. **Board:** Arduino Nano | **Processor:** ATmega328P
3. Disconnect wires from D0/D1
4. Upload, then reconnect D0/D1

---

## Libraries (ESP32 HMI)

| Library | Version |
|---------|---------|
| LovyanGFX | 1.1.12 |
| TAMC_GT911 | ^1.0.2 |
| Adafruit BusIO | 1.16.0 |

---

*Ride Or Die Customs — gfrank9610-gif/Truck-HMI*
