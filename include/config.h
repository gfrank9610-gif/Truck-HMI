#pragma once

// ---- Screen ----
#define SCREEN_W  800
#define SCREEN_H  480

// ---- Serial ----
#define DEBUG_BAUD    115200
#define NANO_BAUD     9600
#define NANO_RX       18    // GPIO18 → Serial2 RX  (connects to Nano TX)
#define NANO_TX       17    // GPIO17 → Serial2 TX  (connects to Nano RX)

// ---- Outputs ----
#define OUTPUT_COUNT  6

// ---- Button layout (fills 800x480) ----
// Header: 0–89  (90px)
// Row 1 : 90–239 (150px)
// Gap   : 240–247 (8px)
// Row 2 : 248–397 (150px)
// Gap   : 398–405 (8px)
// Master: 406–473 (68px)
// Margin: 474–479 (6px)
#define BTN_COLS   3
#define BTN_ROWS   2
#define BTN_X      8
#define BTN_Y      90
#define BTN_W      256    // (800 - 2*8 margins - 2*8 gaps) / 3 = 256
#define BTN_H      150
#define BTN_GAP    8

// ---- Master button row ----
#define MASTER_Y   406
#define MASTER_H   68

// ---- Touch ----
#define TOUCH_DEBOUNCE_MS  200

// ---- Colors (RGB565) ----
#define COLOR_BG          0x0000   // near black
#define COLOR_NEON_GREEN  0x07E0   // bright green
#define COLOR_WHITE       0xFFFF
#define COLOR_DARK_GREEN  0x0200
#define COLOR_RED         0xF800
#define COLOR_DARK_RED    0x2000
#define COLOR_DIM_GRAY    0x39E7
#define COLOR_DARK_PANEL  0x1082
