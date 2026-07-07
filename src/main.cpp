#include <Arduino.h>
#include <Preferences.h>
#include "config.h"

#define TFT_BL        2
#define BL_LEDC_CHAN  0
#define BL_LEDC_FREQ  5000
#define BL_LEDC_BITS  8
#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <lgfx/v1/platforms/esp32s3/Panel_RGB.hpp>
#include <lgfx/v1/platforms/esp32s3/Bus_RGB.hpp>

class LGFX : public lgfx::LGFX_Device {
public:
    lgfx::Bus_RGB   _bus_instance;
    lgfx::Panel_RGB _panel_instance;
    LGFX(void) {
        { auto cfg = _bus_instance.config();
          cfg.panel = &_panel_instance;
          cfg.pin_d0=GPIO_NUM_8;  cfg.pin_d1=GPIO_NUM_3;  cfg.pin_d2=GPIO_NUM_46;
          cfg.pin_d3=GPIO_NUM_9;  cfg.pin_d4=GPIO_NUM_1;  cfg.pin_d5=GPIO_NUM_5;
          cfg.pin_d6=GPIO_NUM_6;  cfg.pin_d7=GPIO_NUM_7;  cfg.pin_d8=GPIO_NUM_15;
          cfg.pin_d9=GPIO_NUM_16; cfg.pin_d10=GPIO_NUM_4; cfg.pin_d11=GPIO_NUM_45;
          cfg.pin_d12=GPIO_NUM_48;cfg.pin_d13=GPIO_NUM_47;cfg.pin_d14=GPIO_NUM_21;
          cfg.pin_d15=GPIO_NUM_14;
          cfg.pin_henable=GPIO_NUM_40; cfg.pin_vsync=GPIO_NUM_41;
          cfg.pin_hsync=GPIO_NUM_39;   cfg.pin_pclk=GPIO_NUM_0;
          cfg.freq_write=15000000;
          cfg.hsync_polarity=0; cfg.hsync_front_porch=8;
          cfg.hsync_pulse_width=4; cfg.hsync_back_porch=43;
          cfg.vsync_polarity=0; cfg.vsync_front_porch=8;
          cfg.vsync_pulse_width=4; cfg.vsync_back_porch=12;
          cfg.pclk_active_neg=1; cfg.de_idle_high=0; cfg.pclk_idle_high=0;
          _bus_instance.config(cfg); }
        { auto cfg = _panel_instance.config();
          cfg.memory_width=800;  cfg.memory_height=480;
          cfg.panel_width=800;   cfg.panel_height=480;
          cfg.offset_x=0;        cfg.offset_y=0;
          _panel_instance.config(cfg); }
        _panel_instance.setBus(&_bus_instance);
        setPanel(&_panel_instance);
    }
};

LGFX lcd;
#include "touch.h"

// ---- Fonts ----
#define FONT_TITLE  lgfx::fonts::FreeSerifBold18pt7b
#define FONT_BTN    lgfx::fonts::FreeSerifBold12pt7b
#define FONT_SMALL  lgfx::fonts::FreeSans9pt7b

// ---- Colour helpers ----
#define COL_ORANGE  0xFD20

// ---- State machine ----
enum AppState { ST_MAIN, ST_PIN, ST_ENG, ST_OSK, ST_SAVER };
static AppState appState = ST_MAIN;

// ---- Serial ----
#define NANO_SERIAL Serial2

// ---- Relay state ----
bool relayState[OUTPUT_COUNT] = {};

// ---- Channel labels ----
static const char* DEFAULT_LABELS[OUTPUT_COUNT] = {
    "Fog Lights","Top Light Bar","Wheel Lights",
    "Reverse Lights","Bed Lights","Train Horn"
};
static char channelNames[OUTPUT_COUNT][17];
static char engDraft[OUTPUT_COUNT][17];
static Preferences prefs;

void loadLabels() {
    prefs.begin("roc", true);
    for (int i = 0; i < OUTPUT_COUNT; i++) {
        char key[4]; snprintf(key, sizeof(key), "c%d", i);
        String s = prefs.getString(key, DEFAULT_LABELS[i]);
        strncpy(channelNames[i], s.c_str(), 16);
        channelNames[i][16] = '\0';
    }
    prefs.end();
}

void saveLabels() {
    prefs.begin("roc", false);
    for (int i = 0; i < OUTPUT_COUNT; i++) {
        char key[4]; snprintf(key, sizeof(key), "c%d", i);
        prefs.putString(key, channelNames[i]);
    }
    prefs.end();
}

// ---- Screen saver ----
#define SAVER_TIMEOUT_MS (3UL * 60UL * 1000UL)  // 3 minutes
static uint32_t lastActivityMs = 0;
static AppState preSaverState  = ST_MAIN;

// ---- Hold-to-enter (long press header 4 s) ----
#define HOLD_MS          4000
#define FINGER_UP_MS     150   // consider finger lifted if no touch signal for this long
#define HOLD_BAR_H       8
#define HOLD_BAR_Y       (BTN_Y - 1 - HOLD_BAR_H)  // just above the divider line

static bool fingerDown         = false;
static uint32_t lastRawTouchMs = 0;
static bool holdActive         = false;
static uint32_t holdStartMs    = 0;
static int holdBarW            = 0;

// ---- Button hold-to-flash (hold 1 s → flash while held; hold 2 s → latched flash) ----
#define BTN_HOLD_MS        1000
#define BTN_HOLD_LATCH_MS  2000
#define FLASH_PERIOD       375
#define MOMENTARY_CH       5    // channel 6 (0-indexed): press=ON, release=OFF, no flash

static int      holdBtnIdx     = -1;
static uint32_t holdBtnStartMs = 0;
static bool     flashActive    = false;
static bool     flashOn        = false;
static uint32_t flashLastMs    = 0;
static bool     flashLatched[OUTPUT_COUNT]    = {};
static bool     latchFlashOn[OUTPUT_COUNT]    = {};
static uint32_t latchFlashLastMs[OUTPUT_COUNT] = {};

// ---- Master strobe mode (hold Light Em Up 4 s) ----
#define MASTER_STROBE_HOLD_MS  4000
#define STROBE_ON_MS           120
#define STROBE_OFF_MS           80

static bool     masterHoldActive   = false;
static uint32_t masterHoldStartMs  = 0;
static bool     masterStrobeActive = false;
static int      strobeChIdx        = 0;
static uint32_t strobeLastMs       = 0;
static bool     strobePhaseOn      = false;

// ---- PIN ----
static const char CORRECT_PIN[] = "111111";
static char pinEntry[7] = "";
static int  pinLen      = 0;

// PIN key grid: 3 cols × 4 rows, centred on 800 px
//   key 240×68, gap 10  →  3*240+2*10=740, margin=(800-740)/2=30
//   rows: y0=155, ends at 155+4*68+3*10=155+302=457
#define PKEY_W   240
#define PKEY_H   68
#define PKEY_GAP 10
#define PKEY_X0  30
#define PKEY_Y0  155

static const char PIN_LABELS[4][3][8] = {
    {"1","2","3"},{"4","5","6"},{"7","8","9"},{"DEL","0","CANCEL"}
};
static const char PIN_CHARS[4][3] = {
    {'1','2','3'},{'4','5','6'},{'7','8','9'},{'D','0','C'}
};

// ---- OSK ----
static int  oskTargetIdx = -1;
static char oskBuf[17]   = "";
static bool oskShift     = true;  // true = uppercase

// OSK key geometry
//   Row 0 (QWERTYUIOP): 10 keys × 73 + 9 gaps × 6 = 784  → x0=8
//   Row 1 (ASDFGHJKL):  9 keys  × 73 + 8 gaps × 6 = 705  → x0=48 (centred)
//   Row 2 (ZXCVBNM):    7 keys  × 73 + 6 gaps × 6 = 547  → x0=48, BKSP=152px at x=601
//   Row 3 (CLR/SPC/DONE): CLR=130, SPC=512, DONE=130 + 2*6 = 784 → x0=8
//   Key H=52, row gap=6
//   Rows Y: 238 / 296 / 354 / 412    (bottom of row3 = 464, fits in 480)
#define OSK_KEY_W   73
#define OSK_KEY_H   52
#define OSK_KEY_GAP 6
#define OSK_ROW_GAP 6
#define OSK_R0_Y    238
#define OSK_R1_Y    296
#define OSK_R2_Y    354
#define OSK_R3_Y    412
#define OSK_BKSP_X  601
#define OSK_BKSP_W  152
#define OSK_CLR_W   130
#define OSK_SHIFT_W 130
#define OSK_SPC_W   376
#define OSK_DONE_W  130
#define OSK_CLR_X   8
#define OSK_SHIFT_X 144   // 8+130+6
#define OSK_SPC_X   280   // 8+130+6+130+6
#define OSK_DONE_X  662   // 8+130+6+130+6+376+6

static const char OSK_R0[] = "QWERTYUIOP";
static const char OSK_R1[] = "ASDFGHJKL";
static const char OSK_R2[] = "ZXCVBNM";

static inline int oskR0X(int i) { return 8  + i * (OSK_KEY_W + OSK_KEY_GAP); }
static inline int oskR1X(int i) { return 48 + i * (OSK_KEY_W + OSK_KEY_GAP); }
static inline int oskR2X(int i) { return 48 + i * (OSK_KEY_W + OSK_KEY_GAP); }

// ---- Layout helpers ----
static inline int btnX(int col) { return BTN_X + col * (BTN_W + BTN_GAP); }
static inline int btnY(int row) { return BTN_Y + row * (BTN_H + BTN_GAP); }

// ============================================================
//  RELAY COMMAND
// ============================================================
void sendRelayCommand(int ch, bool st) {
    NANO_SERIAL.printf("R%d:%d\n", ch, st ? 1 : 0);
}

// ============================================================
//  MAIN SCREEN
// ============================================================
void drawOutputButton(int idx) {
    int col = idx % BTN_COLS;
    int row = idx / BTN_COLS;
    int x = btnX(col), y = btnY(row);
    bool on = relayState[idx];

    lcd.fillRoundRect(x, y, BTN_W, BTN_H, 10, on ? COLOR_DARK_GREEN : COLOR_DARK_PANEL);
    lcd.drawRoundRect(x, y, BTN_W, BTN_H, 10, on ? COLOR_NEON_GREEN : COLOR_DIM_GRAY);

    lcd.fillRoundRect(x+8, y+8, 28, 20, 4, on ? COLOR_NEON_GREEN : COLOR_DIM_GRAY);
    lcd.setFont(&FONT_SMALL);
    lcd.setTextDatum(MC_DATUM);
    lcd.setTextColor(COLOR_BG);
    char ch[3]; snprintf(ch, sizeof(ch), "%d", idx+1);
    lcd.drawString(ch, x+22, y+18);

    lcd.setFont(&FONT_BTN);
    lcd.setTextDatum(MC_DATUM);
    lcd.setTextColor(on ? COLOR_NEON_GREEN : COLOR_WHITE);
    lcd.drawString(channelNames[idx], x+BTN_W/2, y+BTN_H/2);

    lcd.setFont(&FONT_SMALL);
    bool flashing = (flashActive && holdBtnIdx == idx) || flashLatched[idx];
    if (flashing) {
        lcd.setTextColor(on ? COLOR_NEON_GREEN : COLOR_DIM_GRAY);
        lcd.drawString("FLASH", x+BTN_W/2, y+BTN_H-18);
    } else {
        lcd.setTextColor(on ? COLOR_NEON_GREEN : COLOR_DIM_GRAY);
        lcd.drawString(on ? "ON" : "OFF", x+BTN_W/2, y+BTN_H-18);
    }
}

void drawMasterButtons() {
    int halfW = (800 - BTN_X*2 - BTN_GAP) / 2;
    int lx = BTN_X, rx = BTN_X + halfW + BTN_GAP;

    lcd.fillRoundRect(lx, MASTER_Y, halfW, MASTER_H, 10, COLOR_DARK_GREEN);
    lcd.drawRoundRect(lx, MASTER_Y, halfW, MASTER_H, 10, COLOR_NEON_GREEN);
    lcd.setFont(&FONT_BTN); lcd.setTextDatum(MC_DATUM);
    lcd.setTextColor(COLOR_NEON_GREEN);
    lcd.drawString("Light Em Up!", lx+halfW/2, MASTER_Y+MASTER_H/2);

    lcd.fillRoundRect(rx, MASTER_Y, halfW, MASTER_H, 10, COLOR_DARK_RED);
    lcd.drawRoundRect(rx, MASTER_Y, halfW, MASTER_H, 10, COLOR_RED);
    lcd.setTextColor(COLOR_RED);
    lcd.drawString("Master Off", rx+halfW/2, MASTER_Y+MASTER_H/2);
}

void drawMainScreen() {
    lcd.fillScreen(COLOR_BG);
    lcd.fillRect(0, 0, SCREEN_W, BTN_Y-1, COLOR_DARK_PANEL);

    lcd.setFont(&FONT_TITLE);
    lcd.setTextDatum(MC_DATUM);
    lcd.setTextColor(COLOR_NEON_GREEN);
    lcd.drawString("RIDE OR DIE CUSTOMS", SCREEN_W/2, 32);

    lcd.setFont(&FONT_SMALL);
    lcd.setTextColor(COLOR_DIM_GRAY);
    lcd.drawString("Light Control System", SCREEN_W/2, 66);

    lcd.drawFastHLine(0, BTN_Y-1, SCREEN_W, COLOR_NEON_GREEN);

    for (int i = 0; i < OUTPUT_COUNT; i++) drawOutputButton(i);
    drawMasterButtons();
}

// ============================================================
//  HOLD-TO-ENTER PROGRESS BAR
// ============================================================
void cancelHold() {
    if (!holdActive) return;
    holdActive = false;
    holdBarW   = 0;
    // Restore header panel colour + divider (don't leave a black stripe)
    lcd.fillRect(0, HOLD_BAR_Y, 800, HOLD_BAR_H, COLOR_DARK_PANEL);
    lcd.drawFastHLine(0, BTN_Y-1, 800, COLOR_NEON_GREEN);
}

void updateHoldProgress(int ty) {
    if (ty >= BTN_Y) { cancelHold(); return; }

    if (!holdActive) {
        holdActive  = true;
        holdStartMs = millis();
        holdBarW    = 0;
    }
    uint32_t elapsed = millis() - holdStartMs;
    int newW = (int)((uint32_t)elapsed * 800 / HOLD_MS);
    if (newW > 800) newW = 800;
    if (newW > holdBarW) {
        lcd.fillRect(holdBarW, HOLD_BAR_Y, newW - holdBarW, HOLD_BAR_H, COL_ORANGE);
        holdBarW = newW;
    }
    if (elapsed >= HOLD_MS) {
        holdActive = false;
        holdBarW   = 0;
        pinLen = 0; pinEntry[0] = '\0';
        appState = ST_PIN;
    }
}

// ============================================================
//  PIN SCREEN
// ============================================================
void drawPinDots(bool error = false) {
    int cx = 400, step = 46;
    int x0 = cx - step * 5 / 2;
    for (int i = 0; i < 6; i++) {
        int dx = x0 + i * step;
        uint16_t col = error ? COLOR_RED : (i < pinLen ? COL_ORANGE : 0x2945);
        if (i < pinLen || error) {
            lcd.fillCircle(dx, 115, 14, col);
        } else {
            lcd.fillCircle(dx, 115, 14, COLOR_BG);
            lcd.drawCircle(dx, 115, 14, col);
        }
    }
}

void drawPinScreen() {
    lcd.fillScreen(COLOR_BG);

    lcd.fillRect(0, 0, 800, 78, COLOR_DARK_PANEL);
    lcd.setFont(&FONT_TITLE);
    lcd.setTextDatum(MC_DATUM);
    lcd.setTextColor(COL_ORANGE);
    lcd.drawString("ENGINEERING ACCESS", 400, 38);
    lcd.drawFastHLine(0, 78, 800, COL_ORANGE);

    drawPinDots();

    for (int r = 0; r < 4; r++) {
        for (int c = 0; c < 3; c++) {
            int kx = PKEY_X0 + c*(PKEY_W+PKEY_GAP);
            int ky = PKEY_Y0 + r*(PKEY_H+PKEY_GAP);
            char k = PIN_CHARS[r][c];
            uint16_t bdCol = (k=='C') ? COLOR_RED : (k=='D') ? COLOR_DIM_GRAY : COL_ORANGE;
            uint16_t txCol = (k=='C') ? COLOR_RED : (k=='D') ? COLOR_DIM_GRAY : COLOR_WHITE;
            lcd.fillRoundRect(kx, ky, PKEY_W, PKEY_H, 8, COLOR_DARK_PANEL);
            lcd.drawRoundRect(kx, ky, PKEY_W, PKEY_H, 8, bdCol);
            lcd.setFont(&FONT_BTN);
            lcd.setTextDatum(MC_DATUM);
            lcd.setTextColor(txCol);
            lcd.drawString(PIN_LABELS[r][c], kx+PKEY_W/2, ky+PKEY_H/2);
        }
    }
}

void handlePinTouch(int tx, int ty) {
    for (int r = 0; r < 4; r++) {
        for (int c = 0; c < 3; c++) {
            int kx = PKEY_X0 + c*(PKEY_W+PKEY_GAP);
            int ky = PKEY_Y0 + r*(PKEY_H+PKEY_GAP);
            if (tx < kx || tx >= kx+PKEY_W || ty < ky || ty >= ky+PKEY_H) continue;

            char k = PIN_CHARS[r][c];
            if (k == 'C') {
                appState = ST_MAIN;
                drawMainScreen();
                return;
            }
            if (k == 'D') {
                if (pinLen > 0) { pinLen--; pinEntry[pinLen] = '\0'; drawPinDots(); }
                return;
            }
            if (pinLen < 6) {
                pinEntry[pinLen++] = k;
                pinEntry[pinLen]   = '\0';
                drawPinDots();
                if (pinLen == 6) {
                    delay(200);
                    if (strcmp(pinEntry, CORRECT_PIN) == 0) {
                        for (int i = 0; i < OUTPUT_COUNT; i++) {
                            strncpy(engDraft[i], channelNames[i], 16);
                            engDraft[i][16] = '\0';
                        }
                        appState = ST_ENG;
                        lcd.fillScreen(COLOR_BG);
                    } else {
                        drawPinDots(true);
                        delay(800);
                        pinLen = 0; pinEntry[0] = '\0';
                        drawPinDots();
                    }
                }
            }
            return;
        }
    }
}

// ============================================================
//  ENGINEERING SCREEN
// ============================================================
void drawEngField(int idx) {
    int x = btnX(idx % BTN_COLS), y = btnY(idx / BTN_COLS);
    lcd.fillRoundRect(x, y, BTN_W, BTN_H, 10, COLOR_DARK_PANEL);
    lcd.drawRoundRect(x, y, BTN_W, BTN_H, 10, COL_ORANGE);

    lcd.fillRoundRect(x+8, y+8, 28, 20, 4, COL_ORANGE);
    lcd.setFont(&FONT_SMALL); lcd.setTextDatum(MC_DATUM);
    lcd.setTextColor(COLOR_BG);
    char ch[3]; snprintf(ch, sizeof(ch), "%d", idx+1);
    lcd.drawString(ch, x+22, y+18);

    lcd.setFont(&FONT_BTN); lcd.setTextColor(COL_ORANGE);
    lcd.drawString(engDraft[idx], x+BTN_W/2, y+BTN_H/2);

    lcd.setFont(&FONT_SMALL); lcd.setTextColor(0x4208);
    lcd.drawString("TAP TO RENAME", x+BTN_W/2, y+BTN_H-18);
}

void drawEngScreen() {
    lcd.fillScreen(COLOR_BG);
    lcd.fillRect(0, 0, 800, BTN_Y-1, COLOR_DARK_PANEL);
    lcd.setFont(&FONT_TITLE); lcd.setTextDatum(MC_DATUM);
    lcd.setTextColor(COL_ORANGE);
    lcd.drawString("ENGINEERING MODE", 400, 32);
    lcd.setFont(&FONT_SMALL); lcd.setTextColor(0x4208);
    lcd.drawString("HOLD CHANNEL TO RENAME  |  ACCEPT = SAVE  |  EXIT = CANCEL", 400, 66);
    lcd.drawFastHLine(0, BTN_Y-1, 800, COL_ORANGE);

    for (int i = 0; i < OUTPUT_COUNT; i++) drawEngField(i);

    int halfW = (800 - BTN_X*2 - BTN_GAP) / 2;
    int rx = BTN_X + halfW + BTN_GAP;
    lcd.fillRoundRect(BTN_X, MASTER_Y, halfW, MASTER_H, 10, COLOR_DARK_GREEN);
    lcd.drawRoundRect(BTN_X, MASTER_Y, halfW, MASTER_H, 10, COLOR_NEON_GREEN);
    lcd.setFont(&FONT_BTN); lcd.setTextDatum(MC_DATUM);
    lcd.setTextColor(COLOR_NEON_GREEN);
    lcd.drawString("ACCEPT", BTN_X+halfW/2, MASTER_Y+MASTER_H/2);

    lcd.fillRoundRect(rx, MASTER_Y, halfW, MASTER_H, 10, COLOR_DARK_RED);
    lcd.drawRoundRect(rx, MASTER_Y, halfW, MASTER_H, 10, COLOR_RED);
    lcd.setTextColor(COLOR_RED);
    lcd.drawString("EXIT", rx+halfW/2, MASTER_Y+MASTER_H/2);
}

void handleEngTouch(int tx, int ty) {
    for (int i = 0; i < OUTPUT_COUNT; i++) {
        int x = btnX(i % BTN_COLS), y = btnY(i / BTN_COLS);
        if (tx >= x && tx < x+BTN_W && ty >= y && ty < y+BTN_H) {
            oskTargetIdx = i;
            strncpy(oskBuf, engDraft[i], 16); oskBuf[16] = '\0';
            appState = ST_OSK;
            lcd.fillScreen(COLOR_BG);
            return;
        }
    }
    int halfW = (800 - BTN_X*2 - BTN_GAP) / 2;
    if (ty >= MASTER_Y && ty < MASTER_Y+MASTER_H) {
        if (tx >= BTN_X && tx < BTN_X+halfW) {
            // ACCEPT
            for (int i = 0; i < OUTPUT_COUNT; i++) {
                strncpy(channelNames[i], engDraft[i], 16);
                channelNames[i][16] = '\0';
            }
            saveLabels();
            appState = ST_MAIN;
            drawMainScreen();
        } else {
            // EXIT
            appState = ST_MAIN;
            drawMainScreen();
        }
    }
}

// ============================================================
//  ON-SCREEN KEYBOARD
// ============================================================
void drawOskTextBox() {
    lcd.fillRect(10, 60, 780, 105, 0x0821);
    lcd.drawRect(10, 60, 780, 105, COLOR_NEON_GREEN);
    lcd.setFont(&FONT_BTN); lcd.setTextDatum(ML_DATUM);
    lcd.setTextColor(COLOR_NEON_GREEN);
    lcd.drawString(oskBuf, 22, 112);

    // char count bottom-right of box
    char cnt[12]; snprintf(cnt, sizeof(cnt), "%d/16", (int)strlen(oskBuf));
    lcd.setFont(&FONT_SMALL); lcd.setTextDatum(MR_DATUM);
    lcd.setTextColor(0x4208);
    lcd.drawString(cnt, 788, 190);
}

void drawOskRow(const char* keys, int nkeys, int x0, int y) {
    for (int i = 0; i < nkeys; i++) {
        int kx = x0 + i*(OSK_KEY_W+OSK_KEY_GAP);
        char c = oskShift ? keys[i] : (char)(keys[i] + 32);
        char s[2] = {c, '\0'};
        lcd.fillRoundRect(kx, y, OSK_KEY_W, OSK_KEY_H, 6, COLOR_DARK_PANEL);
        lcd.drawRoundRect(kx, y, OSK_KEY_W, OSK_KEY_H, 6, 0x2945);
        lcd.setFont(&FONT_BTN); lcd.setTextDatum(MC_DATUM);
        lcd.setTextColor(COLOR_WHITE);
        lcd.drawString(s, kx+OSK_KEY_W/2, y+OSK_KEY_H/2);
    }
}

void drawOskScreen() {
    lcd.fillScreen(COLOR_BG);

    // Header
    lcd.fillRect(0, 0, 800, 50, COLOR_DARK_PANEL);
    lcd.setFont(&FONT_SMALL); lcd.setTextDatum(MC_DATUM);
    lcd.setTextColor(COL_ORANGE);
    char hdr[32]; snprintf(hdr, sizeof(hdr), "RENAME  CHANNEL  %d", oskTargetIdx+1);
    lcd.drawString(hdr, 400, 25);

    drawOskTextBox();

    // Letter rows
    drawOskRow(OSK_R0, 10, 8,  OSK_R0_Y);
    drawOskRow(OSK_R1, 9,  48, OSK_R1_Y);
    drawOskRow(OSK_R2, 7,  48, OSK_R2_Y);

    // Backspace
    lcd.fillRoundRect(OSK_BKSP_X, OSK_R2_Y, OSK_BKSP_W, OSK_KEY_H, 6, COLOR_DARK_PANEL);
    lcd.drawRoundRect(OSK_BKSP_X, OSK_R2_Y, OSK_BKSP_W, OSK_KEY_H, 6, COLOR_DIM_GRAY);
    lcd.setFont(&FONT_SMALL); lcd.setTextDatum(MC_DATUM); lcd.setTextColor(COLOR_DIM_GRAY);
    lcd.drawString("< DEL", OSK_BKSP_X+OSK_BKSP_W/2, OSK_R2_Y+OSK_KEY_H/2);

    // Bottom row: CLR, SHIFT, SPACE, DONE
    lcd.fillRoundRect(OSK_CLR_X,   OSK_R3_Y, OSK_CLR_W,   OSK_KEY_H, 6, COLOR_DARK_RED);
    lcd.drawRoundRect(OSK_CLR_X,   OSK_R3_Y, OSK_CLR_W,   OSK_KEY_H, 6, COLOR_RED);
    lcd.setFont(&FONT_SMALL); lcd.setTextColor(COLOR_RED);
    lcd.drawString("CLR", OSK_CLR_X+OSK_CLR_W/2, OSK_R3_Y+OSK_KEY_H/2);

    uint16_t shiftFill = oskShift ? COLOR_DARK_GREEN  : COLOR_DARK_PANEL;
    uint16_t shiftBord = oskShift ? COLOR_NEON_GREEN  : COLOR_DIM_GRAY;
    uint16_t shiftText = oskShift ? COLOR_NEON_GREEN  : COLOR_DIM_GRAY;
    lcd.fillRoundRect(OSK_SHIFT_X, OSK_R3_Y, OSK_SHIFT_W, OSK_KEY_H, 6, shiftFill);
    lcd.drawRoundRect(OSK_SHIFT_X, OSK_R3_Y, OSK_SHIFT_W, OSK_KEY_H, 6, shiftBord);
    lcd.setTextColor(shiftText);
    lcd.drawString(oskShift ? "ABC" : "abc", OSK_SHIFT_X+OSK_SHIFT_W/2, OSK_R3_Y+OSK_KEY_H/2);

    lcd.fillRoundRect(OSK_SPC_X,   OSK_R3_Y, OSK_SPC_W,   OSK_KEY_H, 6, COLOR_DARK_PANEL);
    lcd.drawRoundRect(OSK_SPC_X,   OSK_R3_Y, OSK_SPC_W,   OSK_KEY_H, 6, 0x2945);
    lcd.setTextColor(COLOR_DIM_GRAY);
    lcd.drawString("SPACE", OSK_SPC_X+OSK_SPC_W/2, OSK_R3_Y+OSK_KEY_H/2);

    lcd.fillRoundRect(OSK_DONE_X,  OSK_R3_Y, OSK_DONE_W,  OSK_KEY_H, 6, COLOR_DARK_GREEN);
    lcd.drawRoundRect(OSK_DONE_X,  OSK_R3_Y, OSK_DONE_W,  OSK_KEY_H, 6, COLOR_NEON_GREEN);
    lcd.setTextColor(COLOR_NEON_GREEN);
    lcd.drawString("DONE", OSK_DONE_X+OSK_DONE_W/2, OSK_R3_Y+OSK_KEY_H/2);
}

// Returns char, '\b'=backspace, '\x01'=CLR, '\x02'=DONE, '\x03'=SHIFT, ' '=space, 0=miss
char oskHitTest(int tx, int ty) {
    if (ty >= OSK_R0_Y && ty < OSK_R0_Y+OSK_KEY_H) {
        for (int i = 0; i < 10; i++) {
            int kx = oskR0X(i);
            if (tx >= kx && tx < kx+OSK_KEY_W) return OSK_R0[i];
        }
    }
    if (ty >= OSK_R1_Y && ty < OSK_R1_Y+OSK_KEY_H) {
        for (int i = 0; i < 9; i++) {
            int kx = oskR1X(i);
            if (tx >= kx && tx < kx+OSK_KEY_W) return OSK_R1[i];
        }
    }
    if (ty >= OSK_R2_Y && ty < OSK_R2_Y+OSK_KEY_H) {
        for (int i = 0; i < 7; i++) {
            int kx = oskR2X(i);
            if (tx >= kx && tx < kx+OSK_KEY_W) return OSK_R2[i];
        }
        if (tx >= OSK_BKSP_X && tx < OSK_BKSP_X+OSK_BKSP_W) return '\b';
    }
    if (ty >= OSK_R3_Y && ty < OSK_R3_Y+OSK_KEY_H) {
        if (tx >= OSK_CLR_X   && tx < OSK_CLR_X+OSK_CLR_W)     return '\x01';
        if (tx >= OSK_SHIFT_X && tx < OSK_SHIFT_X+OSK_SHIFT_W)  return '\x03';
        if (tx >= OSK_SPC_X   && tx < OSK_SPC_X+OSK_SPC_W)     return ' ';
        if (tx >= OSK_DONE_X  && tx < OSK_DONE_X+OSK_DONE_W)   return '\x02';
    }
    return 0;
}

void handleOskTouch(int tx, int ty) {
    char c = oskHitTest(tx, ty);
    if (!c) return;
    int len = strlen(oskBuf);

    if (c == '\x03') {
        oskShift = !oskShift;
        drawOskScreen();
        return;
    } else if (c == '\x01') {
        oskBuf[0] = '\0';
    } else if (c == '\b') {
        if (len > 0) oskBuf[len-1] = '\0';
    } else if (c == '\x02') {
        // DONE — commit
        const char* val = (strlen(oskBuf) > 0) ? oskBuf : DEFAULT_LABELS[oskTargetIdx];
        strncpy(engDraft[oskTargetIdx], val, 16);
        engDraft[oskTargetIdx][16] = '\0';
        appState = ST_ENG;
        drawEngScreen();
        return;
    } else {
        char out = (c >= 'A' && c <= 'Z' && !oskShift) ? (char)(c + 32) : c;
        if (len < 16) { oskBuf[len] = out; oskBuf[len+1] = '\0'; }
    }
    drawOskTextBox();
}

// ============================================================
//  BACKLIGHT PWM HELPERS
// ============================================================
void blSet(int duty) {
    ledcWrite(BL_LEDC_CHAN, duty);
}

void blFade(int from, int to, int ms) {
    int diff = to - from;
    if (diff == 0) return;
    int steps = abs(diff);
    int stepDelay = ms / steps;
    if (stepDelay < 1) stepDelay = 1;
    int dir = (diff > 0) ? 1 : -1;
    for (int v = from; v != to; v += dir) {
        ledcWrite(BL_LEDC_CHAN, v);
        delay(stepDelay);
    }
    ledcWrite(BL_LEDC_CHAN, to);
}

// ============================================================
//  STARTUP SCREEN  — static text, backlight fade in/out
// ============================================================
void playStartupAnimation() {
    // Draw text once while backlight is off
    lcd.fillScreen(COLOR_BG);
    lcd.setTextDatum(MC_DATUM);

    lcd.setFont(&FONT_TITLE);
    lcd.setTextColor(COLOR_NEON_GREEN);
    lcd.drawString("RIDE OR DIE", SCREEN_W / 2, 175);
    lcd.drawString("CUSTOMS", SCREEN_W / 2, 315);

    // Fade in
    blFade(0, 255, 1200);

    // Hold
    delay(1800);

    // Fade out
    blFade(255, 0, 800);
}

// ============================================================
//  MATRIX RAIN SCREEN SAVER
// ============================================================
#define MTX_CW   12               // cell width  (px) — 6×8 font at size 2
#define MTX_CH   16               // cell height (px)
#define MTX_COLS (800 / MTX_CW)  // 66 columns
#define MTX_ROWS (480 / MTX_CH)  // 30 rows

struct MtxCol {
    int16_t  head;
    uint8_t  tail;
    uint16_t speedMs;
    uint32_t nextMs;
};
static MtxCol mtx[MTX_COLS];

// Green fade palette: index 0 = head (white), ascending = dimmer
static const uint16_t MTX_PAL[] = {
    0xFFFF, 0x67E0, 0x07E0, 0x0580, 0x0360, 0x01A0, 0x00C0, 0x0060
};
#define MTX_PAL_SZ 8

static char mtxRandChar() {
    static const char pool[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789@#$%&*<>?/";
    return pool[random(sizeof(pool) - 1)];
}

void initMatrix() {
    lcd.fillScreen(0x0000);
    for (int i = 0; i < MTX_COLS; i++) {
        mtx[i].head    = -(int16_t)random(MTX_ROWS);
        mtx[i].tail    = 4 + random(10);
        mtx[i].speedMs = 40 + random(120);
        mtx[i].nextMs  = millis() + random(3000);
    }
}

void updateMatrix() {
    uint32_t now = millis();
    for (int i = 0; i < MTX_COLS; i++) {
        if (now < mtx[i].nextMs) continue;
        mtx[i].nextMs = now + mtx[i].speedMs;

        int16_t h = mtx[i].head;
        int     t = mtx[i].tail;
        int     x = i * MTX_CW;

        // Draw from tail up to head so head char overwrites tail
        for (int j = t; j >= 0; j--) {
            int row = h - j;
            if (row < 0 || row >= MTX_ROWS) continue;
            int pal = (j == 0) ? 0 : min(j, MTX_PAL_SZ - 1);
            lcd.drawChar(x, row * MTX_CH, mtxRandChar(), MTX_PAL[pal], (uint16_t)0x0000, 2);
        }

        // Erase cell just behind the tail
        int eraseRow = h - t - 1;
        if (eraseRow >= 0 && eraseRow < MTX_ROWS)
            lcd.fillRect(x, eraseRow * MTX_CH, MTX_CW, MTX_CH, 0x0000);

        mtx[i].head++;

        // Reset column once tail has scrolled off screen
        if (h - t > MTX_ROWS) {
            mtx[i].head    = -(int16_t)random(MTX_ROWS / 2);
            mtx[i].tail    = 4 + random(10);
            mtx[i].speedMs = 40 + random(120);
        }
    }
}

// ============================================================
//  SETUP / LOOP
// ============================================================
static uint32_t lastTouchMs = 0;

void setup() {
    Serial.begin(DEBUG_BAUD);
    NANO_SERIAL.begin(NANO_BAUD, SERIAL_8N1, NANO_RX, NANO_TX);
    loadLabels();
    ledcSetup(BL_LEDC_CHAN, BL_LEDC_FREQ, BL_LEDC_BITS);
    ledcAttachPin(TFT_BL, BL_LEDC_CHAN);
    blSet(0);              // backlight off while display initialises

    lcd.begin();
    lcd.fillScreen(COLOR_BG);
    touch_init();

    playStartupAnimation(); // draws text in the dark, fades in, holds, fades out

    drawMainScreen();       // draw ops screen in the dark
    blFade(0, 255, 500);   // smooth fade up to full brightness
}

void loop() {
    uint32_t now = millis();

    // GT911 only reports at ~100Hz; the loop runs much faster.
    // Use a 150ms sticky window so a missed poll doesn't cancel the hold timer.
    bool rawTouch = touch_has_signal() && touch_touched();
    if (rawTouch) { lastRawTouchMs = now; lastActivityMs = now; }

    bool wasDown   = fingerDown;
    fingerDown     = rawTouch || (now - lastRawTouchMs < FINGER_UP_MS);
    bool newTouch  = fingerDown && !wasDown;
    bool released  = !fingerDown && wasDown;
    int tx = touch_last_x, ty = touch_last_y;

    // ---- Screen saver: enter after 3 min idle ----
    if (appState != ST_SAVER && (now - lastActivityMs) >= SAVER_TIMEOUT_MS) {
        preSaverState = appState;
        appState = ST_SAVER;
        initMatrix();
    }

    // Deferred screen draws (after state change inside handlers)
    static AppState lastDrawnState = ST_MAIN;
    if (appState != lastDrawnState) {
        lastDrawnState = appState;
        if (appState == ST_PIN)   drawPinScreen();
        if (appState == ST_ENG)   drawEngScreen();
        if (appState == ST_OSK)   drawOskScreen();
        // ST_SAVER and ST_MAIN handle their own draws
    }

    switch (appState) {
    case ST_MAIN:
        // ---- Header hold → engineering access ----
        if (fingerDown) {
            if (ty < BTN_Y) updateHoldProgress(ty);
            else cancelHold();
        }
        if (released) cancelHold();

        // ---- New touch: record which button was pressed ----
        if (newTouch && ty >= BTN_Y) {
            holdBtnIdx = -1;
            // channel buttons
            for (int i = 0; i < OUTPUT_COUNT; i++) {
                int x = btnX(i%BTN_COLS), y = btnY(i/BTN_COLS);
                if (tx>=x && tx<x+BTN_W && ty>=y && ty<y+BTN_H) {
                    if (i == MOMENTARY_CH) {
                        // Momentary: ON on press, OFF on release, no flash
                        holdBtnIdx     = i;
                        holdBtnStartMs = now;
                        relayState[i]  = true;
                        sendRelayCommand(i+1, true);
                        drawOutputButton(i);
                        break;
                    }
                    // Tapping a latched button cancels its latch
                    if (flashLatched[i]) {
                        flashLatched[i]  = false;
                        latchFlashOn[i]  = false;
                        relayState[i]    = false;
                        sendRelayCommand(i+1, false);
                        drawOutputButton(i);
                        break;
                    }
                    holdBtnIdx     = i;
                    holdBtnStartMs = now;
                    flashActive    = false;
                    break;
                }
            }
            // master buttons
            int halfW = (800 - BTN_X*2 - BTN_GAP) / 2;
            if (ty >= MASTER_Y && ty < MASTER_Y+MASTER_H) {
                if (tx >= BTN_X && tx < BTN_X+halfW) {
                    // Light Em Up — all ON except momentary ch6, start hold timer for strobe
                    masterHoldActive   = true;
                    masterHoldStartMs  = now;
                    masterStrobeActive = false;
                    for (int i=0;i<OUTPUT_COUNT;i++) {
                        if (i == MOMENTARY_CH) continue;
                        relayState[i] = true;
                        sendRelayCommand(i+1, true);
                    }
                    for (int i=0;i<OUTPUT_COUNT;i++) drawOutputButton(i);
                } else {
                    // Master Off — cancel strobe and all latched flashes
                    masterStrobeActive = false;
                    masterHoldActive   = false;
                    for (int j=0;j<OUTPUT_COUNT;j++) {
                        flashLatched[j] = false;
                        latchFlashOn[j] = false;
                    }
                    for (int i=0;i<OUTPUT_COUNT;i++) relayState[i]=false;
                    sendRelayCommand(0, false);
                    for (int i=0;i<OUTPUT_COUNT;i++) drawOutputButton(i);
                }
            }
        }

        // ---- While holding a channel button ----
        if (fingerDown && holdBtnIdx >= 0 && holdBtnIdx != MOMENTARY_CH) {
            uint32_t held = now - holdBtnStartMs;
            // Activate flash after 2 s
            if (!flashActive && held >= BTN_HOLD_MS) {
                flashActive = true;
                flashOn     = true;
                flashLastMs = now;
                relayState[holdBtnIdx] = true;
                sendRelayCommand(holdBtnIdx+1, true);
                drawOutputButton(holdBtnIdx);
            }
            // Toggle held flash every 500 ms
            if (flashActive && (now - flashLastMs >= FLASH_PERIOD)) {
                flashOn     = !flashOn;
                flashLastMs = now;
                relayState[holdBtnIdx] = flashOn;
                sendRelayCommand(holdBtnIdx+1, flashOn);
                drawOutputButton(holdBtnIdx);
            }
        }

        // ---- Latched flash: each channel runs independently ----
        for (int i = 0; i < OUTPUT_COUNT; i++) {
            if (flashLatched[i] && (now - latchFlashLastMs[i] >= FLASH_PERIOD)) {
                latchFlashOn[i]    = !latchFlashOn[i];
                latchFlashLastMs[i] = now;
                relayState[i]       = latchFlashOn[i];
                sendRelayCommand(i+1, latchFlashOn[i]);
                drawOutputButton(i);
            }
        }

        // ---- Light Em Up hold → master strobe at 4 s ----
        if (fingerDown && masterHoldActive && !masterStrobeActive) {
            if (now - masterHoldStartMs >= MASTER_STROBE_HOLD_MS) {
                masterStrobeActive = true;
                for (int i = 0; i < OUTPUT_COUNT; i++) {
                    flashLatched[i] = false;
                    latchFlashOn[i] = false;
                    relayState[i]   = false;
                }
                sendRelayCommand(0, false);
                strobeChIdx   = 0;
                strobePhaseOn = true;
                strobeLastMs  = now;
                sendRelayCommand(1, true);
                relayState[0] = true;
                for (int i = 0; i < OUTPUT_COUNT; i++) drawOutputButton(i);
            }
        }
        if (released && masterHoldActive) masterHoldActive = false;

        // ---- Master strobe: sequential chase through all channels ----
        if (masterStrobeActive) {
            if (strobePhaseOn && (now - strobeLastMs) >= STROBE_ON_MS) {
                sendRelayCommand(strobeChIdx + 1, false);
                relayState[strobeChIdx] = false;
                drawOutputButton(strobeChIdx);
                strobePhaseOn = false;
                strobeLastMs  = now;
            } else if (!strobePhaseOn && (now - strobeLastMs) >= STROBE_OFF_MS) {
                do { strobeChIdx = (strobeChIdx + 1) % OUTPUT_COUNT; } while (strobeChIdx == MOMENTARY_CH);
                strobePhaseOn = true;
                strobeLastMs  = now;
                sendRelayCommand(strobeChIdx + 1, true);
                relayState[strobeChIdx] = true;
                drawOutputButton(strobeChIdx);
            }
        }

        // ---- Release a channel button ----
        if (released && holdBtnIdx >= 0) {
            if (holdBtnIdx == MOMENTARY_CH) {
                // Momentary: OFF on release
                relayState[holdBtnIdx] = false;
                sendRelayCommand(holdBtnIdx+1, false);
                drawOutputButton(holdBtnIdx);
                holdBtnIdx = -1;
            } else if (flashActive) {
                if (now - holdBtnStartMs >= BTN_HOLD_LATCH_MS) {
                    // Held 4 s+: latch the flash, keep it going after release
                    int b = holdBtnIdx;
                    flashLatched[b]     = true;
                    latchFlashOn[b]     = flashOn;
                    latchFlashLastMs[b] = flashLastMs;
                    flashActive         = false;
                } else {
                    // Held 2–4 s: stop flash, relay off
                    flashActive            = false;
                    relayState[holdBtnIdx] = false;
                    sendRelayCommand(holdBtnIdx+1, false);
                    drawOutputButton(holdBtnIdx);
                }
            } else {
                // Short tap — normal toggle
                relayState[holdBtnIdx] = !relayState[holdBtnIdx];
                sendRelayCommand(holdBtnIdx+1, relayState[holdBtnIdx]);
                drawOutputButton(holdBtnIdx);
            }
            holdBtnIdx = -1;
        }
        break;

    case ST_PIN:
        if (newTouch) handlePinTouch(tx, ty);
        break;

    case ST_ENG:
        if (newTouch) handleEngTouch(tx, ty);
        break;

    case ST_OSK:
        if (newTouch) handleOskTouch(tx, ty);
        break;

    case ST_SAVER:
        updateMatrix();
        if (newTouch) {
            appState = preSaverState;
            lastActivityMs = now;
            lastDrawnState = ST_SAVER;
            lcd.fillScreen(COLOR_BG);
            if (preSaverState == ST_MAIN)     drawMainScreen();
            else if (preSaverState == ST_PIN) drawPinScreen();
            else if (preSaverState == ST_ENG) drawEngScreen();
            else if (preSaverState == ST_OSK) drawOskScreen();
        }
        break;
    }

    while (NANO_SERIAL.available()) NANO_SERIAL.read();
}
