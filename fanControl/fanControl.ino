/*
 * Chasis Fan Control — Simple dual PWM fan controller with WS2812 speed gauge.
 *
 * MCU:  CH32V003F4P6 @ 48 MHz (TSSOP-20)
 * Fans: 2× 4-pin PWM at 25 kHz (PD5, PD6)
 * LEDs: 4× WS2812C via SPI1_MOSI (PC6) at 3 MHz
 * UI:   Single button on PC3 (INPUT_PULLUP):
 *         click        → cycle 0% → 25% → 50% → 75% → 100%
 *         double-click → 100 %
 *         long-press   →   0 %
 *
 * Pin map:
 *   PD5  TIM1_CH1   Fan 1 PWM
 *   PD6  TIM1_CH2   Fan 2 PWM
 *   PC6  SPI1_MOSI  WS2812 data
 *   PC3  GPIO       Button
 *   PD1  SWIO       Debug
 *   PD7  NRST       Reset
 */

/* ================================================================
 * SECTION 1 — Constants
 * ================================================================ */

// ---- Fan PWM (TIM1, 25 kHz) ----
#define PWM_PERIOD        1919    // 48 MHz / 25000 - 1

// ---- Speed levels (5 levels, 0–100 %) ----
#define NUM_LEVELS        5
#define LEVEL_OFF         0
#define LEVEL_25          1
#define LEVEL_50          2
#define LEVEL_75          3
#define LEVEL_100         4

static const uint8_t  levelPct[NUM_LEVELS]  = {   0,  25,  50,  75, 100 };
static const uint16_t levelDuty[NUM_LEVELS] = {   0, 480, 960,1440,1919 };

// ---- Level colours (GRB order: [0]=G, [1]=R, [2]=B) ----
// 0%:off  25%:blue  50%:green  75%:yellow  100%:red
static const uint8_t levelColor[NUM_LEVELS][3] = {
    {   0,   0,   0 },   //  0% — off
    {   0,   0, 255 },   // 25% — blue
    { 255,   0,   0 },   // 50% — green
    { 128, 255,   0 },   // 75% — yellow
    {   0, 255,   0 },   // 100% — red
};

// ---- WS2812 via SPI ----
#define NUM_LEDS          4
#define SPI_BYTES_PER_LED 12     // 24 bits × 4 SPI-bits/bit / 8

// ---- Button ----
#define BTN_PIN           PC3
#define BTN_DEBOUNCE_MS   30
#define BTN_SHORT_MIN_MS  50
#define BTN_SHORT_MAX_MS  500
#define BTN_DOUBLE_GAP_MS 300
#define BTN_LONG_MS       1000

// ---- WS2812 bit-expansion LUT (2 input bits → 1 SPI byte) ----
// 1→1110, 0→1000.  Upper nibble = bit1, lower nibble = bit0.
static const uint8_t wsLUT[4] = {
    0x88,  // 00
    0x8E,  // 01
    0xE8,  // 10
    0xEE,  // 11
};

/* ================================================================
 * SECTION 2 — Global State
 * ================================================================ */

static uint8_t  g_level  = LEVEL_OFF;   // 0–4
static bool     g_dirty  = true;

// Display buffers
static uint8_t  g_leds[NUM_LEDS][3];                    // GRB: [i][0]=G, [1]=R, [2]=B
static uint8_t  g_spiBuf[NUM_LEDS * SPI_BYTES_PER_LED];

/* ================================================================
 * SECTION 3 — Fan PWM (TIM1, register-level)
 * ================================================================ */

static void pwm_begin(void) {
    // Enable GPIOD + TIM1 + AFIO clocks
    RCC->APB2PCENR |= (1 << 5)    // GPIOD
                   |  (1 << 11)   // TIM1
                   |  (1 << 0);   // AFIO

    // PD5, PD6 → alternate push-pull 50 MHz
    uint32_t cfglr = GPIOD->CFGLR;
    cfglr &= ~(0xFFUL << 20);
    cfglr |=  (0xBBUL << 20);     // CNF=10(af-pp), MODE=11(50 MHz)
    GPIOD->CFGLR = cfglr;

    // Remap: TIM1_CH1 → PD5, TIM1_CH2 → PD6 (PartialRemap1, bit 6)
    AFIO->PCFR1 |= (1 << 6);

    // Timer base: 48 MHz / 1 / 1920 = 25 kHz
    TIM1->PSC   = 0;
    TIM1->ATRLR = PWM_PERIOD;
    TIM1->CNT   = 0;

    // CH1 + CH2: PWM mode 1, preload
    TIM1->CHCTLR1 = (6 << 4) | (1 << 3);
    TIM1->CHCTLR2 = (6 << 4) | (1 << 3);

    // Enable outputs CC1E + CC2E
    TIM1->CCER |= (1 << 0) | (1 << 4);

    // Start at 0 %
    TIM1->CH1CVR = 0;
    TIM1->CH2CVR = 0;

    // MOE + ARPE + CEN
    TIM1->BDTR  |= (1 << 15);      // MOE
    TIM1->CTLR1 |= (1 << 7)        // ARPE
                |  (1 << 0);       // CEN
}

static void pwm_set(uint8_t pct) {
    uint16_t ccr;
    if      (pct >= 100) ccr = levelDuty[LEVEL_100];
    else if (pct == 0)   ccr = levelDuty[LEVEL_OFF];
    else                 ccr = (uint16_t)((uint32_t)pct * PWM_PERIOD / 100);
    TIM1->CH1CVR = ccr;
    TIM1->CH2CVR = ccr;
}

/* ================================================================
 * SECTION 4 — WS2812 via SPI1 (register-level)
 * ================================================================ */

static void ws2812_begin(void) {
    // Enable SPI1 + GPIOC clocks
    RCC->APB2PCENR |= (1 << 12)   // SPI1
                   |  (1 << 4);   // GPIOC

    // PC6 → alternate push-pull 50 MHz (SPI1_MOSI)
    uint32_t cfglr = GPIOC->CFGLR;
    cfglr &= ~(0xFU << 24);
    cfglr |=  (0xBU << 24);       // CNF=10(af-pp), MODE=11(50 MHz)
    GPIOC->CFGLR = cfglr;

    // SPI1: master, mode 0, MSB first, BR=011=fPCLK/16=3 MHz
    SPI1->CTLR1 = (1 << 2)        // MSTR
               |  (3 << 3);       // BR = 011 = /16
    SPI1->CTLR2 = 0;
    SPI1->CTLR1 |= (1 << 6);      // SPE
}

static void ws2812_send(void) {
    // Expand g_leds[NUM_LEDS][3] (GRB) → g_spiBuf
    uint8_t *p = g_spiBuf;
    for (uint8_t i = 0; i < NUM_LEDS; i++) {
        for (uint8_t c = 0; c < 3; c++) {          // G, R, B
            uint8_t v = g_leds[i][c];
            *p++ = wsLUT[(v >> 6) & 3];
            *p++ = wsLUT[(v >> 4) & 3];
            *p++ = wsLUT[(v >> 2) & 3];
            *p++ = wsLUT[ v       & 3];
        }
    }

    // Transmit via hardware SPI
    for (uint16_t i = 0; i < sizeof(g_spiBuf); i++) {
        while (!(SPI1->STATR & (1 << 1)));          // wait TXE
        SPI1->DATAR = g_spiBuf[i];
    }
    while (SPI1->STATR & (1 << 7));                 // wait BSY

    delayMicroseconds(60);                           // RESET > 50 µs
}

/* ================================================================
 * SECTION 5 — Display update
 * ================================================================ */

static void dsp_update(void) {
    if (!g_dirty) return;
    g_dirty = false;

    // Bar graph: light N LEDs for level N, all others off.
    // Level 0 = all off; Level 4 = all 4 lit.
    for (uint8_t i = 0; i < NUM_LEDS; i++) {
        if (i < g_level) {
            g_leds[i][0] = levelColor[g_level][0];   // G
            g_leds[i][1] = levelColor[g_level][1];   // R
            g_leds[i][2] = levelColor[g_level][2];   // B
        } else {
            g_leds[i][0] = 0;
            g_leds[i][1] = 0;
            g_leds[i][2] = 0;
        }
    }

    ws2812_send();
}

/* ================================================================
 * SECTION 6 — Button gesture detection
 * ================================================================ */

static void btn_poll(uint32_t now) {
    static uint32_t tLast      = 0;
    static uint8_t  raw        = HIGH, lastRaw = HIGH;
    static uint8_t  deb        = HIGH, lastDeb = HIGH;
    static uint8_t  dbCnt      = 0;
    static uint32_t tPress     = 0;
    static uint32_t tRelease   = 0;
    static uint8_t  clicks     = 0;
    static bool     longDone   = false;

    // Limit to ~1 ms sampling
    if (now - tLast < 1) return;
    tLast = now;

    raw = digitalRead(BTN_PIN);   // LOW = pressed

    // Debounce
    if (raw == lastRaw) { if (dbCnt < BTN_DEBOUNCE_MS) dbCnt++; }
    else                { dbCnt = 0; }
    lastRaw = raw;
    if (dbCnt >= BTN_DEBOUNCE_MS) deb = raw;

    // Edge detection
    if (deb == LOW  && lastDeb == HIGH) { tPress = now; longDone = false; }
    if (deb == HIGH && lastDeb == LOW)  {
        uint32_t held = now - tPress;
        if (held >= BTN_SHORT_MIN_MS && held <= BTN_SHORT_MAX_MS) clicks++;
        tRelease = now;
    }
    lastDeb = deb;

    // Long-press → 0 %  (fires once while held)
    if (deb == LOW && !longDone && (now - tPress) > BTN_LONG_MS) {
        longDone  = true;
        clicks    = 0;
        g_level   = LEVEL_OFF;
        pwm_set(levelPct[g_level]);
        g_dirty   = true;
    }

    // Resolve clicks after gap
    if (clicks && (now - tRelease) > BTN_DOUBLE_GAP_MS) {
        if (clicks == 1) {
            // Single click → cycle 0→1→2→3→4→0…
            g_level = (g_level + 1) % NUM_LEVELS;
        } else {
            // Double click → 100 %
            g_level = LEVEL_100;
        }
        pwm_set(levelPct[g_level]);
        g_dirty = true;
        clicks  = 0;
    }
}

/* ================================================================
 * SECTION 7 — Arduino entry points
 * ================================================================ */

void setup(void) {
    pinMode(BTN_PIN, INPUT_PULLUP);

    pwm_begin();
    ws2812_begin();

    // Show initial state (level 0 = all off)
    dsp_update();
}

void loop(void) {
    uint32_t now = millis();
    btn_poll(now);
    dsp_update();
}
