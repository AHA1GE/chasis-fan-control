/*
 * Chasis Fan Control — Dual PWM fan controller with WS2812 battery gauge.
 *
 * MCU:  CH32V003F4P6 @ 48 MHz (TSSOP-20)
 * Fans: 2× 4-pin PWM at 25 kHz, 2× tachometer pulse-counting
 * LEDs: 4× WS2812C daisy-chained via SPI1_MOSI (PC6) at 3 MHz
 * Batt: 1S–3S LiPo auto-detect on PA2 (10k / 4k7 divider)
 * UI:   Single button on PC3:
 *         click       → cycle 0% → 25% → 50% → 75%
 *         double-click → 100%
 *         long-press   → 0%
 *
 * Pin map:
 *   PD5  TIM1_CH1   Fan 1 PWM     PD2  EXTI2   Fan 1 tach
 *   PD6  TIM1_CH2   Fan 2 PWM     PA1  EXTI1   Fan 2 tach
 *   PC6  SPI1_MOSI  WS2812 data   PA2  ADC     Battery voltage
 *   PC3  GPIO       Button         PD1  SWIO   Debug
 *   PD7  NRST       Reset          PC2  —      (spare)
 */

/* ================================================================
 * SECTION 1 — Constants
 * ================================================================ */

// ---- Fan PWM (TIM1, 25 kHz) ----
#define PWM_PERIOD        1919    // 48 MHz / 25000 − 1
#define DUTY_0             0
#define DUTY_25          480
#define DUTY_50          960
#define DUTY_75         1440
#define DUTY_100        1919

// ---- Tach ----
#define TACH1_PIN         PD2    // Fan 1
#define TACH2_PIN         PA1    // Fan 2
#define TACH_WINDOW_MS    2000

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

// ---- Battery ADC ----
#define BATT_PIN          PA2
// Divider: BAT+ → 10k → VDETECT(PA2) → 4k7 → GND.  Ratio = 4.7/14.7
// vBatt_mV = adc × 5000 × 147 / (1024 × 47)
#define ADC_3S_THRESHOLD  570     // >570 → 3S  (VDETECT > 2.78 V)
#define ADC_2S_THRESHOLD  370     // 370–570 → 2S
#define ADC_1S_MIN        200     // <200 → invalid

#define BATT_FULL_MV      4200    // 4.2 V/cell = 100 %
#define BATT_EMPTY_MV     3000    // 3.0 V/cell =   0 %

// ---- Intervals ----
#define ADC_INTERVAL_MS    200
#define DSP_INTERVAL_MS    100
#define RAINBOW_FRAME_MS   50
#define RAINBOW_FRAMES     40     // 40 × 50 ms = 2 s
#define ERROR_BLINK_MS     100    // 5 Hz

// ---- Display / colour ----
#define MIN_RPM           100     // fan minimum speed at 0% PWM; peg to blue
#define MAX_RPM           20000   // fan maximum speed at 100% PWM / 12V; peg to red
#define RPM_HALF_SPAN     ((MAX_RPM - MIN_RPM) / 2)   
#define RPM_MIDPOINT      ((MIN_RPM + MAX_RPM) / 2)   

// ---- Duty cycle table (single-click) ----
static const uint8_t dutyTable[4] = { 0, 25, 50, 75 };

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

enum State : uint8_t {
    S_STARTUP = 0,
    S_RUNNING,
    S_ERROR,
};
static State    g_state   = S_STARTUP;

// Battery
static uint8_t  g_cells   = 0;     // 0=error, 1, 2, 3
static uint8_t  g_socPct  = 0;     // 0–100
static uint8_t  g_gauge   = 0;     // 0–4 LEDs lit
static uint32_t g_tAdc    = 0;

// Fan / tach
static volatile uint16_t g_tach[2] = {0, 0};  // [0]=Fan1, [1]=Fan2
static uint16_t g_rpm     = 0;
static uint8_t  g_fanMode = 0;      // 1=single, 2=dual
static uint32_t g_tTach    = 0;

// PWM
static uint8_t  g_dutyIdx = 0;      // 0-3=cycle(25/50/75), 4=100%

// Display
static uint8_t  g_leds[NUM_LEDS][3];      // GRB order: [i][0]=G, [1]=R, [2]=B
static uint8_t  g_spiBuf[NUM_LEDS * SPI_BYTES_PER_LED];
static uint32_t g_tDsp    = 0;
static bool     g_dirty   = true;

// Error blink
static uint32_t g_tBlink  = 0;
static bool     g_blinkOn = false;

// Startup phases
static uint32_t g_tPhase  = 0;
static uint8_t  g_phase   = 0;       // 0=rainbow, 1=spinup, 2=count, 3=done

/* ================================================================
 * SECTION 3 — Interrupt lock helpers (RISC-V)
 * ================================================================ */

// RISC-V: disable machine-mode interrupts via mstatus.MIE, return previous state
static inline uint32_t irq_lock(void) {
    uint32_t mstatus;
    __asm__ volatile("csrr %0, mstatus" : "=r"(mstatus));
    __asm__ volatile("csrw mstatus, %0" : : "r"(mstatus & ~0x8));
    return mstatus;
}
static inline void irq_restore(uint32_t prev) {
    __asm__ volatile("csrw mstatus, %0" : : "r"(prev));
}

/* ================================================================
 * SECTION 4 — Fan PWM (TIM1, register-level init)
 * ================================================================ */

static void pwm_begin(void) {
    // Enable GPIOD + TIM1 + AFIO clocks
    RCC->APB2PCENR |= (1 << 5)   // GPIOD
                   |  (1 << 11)  // TIM1
                   |  (1 << 0);  // AFIO

    // PD5, PD6 → alternate push-pull 50 MHz
    // GPIOD CFGLR: PD5=nibble 5 (bits 20-23), PD6=nibble 6 (bits 24-27)
    uint32_t cfglr = GPIOD->CFGLR;
    cfglr &= ~(0xFFUL << 20);
    cfglr |=  (0xBBUL << 20);      // CNF=10(af-pp), MODE=11(50 MHz)
    GPIOD->CFGLR = cfglr;

    // Remap: TIM1_CH1 → PD5, TIM1_CH2 → PD6 (PartialRemap1, bit 6)
    AFIO->PCFR1 |= (1 << 6);

    // Timer base: 48 MHz / 1 / 1920 = 25 kHz
    TIM1->PSC   = 0;
    TIM1->ATRLR = PWM_PERIOD;
    TIM1->CNT   = 0;

    // CH1 (PD5): PWM mode 1, preload
    // CHCTLR1: OC1M[2:0]=110(PWM1), OC1PE=1
    TIM1->CHCTLR1 = (6 << 4) | (1 << 3);

    // CH2 (PD6): PWM mode 1, preload
    TIM1->CHCTLR2 = (6 << 4) | (1 << 3);

    // Enable outputs CC1E + CC2E
    TIM1->CCER |= (1 << 0) | (1 << 4);

    // Start at 0 %
    TIM1->CH1CVR = 0;
    TIM1->CH2CVR = 0;

    // MOE + ARPE + CEN
    TIM1->BDTR  |= (1 << 15);      // MOE  (main output enable)
    TIM1->CTLR1 |= (1 << 7)        // ARPE (auto-reload preload)
                |  (1 << 0);       // CEN  (counter enable)
}

static void pwm_set(uint8_t pct) {
    uint16_t ccr;
    if (pct >= 100)     ccr = DUTY_100;
    else if (pct == 0)  ccr = DUTY_0;
    else                ccr = (uint16_t)((uint32_t)pct * PWM_PERIOD / 100);
    TIM1->CH1CVR = ccr;
    TIM1->CH2CVR = ccr;
}

/* ================================================================
 * SECTION 5 — WS2812 via SPI1 (register-level init)
 * ================================================================ */

static void ws2812_begin(void) {
    // Enable SPI1 + GPIOC clocks
    RCC->APB2PCENR |= (1 << 12)  // SPI1
                   |  (1 << 4);  // GPIOC

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
    // Expand g_leds[4][3] (GRB) → g_spiBuf[48]
    // Each colour byte → 4 SPI bytes (2 input bits per output byte)
    uint8_t *p = g_spiBuf;
    for (uint8_t i = 0; i < NUM_LEDS; i++) {
        for (uint8_t c = 0; c < 3; c++) {       // G, R, B
            uint8_t v = g_leds[i][c];
            *p++ = wsLUT[(v >> 6) & 3];
            *p++ = wsLUT[(v >> 4) & 3];
            *p++ = wsLUT[(v >> 2) & 3];
            *p++ = wsLUT[ v       & 3];
        }
    }

    // Transmit via hardware SPI — no IRQs disabled
    for (uint16_t i = 0; i < sizeof(g_spiBuf); i++) {
        while (!(SPI1->STATR & (1 << 1)));   // wait TXE
        SPI1->DATAR = g_spiBuf[i];
    }
    while (SPI1->STATR & (1 << 7));           // wait BSY

    delayMicroseconds(60);                     // RESET > 50 µs
}

/* ================================================================
 * SECTION 6 — Colour helpers
 * ================================================================ */

// HSV colour wheel → GRB.  pos 0..255 → R→G→B→R
static void colorWheel(uint8_t pos, uint8_t grb[3]) {
    if (pos < 85) {
        grb[0] = pos * 3;        grb[1] = 255 - pos * 3;  grb[2] = 0;
    } else if (pos < 170) {
        pos -= 85;
        grb[0] = 255 - pos * 3;  grb[1] = 0;              grb[2] = pos * 3;
    } else {
        pos -= 170;
        grb[0] = 0;              grb[1] = pos * 3;        grb[2] = 255 - pos * 3;
    }
}

// Linear heatmap: blue → green → red  (GRB order)
// ≤ MIN_RPM = pure blue,  ≥ MAX_RPM = pure red
static void rpmToHeatColor(uint16_t rpm, uint8_t grb[3]) {
    if (rpm <= MIN_RPM) {
        grb[0] = 0;   grb[1] = 0;   grb[2] = 255;   // pure blue
        return;
    }
    if (rpm >= MAX_RPM) {
        grb[0] = 0;   grb[1] = 255; grb[2] = 0;     // pure red
        return;
    }

    if (rpm < RPM_MIDPOINT) {
        // Blue → Green:  G rises 0→255,  B falls 255→0
        uint32_t t = rpm - MIN_RPM;
        uint8_t  g = (uint8_t)(t * 255 / RPM_HALF_SPAN);
        grb[0] = g;        grb[1] = 0;   grb[2] = 255 - g;
    } else {
        // Green → Red:  R rises 0→255,  G falls 255→0
        uint32_t t = rpm - RPM_MIDPOINT;
        uint8_t  r = (uint8_t)(t * 255 / RPM_HALF_SPAN);
        grb[0] = 255 - r;  grb[1] = r;   grb[2] = 0;
    }
}

/* ================================================================
 * SECTION 7 — Battery / ADC
 * ================================================================ */

static uint8_t batt_detect(void) {
    uint32_t sum = 0;
    for (int i = 0; i < 4; i++) {
        sum += analogRead(BATT_PIN);
        delay(2);
    }
    uint16_t avg = (uint16_t)(sum >> 2);
    if      (avg > ADC_3S_THRESHOLD)  return 3;
    else if (avg > ADC_2S_THRESHOLD)  return 2;
    else if (avg > ADC_1S_MIN)       return 1;
    else                              return 0;
}

static void batt_sample(void) {
    uint16_t raw = analogRead(BATT_PIN);

    // 4-sample rolling average
    static uint16_t buf[4] = {0};
    static uint8_t  idx    = 0;
    buf[idx] = raw;
    idx = (idx + 1) & 3;
    uint16_t adcAvg = (uint16_t)(((uint32_t)buf[0] + buf[1] + buf[2] + buf[3]) >> 2);

    // Battery mV
    uint32_t vBat  = ((uint32_t)adcAvg * 5000UL * 147UL) / (1024UL * 47UL);
    uint32_t vCell = vBat / g_cells;

    // SoC %
    if (vCell <= BATT_EMPTY_MV)      g_socPct = 0;
    else if (vCell >= BATT_FULL_MV)  g_socPct = 100;
    else g_socPct = (uint8_t)((100UL * (vCell - BATT_EMPTY_MV))
                               / (BATT_FULL_MV - BATT_EMPTY_MV));

    // Gauge LED count
    if      (g_socPct >= 75) g_gauge = 4;
    else if (g_socPct >= 50) g_gauge = 3;
    else if (g_socPct >= 25) g_gauge = 2;
    else if (g_socPct >= 5)  g_gauge = 1;
    else                     g_gauge = 0;
}

/* ================================================================
 * SECTION 8 — Tachometer ISRs + RPM
 * ================================================================ */

// Tach ISR callbacks — called from core's EXTI7_0_IRQHandler
static void tach1_cb(void) { g_tach[0]++; }
static void tach2_cb(void) { g_tach[1]++; }

static void tach_begin(void) {
    // Fan 1 tach: PD2 → EXTI2, rising edge
    attachInterrupt(TACH1_PIN, GPIO_Mode_IPU, tach1_cb, EXTI_Mode_Interrupt, EXTI_Trigger_Rising);
    // Fan 2 tach: PA1 → EXTI1, rising edge
    attachInterrupt(TACH2_PIN, GPIO_Mode_IPU, tach2_cb, EXTI_Mode_Interrupt, EXTI_Trigger_Rising);
}

static void tach_update(void) {
    // Atomically read + reset both counters
    uint32_t prev = irq_lock();
    uint16_t c1 = g_tach[0];
    uint16_t c2 = g_tach[1];
    g_tach[0] = 0;
    g_tach[1] = 0;
    irq_restore(prev);

    // RPM = pulses × 60 / (2 pulses/rev × window_seconds)
    // With 2 s window:  multiplier = 30 / 2 = 15
    uint16_t rpm1 = c1 * 15;
    uint16_t rpm2 = c2 * 15;

    g_rpm = (g_fanMode == 2) ? ((rpm1 + rpm2) / 2) : rpm1;
}

// Atomically reset both tach counters
static void tach_reset(void) {
    uint32_t prev = irq_lock();
    g_tach[0] = 0;
    g_tach[1] = 0;
    irq_restore(prev);
}

// Atomically snapshot a single counter
static uint16_t tach_snap(uint8_t ch) {
    uint32_t prev = irq_lock();
    uint16_t v = g_tach[ch];
    irq_restore(prev);
    return v;
}

/* ================================================================
 * SECTION 9 — Button gesture detection
 * ================================================================ */

static void btn_poll(uint32_t now) {
    static uint32_t tLast       = 0;
    static uint8_t  raw       = HIGH, lastRaw = HIGH;
    static uint8_t  deb       = HIGH, lastDeb = HIGH;
    static uint8_t  dbCnt     = 0;
    static uint32_t tPress    = 0;
    static uint32_t tRelease  = 0;
    static uint8_t  clicks    = 0;
    static bool     longDone  = false;

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
    if (deb == LOW  && lastDeb == HIGH) { tPress = now; longDone = false; }   // press
    if (deb == HIGH && lastDeb == LOW)  {                                      // release
        uint32_t held = now - tPress;
        if (held >= BTN_SHORT_MIN_MS && held <= BTN_SHORT_MAX_MS) clicks++;
        tRelease = now;
    }
    lastDeb = deb;

    // Long-press: fire ONCE while held
    if (deb == LOW && !longDone && (now - tPress) > BTN_LONG_MS) {
        longDone = true;
        clicks   = 0;
        g_dutyIdx = 0;
        pwm_set(0);
        g_dirty   = true;
    }

    // Resolve after double-click gap
    if (clicks && (now - tRelease) > BTN_DOUBLE_GAP_MS) {
        if (clicks == 1) {               // single: cycle
            g_dutyIdx = (g_dutyIdx + 1) % 4;
            pwm_set(dutyTable[g_dutyIdx]);
        } else {                          // double: 100 %
            g_dutyIdx = 4;
            pwm_set(100);
        }
        g_dirty = true;
        clicks  = 0;
    }
}

/* ================================================================
 * SECTION 10 — Display update (RUNNING state)
 * ================================================================ */

static void dsp_update(uint32_t now) {
    if (!g_dirty && (now - g_tDsp < DSP_INTERVAL_MS)) return;
    g_tDsp  = now;
    g_dirty = false;

    // Compute RPM colour once — applies to all lit LEDs
    uint8_t rpmColour[3];
    rpmToHeatColor(g_rpm, rpmColour);

    // Battery gauge → LED count only;  RPM → colour
    for (uint8_t i = 0; i < NUM_LEDS; i++) {
        if (i < g_gauge) {
            g_leds[i][0] = rpmColour[0];
            g_leds[i][1] = rpmColour[1];
            g_leds[i][2] = rpmColour[2];
        } else {
            g_leds[i][0] = 0;
            g_leds[i][1] = 0;
            g_leds[i][2] = 0;
        }
    }

    // Critical battery: blink topmost gauge LED at 2 Hz (still in RPM colour)
    if (g_socPct < 10 && g_gauge == 1) {
        if ((now / 250) & 1) {
            g_leds[0][0] = 0;
            g_leds[0][1] = 0;
            g_leds[0][2] = 0;
        }
    }

    ws2812_send();
}

/* ================================================================
 * SECTION 11 — State runners
 * ================================================================ */

static void run_startup(uint32_t now) {
    switch (g_phase) {

    // ——— Phase 0: rainbow animation (2 s) ———
    case 0: {
        uint8_t frame = (uint8_t)((now - g_tPhase) / RAINBOW_FRAME_MS);
        if (frame >= RAINBOW_FRAMES) {
            // Done. Run battery + fan checks.
            g_cells = batt_detect();
            pwm_set(100);                      // spin fans for detection
            g_tPhase = now;
            g_phase = 1;
            return;
        }
        // Render frame
        static uint8_t lastFrame = 0xFF;       // force first frame
        if (frame != lastFrame) {
            lastFrame = frame;
            for (uint8_t i = 0; i < NUM_LEDS; i++) {
                uint8_t pos = (frame * 8 + i * 64) & 0xFF;
                colorWheel(pos, g_leds[i]);
            }
            ws2812_send();
        }
        return;
    }

    // ——— Phase 1: spin-up delay (500 ms at 100 %) ———
    case 1:
        if (now - g_tPhase < 500) return;
        tach_reset();
        g_tPhase = now;
        g_phase = 2;
        return;

    // ——— Phase 2: count tach for 1 s ———
    case 2:
        if (now - g_tPhase < 1000) return;
        {
            uint16_t c1 = tach_snap(0);
            uint16_t c2 = tach_snap(1);
            pwm_set(0);                        // stop fans

            if      (c1 > 3 && c2 > 3) g_fanMode = 2;
            else if (c1 > 3)           g_fanMode = 1;
            else                       g_fanMode = 0;

            // if (g_cells == 0 || g_fanMode == 0) 
            // {
            //     g_state = S_ERROR;
            // } 
            // else 
            {
                g_state = S_RUNNING;
                g_dirty = true;
            }
            g_phase = 3;
        }
        return;
    }
}

static void run_running(uint32_t now) {
    btn_poll(now);

    if (now - g_tAdc >= ADC_INTERVAL_MS) {
        g_tAdc = now;
        batt_sample();
        g_dirty = true;
    }

    if (now - g_tTach >= TACH_WINDOW_MS) {
        g_tTach = now;
        tach_update();
        g_dirty = true;
    }

    dsp_update(now);
}

static void run_error(uint32_t now) {
    if (now - g_tBlink >= ERROR_BLINK_MS) {
        g_tBlink = now;
        g_blinkOn = !g_blinkOn;
        for (uint8_t i = 0; i < NUM_LEDS; i++) {
            g_leds[i][0] = 0;
            g_leds[i][1] = g_blinkOn ? 128 : 0;
            g_leds[i][2] = 0;
        }
        ws2812_send();
    }
}

/* ================================================================
 * SECTION 12 — Arduino entry points
 * ================================================================ */

void setup(void) {
    pinMode(BTN_PIN,   INPUT_PULLUP);
    pinMode(TACH1_PIN, INPUT_PULLUP);
    pinMode(TACH2_PIN, INPUT_PULLUP);
    pinMode(BATT_PIN,  INPUT);

    pwm_begin();
    ws2812_begin();
    tach_begin();

    g_state  = S_STARTUP;
    g_phase  = 0;
    g_tPhase = millis();
}

void loop(void) {
    uint32_t now = millis();

    switch (g_state) {
        case S_STARTUP: run_startup(now);  break;
        case S_RUNNING: run_running(now);  break;
        case S_ERROR:   run_error(now);    break;
    }
}
