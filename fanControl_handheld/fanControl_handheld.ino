/*
 * Chasis Fan Control — Handheld single-fan controller with WS2812 battery gauge.
 *
 * MCU:  CH32V003J4M6 @ 48 MHz (TSSOP-8)
 * Fan:  1× 4-pin PWM on PC4 (TIM1_CH4), No tachometer
 * LED:  1× WS2812C on PD4 — bitbang timed against RISC-V mcycle CSR
 * Batt: 1S 21700 LiPo on PA2 (10k / 10k divider, 2.8 V internal ADC ref)
 * Btn:  Single button on PD6, active low, with EXTI (wake from deep sleep)
 * Chg:  Charge detect on PC1, active low, EXTI for instant detection + STANDBY wake
 *
 * WS2812 technique: bitbang on PD4 using the RISC-V mcycle CSR as timing reference.
 * mcycle is a mandatory 64-bit hardware counter at CPU frequency (48 MHz),
 * immune to pipeline depth, compiler optimisation level, and cache.  No timer
 * interrupts, no DMA, no NOPs — a simple while-loop polling mcycle for deadlines.
 *
 * UX:
 *       - Power on(POR) or exit deep sleep: pure blue breath brightness 1 Hz
 *       - click       → ON (FAN_EN high, duty 0%), solid CYAN
 *       - double-click → ON at 100% (one-key-turbo)
 *       - ON state:  click cycles 0%→25%→50%→75%→0%
 *                    double-click → 100%
 *                    long-press → back to blue breath
 *       - 0%=CYAN, 25%=green, 50%=yellow, 75%=orange
 *       - 100% duty: breathing color red↔white at 2 Hz
 *       - charging: ignore button, force fan OFF, breath battery SoC colour at 1 Hz
 *         (red=low → green=full), solid green when fully charged
 */

/* ================================================================
 * SECTION 1 — Constants
 * ================================================================ */

// ---- Fan PWM (TIM1_CH4 on PC4, 25 kHz) ----
#define FAN_EN_PIN    PC2         // active high MOSFET gate
#define FAN_PWM_PIN   PC4         // TIM1_CH4

#define PWM_PERIOD    1919        // 48 MHz / 25000 − 1
#define DUTY_0        0
#define DUTY_25       480
#define DUTY_50       960
#define DUTY_75       1440
#define DUTY_100      1919

// ---- WS2812 via mcycle bitbang on PD4 ----
#define LED_PIN       PD4

// 48 MHz mcycle ticks → nanoseconds:  1 tick = 20.833 ns
// WS2812 bit period = 1.25 µs = 60 ticks
#define WS_BIT_TICKS   60

// High-pulse durations (ticks at 48 MHz):
// T0H nominal 0.35 µs → 17 ticks.  Spec range 0.20–0.50 µs (10–24 ticks).
// T1H nominal 0.75 µs → 36 ticks.  Spec range 0.55–0.85 µs (26–41 ticks).
#define WS_T0H_TICKS   17
#define WS_T1H_TICKS   36

// Reset: hold LOW >50 µs → >2400 ticks.  Use 3000 (62.5 µs) for margin.
#define WS_RESET_TICKS 3000

// ---- Display / colour ----
// Duty colours in GRB order: [G, R, B]
static const uint8_t dutyColors[4][3] = {
    {255,   0, 255},   // 0%   = CYAN
    {255,   0,   0},   // 25%  = GREEN
    {255, 255,   0},   // 50%  = YELLOW
    {165, 255,   0},   // 75%  = ORANGE
};
// 100% = red↔white breathing, computed live

// ---- Button ----
#define BTN_PIN            PD6   // active low
#define BTN_DEBOUNCE_MS    30
#define BTN_SHORT_MIN_MS   50
#define BTN_SHORT_MAX_MS   500
#define BTN_DOUBLE_GAP_MS  300
#define BTN_LONG_MS        1000

// ---- Battery ADC ----
// Divider: BAT+ → 10k → VDETECT(PA2) → 10k → GND.  Ratio = 1/2
// vBatt_mV = adc × ADC_REF × 2 / 1024
#define ADC_REF        2800
#define BATT_PIN       PA2
#define BATT_MAX       4300
#define BATT_MIN       3000
#define CHARGE_PIN      PC1    // active low

// ---- Intervals ----
#define ADC_INTERVAL_MS     200
#define DSP_INTERVAL_MS     50     // 50 ms for smooth breathing
#define OFF_TIMEOUT_MS      10000  // 10 s idle in S_WAIT(from S_ON) → deep sleep

// ---- Breath effect periods ----
#define WAKE_BREATH_PERIOD    1000  // 1 Hz
#define CHARGE_BREATH_PERIOD  1000  // 1 Hz
#define TURBO_BREATH_PERIOD   500   // 2 Hz

// ---- Duty table (single-click cycle) ----
static const uint8_t dutyTable[4] = { 0, 25, 50, 75 };

/* ================================================================
 * SECTION 2 — Global State
 * ================================================================ */

enum State : uint8_t {
    S_WAIT     = 0,
    S_ON       = 1,
    S_CHARGING = 2,
};
static State    g_state         = S_WAIT;
static State    g_prevState     = S_WAIT;   // in S_WAIT: if==S_ON → 10s auto-sleep timer active
static State    g_chgPrevState  = S_WAIT;   // state to restore after charger unplug

// Battery
static uint8_t  g_socPct     = 0;          // 0–100
static uint32_t g_tAdc       = 0;

// PWM
static uint8_t  g_dutyIdx    = 0;          // 0–3 = cycle (0/25/50/75), 4 = 100 %

// Display — single LED colour (GRB order)
static uint8_t  g_ledG       = 0;
static uint8_t  g_ledR       = 0;
static uint8_t  g_ledB       = 0;
static uint32_t g_tDsp       = 0;
static bool     g_dirty      = true;

// S_WAIT timer (10 s countdown when g_prevState == S_ON)
static uint32_t g_tWait      = 0;

// Charge detection
static volatile bool g_chargeFlag = false;

/* ================================================================
 * SECTION 3 — Interrupt lock helpers (RISC-V mstatus.MIE)
 * ================================================================ */

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
 * SECTION 4 — Fan PWM (TIM1_CH4 on PC4, 25 kHz)
 * ================================================================ */

static void pwm_begin(void) {
    // Enable GPIOC + TIM1 + AFIO clocks
    RCC->APB2PCENR |= (1 << 4)    // GPIOC
                   |  (1 << 11)   // TIM1
                   |  (1 << 0);   // AFIO

    // PC4 → alternate push-pull 50 MHz (nibble 4, bits 16–19)
    uint32_t cfglr = GPIOC->CFGLR;
    cfglr &= ~(0xFU << 16);
    cfglr |=  (0xBU << 16);       // CNF=10(AF-PP), MODE=11(50 MHz)
    GPIOC->CFGLR = cfglr;

    // Timer base: 48 MHz / 1 / 1920 = 25 kHz
    TIM1->PSC   = 0;
    TIM1->ATRLR = PWM_PERIOD;
    TIM1->CNT   = 0;

    // CH4: PWM mode 1, preload
    // CHCTLR2 bits 8–10: OC4M=110 (PWM1), bit 11: OC4PE=1
    TIM1->CHCTLR2 = (6 << 8) | (1 << 11);

    // Enable CH4 output: CC4E (bit 12)
    TIM1->CCER |= (1 << 12);

    // Start at 0 %
    TIM1->CH4CVR = 0;

    // MOE + ARPE + CEN
    TIM1->BDTR  |= (1 << 15);      // MOE  (main output enable)
    TIM1->CTLR1 |= (1 << 7)        // ARPE (auto-reload preload)
                |  (1 << 0);       // CEN  (counter enable)
}

static inline void fan_pwm_write(uint8_t pct) {
    uint16_t ccr;
    if (pct >= 100)     ccr = DUTY_100;
    else if (pct == 0)  ccr = DUTY_0;
    else                ccr = (uint16_t)((uint32_t)pct * PWM_PERIOD / 100);
    TIM1->CH4CVR = ccr;
}

/* ================================================================
 * SECTION 5 — WS2812 bitbang on PD4 (mcycle-timed, zero NOPs)
 * ================================================================ */

// Read RISC-V mcycle CSR (lower 32 bits of 64-bit hardware cycle counter)
static inline uint32_t mcycle_read(void) {
    uint32_t val;
    __asm__ volatile("csrr %0, mcycle" : "=r"(val));
    return val;
}

static void ws2812_begin(void) {
    // PD4 → push-pull output, 50 MHz
    uint32_t cfglr = GPIOD->CFGLR;
    cfglr &= ~(0xFU << 16);
    cfglr |=  (0x3U << 16);       // CNF=00(GP-PP), MODE=11(50 MHz)
    GPIOD->CFGLR = cfglr;
    GPIOD->BSHR = (1 << (16 + 4)); // start LOW
}

// Send 24-bit GRB value to single WS2812.  Disables interrupts for
// ~30 µs to avoid pulse corruption.  All timing is referenced to the
// mcycle hardware counter — zero NOPs, immune to pipeline/compiler.
static void ws2812_send_grb(uint8_t g, uint8_t r, uint8_t b) {
    // Pack GRB: G sent first (WS2812C), MSB-first per byte
    uint32_t packed = ((uint32_t)g << 16) | ((uint32_t)r << 8) | b;

    uint32_t prev = irq_lock();

    // --- 24 data bits ---
    for (uint8_t bit = 0; bit < 24; bit++) {
        uint32_t tick = mcycle_read();

        // Set HIGH
        GPIOD->BSHR = (1 << 4);

        // Hold HIGH for T0H or T1H duration
        uint32_t high = (packed & 0x800000) ? WS_T1H_TICKS : WS_T0H_TICKS;
        packed <<= 1;
        while ((int32_t)(mcycle_read() - (tick + high)) < 0) { /* spin */ }

        // Set LOW — rests for remainder of bit period
        GPIOD->BSHR = (1 << (16 + 4));

        // Wait until end of 1.25 µs bit period
        while ((int32_t)(mcycle_read() - (tick + WS_BIT_TICKS)) < 0) { /* spin */ }
    }

    // --- Reset: hold LOW >50 µs ---
    uint32_t rst_tick = mcycle_read();
    while ((int32_t)(mcycle_read() - (rst_tick + WS_RESET_TICKS)) < 0) { /* spin */ }

    irq_restore(prev);
}

/* ================================================================
 * SECTION 6 — Colour helpers
 * ================================================================ */

// Triangle wave: 0→255→0 over period_ms.  Returns current brightness 0–255.
static uint8_t breath_triangle(uint32_t period_ms, uint32_t now) {
    uint32_t t = now % period_ms;
    uint32_t half = period_ms / 2;
    if (t < half)
        return (uint8_t)(t * 255 / half);
    else
        return (uint8_t)((period_ms - t) * 255 / half);
}

// Battery SoC → GRB colour: red (0%) → green (100%)
static void soc_to_color(uint8_t socPct, uint8_t *g, uint8_t *r, uint8_t *b) {
    uint8_t pct = (socPct > 100) ? 100 : socPct;
    *r = (uint8_t)(255 - ((uint16_t)pct * 255 / 100));
    *g = (uint8_t)((uint16_t)pct * 255 / 100);
    *b = 0;
}

/* ================================================================
 * SECTION 7 — Battery ADC (1S only)
 * ================================================================ */

static void batt_sample(void) {
    uint16_t raw = analogRead(BATT_PIN);

    // 4-sample rolling average
    static uint16_t buf[4] = {0};
    static uint8_t  idx    = 0;
    buf[idx] = raw;
    idx = (idx + 1) & 3;
    uint16_t adcAvg = (uint16_t)(((uint32_t)buf[0] + buf[1] + buf[2] + buf[3]) >> 2);

    // vBatt_mV = adc * Vref * 2 / 1024  (Vref = 2800 mV internal, divider ratio = 0.5)
    uint32_t vBatt = ((uint32_t)adcAvg * (uint32_t)ADC_REF * 2UL) / 1024UL;

    // SoC: linear 0–100 % between BATT_MIN and BATT_MAX
    if (vBatt <= BATT_MIN)      g_socPct = 0;
    else if (vBatt >= BATT_MAX) g_socPct = 100;
    else g_socPct = (uint8_t)((100UL * (vBatt - BATT_MIN))
                               / (BATT_MAX - BATT_MIN));
}

/* ================================================================
 * SECTION 8 — Omitted (no tachometer)
 * ================================================================ */

/* ================================================================
 * SECTION 9 — Button gesture detection
 * ================================================================ */

// Forward declarations for state transitions
static void enter_on(uint8_t dutyIdx);
static void enter_wait(void);
static void deep_sleep_enter(void);

static void btn_poll(uint32_t now) {
    // Ignore button entirely during charging
    if (g_state == S_CHARGING) return;

    static uint32_t tLast       = 0;
    static uint8_t  raw         = HIGH, lastRaw = HIGH;
    static uint8_t  deb         = HIGH, lastDeb = HIGH;
    static uint8_t  dbCnt       = 0;
    static uint32_t tPress      = 0;
    static uint32_t tRelease    = 0;
    static uint8_t  clicks      = 0;
    static bool     longDone    = false;

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

        if (g_state == S_ON) {
            enter_wait();                    // S_ON → S_WAIT (10 s timer)
        } else if (g_state == S_WAIT) {
            deep_sleep_enter();              // S_WAIT → STANDBY immediately
        }
        return;  // state changed — skip click resolution
    }

    // Resolve after double-click gap
    if (clicks && (now - tRelease) > BTN_DOUBLE_GAP_MS) {
        g_dirty = true;  // state or duty changed — refresh display immediately
        if (g_state == S_ON) {
            if (clicks == 1) {               // single: cycle duty
                g_dutyIdx = (g_dutyIdx + 1) % 4;
                fan_pwm_write(dutyTable[g_dutyIdx]);
            } else {                          // double: 100 %
                g_dutyIdx = 4;
                fan_pwm_write(100);
            }
        } else if (g_state == S_WAIT) {
            // Any click cancels the 10 s auto-sleep timer (implicit by leaving S_WAIT)
            if (clicks == 1) {               // single: ON at 0 %
                enter_on(0);
            } else {                          // double: ON at 100 % (one-key-turbo)
                enter_on(4);
            }
        }
        clicks = 0;
    }
}

/* ================================================================
 * SECTION 10 — Display update
 * ================================================================ */

static void dsp_update(uint32_t now) {
    if (!g_dirty && (now - g_tDsp < DSP_INTERVAL_MS)) return;
    g_tDsp  = now;
    g_dirty = false;

    switch (g_state) {

    case S_WAIT: {
        // Blue breath 1 Hz (always, regardless of how we entered)
        uint8_t bright = breath_triangle(WAKE_BREATH_PERIOD, now);
        g_ledG = 0;
        g_ledR = 0;
        g_ledB = bright;
        break;
    }

    case S_ON: {
        if (g_dutyIdx == 4) {
            // 100 %: red↔white colour breath at 2 Hz (full brightness)
            uint32_t t = now % TURBO_BREATH_PERIOD;   // 500 ms period
            uint8_t mix;
            if (t < 250)
                mix = (uint8_t)(t * 255 / 250);        // 0→255: red → white
            else
                mix = (uint8_t)((500 - t) * 255 / 250); // 255→0: white → red
            g_ledR = 255;
            g_ledG = mix;
            g_ledB = mix;
        } else {
            // 0 %–75 %: solid duty colour
            g_ledG = dutyColors[g_dutyIdx][0];
            g_ledR = dutyColors[g_dutyIdx][1];
            g_ledB = dutyColors[g_dutyIdx][2];
        }
        break;
    }

    case S_CHARGING: {
        if (g_socPct >= 100) {
            // Fully charged: solid green
            g_ledR = 0;
            g_ledG = 255;
            g_ledB = 0;
        } else {
            // Breathing battery SoC colour at 1 Hz
            uint8_t bright = breath_triangle(CHARGE_BREATH_PERIOD, now);
            uint8_t r, g, b;
            soc_to_color(g_socPct, &g, &r, &b);
            g_ledR = (uint8_t)((uint16_t)r * bright / 255);
            g_ledG = (uint8_t)((uint16_t)g * bright / 255);
            g_ledB = (uint8_t)((uint16_t)b * bright / 255);
        }
        break;
    }
    }

    ws2812_send_grb(g_ledG, g_ledR, g_ledB);
}

/* ================================================================
 * SECTION 11 — State runners
 * ================================================================ */

static void enter_on(uint8_t dutyIdx) {
    digitalWrite(FAN_EN_PIN, HIGH);
    g_dutyIdx = dutyIdx;
    if (dutyIdx == 4)  fan_pwm_write(100);
    else               fan_pwm_write(dutyTable[dutyIdx]);
    g_state = S_ON;
    g_dirty = true;
    g_tAdc  = 0;  // sample immediately
}

static void enter_wait(void) {
    digitalWrite(FAN_EN_PIN, LOW);
    fan_pwm_write(0);
    g_prevState = S_ON;   // signals: 10 s auto-sleep timer is active
    g_tWait     = millis();
    g_state     = S_WAIT;
    g_dirty     = true;
}

static void run_wait(uint32_t now) {
    btn_poll(now);

    // 10 s auto-sleep timer (only when coming from S_ON)
    if (g_prevState == S_ON && (now - g_tWait >= OFF_TIMEOUT_MS)) {
        deep_sleep_enter();
        return;
    }

    dsp_update(now);
}

static void run_on(uint32_t now) {
    btn_poll(now);

    if (now - g_tAdc >= ADC_INTERVAL_MS) {
        g_tAdc = now;
        batt_sample();
    }

    dsp_update(now);
}

static void run_charging(uint32_t now) {
    // Fan forced off during charging
    // Button ignored (handled in btn_poll)

    if (now - g_tAdc >= ADC_INTERVAL_MS) {
        g_tAdc = now;
        batt_sample();
    }

    dsp_update(now);
}

/* ================================================================
 * SECTION 12 — Deep sleep entry (STANDBY mode)
 * ================================================================ */

static void deep_sleep_enter(void) {
    // Turn everything off
    digitalWrite(FAN_EN_PIN, LOW);
    fan_pwm_write(0);
    GPIOD->BSHR = (1 << (16 + 4));  // LED off

    // Configure EXTI wake sources: PD6 (EXTI6) + PC1 (EXTI1), falling edge
    RCC->APB2PCENR |= (1 << 0);     // AFIO

    // PD6 → EXTI6: falling edge trigger (button press)
    EXTI->FTENR  |= (1 << 6);
    EXTI->INTENR |= (1 << 6);
    EXTI->SWIEVR &= ~(1 << 6);      // clear pending

    // PC1 → EXTI1: falling edge trigger (charger plug-in)
    EXTI->FTENR  |= (1 << 1);
    EXTI->INTENR |= (1 << 1);
    EXTI->SWIEVR &= ~(1 << 1);      // clear pending

    // Enter STANDBY: SLEEPDEEP=1, PDDS=1
    // PWR->CTLR: bit 2 = SLEEPDEEP, bit 1 = PDDS
    RCC->APB1PCENR |= (1 << 28);    // PWR clock enable
    PWR->CTLR |= (1 << 2);          // SLEEPDEEP
    PWR->CTLR |= (1 << 1);          // PDDS = STANDBY (not STOP)

    // Disable all interrupts to prevent spurious wake
    __asm__ volatile("csrw mie, %0" : : "r"(0));

    // Execute WFI
    __asm__ volatile("wfi");

    // Should never reach here — STANDBY wake triggers POR reset
    while (1) { /* wait for reset */ }
}

/* ================================================================
 * SECTION 13 — Arduino entry points
 * ================================================================ */

// CHARGE_PIN EXTI ISR — sets flag for instant charge detection
static void charge_isr(void) {
    g_chargeFlag = true;
}

void setup(void) {
    // GPIO init (Arduino API)
    pinMode(FAN_EN_PIN, OUTPUT);
    digitalWrite(FAN_EN_PIN, LOW);     // fan off initially

    pinMode(BTN_PIN,   INPUT_PULLUP);  // button: active-low, internal pull-up
    pinMode(BATT_PIN,  INPUT);         // battery ADC
    pinMode(CHARGE_PIN, INPUT_PULLUP); // charge detect: active-low, pull-up

    // Register-level peripherals
    pwm_begin();
    ws2812_begin();

    // CHARGE_PIN EXTI for instant charge detection (falling edge = plug-in)
    attachInterrupt(CHARGE_PIN, GPIO_Mode_IPU, charge_isr,
                    EXTI_Mode_Interrupt, EXTI_Trigger_Falling);

    // Initial state: check if charger is plugged in at boot
    g_prevState = S_WAIT;   // signals: no auto-sleep timer
    g_tAdc = 0;

    if (digitalRead(CHARGE_PIN) == LOW) {
        // Woke from STANDBY by charger plug-in, or powered on while charging
        digitalWrite(FAN_EN_PIN, LOW);
        fan_pwm_write(0);
        g_state = S_CHARGING;
    } else {
        g_state = S_WAIT;
    }
}

void loop(void) {
    uint32_t now = millis();

    // CHARGE_PIN: instant transition via EXTI flag, or poll fallback
    if (g_chargeFlag || digitalRead(CHARGE_PIN) == LOW) {
        g_chargeFlag = false;
        if (digitalRead(CHARGE_PIN) == LOW && g_state != S_CHARGING) {
            g_chgPrevState = g_state;  // save state to restore on unplug
            digitalWrite(FAN_EN_PIN, LOW);
            fan_pwm_write(0);
            g_state = S_CHARGING;
            g_dirty = true;
        }
    }

    // Charger unplugged → restore previous state
    if (g_state == S_CHARGING && digitalRead(CHARGE_PIN) == HIGH) {
        g_state = g_chgPrevState;
        g_dirty = true;
        if (g_state == S_ON) {
            digitalWrite(FAN_EN_PIN, HIGH);
            if (g_dutyIdx == 4)  fan_pwm_write(100);
            else                 fan_pwm_write(dutyTable[g_dutyIdx]);
        }
        // If previous was S_WAIT, g_prevState is preserved (10s timer state intact)
    }

    switch (g_state) {
        case S_WAIT:     run_wait(now);     break;
        case S_ON:       run_on(now);       break;
        case S_CHARGING: run_charging(now); break;
    }
}
