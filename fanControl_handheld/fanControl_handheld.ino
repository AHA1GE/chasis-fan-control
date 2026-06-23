/*
 * Chasis Fan Control — Handheld single-fan controller with WS2812 battery gauge.
 *
 * MCU:  CH32V003J4M6 @ 48 MHz (TSSOP-8)
 * Fan:  1× 4-pin PWM on PC4 (TIM1_CH4), No tachometer
 * LED:  1× WS2812C on PD4 — timer-overflow ISR bitbang (TIM1 counter as timing reference)
 * Batt: 1S 21700 LiPo on PA2 (10k / 10k divider, 2.8 V internal ADC ref)
 * Btn:  Single button on PD6, active low, with EXTI (wake from deep sleep)
 * Chg:  Charge detect on PC1, active low, EXTI for instant detection + STANDBY wake
 *
 * WS2812 technique: TIM1 at 800 kHz overflow interrupt. ISR drives PD4 via BSHR,
 * polling TIM1->CNT for pulse-width timing. Zero NOPs — all timing referenced to
 * the hardware counter, immune to pipeline depth / compiler optimisation / cache.
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

// ---- Fan PWM (TIM1_CH4 on PC4, 800 kHz base, period = 59) ----
#define FAN_EN_PIN    PC2         // active high MOSFET gate
#define FAN_PWM_PIN   PC4         // TIM1_CH4;  0 %=0,  25 %=15,  50 %=30,  75 %=45,  100 %=59

// ---- WS2812 via TIM1 overflow ISR ----
#define LED_PIN       PD4         // bit-banged in ISR via GPIOD->BSHR
#define WS2812_BITS   24
#define WS2812_RESET  50          // 50 × 1.25 µs = 62.5 µs reset
#define WS2812_SLOTS  (WS2812_BITS + WS2812_RESET)  // 74 slots total

// TIM1 counter targets for WS2812 pulse widths (48 MHz ticks = 20.833 ns each)
// Account for ~6 cycle ISR overhead from overflow to first GPIO write.
#define WS_T0H_CNT    15          // target CNT for end of 0-bit high (~0.31 µs)
#define WS_T1H_CNT    33          // target CNT for end of 1-bit high (~0.69 µs)

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
static uint8_t  g_socPct     = 0;        // 0–100
static uint32_t g_tAdc       = 0;

// PWM
static uint8_t  g_dutyIdx    = 0;        // 0–3 = cycle (0/25/50/75), 4 = 100 %

// Display — single LED colour (GRB order)
static uint8_t  g_ledG       = 0;
static uint8_t  g_ledR       = 0;
static uint8_t  g_ledB       = 0;
static uint32_t g_tDsp       = 0;

// S_WAIT timer (10 s countdown when g_prevState == S_ON)
static uint32_t g_tWait      = 0;

// Charge detection
static volatile bool g_chargeFlag = false;

// ---- WS2812 ISR state (driven by TIM1 overflow) ----
static volatile uint8_t  g_wsSlot;        // 0 .. WS2812_SLOTS-1
static volatile uint32_t g_wsPacked;      // current 24-bit GRB value (MSB = next bit)
static volatile bool     g_wsBusy;        // true while transmission in progress

/* ================================================================
 * SECTION 3 — TIM1 init (shared: ISR-WS2812 on PD4 + fan PWM on PC4)
 * ================================================================ */

static void tim1_begin(void) {
    // Clock enables
    RCC->APB2PCENR |= (1 << 4)    // GPIOC
                   |  (1 << 5)    // GPIOD
                   |  (1 << 11)   // TIM1
                   |  (1 << 0);   // AFIO

    // PD4 → push-pull output, 50 MHz (manually toggled in ISR)
    uint32_t cfglr_d = GPIOD->CFGLR;
    cfglr_d &= ~(0xFU << 16);
    cfglr_d |=  (0x3U << 16);       // CNF=00(GP-PP), MODE=11(50 MHz)
    GPIOD->CFGLR = cfglr_d;
    GPIOD->BSHR = (1 << (16 + 4));  // start LOW

    // PC4 → alternate push-pull 50 MHz (TIM1_CH4 for fan PWM)
    uint32_t cfglr_c = GPIOC->CFGLR;
    cfglr_c &= ~(0xFU << 16);
    cfglr_c |=  (0xBU << 16);       // CNF=10(AF-PP), MODE=11(50 MHz)
    GPIOC->CFGLR = cfglr_c;

    // TIM1 timebase: 48 MHz / 1 / 60 = 800 kHz, 1.25 µs per slot
    TIM1->PSC   = 0;
    TIM1->ATRLR = 59;               // period = 60 ticks
    TIM1->CNT   = 0;

    // TIM1_CH4 (PC4): PWM mode 1, preload enabled, for fan
    TIM1->CHCTLR2 = (6 << 8)        // OC4M=110 (PWM mode 1)
                  | (1 << 11);      // OC4PE (preload enable)
    TIM1->CCER |= (1 << 12);        // CC4E
    TIM1->CH4CVR = 0;

    // MOE + ARPE
    TIM1->BDTR  |= (1 << 15);       // MOE
    TIM1->CTLR1 |= (1 << 7);        // ARPE

    // Enable TIM1 overflow interrupt
    TIM1->DMAINTENR |= (1 << 0);    // UIE (Update interrupt enable)
    NVIC_EnableIRQ(TIM1_UP_IRQn);   // enable in NVIC
}

/* ================================================================
 * SECTION 4 — Fan PWM (TIM1_CH4, shared 800 kHz timer)
 * ================================================================ */

// TIM1 period = 59 → duty range 0–59 (60 steps).
static inline void fan_pwm_write(uint8_t pct) {
    uint8_t ccr;
    if (pct >= 100)     ccr = 59;
    else if (pct == 0)  ccr = 0;
    else                ccr = (uint8_t)(((uint16_t)pct * 59) / 100);
    TIM1->CH4CVR = ccr;

    // Ensure timer is running
    if (!(TIM1->CTLR1 & 1)) {
        TIM1->CTLR1 |= (1 << 0);          // CEN = 1
    }
}

/* ================================================================
 * SECTION 5 — WS2812 send (kick off ISR-driven transmission)
 * ================================================================ */

static void ws2812_send_grb(uint8_t g, uint8_t r, uint8_t b) {
    // Pack GRB: G sent first (WS2812C order), MSB-first per byte
    uint32_t packed = ((uint32_t)g << 16) | ((uint32_t)r << 8) | b;

    // Load ISR state
    g_wsPacked = packed;
    g_wsSlot   = 0;
    g_wsBusy   = true;

    // Start timer (ISR fires on first overflow → ~1.25 µs)
    TIM1->CNT = 0;
    TIM1->CTLR1 |= (1 << 0);              // CEN = 1

    // Wait for transmission to complete
    while (g_wsBusy) { /* spin */ }
}

/* ================================================================
 * SECTION 6 — TIM1 overflow ISR (WS2812 bitbang on PD4)
 * ================================================================ */

extern "C" void TIM1_UP_IRQHandler(void) {
    // Clear update interrupt flag
    TIM1->INTFR &= ~(1 << 0);            // UIF = 0

    uint8_t slot = g_wsSlot;

    if (slot < WS2812_BITS) {
        // --- Data bit ---
        bool bitVal = (g_wsPacked & 0x800000) != 0;
        g_wsPacked <<= 1;

        // Set PD4 HIGH
        GPIOD->BSHR = (1 << 4);

        // Wait for target CNT (hardware counter, immune to pipeline/compiler/cache)
        uint16_t target = bitVal ? WS_T1H_CNT : WS_T0H_CNT;
        while (TIM1->CNT < target) { /* spin on hardware counter */ }

        // Set PD4 LOW — rests for remainder of bit period
        GPIOD->BSHR = (1 << (16 + 4));

        // Wait for next overflow (CNT wraps to 0)
        while (TIM1->CNT >= target) { /* spin — CNT still in this period */ }

    } else if (slot < WS2812_SLOTS) {
        // --- Reset slot — PD4 stays LOW ---
        // Just wait for this overflow to complete
        while (TIM1->CNT > 0)   { /* spin */ }
        while (TIM1->CNT == 0)  { /* ensure we see the wrap */ }

    } else {
        // --- Transmission complete ---
        TIM1->CTLR1 &= ~(1 << 0);         // CEN = 0 (stop timer)
        GPIOD->BSHR = (1 << (16 + 4));    // PD4 LOW
        g_wsBusy = false;
        return;
    }

    g_wsSlot = slot + 1;
}

/* ================================================================
 * SECTION 7 — Colour helpers
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
 * SECTION 8 — Battery ADC (1S only)
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
 * SECTION 9 — Omitted (no tachometer)
 * ================================================================ */

/* ================================================================
 * SECTION 10 — Button gesture detection
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
 * SECTION 11 — Display update
 * ================================================================ */

static void dsp_update(uint32_t now) {
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
 * SECTION 12 — State runners
 * ================================================================ */

static void enter_on(uint8_t dutyIdx) {
    digitalWrite(FAN_EN_PIN, HIGH);
    g_dutyIdx = dutyIdx;
    if (dutyIdx == 4)  fan_pwm_write(100);
    else               fan_pwm_write(dutyTable[dutyIdx]);
    g_state = S_ON;
    g_tAdc  = 0;  // sample immediately
}

static void enter_wait(void) {
    digitalWrite(FAN_EN_PIN, LOW);
    fan_pwm_write(0);
    g_prevState = S_ON;   // signals: 10 s auto-sleep timer is active
    g_tWait     = millis();
    g_state     = S_WAIT;
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
 * SECTION 13 — Deep sleep entry (STANDBY mode)
 * ================================================================ */

static void deep_sleep_enter(void) {
    // Turn everything off
    digitalWrite(FAN_EN_PIN, LOW);
    fan_pwm_write(0);
    TIM1->CTLR1 &= ~(1 << 0);               // stop timer
    NVIC_DisableIRQ(TIM1_UP_IRQn);          // disable timer interrupt
    GPIOD->BSHR = (1 << (16 + 4));          // LED off

    // Configure EXTI wake sources: PD6 (EXTI6) + PC1 (EXTI1), falling edge
    RCC->APB2PCENR |= (1 << 0);             // AFIO

    // PD6 → EXTI6: falling edge trigger (button press)
    EXTI->FTENR  |= (1 << 6);
    EXTI->INTENR |= (1 << 6);
    EXTI->SWIEVR &= ~(1 << 6);              // clear pending

    // PC1 → EXTI1: falling edge trigger (charger plug-in)
    EXTI->FTENR  |= (1 << 1);
    EXTI->INTENR |= (1 << 1);
    EXTI->SWIEVR &= ~(1 << 1);              // clear pending

    // Enter STANDBY: SLEEPDEEP=1, PDDS=1
    // PWR->CTLR: bit 2 = SLEEPDEEP, bit 1 = PDDS
    RCC->APB1PCENR |= (1 << 28);            // PWR clock enable
    PWR->CTLR |= (1 << 2);                  // SLEEPDEEP
    PWR->CTLR |= (1 << 1);                  // PDDS = STANDBY (not STOP)

    // Disable all interrupts to prevent spurious wake
    __asm__ volatile("csrw mie, %0" : : "r"(0));

    // Execute WFI
    __asm__ volatile("wfi");

    // Should never reach here — STANDBY wake triggers POR reset
    while (1) { /* wait for reset */ }
}

/* ================================================================
 * SECTION 14 — Arduino entry points
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

    // TIM1: overflow ISR for WS2812 on PD4 + fan PWM on PC4
    tim1_begin();

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
        }
    }

    // Charger unplugged → restore previous state
    if (g_state == S_CHARGING && digitalRead(CHARGE_PIN) == HIGH) {
        g_state = g_chgPrevState;
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
