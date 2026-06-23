/*
 * Chasis Fan Control — Handheld single-fan controller with WS2812 battery gauge.
 *
 * MCU:  CH32V003J4M6 @ 48 MHz (TSSOP-8)
 * Fan:  1× 4-pin PWM on PC4 (T1_CH4/T1CH1_3), No tachometer
 * LED:  1× WS2812C on PD4(T1CH4_3)/PD5(T2CH4_3)/PD1, all same physical pin
 * Batt: 1S 21700 LiPo on PA2(ADC0) (10k / 10k divider, 2.8 V internal ADC ref)
 * Btn:  Single button on PD6, active low, with EXTI for STANDBY wake
 * Chg:  Charge detect on PC1, active low, EXTI for instant detection + STANDBY wake
 *
 * UX:
 *       - Power on(POR) or exit deep sleep: pure blue, breath brightness, 1 Hz
 *       - click       → ON (FAN_EN high, duty 25%), solid CYAN
 *       - double-click → ON at 100% (quick shortcut)
 *       - ON state:  click cycles 25%→50%→75%→100%→25%
 *                    double-click → 100%
 *                    long-press → back to blue breath(fan OFF)
 *       - 25%=CYAN, 50%=GREEN, 75%=YELLOW, 100%=ORANGE
 *       - charging: ignore button, force fan OFF, breath battery SoC colour at 1 Hz
 *         (red=low → green=full), solid green when fully charged
 */

/* ================================================================
 * Constants
 * ================================================================ */

// ---- Fan PWM (TIM1_CH4 on PC4) ----
#define FAN_EN_PIN PC2  // active high MOSFET gate
#define FAN_PWM_PIN PC4 // TIM1_CH4
// Duty table (single-click cycle)
static const int dutyTable[4] = {25, 50, 75, 100};

// ---- WS2812 via TIM2_CH4 PWM+DMA on PD5 (T2CH4_3) ----
#define NUM_LEDS          1
#define WS2812_T0H        19     // ~0.4 µs at 800 kHz
#define WS2812_T1H        38     // ~0.8 µs at 800 kHz
#define WS2812_RESET_LEN  50     // >50 µs reset = 62.5 µs
#define WS2812_BUF_SIZE   (NUM_LEDS * 24 + WS2812_RESET_LEN)  // 74

// Duty colours in GRB order: [G, R, B]
static const int dutyColors[4][3] = {
    {128, 0, 128}, // 25%  = CYAN
    {255, 0, 0},   // 50%  = GREEN
    {128, 128, 0}, // 75%  = YELLOW
    {100, 154, 0}, // 100% = ORANGE
};

// ---- Button ----
#define BTN_PIN PD6 // active low
#define BTN_DEBOUNCE_MS 30
#define BTN_SHORT_MIN_MS 50
#define BTN_SHORT_MAX_MS 500
#define BTN_DOUBLE_GAP_MS 300
#define BTN_LONG_MS 1000

// ---- Battery ADC ----
// Divider: BAT+ → 10k → VDETECT(PA2) → 10k → GND.  Ratio = 1/2
// vBatt_mV = adc × ADC_REF × 2 / 1024
#define ADC_REF 2800
#define BATT_PIN PA2
#define BATT_MAX 4300
#define BATT_MIN 3000
#define CHARGE_PIN PC1 // active low
#define CHG_DEBOUNCE_MS 50

// ---- Intervals ----
#define ADC_INTERVAL_MS 100
#define LED_INTERVAL_MS 10
#define OFF_TIMEOUT_MS 10000 // 10 s idle in S_WAIT(from S_ON) → deep sleep

// ---- Breath effect periods ----
#define WAKE_BREATH_PERIOD 1000U   // 1 Hz
#define CHARGE_BREATH_PERIOD 1000U // 1 Hz

/* ================================================================
 * Global State
 * ================================================================ */

enum State : int
{
    S_WAIT = 0,
    S_ON = 1,
    S_CHARGING = 2,
};
static State g_state = S_WAIT;
static State g_prevState = S_WAIT;    // in S_WAIT: if==S_ON → 10s auto-sleep timer active
static State g_chgPrevState = S_WAIT; // state to restore after charger unplug

// Battery
static int g_batt_socPct = 0; // 0–100
static bool g_batt_valid = false;
static unsigned long g_batt_tAdc = 0;

// PWM
static int g_dutyIdx = 0; // 0–3 = cycle (25/50/75/100)

// WS2812 LED colour (GRB order)
static int g_led_G = 0;
static int g_led_R = 0;
static int g_led_B = 0;
static bool g_led_dirty = false;
static long g_led_t_last_millis = 0;

// DMA buffer for WS2812 PWM+DMA (one half-word per bit + reset)
static uint16_t g_ws2812_dma_buf[WS2812_BUF_SIZE];

// S_WAIT timer (10 s countdown when g_prevState == S_ON)
static unsigned long g_tWait = 0;

// Charge detection
static volatile bool g_chargeFlag = false;
static int g_chg_last_pin = 1;  // HIGH (pull-up, charger unplugged)
static unsigned long g_chg_tStable = 0;

/* ================================================================
 * Helpers
 * ================================================================ */

static inline long irq_lock(void)
{
    long mstatus;
    __asm__ volatile("csrrc %0, mstatus, %1"
                     : "=r"(mstatus)
                     : "r"(0x8)); // atomically read mstatus, clear MIE (bit 3)
    return mstatus;
}
static inline void irq_restore(long prev)
{
    __asm__ volatile("csrw mstatus, %0"
                     :
                     : "r"(prev));
}

static void deep_sleep_enter(void)
{
    // Turn everything off
    fan_off();

    // Configure EXTI wake sources: PD6 (EXTI6) + PC1 (EXTI1), falling edge
    RCC->APB2PCENR |= (1 << 0); // AFIO

    // PD6 → EXTI6: falling edge trigger (button press)
    // EXTI lines 4-7 on CH32V003 are fixed-port — EXTI6 fires on
    // PD6 (pin 6 of GPIOD).  No AFIO->EXTICR configuration needed.
    EXTI->FTENR |= (1 << 6);
    EXTI->INTENR |= (1 << 6);
    EXTI->INTFR = (1 << 6); // write 1 to clear any pending EXTI6

    // PC1 → EXTI1: falling edge trigger (charger plug-in)
    // EXTI1 routing to PC1 was already set up by attachInterrupt() in setup(),
    // but re-apply here in case standby-gate reset clears it.
    EXTI->FTENR |= (1 << 1);
    EXTI->INTENR |= (1 << 1);
    EXTI->INTFR = (1 << 1); // write 1 to clear any pending EXTI1

    // Enter STANDBY: SLEEPDEEP=1 (in PFIC SCTLR), PDDS=1 (in PWR->CTLR)
    RCC->APB1PCENR |= (1 << 28); // PWR clock enable

    PWR->CTLR &= ~(1UL << 0);    // Clear LPDS  — ensure correct standby mode
    PWR->CTLR |= (1UL << 1);     // Set   PDDS  — Power-Down Deepsleep = Standby

    // CRITICAL: SLEEPDEEP is in the PFIC System Control Register (NVIC->SCTLR
    // bit 2), NOT in PWR->CTLR.  The original code wrote to PWR->CTLR bit 2
    // which is CWUF (Clear Wakeup Flag) — a no-op for standby entry.
    NVIC->SCTLR |= (1 << 2);     // SLEEPDEEP = 1

    // Execute WFI to enter standby.
    // Do NOT disable interrupts (csrw mie,0) — in Standby mode the EXTI
    // line hardware directly wakes the voltage regulator; CPU interrupt
    // delivery is irrelevant.  Disabling interrupts before WFI in Sleep
    // mode makes WFI a NOP (returns immediately).
    __asm__ volatile("wfi");

    // Should never reach here — STANDBY wake triggers a full POR-like reset.
    // If we do reach here (standby entry failed), force a system reset so
    // the device doesn't appear dead.
    NVIC_SystemReset();
}

// Triangle wave: 0→255→0 over period_ms.  Returns current brightness 0–255.
static int get_breath_brightness(unsigned long period_ms, unsigned long nowMillis)
{
    long t = nowMillis % period_ms;
    long half = period_ms / 2;
    if (t < half)
        return (int)(t * 255 / half);
    else
        return (int)((period_ms - t) * 255 / half);
}

// Battery SoC → GRB colour: red (0%) → green (100%)
static void soc_to_color(int socPct, int *g, int *r, int *b)
{
    int pct = (socPct > 100) ? 100 : socPct;
    *r = (int)(255 - ((int)pct * 255 / 100));
    *g = (int)((int)pct * 255 / 100);
    *b = 0;
}

static void fan_pwm_write(int pct)
{
    // default is 12-bit resolution
    analogWrite(FAN_PWM_PIN, (long)pct * 4095 / 100);
}

static void fan_off(void)
{
    digitalWrite(FAN_PWM_PIN, LOW);
    digitalWrite(FAN_EN_PIN, LOW);
}

/* ================================================================
 * WS2812 PWM+DMA driver (TIM2_CH4 on PD5, 800 kHz carrier)
 * ================================================================ */

static void ws2812_begin(void)
{
    // ---- Clocks ----
    RCC->APB2PCENR |= (1 << 5)   // GPIOD
                   |  (1 << 0);  // AFIO
    RCC->APB1PCENR |= (1 << 0);  // TIM2
    RCC->AHBPCENR  |= (1 << 0);  // DMA1

    // ---- AFIO: TIM2 Full Remap (TIM2_RM[1:0] = 11 → CH4 on PD5) ----
    AFIO->PCFR1 = (AFIO->PCFR1 & ~(3UL << 8)) | (3UL << 8);

    // ---- PD5 → alternate push-pull, 50 MHz (nibble 5, bits 23:20) ----
    GPIOD->CFGLR = (GPIOD->CFGLR & ~(0xFUL << 20)) | (0xBUL << 20);

    // ---- TIM2: PSC=0, ARR=59 → 48 MHz / 60 = 800 kHz ----
    TIM2->PSC   = 0;
    TIM2->ATRLR = 59;
    TIM2->CNT   = 0;

    // ---- CH4: PWM mode 1, no preload (OC4PE=0) for direct DMA updates ----
    // Preload off avoids the "first DMA value used for 3 periods" issue.
    // CHCTLR2 bits 15:12 control CH4: OC4M[2:0]=110(PWM1), OC4PE=0
    TIM2->CHCTLR2 = (6UL << 12);

    // ---- CH4 output enable (CC4E) ----
    TIM2->CCER |= (1UL << 12);

    // ---- DMA request on update (UDE) ----
    TIM2->DMAINTENR |= TIM_UDE;

    // ---- DMA1 Channel 2: TIM2_UP → Memory→TIM2_CH4CVR, 16-bit, normal mode ----
    // CH32V003 RM Table 8-2: TIM2_UP request is hardwired to DMA1 Channel 2.
    // (Channel 5, which was used previously, only serves TIM1_CH3/CH4/USART1_TX.)
    DMA1_Channel2->PADDR = (uint32_t)&TIM2->CH4CVR;
    DMA1_Channel2->MADDR = (uint32_t)g_ws2812_dma_buf;
    // CFGR: DIR=M2P(bit4), MINC(bit7), PSIZE=16b(bit8), MSIZE=16b(bit10), PL=High(bits13:12=10)
    DMA1_Channel2->CFGR  = (1UL << 4) | (1UL << 7) | (1UL << 8) | (1UL << 10) | (2UL << 12);
}

static void ws2812_send(uint8_t g, uint8_t r, uint8_t b)
{
    // Fill DMA buffer: 24 bits GRB MSB-first + 50 reset slots
    uint32_t packed = ((uint32_t)g << 16) | ((uint32_t)r << 8) | b;
    uint16_t pos = 0;
    for (uint8_t i = 0; i < 24; i++) {
        g_ws2812_dma_buf[pos++] = (packed & 0x800000UL) ? WS2812_T1H : WS2812_T0H;
        packed <<= 1;
    }
    for (uint8_t i = 0; i < WS2812_RESET_LEN; i++) {
        g_ws2812_dma_buf[pos++] = 0;
    }

    // Preload the first buffer value into CH4CVR so the first PWM period
    // uses the correct duty (with OC4PE=0 this writes directly to the active
    // compare register — no preload shadow to worry about).
    TIM2->CH4CVR = g_ws2812_dma_buf[0];

    // Re-arm DMA (Channel 2 — TIM2_UP request destination)
    DMA1_Channel2->CFGR &= ~(1UL << 0);          // CHEN = 0
    DMA1_Channel2->CNTR  = WS2812_BUF_SIZE;      // transfer count
    DMA1_Channel2->MADDR = (uint32_t)g_ws2812_dma_buf;  // reset memory ptr
    DMA1_Channel2->CFGR |= (1UL << 0);           // CHEN = 1

    // Start timer (was stopped after previous transmission)
    TIM2->CTLR1 |= (1UL << 0);                   // CEN

    // Poll transfer-complete (TC2 = INTFR bit 5 for Channel 2)
    while (!(DMA1->INTFR & (1UL << 5)));
    DMA1->INTFCR = (1UL << 5);                   // clear TC2

    // Stop timer
    TIM2->CTLR1 &= ~(1UL << 0);                  // CEN = 0
}

/* ================================================================
 * Battery ADC (1S only)
 * ================================================================ */
static void batt_task(void)
{
    int raw = analogRead(BATT_PIN);

    g_batt_valid = true; // first ADC sample acquired — SoC data is now valid

    // 4-sample rolling average
    static int buf[4] = {0};
    static int idx = 0;
    buf[idx] = raw;
    idx = (idx + 1) & 3;
    int adcAvg = (int)(((long)buf[0] + buf[1] + buf[2] + buf[3]) >> 2);

    // vBatt_mV = adc * Vref * 2 / 1024  (Vref = 2800 mV internal, divider ratio = 0.5)
    long vBatt = ((long)adcAvg * (long)ADC_REF * 2UL) / 1024UL;

    // SoC: linear 0–100 % between BATT_MIN and BATT_MAX
    if (vBatt <= BATT_MIN)
    {
        g_batt_socPct = 0;
    }
    else if (vBatt >= BATT_MAX)
    {
        g_batt_socPct = 100;
    }
    else
    {
        g_batt_socPct = (int)((100UL * (vBatt - BATT_MIN)) / (BATT_MAX - BATT_MIN));
    }
}

/* ================================================================
 * Button gesture detection
 * ================================================================ */
static void btn_task(unsigned long nowMillis)
{
    // Ignore button entirely during charging
    if (g_state == S_CHARGING)
        return;

    static long tLast = 0;
    static int raw = HIGH, lastRaw = HIGH;
    static int deb = HIGH, lastDeb = HIGH;
    static unsigned long tStable = 0;
    static long tPress = 0;
    static long tRelease = nowMillis;
    static int clicks = 0;
    static bool longDone = false;

    // Limit to ~1 ms sampling
    if (nowMillis - tLast < 1)
        return;
    tLast = nowMillis;

    raw = digitalRead(BTN_PIN); // LOW = pressed

    // Debounce: time-based, independent of loop rate
    if (raw != lastRaw)
    {
        tStable = nowMillis;
    }
    lastRaw = raw;
    if (nowMillis - tStable >= BTN_DEBOUNCE_MS)
        deb = raw;

    // Edge detection
    if (deb == LOW && lastDeb == HIGH)
    {
        tPress = nowMillis;
        longDone = false;
    } // press
    if (deb == HIGH && lastDeb == LOW)
    { // release
        long held = nowMillis - tPress;
        if (held >= BTN_SHORT_MIN_MS && held <= BTN_SHORT_MAX_MS)
            clicks++;
        tRelease = nowMillis;
    }
    lastDeb = deb;

    // Long-press: fire ONCE while held
    if (deb == LOW && !longDone && (nowMillis - tPress) > BTN_LONG_MS)
    {
        longDone = true;
        clicks = 0;

        if (g_state == S_ON)
        {
            fan_off();
            g_prevState = S_ON; // signals: 10 s auto-sleep timer is active
            g_tWait = nowMillis;
            g_state = S_WAIT;
        }
        else if (g_state == S_WAIT)
        {
            deep_sleep_enter(); // S_WAIT → STANDBY immediately
        }
        return; // state changed — skip click resolution
    }

    // Resolve after double-click gap
    if (clicks && (nowMillis - tRelease) > BTN_DOUBLE_GAP_MS)
    {
        g_led_dirty = true; // state or duty changed — refresh display immediately
        if (g_state == S_ON)
        {
            if (clicks == 1)
            { // single: cycle duty
                g_dutyIdx = (g_dutyIdx + 1) % 4;
                fan_pwm_write(dutyTable[g_dutyIdx]);
            }
            else
            { // double: 100 %
                g_dutyIdx = 3;
                fan_pwm_write(dutyTable[3]);
            }
        }
        else if (g_state == S_WAIT)
        {
            // Any click cancels the 10 s auto-sleep timer (implicit by leaving S_WAIT)
            if (clicks == 1)
            { // single: ON at 25 %
                g_dutyIdx = 0;
                fan_pwm_write(dutyTable[0]);
            }
            else
            { // double: ON at 100 % (quick shortcut)
                g_dutyIdx = 3;
                fan_pwm_write(dutyTable[3]);
            }
            digitalWrite(FAN_EN_PIN, HIGH);
            g_state = S_ON;
        }
        clicks = 0;
    }
}

/* ================================================================
 * Display update
 * ================================================================ */

static void led_task(unsigned long nowMillis)
{
    if (!g_led_dirty && (nowMillis - g_led_t_last_millis < LED_INTERVAL_MS))
        return;
    g_led_t_last_millis = nowMillis;
    g_led_dirty = false;

    // Low-battery warning: 2 Hz red blink overrides normal display
    if (g_batt_valid && g_batt_socPct < 20 && (g_state == S_WAIT || g_state == S_ON))
    {
        if ((nowMillis % 500) < 250)
        {
            // Red on (GRB order: G=0, R=255, B=0)
            g_led_G = 0;
            g_led_R = 255;
            g_led_B = 0;
        }
        else
        {
            // Off
            g_led_G = 0;
            g_led_R = 0;
            g_led_B = 0;
        }
    }
    else switch (g_state)
    {

    case S_WAIT:
    {
        // Blue breath 1 Hz (always, regardless of how we entered)
        int bright = get_breath_brightness(WAKE_BREATH_PERIOD, nowMillis);
        g_led_G = 0;
        g_led_R = 0;
        g_led_B = bright;
        break;
    }

    case S_ON:
    {
        // Solid duty colour
        g_led_G = dutyColors[g_dutyIdx][0];
        g_led_R = dutyColors[g_dutyIdx][1];
        g_led_B = dutyColors[g_dutyIdx][2];
        break;
    }

    case S_CHARGING:
    {
        if (g_batt_socPct >= 100)
        {
            // Fully charged: solid green
            g_led_R = 0;
            g_led_G = 255;
            g_led_B = 0;
        }
        else
        {
            // Breathing battery SoC colour at 1 Hz
            int bright = get_breath_brightness(CHARGE_BREATH_PERIOD, nowMillis);
            int r, g, b;
            soc_to_color(g_batt_socPct, &g, &r, &b);
            g_led_R = (r * bright / 255);
            g_led_G = (g * bright / 255);
            g_led_B = (b * bright / 255);
        }
        break;
    }
    }

    // Push to WS2812 via PWM+DMA
    ws2812_send((uint8_t)g_led_G, (uint8_t)g_led_R, (uint8_t)g_led_B);
}

/* ================================================================
 * State runners
 * ================================================================ */
static void state_run_wait(unsigned long nowMillis)
{
    btn_task(nowMillis);

    // 10 s auto-sleep timer (only when coming from S_ON)
    if (g_prevState == S_ON && (nowMillis - g_tWait >= OFF_TIMEOUT_MS))
    {
        deep_sleep_enter();
        return;
    }

    led_task(nowMillis);
}

static void state_run_on(unsigned long nowMillis)
{
    btn_task(nowMillis);

    if (nowMillis - g_batt_tAdc >= ADC_INTERVAL_MS)
    {
        g_batt_tAdc = nowMillis;
        batt_task();
    }

    led_task(nowMillis);
}

static void state_run_charging(unsigned long nowMillis)
{
    // Fan forced off during charging
    // Button ignored (handled in btn_task)

    if (nowMillis - g_batt_tAdc >= ADC_INTERVAL_MS)
    {
        g_batt_tAdc = nowMillis;
        batt_task();
    }

    led_task(nowMillis);
}

/* ================================================================
 * Arduino entry points
 * ================================================================ */

// CHARGE_PIN EXTI ISR — sets flag for instant charge detection
static void charge_isr(void)
{
    g_chargeFlag = true;
    EXTI->INTFR = (1 << 1); // write 1 to clear any pending EXTI1
}

void setup(void)
{
    // ---- STANDBY-wake detection ----
    // After waking from Standby the MCU executes a full POR-like reset.
    // RCC->RSTSCKR flags tell us WHY we reset.
    //   Bit 31 (LPWRRSTF): low-power / standby reset
    //   Bit 27 (PORRSTF):  cold power-on reset
    //   Bit 26 (PINRSTF):  NRST pin reset
    // Clear all reset flags so the next reset cause can be read cleanly
    RCC->RSTSCKR |= (1UL << 24); // RMVF bit — clears all reset flags

    // GPIO init (Arduino API)
    pinMode(FAN_EN_PIN, OUTPUT);
    pinMode(FAN_PWM_PIN, OUTPUT);
    fan_off(); // fan off initially

    pinMode(BATT_PIN, INPUT);          // battery ADC
    pinMode(BTN_PIN, INPUT_PULLUP);    // button: active-low, internal pull-up
    pinMode(CHARGE_PIN, INPUT_PULLUP); // charge detect: active-low, pull-up

    // CHARGE_PIN EXTI for instant charge detection (falling edge = plug-in)
    // This also routes PC1 → EXTI1 via AFIO->EXTICR, which is required for
    // Standby wake via charger plug-in.
    attachInterrupt(CHARGE_PIN, GPIO_Mode_IPU, charge_isr,
                    EXTI_Mode_Interrupt, EXTI_Trigger_Falling);

    // WS2812 PWM+DMA driver init (TIM2_CH4 on PD5 via TIM2 Full Remap)
    ws2812_begin();

    // Initial state: check if charger is plugged in at boot
    g_prevState = S_WAIT; // signals: no auto-sleep timer
    g_batt_tAdc = 0;

    // If we woke from standby AND the charger is not plugged in, it was a
    // button-press wake.  Enter S_WAIT (blue breath) so the user sees the
    // device is alive.
    if (digitalRead(CHARGE_PIN) == LOW)
    {
        // Charger plugged in — either woke from standby by charger, or
        // powered on while charging
        fan_off();
        g_state = S_CHARGING;
    }
    else
    {
        // No charger present.  If this was a standby wake, the button
        // caused it — start in S_WAIT (idle).  Cold power-on also starts
        // in S_WAIT, so the user experience is identical.
        g_state = S_WAIT;
    }
}

void loop(void)
{
    unsigned long nowMillis = millis();

    // Charge pin debounce
    int chg_raw = digitalRead(CHARGE_PIN);
    if (chg_raw != g_chg_last_pin)
    {
        g_chg_tStable = nowMillis;
    }
    g_chg_last_pin = chg_raw;
    bool chg_low = (chg_raw == LOW) && (nowMillis - g_chg_tStable >= CHG_DEBOUNCE_MS);
    bool chg_high = (chg_raw == HIGH) && (nowMillis - g_chg_tStable >= CHG_DEBOUNCE_MS);

    // Clear EXTI flag unconditionally — its job is to wake the main loop
    if (g_chargeFlag)
    {
        g_chargeFlag = false;
    }

    // Charger plugged in (debounced)
    if (chg_low && g_state != S_CHARGING)
    {
        g_chgPrevState = g_state; // save state to restore on unplug
        fan_off();
        g_state = S_CHARGING;
        g_led_dirty = true;
    }

    // Charger unplugged (debounced)
    if (g_state == S_CHARGING && chg_high)
    {
        g_state = g_chgPrevState;
        g_led_dirty = true;
        if (g_state == S_ON)
        {
            digitalWrite(FAN_EN_PIN, HIGH);
            fan_pwm_write(dutyTable[g_dutyIdx]);
        }
        // If previous was S_WAIT, g_prevState is preserved (10s timer state intact)
    }

    switch (g_state)
    {
    case S_WAIT:
        state_run_wait(nowMillis);
        break;
    case S_ON:
        state_run_on(nowMillis);
        break;
    case S_CHARGING:
        state_run_charging(nowMillis);
        break;
    }
}
