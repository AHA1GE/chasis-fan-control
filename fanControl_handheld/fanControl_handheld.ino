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

// ---- WS2812 ----
#define LED_PIN PD4
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
    EXTI->FTENR |= (1 << 6);
    EXTI->INTENR |= (1 << 6);
    EXTI->INTFR = (1 << 6); // write 1 to clear any pending EXTI6

    // PC1 → EXTI1: falling edge trigger (charger plug-in)
    EXTI->FTENR |= (1 << 1);
    EXTI->INTENR |= (1 << 1);
    EXTI->INTFR = (1 << 1); // write 1 to clear any pending EXTI1

    // Enter STANDBY: SLEEPDEEP=1, PDDS=1
    // PWR->CTLR: bit 2 = SLEEPDEEP, bit 1 = PDDS
    RCC->APB1PCENR |= (1 << 28); // PWR clock enable
    PWR->CTLR |= (1 << 2);       // SLEEPDEEP
    PWR->CTLR |= (1 << 1);       // PDDS = STANDBY (not STOP)

    // Disable all interrupts to prevent spurious wake
    __asm__ volatile("csrw mie, %0"
                     :
                     : "r"(0));

    // Execute WFI
    __asm__ volatile("wfi");

    // Should never reach here — STANDBY wake triggers POR reset
    while (1)
    { /* wait for reset */
    }
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
        return;
    }

    switch (g_state)
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
    // GPIO init (Arduino API)
    pinMode(FAN_EN_PIN, OUTPUT);
    pinMode(FAN_PWM_PIN, OUTPUT);
    fan_off(); // fan off initially

    pinMode(BATT_PIN, INPUT);          // battery ADC
    pinMode(BTN_PIN, INPUT_PULLUP);    // button: active-low, internal pull-up
    pinMode(CHARGE_PIN, INPUT_PULLUP); // charge detect: active-low, pull-up

    // CHARGE_PIN EXTI for instant charge detection (falling edge = plug-in)
    attachInterrupt(CHARGE_PIN, GPIO_Mode_IPU, charge_isr,
                    EXTI_Mode_Interrupt, EXTI_Trigger_Falling);

    // Initial state: check if charger is plugged in at boot
    g_prevState = S_WAIT; // signals: no auto-sleep timer
    g_batt_tAdc = 0;

    if (digitalRead(CHARGE_PIN) == LOW)
    {
        // Woke from STANDBY by charger plug-in, or powered on while charging
        fan_off();
        g_state = S_CHARGING;
    }
    else
    {
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
