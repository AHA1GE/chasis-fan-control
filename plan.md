# Firmware Implementation Plan: Chasis Fan Control (CH32V003F4P6)

## Context

The hardware (`hardware/pins.md`, `hardware/Netlist_fan-control.asc`) defines a dual-fan controller with:
- CH32V003F4P6 MCU (48 MHz RISC-V, 16 KB flash, 2 KB SRAM)
- 2× PWM outputs (PD5 = Fan1, PD6 = Fan2) at 25 kHz via TIM1 (PartialRemap1)
- 2× tachometer inputs (PD2 = Fan1, PA1 = Fan2) via interrupt pulse counting
- 4× WS2812C LEDs daisy-chained on **PC6 / SPI1_MOSI** — driven via hardware SPI at 3 MHz
- 1× button on PC3 (active-low, internal pull-up) for speed selection
- ADC on PA2 for battery voltage (10k/4.7k divider), auto-detect 1S/2S/3S
- NRST on PD7; SWD/SWIO on PD1
- Physical on/off via MOSFET + slide switch (no firmware involvement)

## Code Style & Philosophy

- **Arduino API everywhere possible** — `pinMode()`, `digitalRead()`, `analogRead()`, `millis()`, `delay()`, `map()`, `noInterrupts()`/`interrupts()`, `attachInterrupt()`
- **Lean helpers, not bare-metal sprawl** — encapsulate low-level init in small `static` functions or lightweight classes (like the `Servo` pattern); call them once in `setup()`, then use clean high-level calls in `loop()`
- **`static` file-scope globals** — all state as static variables, no heap allocation
- **`enum : uint8_t`** for compact state machines
- **`#define` for all constants** at the top of the file
- **Non-blocking `loop()`** — use `millis()` interval checks instead of `delay()` so the button stays responsive
- **Optional debug toggles** — `#if ENABLE_SCREEN` pattern can be adapted for serial/screen debug if needed

The only modules that require low-level register access are **fan PWM init** and **SPI init** (both encapsulated once in `setup()`, like the `Servo` library pattern). The WS2812 data path uses pure SPI byte transfers — no inline assembly, no cycle counting, and most importantly **no interrupt disabling**.

---

## Module Breakdown

### 1. Pin Definitions & Constants

```cpp
// ---- Fan PWM (TIM1 with PartialRemap1: CH1→PD5, CH2→PD6) ----
#define PWM_FAN1_PIN      PD5   // TIM1_CH1, Fan 1
#define PWM_FAN2_PIN      PD6   // TIM1_CH2, Fan 2
#define PWM_FREQ_HZ       25000
#define PWM_PERIOD        1919  // 48MHz / 25000 - 1
#define DUTY_0_PERCENT    0
#define DUTY_25_PERCENT   480
#define DUTY_50_PERCENT   960
#define DUTY_75_PERCENT   1440
#define DUTY_100_PERCENT  1919

// ---- Tachometer ----
#define TACH1_PIN         PD2   // Fan 1 tach
#define TACH2_PIN         PA1   // Fan 2 tach
#define TACH_WINDOW_MS    2000  // RPM measurement interval

// ---- WS2812 via SPI ----
#define RGB_PIN           PC6   // SPI1_MOSI
#define NUM_LEDS          4
#define WS2812_SPI_CLK_HZ 3000000  // 48MHz / 16 = 3MHz → ~750kbps WS2812

// ---- Button ----
#define BTN_PIN           PC3
#define BTN_DEBOUNCE_MS   30
#define BTN_SHORT_MIN_MS  50
#define BTN_SHORT_MAX_MS  500
#define BTN_DOUBLE_GAP_MS 300
#define BTN_LONG_MS       1000

// ---- Battery ADC ----
#define BATT_PIN          PA2
#define ADC_MAX           1023
// Divider: BAT+ → 10k → VDETECT → 4k7 → GND  →  ratio = 4.7/(10+4.7) = 0.3197
// Vref = 5.0V (buck-boost output).  vBatt = adcRaw * 5.0 / 1024 / 0.3197
// Simplified integer math: vBatt_mV = adcRaw * 5000 * 147 / (1024 * 47)
#define BATT_FULL_MV_PER_CELL   4200
#define BATT_EMPTY_MV_PER_CELL  3000
#define BATT_CRITICAL_MV_PER_CELL 3200

// Cell-detect ADC thresholds (10-bit, Vref=5V):
#define ADC_3S_THRESHOLD   570   // >570 → 3S
#define ADC_2S_THRESHOLD   370   // 370-570 → 2S
#define ADC_1S_THRESHOLD   200   // 200-340 → 1S (below 200 = invalid)

// ---- Display update intervals ----
#define ADC_INTERVAL_MS    200
#define DISPLAY_INTERVAL_MS 100
#define RAINBOW_FRAME_MS   50
#define RAINBOW_DURATION_MS 2000

// ---- Duty cycle table (single-click cycling) ----
static const uint8_t dutyCycleTable[4] = { 0, 25, 50, 75 };
```

### 2. Global State

```cpp
enum SystemState : uint8_t {
    STATE_STARTUP = 0,
    STATE_RUNNING,
    STATE_ERROR
};
static SystemState systemState = STATE_STARTUP;

// Battery
static uint8_t  battCellCount = 0;    // 0=invalid, 1, 2, 3
static uint16_t battAdcAvg = 0;       // rolling average ADC value
static uint8_t  battPercent = 0;      // 0-100 SoC
static uint8_t  battGaugeLeds = 0;    // 0-4 LEDs lit for gauge
static uint32_t lastAdcMs = 0;

// Fan / tach
static volatile uint16_t tachPulses1 = 0;  // incremented in ISR
static volatile uint16_t tachPulses2 = 0;
static uint16_t fanRpm = 0;
static uint8_t  fanMode = 0;              // 1=single, 2=dual
static uint32_t lastTachMs = 0;

// PWM
static uint8_t  pwmDutyPercent = 0;
static uint8_t  pwmDutyIndex = 0;         // 0-3=cycle, 4=100%

// Button
static uint32_t lastBtnSampleMs = 0;

// WS2812 display buffer — color in GRB order (pre-expansion)
static uint8_t ledBuf[NUM_LEDS][3];       // [i][0]=G, [i][1]=R, [i][2]=B
// SPI output buffer — 24 color bits × 4 SPI-bits/color-bit / 8 = 12 bytes per LED
static uint8_t spiBuf[NUM_LEDS * 12];     // 4 LEDs × 12 = 48 bytes
static uint32_t lastDisplayMs = 0;
static bool     displayDirty = true;
```

### 3. Fan PWM Helper (encapsulated low-level init)

Like the reference `Servo` class, a small static helper configures TIM1 once in `setup()`. The actual duty updates in `loop()` use simple register writes.

```cpp
static void fanPWM_begin() {
    // Enable GPIOD + TIM1 clocks
    RCC->APB2PCENR |= RCC_APB2Periph_GPIOD | RCC_APB2Periph_TIM1;

    // PD5, PD6 as alternate push-pull (needed even with remap)
    // ... configure GPIOD CFGLR for PD5=AF_PP, PD6=AF_PP

    // Remap TIM1_CH1→PD5, TIM1_CH2→PD6
    AFIO->PCFR1 |= AFIO_PCFR1_TIM1_REMAP_PARTIAL1;

    // TIM1: 48 MHz, prescaler=0, period=1919 → 25 kHz
    TIM1->PSC = 0;
    TIM1->ATRLR = PWM_PERIOD;
    TIM1->CNT = 0;

    // CH1 + CH2: PWM mode 1, active high, preload enabled
    TIM1->CHCTLR1 = TIM_OCMode_PWM1 | TIM_OCPreload_Enable;  // CH1
    TIM1->CHCTLR2 = TIM_OCMode_PWM1 | TIM_OCPreload_Enable;  // CH2
    TIM1->CCER |= TIM_CCx_Enable(CH1) | TIM_CCx_Enable(CH2);

    // Set 0% duty initially
    TIM1->CH1CVR = 0;
    TIM1->CH2CVR = 0;

    // Main output enable + start
    TIM1->BDTR |= TIM_BDTR_MOE;
    TIM1->CTLR1 |= TIM_CEN;
}

static void fanPWM_setDuty(uint8_t percent) {
    uint16_t ccr = (uint16_t)((uint32_t)percent * PWM_PERIOD / 100);
    TIM1->CH1CVR = ccr;
    TIM1->CH2CVR = ccr;
}
```

> **Note:** The exact register names depend on the CH32V003 Arduino core headers. Adjust to match `ch32v003_gpio.h` / `ch32v003_tim.h` field names. The pattern is identical to what `CH32V003_SERVO.h` does internally, just with different TIM1 parameters (25 kHz vs 50 Hz).

### 4. GPIO Initialization (`setup()`)

Uses Arduino `pinMode()` everywhere:

```cpp
void setup() {
    // Button: input with internal pull-up
    pinMode(BTN_PIN, INPUT_PULLUP);

    // WS2812 SPI (PC6 = MOSI): configured in ws2812_spi_begin()
    // No digitalWrite needed — SPI hardware drives the pin

    // Tach inputs: pull-up (open-drain from fan) — PD2 + PA1
    pinMode(TACH1_PIN, INPUT_PULLUP);
    pinMode(TACH2_PIN, INPUT_PULLUP);

    // Battery ADC: analog input
    pinMode(BATT_PIN, INPUT_ANALOG);

    // PWM init (register-level, encapsulated)
    fanPWM_begin();

    // SPI init for WS2812 (register-level, encapsulated)
    ws2812_spi_begin();

    // Tach interrupts
    attachInterrupt(digitalPinToInterrupt(TACH1_PIN), tach1_ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(TACH2_PIN), tach2_ISR, RISING);

    systemState = STATE_STARTUP;
}
```

> If `attachInterrupt()` is not available on PD2/PA1 in the specific Arduino core, fall back to configuring EXTI directly (small init block, same pattern as `fanPWM_begin`). The ISR bodies are identical either way.

### 5. Non-Blocking Main Loop

```cpp
void loop() {
    uint32_t now = millis();

    switch (systemState) {
        case STATE_STARTUP:
            startup_run(now);
            break;
        case STATE_RUNNING:
            running_run(now);
            break;
        case STATE_ERROR:
            error_run(now);
            break;
    }
}
```

No `delay()` calls — each subsystem checks `millis()` intervals.

### 6. Button Gesture Detection (polled in `running_run`)

Since `loop()` runs continuously without blocking, button is sampled every ~2ms:

```cpp
static void button_poll(uint32_t now) {
    // Limit sampling rate to ~1ms for debounce counting
    if (now - lastBtnSampleMs < 1) return;
    lastBtnSampleMs = now;

    static uint8_t  raw = HIGH, lastRaw = HIGH;
    static uint8_t  debounced = HIGH, lastDebounced = HIGH;
    static uint8_t  debounceCnt = 0;
    static uint32_t pressStartMs = 0;
    static uint32_t lastReleaseMs = 0;
    static uint8_t  pendingClicks = 0;
    static bool     longFired = false;

    raw = digitalRead(BTN_PIN);  // LOW = pressed

    // Debounce: count consecutive same-samples
    if (raw == lastRaw) {
        if (debounceCnt < BTN_DEBOUNCE_MS) debounceCnt++;
    } else {
        debounceCnt = 0;
    }
    lastRaw = raw;
    if (debounceCnt >= BTN_DEBOUNCE_MS) debounced = raw;

    // Falling edge → press started
    if (debounced == LOW && lastDebounced == HIGH) {
        pressStartMs = now;
        longFired = false;
    }
    // Rising edge → released
    if (debounced == HIGH && lastDebounced == LOW) {
        uint32_t held = now - pressStartMs;
        if (held >= BTN_SHORT_MIN_MS && held <= BTN_SHORT_MAX_MS) {
            pendingClicks++;
        }
        lastReleaseMs = now;
    }
    lastDebounced = debounced;

    // Long press while held (fire once)
    if (debounced == LOW && !longFired && (now - pressStartMs) > BTN_LONG_MS) {
        longFired = true;
        pendingClicks = 0;          // cancel any pending clicks
        pwmDutyIndex = 0;
        pwmDutyPercent = dutyCycleTable[0];
        fanPWM_setDuty(0);
        displayDirty = true;
    }

    // Resolve gestures after double-click gap timeout
    if (pendingClicks > 0 && (now - lastReleaseMs) > BTN_DOUBLE_GAP_MS) {
        if (pendingClicks == 1) {
            // Single click → cycle 0→25→50→75
            pwmDutyIndex = (pwmDutyIndex + 1) % 4;
            pwmDutyPercent = dutyCycleTable[pwmDutyIndex];
        } else {
            // Double click → 100%
            pwmDutyIndex = 4;
            pwmDutyPercent = 100;
        }
        fanPWM_setDuty(pwmDutyPercent);
        displayDirty = true;
        pendingClicks = 0;
    }
}
```

**Gesture-to-action map:**

| Gesture | Action |
|---------|--------|
| Single click | Cycle `duty_index` (0→1→2→3→0) → 0%, 25%, 50%, 75% |
| Double click | Set `duty_index=4` → 100% |
| Long press (>1s) | Fire immediately → 0% (cancels any pending clicks) |

### 7. WS2812 via SPI (PC6 / SPI1_MOSI)

**Why SPI over bit-bang:** Hardware SPI at 3 MHz eliminates interrupt blocking entirely. The bit-bang approach disables IRQs for ~172 µs per LED update, which causes tach pulse loss. SPI transfers happen autonomously in hardware — tach ISRs fire without interference. No inline assembly, no cycle counting, no calibration needed.

**Principle** (from `drive ws2812 use spi.md`): Each WS2812 data bit is expanded to 4 SPI bits:
- `1` → `1110` (3/4 HIGH, 1/4 LOW ≈ 1.0 µs / 0.33 µs)
- `0` → `1000` (1/4 HIGH, 3/4 LOW ≈ 0.33 µs / 1.0 µs)

SPI clock = 48 MHz / 16 = **3 MHz** → 1 SPI bit = 333 ns. Each WS2812 bit = 4 × 333 = 1.33 µs ≈ 750 kbps (within tolerance of the 800 kbps target). Verified working in practice.

Each color byte (8 bits) → 4 SPI bytes (2 bits per SPI byte). One LED (GRB = 3 bytes) → 12 SPI bytes. 4 LEDs → **48 bytes** total.

#### 7a. SPI initialization (called once in `setup()`)

```cpp
static void ws2812_spi_begin() {
    // Enable SPI1 + GPIOC clocks
    RCC->APB2PCENR |= RCC_APB2Periph_SPI1 | RCC_APB2Periph_GPIOC;

    // PC6 = alternate push-pull (SPI1_MOSI, GPIO Port C)
    // Configure GPIOC CFGLR: PC6 → AF_PP, 50MHz
    // (exact register writes depend on core header names)

    // SPI1: master, TX-only, MSB first, baud = 48MHz/16 = 3MHz
    SPI1->CTLR1 = SPI_CTLR1_MSTR | SPI_CTLR1_BR_DIV16 | SPI_CTLR1_MSB_FIRST;
    SPI1->CTLR2 = 0;  // no NSS, no interrupts needed
    SPI1->CTLR1 |= SPI_CTLR1_SPE;  // enable
}
```

> Register field names match the CH32V003 Arduino core's `ch32v003_spi.h`. Adjust to the core's actual constants. The pattern is identical to what `fanPWM_begin()` does.

#### 7b. Bit-expansion + SPI send

```cpp
// 4-entry LUT: maps 2 color bits → 1 SPI byte
// Index = bits[1:0] from the color byte
static const uint8_t ws2812LUT[4] = {
    0x88,  // 00 → 1000_1000
    0x8E,  // 01 → 1000_1110
    0xE8,  // 10 → 1110_1000
    0xEE,  // 11 → 1110_1110
};

static void ws2812_send() {
    // Expand ledBuf[4][3] (GRB) → spiBuf[48]
    uint8_t *out = spiBuf;
    for (uint8_t i = 0; i < NUM_LEDS; i++) {
        // Order: G, R, B (WS2812C expects Green first)
        for (uint8_t comp = 0; comp < 3; comp++) {
            uint8_t c = ledBuf[i][comp];
            // Expand 8 bits → 4 SPI bytes (2 input bits per output byte)
            for (uint8_t j = 0; j < 4; j++) {
                *out++ = ws2812LUT[(c >> 6) & 0x03];
                c <<= 2;
            }
        }
    }

    // Send all 48 bytes via SPI (hardware handles timing, no IRQs disabled)
    for (uint8_t i = 0; i < sizeof(spiBuf); i++) {
        while (!(SPI1->STATR & SPI_STATR_TXE));  // wait TX empty
        SPI1->DATAR = spiBuf[i];
    }
    // Wait for last byte to finish
    while (SPI1->STATR & SPI_STATR_BSY);

    // RESET: hold MOSI low >50 µs before next frame
    delayMicroseconds(60);
}
```

> **No `noInterrupts()`!** SPI hardware runs autonomously. Tach ISRs fire freely during the 48-byte transfer (~128 µs at 3 MHz). Zero pulse loss.

### 8. Startup Sequence

```cpp
static void startup_run(uint32_t now) {
    static uint32_t startMs = 0;
    static uint8_t  frame = 0;
    static uint32_t lastFrameMs = 0;

    // Init on first call
    if (startMs == 0) {
        startMs = now;
        lastFrameMs = now;
        frame = 0;
        fanPWM_setDuty(0);  // fans off during animation
    }

    // Rainbow animation (2 seconds)
    if (frame < (RAINBOW_DURATION_MS / RAINBOW_FRAME_MS)) {
        if (now - lastFrameMs >= RAINBOW_FRAME_MS) {
            lastFrameMs = now;
            for (uint8_t i = 0; i < NUM_LEDS; i++) {
                uint8_t pos = (frame * 8 + i * 64) & 0xFF;
                colorWheel(pos, ledBuf[i]);
            }
            ws2812_send();
            frame++;
        }
        return;
    }

    // Animation done — run checks
    battCellCount = battery_detectCells();  // 4-sample avg ADC

    // Fan presence: 100% PWM for 500ms, count tach for 1s
    fanPWM_setDuty(100);
    delay(500);
    noInterrupts();
    tachPulses1 = 0;
    tachPulses2 = 0;
    interrupts();
    delay(1000);
    noInterrupts();
    uint16_t c1 = tachPulses1, c2 = tachPulses2;
    tachPulses1 = tachPulses2 = 0;
    interrupts();
    fanPWM_setDuty(0);

    if (c1 > 3 && c2 > 3)      fanMode = 2;
    else if (c1 > 3)           fanMode = 1;
    else                       fanMode = 0;

    // Error check
    if (battCellCount == 0 || fanMode == 0) {
        systemState = STATE_ERROR;
        return;
    }

    // Success → RUNNING
    systemState = STATE_RUNNING;
    displayDirty = true;
}
```

`colorWheel()` — HSV→RGB for rainbow (same algorithm as plan v1, returning `{g, r, b}` for GRB order).

### 9. Battery ADC (Arduino `analogRead`)

```cpp
static uint8_t battery_detectCells() {
    uint32_t sum = 0;
    for (int i = 0; i < 4; i++) sum += analogRead(BATT_PIN);
    uint16_t avg = (uint16_t)(sum >> 2);

    if (avg > ADC_3S_THRESHOLD)      return 3;
    else if (avg > ADC_2S_THRESHOLD) return 2;
    else if (avg > ADC_1S_THRESHOLD) return 1;
    else                             return 0;  // invalid
}

static void battery_sample() {
    uint16_t raw = analogRead(BATT_PIN);
    // Rolling average: 4-sample
    static uint16_t buf[4] = {0};
    static uint8_t idx = 0;
    buf[idx] = raw;
    idx = (idx + 1) & 3;
    battAdcAvg = (buf[0] + buf[1] + buf[2] + buf[3]) >> 2;

    // Compute battery mV:  vBatt = adc * 5000mV * (10k+4.7k) / (1024 * 4.7k)
    uint32_t vBattMv = ((uint32_t)battAdcAvg * 5000UL * 147UL) / (1024UL * 47UL);
    uint32_t vCellMv = vBattMv / battCellCount;

    // SoC per cell: 0% at 3.0V, 100% at 4.2V
    battPercent = (uint8_t)constrain(
        map((long)vCellMv, BATT_EMPTY_MV_PER_CELL, BATT_FULL_MV_PER_CELL, 0L, 100L),
        0L, 100L
    );

    // Gauge LED count
    if      (battPercent >= 75) battGaugeLeds = 4;
    else if (battPercent >= 50) battGaugeLeds = 3;
    else if (battPercent >= 25) battGaugeLeds = 2;
    else if (battPercent >= 5)  battGaugeLeds = 1;
    else                        battGaugeLeds = 0;  // critical low
}
```

### 10. Tachometer ISRs

```cpp
static void tach1_ISR() { tachPulses1++; }
static void tach2_ISR() { tachPulses2++; }
```

RPM computation (called every `TACH_WINDOW_MS` in `running_run`):

```cpp
static void tach_update() {
    noInterrupts();
    uint16_t c1 = tachPulses1, c2 = tachPulses2;
    tachPulses1 = tachPulses2 = 0;
    interrupts();

    // RPM = pulses * 60 / (2 pulses/rev * window_seconds)
    uint16_t rpm1 = c1 * (60000UL / (2 * TACH_WINDOW_MS));
    uint16_t rpm2 = c2 * (60000UL / (2 * TACH_WINDOW_MS));

    if (fanMode == 2) fanRpm = (rpm1 + rpm2) / 2;
    else              fanRpm = rpm1;
}
```

With 2-second window: `RPM = pulses × 15`.

### 11. Display Compute (RUNNING state, every 100ms)

```cpp
// Speed color palette (GRB order)
static const uint8_t speedPalette[5][3] = {
    {0,   0,   128},   // Blue   (0-1000 RPM)
    {128, 0,   128},   // Cyan   (1000-2000)
    {128, 0,   0  },   // Green  (2000-3000)
    {128, 128, 0  },   // Yellow (3000-5000)
    {0,   128, 0  },   // Red    (5000+)
};

static uint8_t rpmToColorIdx(uint16_t rpm) {
    if (rpm < 1000) return 0;
    if (rpm < 2000) return 1;
    if (rpm < 3000) return 2;
    if (rpm < 5000) return 3;
    return 4;
}

static void display_update(uint32_t now) {
    if (!displayDirty && (now - lastDisplayMs < DISPLAY_INTERVAL_MS)) return;
    lastDisplayMs = now;
    displayDirty = false;

    // Fill gauge LEDs as dim white
    for (uint8_t i = 0; i < NUM_LEDS; i++) {
        if (i < battGaugeLeds) {
            uint8_t bright = 24 + i * 16;  // increasing brightness
            ledBuf[i][0] = bright;  // G
            ledBuf[i][1] = bright;  // R
            ledBuf[i][2] = bright;  // B  → dim white
        } else {
            ledBuf[i][0] = ledBuf[i][1] = ledBuf[i][2] = 0;
        }
    }

    // Overlay "upper" (last lit) LED with speed color
    if (battGaugeLeds > 0) {
        uint8_t upper = battGaugeLeds - 1;
        uint8_t ci = rpmToColorIdx(fanRpm);
        ledBuf[upper][0] = speedPalette[ci][0];
        ledBuf[upper][1] = speedPalette[ci][1];
        ledBuf[upper][2] = speedPalette[ci][2];
    }

    // Critical battery blink (<10%): flash last LED off at 2Hz
    if (battPercent < 10 && battGaugeLeds <= 1) {
        if ((now / 250) & 1) {  // 250ms on, 250ms off
            ledBuf[0][0] = ledBuf[0][1] = ledBuf[0][2] = 0;
        }
    }

    ws2812_send();
}
```

### 12. RUNNING State (`running_run`)

```cpp
static void running_run(uint32_t now) {
    button_poll(now);

    if (now - lastAdcMs >= ADC_INTERVAL_MS) {
        lastAdcMs = now;
        battery_sample();
        displayDirty = true;
    }

    if (now - lastTachMs >= TACH_WINDOW_MS) {
        lastTachMs = now;
        tach_update();
        displayDirty = true;
    }

    display_update(now);
}
```

### 13. ERROR State

```cpp
static void error_run(uint32_t now) {
    // Fast blink all 4 LEDs red at ~5 Hz (100ms period)
    static uint32_t lastBlinkMs = 0;
    static bool on = false;

    if (now - lastBlinkMs >= 100) {
        lastBlinkMs = now;
        on = !on;
        for (uint8_t i = 0; i < NUM_LEDS; i++) {
            ledBuf[i][0] = 0;       // G=0
            ledBuf[i][1] = on ? 128 : 0;  // R
            ledBuf[i][2] = 0;       // B=0
        }
        ws2812_send();
    }

    // PWM stays at 0%, button ignored, only exit is power cycle
}
```

### 14. Display Behavior Summary

| What you see | Meaning |
|-------------|---------|
| 4× rainbow rotating (2s) | Power-on startup |
| All 4 red fast blink (5 Hz) | Error: bad battery OR no fans detected |
| 1 LED white | Battery 5–25% |
| 2 LEDs white | Battery 25–50% |
| 3 LEDs white | Battery 50–75% |
| 4 LEDs white | Battery 75–100% |
| Top LED = blue → cyan → green → yellow → red | Fan RPM low → high |
| Top LED blinking off/on | Battery critically low (<10%) |

---

## State Machine

```
[POWER ON]
    |
    v
+----------+   bat valid AND fan(s) present   +----------+
| STARTUP  | --------------------------------> | RUNNING  |
| rainbow  |                                   | normal   |
| 2 sec    |   bat invalid OR no fans          | ops      |
+----------+ --------> +-------+              +----------+
                       | ERROR |
                       | fast  |
                       | blink |
                       | red   |
                       | 5 Hz  |
                       +-------+
```

---

## File Structure

Single file: `fanControl/fanControl.ino`

Sections (in order):
1. `#define` constants & pin assignments
2. Global state variables (`static`)
3. Low-level helper: `fanPWM_begin()` / `fanPWM_setDuty()` — register init, encapsulated
4. Low-level helper: `ws2812_spi_begin()` / `ws2812_send()` + `ws2812LUT[]` — SPI init + bit expansion
5. Color helpers: `colorWheel()`, `rpmToColorIdx()`, `speedPalette[]`
6. Battery helpers: `battery_detectCells()`, `battery_sample()`
7. Tach ISRs + `tach_update()`
8. Button: `button_poll()`
9. Display: `display_update()`
10. State runners: `startup_run()`, `running_run()`, `error_run()`
11. `setup()` — Arduino pinMode + one-time inits
12. `loop()` — dispatch to state runner

---

## Implementation Order

| Step | What | Verify with |
|------|------|------------|
| 1 | `fanPWM_begin()` + `fanPWM_setDuty()` | Oscilloscope: 25 kHz on PD5/PD6, duty changes |
| 2 | `ws2812_spi_begin()` + `ws2812_send()` with test pattern | Logic analyzer on PC6; visually on LEDs |
| 3 | `colorWheel()` + rainbow animation | Visual: smooth rotation on all 4 LEDs |
| 4 | `battery_detectCells()` + `battery_sample()` | Variable PSU: verify 1S/2S/3S detection |
| 5 | `tach1_ISR()`/`tach2_ISR()` + `tach_update()` | Signal gen or real fan: verify RPM |
| 6 | `button_poll()` | Serial print or LED blink: single/double/long |
| 7 | `display_update()` — gauge + speed overlay | Visual integration |
| 8 | State machine: `startup_run`/`running_run`/`error_run` | Full system test |
| 9 | `setup()` + `loop()` integration | End-to-end: power-up → rainbow → running → error |

## Verification

1. **Oscilloscope:** 25 kHz PWM on PD5/PD6, duty responds to button gestures
2. **Logic analyzer:** Confirm SPI MOSI (PC6) waveform matches WS2812 timing — 3 MHz clock, correct bit patterns
3. **Visual:** Rainbow animation rotates, error fast-blinks red, speed color transitions across RPM range
4. **Fan test:** Real fans connected via CN1 or CN4, confirm tach read-back and single/dual detection
5. **Battery test:** Variable DC supply at BAT+, verify 1S/2S/3S thresholds, gauge LED count, critical blink
6. **Edge cases:** Power on with no battery → ERROR; power on with battery but no fans → ERROR; hot-plug fan during RUNNING → speed LED falls to blue
7. **Tach integrity:** With SPI-driven WS2812, confirm zero tach pulse loss (scope on PD2/PA1 during WS2812 update — ISR must fire without delay)
