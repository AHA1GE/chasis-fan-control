# Firmware Implementation Plan: Chasis Fan Control (CH32V003F4P6)

## Context

The hardware (`hardware/pins.md`, `hardware/Netlist_fan-control.asc`) defines a dual-fan controller with:
- CH32V003F4P6 MCU (48 MHz RISC-V, 16 KB flash, 2 KB SRAM)
- 2× PWM outputs (PD5 = Fan1, PD6 = Fan2) at 25 kHz
- 2× tachometer inputs (PC7 = Fan1, PD3 = Fan2) via EXTI pulse counting
- 4× WS2812C LEDs daisy-chained on PC2 (battery gauge + speed indicator)
- 1× button on PC1 (active-low) for speed selection
- ADC on PD2 for battery voltage (10k/4.7k divider), auto-detect 1S/2S/3S
- Physical on/off via MOSFET + slide switch
- Existing file `fanControl/fanControl.ino` contains servo-demo code (to be replaced)
- Code style: bare-metal WCH Standard Peripheral Library (RCC, GPIO, TIM, ADC, EXTI, NVIC register-level calls)

## Implementation Strategy

**Single-file firmware** (`fanControl/fanControl.ino`), organized into clearly commented sections. Keep under ~16 KB flash / 2 KB SRAM.

## Module Breakdown (in implementation order)

### 1. Pin Definitions & Constants
- All pin macros, timing constants, duty table `{0, 25, 50, 75}`, ADC thresholds
- GPIO remap: `GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM1, ENABLE)` to route TIM1_CH1/CH2 to PD5/PD6

### 2. Global State Structs
```c
typedef enum { STATE_STARTUP, STATE_RUNNING, STATE_ERROR } SystemState;

typedef struct {
    uint8_t cell_count;     // 0=invalid, 1, 2, 3
    uint16_t adc_avg;       // filtered ADC reading
    uint8_t  percent;       // 0-100 SoC
    uint8_t  gauge_leds;    // 1-4 LEDs to light for gauge
} BatteryState;

typedef struct {
    uint16_t rpm;           // averaged RPM (0 if stalled)
    uint8_t  fan_mode;      // 1=single, 2=dual
} FanState;

typedef struct {
    uint8_t duty_percent;   // 0, 25, 50, 75, 100
    uint8_t duty_index;     // 0-3=cycle, 4=100%
} PwmState;

// WS2812 pixel in GRB order (WS2812C expects Green first)
typedef struct { uint8_t g, r, b; } PixelGRB;

typedef struct {
    SystemState  state;
    BatteryState battery;
    FanState     fan;
    PwmState     pwm;
    PixelGRB     leds[4];
    uint8_t      update_display;
    uint32_t     tick_ms;
} SystemContext;
```

### 3. GPIO Initialization
- PC1: input w/ pull-up (button)
- PC2: push-pull output (WS2812 data)
- PC7: input w/ pull-up (Fan 1 tach)
- PD2: analog input (battery ADC)
- PD3: input w/ pull-up (Fan 2 tach)
- PD5, PD6: AF push-pull (PWM — set in PWM init)

### 4. PWM (TIM1, 25 kHz)
```
TIM1 clock = 48 MHz
TIM_Prescaler = 0
TIM_Period = 1919  →  48 MHz / 1920 = 25 kHz exactly
Resolution: 1920 steps (~0.052% per step)

Duty table (CCR values):
  0%   → 0
  25%  → 480
  50%  → 960
  75%  → 1440
  100% → 1919
```
- CH1 (PD5) = Fan 1, CH2 (PD6) = Fan 2, both PWM mode 1, active-high
- Both channels always set to same duty cycle
- `TIM_CtrlPWMOutputs(TIM1, ENABLE)` required for MOE bit

**Critically: `GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM1, ENABLE)`** — needed to route TIM1_CH1 to PD5 and TIM1_CH2 to PD6 (default mapping is PA8/PA9 which don't exist on TSSOP-20).

### 5. SysTick (1 ms)
- `SysTick_Config(SystemCoreClock / 1000)` for 1 ms tick
- Handler: increments `system_tick_ms`, calls `button_tick()`, updates `ctx.tick_ms`
- Priority 0 (highest) — must not be preempted by EXTI

### 6. Button Gesture Detection (PC1)
Timing thresholds:
| Parameter | Value |
|-----------|-------|
| Debounce | 30 ms |
| Short press min | 50 ms |
| Short press max | 500 ms |
| Double-click gap | 300 ms (max between releases) |
| Long press | 1000 ms |

State machine (processed every 1 ms from SysTick):
- Tracks raw → debounced state (30 consecutive matching samples)
- Detects falling edge (press start), rising edge (release)
- On release: if held 50-500 ms → increment `pending_clicks`
- While held > 1000 ms → fire long-press (set duty=0%, duty_index=0) — fire ONLY once per hold
- After release + 300 ms of silence:
  - `pending_clicks == 1` → single-click: cycle duty_index (0→1→2→3→0)
  - `pending_clicks >= 2` → double-click: duty_index=4, duty=100%

### 7. WS2812 Bit-Bang (PC2)
**Critical:** interrupts disabled during entire transmission (~172 µs for 4 LEDs).

Timing at 48 MHz (WS2812C spec: T0H=200-500ns, T0L=650-950ns, T1H=550-850ns, T1L=450-750ns, RESET>50µs):
```
Bit '0': HIGH 16 cycles (333ns) + LOW 44 cycles (917ns) = 60 cycles
Bit '1': HIGH 32 cycles (667ns) + LOW 28 cycles (583ns) = 60 cycles
Total per LED: 24 bits × 60 = 1440 cycles ≈ 30 µs
4 LEDs: 120 µs + 52 µs reset ≈ 172 µs IRQs off
```
- Pure inline assembly with NOP padding for cycle accuracy
- `delay_cycles(n)` macro: `addi + bnez` loop (2 cycles/iter)
- Calibrate with oscilloscope — adjust constants if needed
- Byte order: **G, R, B** (WS2812C-specific)
- `ws2812_send_buffer(ctx.leds)` sends all 4 pixels in one critical section

### 8. Startup Rainbow Animation
- HSV color wheel, 256 positions
- 4 LEDs offset by 64 (90° apart)
- Every 50 ms: advance by 8 positions → smooth rotation
- 40 frames × 50 ms = 2 seconds total
- Runs in STARTUP state, renders via `ws2812_send_buffer()`

### 9. ADC & Battery Detection (PD2)
Divider: VDETECT = BAT+ × 4.7k / (10k + 4.7k) = BAT+ × 0.3197

Cell detection (averaged over 4 samples at boot):
| Cells | VDETECT range | ADC range (10-bit, Vref=5V) |
|-------|---------------|---------------------------|
| 1S | 0.96 – 1.34 V | 196 – 275 |
| 2S | 1.92 – 2.69 V | 393 – 551 |
| 3S | 2.88 – 4.03 V | 590 – 825 |

Thresholds with guard-bands: ADC > 570 → 3S; 370–570 → 2S; 200–340 → 1S; else → invalid (error)

SoC per cell: `(V_cell - 3.0) / 1.2 × 100`, clamped 0–100.
Gauge LED count: <5%=0 (critical), 5-25%=1, 25-50%=2, 50-75%=3, 75-100%=4.

In RUNNING: sample ADC every 200 ms, 4-sample rolling average. Cell count locked at boot (no dynamic re-detection).

### 10. Tachometer (PC7 + PD3)
- EXTI on both pins, rising edge, priority 1
- ISRs increment `volatile uint16_t` counters
- Every 2 seconds: capture counters atomically, compute delta, RPM = delta × 15
  (60 s/min ÷ 2 pulses/rev ÷ 2 s window = 15)
- Dual fan: RPM = (rpm1 + rpm2) / 2; Single fan: RPM = rpm1
- Startup detection: PWM=100% for 500ms, count pulses for 1s. If both >3: dual. If only PC7 >3: single. If both ≤3: error.

IRQ impact from WS2812 (~172µs every 100ms): at worst (<1% pulse loss) — negligible for RPM accuracy.

### 11. Display Compute Logic (RUNNING, every 100ms)
```
For i = 0..3:
  if i < gauge_leds:
    leds[i] = dim white (brightness scales with i)
  else:
    leds[i] = off

"Upper" LED (index = gauge_leds - 1) overwritten with speed color:
  RPM color map (5 bins): blue → cyan → green → yellow → red
  (0-1000, 1000-2000, 2000-3000, 3000-5000, 5000+)

Critical battery (<10%): blink the last gauge LED at 500ms period.
```

### 12. Error Handling
**Enter ERROR state if at startup:**
- Battery voltage not in valid range for any cell count (ADC outside 200-825), **OR**
- Both tach signals show 0 pulses after 100% PWM for 500ms + 1s measurement

**ERROR state behavior:**
- All 4 LEDs fast-blink red at **~5 Hz** (100ms on, 100ms off) — visually distinct "fast blink"
- PWM stays at 0%
- No button or ADC processing
- Only exit: power cycle

### 13. Main Loop
```c
void loop() {
    uint32_t now = ctx.tick_ms;
    switch (ctx.state) {
        case STATE_STARTUP:
            // Run 2s rainbow animation, then battery+fan checks
            // Transition to RUNNING or ERROR
            break;
        case STATE_RUNNING:
            // 1. ADC sample every 200ms
            // 2. Tach RPM calculation every 2000ms
            // 3. Display update every 100ms (or on duty change)
            break;
        case STATE_ERROR:
            // Blink all red at 5 Hz (100ms period)
            break;
    }
}
```
CPU idle >99.8% of the time.

### 14. ISR Priority Assignment
| Interrupt | Priority | Rationale |
|-----------|----------|-----------|
| SysTick | 0 (highest) | 1ms button timing must not drift |
| EXTI3 (tach2) | 1 | Tach pulses must not be missed |
| EXTI7 (tach1) | 1 | Shared EXTI5_9 vector, same priority |

## State Machine Diagram
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
                       +-------+
```

## Implementation Order
1. GPIO init + PWM (verify 25 kHz on PD5/PD6 with scope)
2. SysTick (1ms, verify with GPIO toggle)
3. Button gesture (verify with debug LED patterns)
4. WS2812 send function (verify with single test pattern)
5. Rainbow animation (visual confirmation)
6. ADC + battery detection (verify thresholds)
7. Tachometer EXTI + RPM calc (verify with signal generator or fan)
8. Display compute (gauge + speed color integration)
9. Main loop + state machine
10. Error handling + final integration

## Verification
1. **Oscilloscope:** Confirm 25 kHz PWM on PD5/PD6, verify duty cycle changes with button
2. **Logic analyzer:** Calibrate WS2812 bit timing (T0H/T0L/T1H/T1L within spec)
3. **Visual:** Rainbow animation rotates smoothly, error blink is fast and clearly red
4. **Fan test:** Connect real fans, confirm RPM read-back and speed color mapping
5. **Battery test:** Power board from variable supply, verify cell-count detection at 1S/2S/3S thresholds
6. **Edge cases:** Power on with no battery, no fans — verify ERROR state triggers
