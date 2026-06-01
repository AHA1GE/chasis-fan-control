# Chasis Fan Control

Dual chassis fan controller based on the **CH32V003** RISC-V microcontroller. Drives two 4-pin PWM fans simultaneously with push-button speed selection, WS2812 battery gauge, and tachometer read-back — all powered from a 1S–3S LiPo with automatic cell-count detection.

## Features

- **Dual fan PWM control** — both fans run at the same duty cycle, driven by hardware PWM from the CH32V003.
- **Push-button speed selection** — single-click cycles through 0% → 25% → 50% → 75%; double-click jumps to 100%; long-press snaps to 0%.
- **Tachometer feedback** — reads fan RPM from the open-drain tach signal and maps speed to color on the WS2812 LEDs (blue → cyan → green → yellow → red for low → high).
- **Battery gauge** — 4× WS2812C LEDs show state-of-charge by count: 1 LED = 5–25%, 2 = 25–50%, 3 = 50–75%, 4 = 75–100%. The appropriate LED blinks when the battery is critically low.
- **Auto cell-count detection** — voltage divider sampled at boot recognizes 1S (3.0–4.2 V), 2S (6.0–8.4 V), or 3S (9.0–12.6 V) LiPo packs.
- **Physical master switch** — a slide switch fully disconnects battery ground via an N-channel MOSFET; zero quiescent draw when off.
- **Buck-boost regulator** — TPS63070 provides a clean 5 V rail from a 2–16 V battery input; works across the full discharge range of any supported pack.
- **AIO header (H1)** — 5-pin all-in-one header combining SWD (WCH-LinkE / J-Link) with a reserved WS2812 extension passthrough on pin 5 (no firmware function yet).
- **Flexible fan connection** — primary CN1 socket for two fans, optional CN4 header for a single fan. Firmware auto-adapts: uses combined tach reading with two fans, falls back to the single tach with one.

## Hardware

| Block | Part | Notes |
|-------|------|-------|
| MCU | CH32V003F4P6 (TSSOP-20) | 48 MHz RISC-V, 16 KB flash, 2 KB SRAM |
| Power | TPS63070RNMR | 2–16 V buck-boost, 5 V / 2 A out |
| Inductor | FTC252012S1R5MBCA | 1.5 µH, 3.7 A rated |
| MOSFET | HD504N075SG | N-channel, 40 V / 40 A, low-side battery disconnect |
| LEDs | 4× WS2812C (5050) | Daisy-chained RGB, 5 mA low-current variant |
| Button | KH-6X6X5H-STM | Tactile SMD, 6×6×5 mm |
| Power switch | SS-12D10L5 | SPDT slide switch, 3 A / 125 V |
| Dual-fan connector (CN1) | SH1.25-8ALT | 8-pin SMD, 1.25 mm pitch, two 4-pin fans |
| Single-fan connector (CN4) | M2510V-04P-N3 | 4-pin TH, 2.54 mm pitch, optional alternate |
| AIO header (H1) | PZ254V-11-05P | 5-pin TH, 2.54 mm pitch, SWD + reserved WS2812 |

Full pinouts and schematic netlist are in [`hardware/pins.md`](hardware/pins.md) and [`hardware/Netlist_fan-control.asc`](hardware/Netlist_fan-control.asc).

## Usage

### Power On

1. Connect your fan(s): plug two fans into CN1, or a single fan into CN4 (the two connectors are alternatives — pick whichever fits your setup).
2. Connect a 1S–3S LiPo battery (battery power enters via CN1 pins 7–8 or solder pads).
3. Flip the slide switch (SW2) to the **ON** position — the buck-boost starts, the MCU boots, and all 4 WS2812 LEDs run a brief power-on animation.
4. The MCU samples the battery voltage divider and determines the cell count. The corresponding number of LEDs light up to show the state of charge.

### Fan Speed Control

| Gesture | Action |
|---------|--------|
| **Single click** | Cycle duty: **0% → 25% → 50% → 75%** (wraps to 0%) |
| **Double click** | Jump to **100%** duty |
| **Long press** | Instantly return to **0%** duty |

After power-up both fans start at 0% PWM (lowest speed / stopped, depending on the fan). The active duty cycle is indicated by the WS2812 LEDs.

### Reading Fan Speed

The "up" LED color encodes the current fan RPM read from the tachometer signal:

| Color | Meaning |
|-------|---------|
| 🔵 Blue | Lowest RPM |
| 🩵 Cyan | Low |
| 🟢 Green | Medium |
| 🟡 Yellow | High |
| 🔴 Red | Highest RPM |

### Battery Gauge

| LEDs lit | State of charge |
|----------|----------------|
| 1 LED | 5% – 25% |
| 2 LEDs | 25% – 50% |
| 3 LEDs | 50% – 75% |
| 4 LEDs | 75% – 100% |

When the battery falls to a critical level, the last remaining LED blinks to warn of low battery.

### Power Off

Flip the slide switch to **OFF** — the MOSFET disconnects battery ground and the entire board draws zero current.

## Firmware

[`fanControl/fanControl.ino`](fanControl/fanControl.ino) — single-file firmware for the CH32V003 Arduino environment.

> **⚠️ Status: freshly written, not yet tested on hardware.** The PCB has not been manufactured. The firmware compiles but has not been validated against real fans, batteries, or WS2812 LEDs. See [Testing](#testing) below.

### Pin assignments

| GPIO | Function | Detail |
|------|----------|--------|
| PD5 | Fan 1 PWM | TIM1_CH1, 25 kHz (PartialRemap1) |
| PD6 | Fan 2 PWM | TIM1_CH2, 25 kHz |
| PD2 | Fan 1 tach | EXTI2, rising-edge pulse counter |
| PA1 | Fan 2 tach | EXTI1, rising-edge pulse counter |
| PC6 | WS2812 data | SPI1_MOSI, 3 MHz hardware SPI (no IRQ blocking) |
| PC3 | Button | Active-low, internal pull-up, polled in `loop()` |
| PA2 | Battery ADC | 10-bit, 10k/4k7 divider, 1S–3S auto-detect |
| PD1 | SWD (SWIO) | Programming/debug via H1 |
| PD7 | NRST | Reset |

### Implementation notes

- **Arduino-first coding style** — `pinMode`, `digitalRead`, `analogRead`, `millis`, `delayMicroseconds` for all high-level logic. Register-level code is isolated to three init helpers (`pwm_begin`, `ws2812_begin`, `tach_begin`) called once in `setup()`.
- **No interrupt blocking** — WS2812 is driven via hardware SPI at 3 MHz. The 48-byte transfer (4 LEDs × 3 colours × 4 SPI-bytes/colour) streams through `SPI1->DATAR` without disabling interrupts. Tach ISRs fire unimpeded.
- **Button debounce** — polled every ~1 ms in `loop()`, 30-sample debounce counter. Edge detection with falling/rising transitions, single-click/double-click/long-press resolved by gap timeout.
- **Tach** — `EXTI1_IRQHandler` (PA1) and `EXTI2_IRQHandler` (PD2) increment atomic pulse counters. Every 2 seconds the main loop snapshots and resets them, computing RPM = pulses × 15.
- **Startup sequence** — 2 s rainbow animation → battery cell-count detection (4-sample averaged ADC) → spin fans at 100% for 500 ms → count tach pulses for 1 s → enter RUNNING or ERROR.

### Display behaviour

| What you see | Meaning |
|-------------|---------|
| 4× rainbow rotating (2 s) | Power-on startup |
| All 4 LEDs fast-blink red (5 Hz) | Error: bad battery voltage OR no fans detected |
| 1–4 LEDs dim white | Battery gauge: 1=5–25%, 2=25–50%, 3=50–75%, 4=75–100% |
| Top LED colour: blue → cyan → green → yellow → red | Fan RPM low → high |
| Top LED blinking off/on | Battery critically low (<10%) |

## Pin Reference

See [**hardware/pins.md**](hardware/pins.md) for the complete pinout tables covering the MCU, connectors, voltage divider, LED chain, buck-boost, and power switching.

## Implementation Plan

See [**plan.md**](plan.md) for the full firmware architecture: state machine, module breakdown, SPI rationale, button timing thresholds, ADC voltage-divider math, and implementation order.

## Testing

The firmware is untested. When the board arrives, verify in this order:

### 1. PWM (oscilloscope)
Probe PD5 and PD6. Confirm 25 kHz, 0% duty at idle. Change duty via button and verify CCR matches `{0, 480, 960, 1440, 1919}` for `{0%, 25%, 50%, 75%, 100%}`.

### 2. WS2812 (logic analyser)
Probe PC6 (SPI1_MOSI). Verify 3 MHz clock, correct SPI byte stream (`0x88`/`0x8E`/`0xE8`/`0xEE` patterns), 48 bytes per frame, ≥50 µs reset gap. Visually confirm rainbow animation and solid colours.

### 3. Battery ADC (variable DC supply)
Apply known voltages to BAT+ and read `analogRead(PA2)`. Verify:
- 1S detection: ADC 200–340
- 2S detection: ADC 370–570
- 3S detection: ADC > 570
- Out-of-range → ERROR state with red fast-blink

### 4. Tachometer (signal generator or real fan)
Apply a 10–500 Hz square wave to PD2 and PA1. Confirm pulse counts match and RPM calculation is correct (RPM = pulses × 15 per 2 s window). Verify single-fan and dual-fan mode detection at startup.

### 5. Button (serial debug or LED feedback)
Test each gesture:
- Single click → duty cycles 0→25→50→75→0%
- Double click → duty jumps to 100%
- Long press (hold >1 s) → duty snaps to 0% immediately

### 6. End-to-end
Full system test: power on → rainbow → battery gauge appears → button controls fan speed → speed colour tracks RPM → low battery triggers blink → power off.

### Edge cases
- Power on with no battery → ERROR
- Power on with battery but no fans connected → ERROR
- Hot-plug fan during RUNNING → speed LED falls to blue (0 RPM)
- Power on at borderline ADC values (near 1S/2S boundary) → correct cell count

## Build & Flash

1. Install the [CH32V003 Arduino board package](https://github.com/openwch/arduino_core_ch32) or [ch32v003fun](https://github.com/cnlohr/ch32v003fun).
2. Open `fanControl/fanControl.ino` in the Arduino IDE (or PlatformIO with the CH32V003 platform).
3. Select board: **CH32V003F4P6**.
4. Connect WCH-LinkE to H1: `GND – 5V – SWD(PD1) – NRST`.
5. Compile and upload.

> **Register-name compatibility:** The firmware uses WCH SPL register names (`TIM1->ATRLR`, `SPI1->CTLR1`, `AFIO->PCFR1`, etc.). If your Arduino core uses different names (e.g. `TIM1->ARR`, `SPI1->CR1`), adjust the three init helpers in sections 4, 5, and 8 of the firmware. `NVIC_EnableIRQ` may need to be replaced with `PFIC_EnableIRQ` depending on the core version.

## License

[Specify your license here]

---

*Schematic and PCB designed with EasyEDA Pro. MCU: WCH CH32V003F4P6 (QingKe V2 RISC-V).*
*Firmware written June 2026 — awaiting hardware validation.*

🤖 Generated with [Claude Code](https://claude.com/claude-code)
