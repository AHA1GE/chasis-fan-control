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

The firmware lives in [`fanControl/fanControl.ino`](fanControl/fanControl.ino) and is written for the Arduino / PlatformIO CH32V003 environment. Key implementation details:

- **PWM:** Hardware timers generate 25 kHz PWM on PD5 (Fan 1) and PD6 (Fan 2) — the Intel-standard PC fan frequency.
- **Tach:** External interrupts on PC7 (Fan 1) and PD3 (Fan 2) count pulses; 2 pulses per revolution for standard fans. When both tach signals are alive the firmware uses a combined (averaged) reading; when only Fan 1 is connected (CN4) it falls back to `SPEED_1` directly.
- **WS2812:** Bit-banged on PC2 using cycle-accurate assembly or the FastLED / NeoPixel library.
- **Button:** Polled or interrupt-driven on PC1 with software debounce and gesture detection (single-click, double-click, long-press).
- **ADC:** Periodic sampling on PD2 for battery voltage; cell-count auto-detection runs once at boot.

## Pin Reference

See [**hardware/pins.md**](hardware/pins.md) for the complete pinout tables covering the MCU, connectors, voltage divider, LED chain, buck-boost, and power switching.

## Build & Flash

1. Install the WCH CH32V003 toolchain (MounRiver Studio or the [ch32v003fun](https://github.com/cnlohr/ch32v003fun) open-source SDK).
2. Open `fanControl/fanControl.ino` in the Arduino IDE with the CH32V003 board package, or use PlatformIO.
3. Connect a WCH-LinkE (or J-Link) to H1: GND, 5V, SWD, NRST.
4. Compile and upload.

## License

[Specify your license here]

---

*Schematic and PCB designed with EasyEDA Pro. MCU: WCH CH32V003F4P6 (QingKe V2 RISC-V).*

🤖 Generated with [Claude Code](https://claude.com/claude-code)
