# Pin Assignments — Chasis Fan Control Board

## MCU: CH32V003F4P6 (U1) — TSSOP-20

| Pin | Net | GPIO | Function | Description |
|-----|-----|------|----------|-------------|
| 2 | `PWM_2` | PD6 | PWM output | Fan 2 PWM speed control |
| 3 | `PWM_1` | PD5 | PWM output | Fan 1 PWM speed control |
| 4 | `NRST` | NRST | Reset | Reset pin, also broken out to H1.4 |
| 5 | `SPEED_2` | PD3 | Tach input | Fan 2 speed feedback (open-drain tach) |
| 6 | `VDETECT` | PD2 | ADC input | Battery voltage sense (divider: 10k / 4.7k) |
| 7 | `GND` | PD1 | GND | Tied to ground |
| 9 | `+5V` | VDD | Power | 5 V supply from buck-boost converter |
| 13 | `Btn` | PC1 | GPIO input | User button (active low, internal pull-up) |
| 14 | `RGB` | PC2 | GPIO output | WS2812 data line (bit-banged) |
| 18 | `SWD` | PC6 | SWIO | SWD debug data |
| 19 | `SPEED_1` | PC7 | Tach input | Fan 1 speed feedback (open-drain tach) |

> **Note:** Pin numbers come from the EasyEDA netlist and align with the TSSOP-20 package pinout. GPIO port/pin names are derived from the CH32V003F4P6 datasheet; verify against the actual firmware `fanControl.ino` for the definitive `pinMode()` / `digitalPinToPort()` mapping.

---

## Dual-Fan Connector — CN1 (SH1.25-8ALT, 8-pin SMD, 1.25 mm pitch)

Primary socket for connecting **two** chassis fans.

| Pin | Net | Function |
|-----|-----|----------|
| 1 | `GND` | Ground |
| 2 | `GND` | Ground |
| 3 | `PWM_1` | Fan 1 PWM |
| 4 | `PWM_2` | Fan 2 PWM |
| 5 | `SPEED_1` | Fan 1 tachometer |
| 6 | `SPEED_2` | Fan 2 tachometer |
| 7 | `BAT+` | Battery positive |
| 8 | `BAT+` | Battery positive |
| (9,10) | `GND` | Shield/mechanical pads |

> Standard 4-pin PC fan pinout per fan: GND, BAT+, TACH, PWM. Fans are powered directly from the battery rail (not the regulated 5 V) so they run at their native voltage.

### Firmware: single vs. dual fan

The MCU always outputs the same PWM duty on both `PWM_1` and `PWM_2`. For speed read-back:
- **Both fans connected (CN1):** both tach signals (`SPEED_1`, `SPEED_2`) are valid — firmware uses the combined/normal value to drive the speed indicator LED.
- **Only Fan 1 connected (CN4):** only `SPEED_1` is valid — firmware falls back to the single tach reading directly.

---

## Optional Single-Fan Connector — CN4 (M2510V-04P-N3, 4-pin TH, 2.54 mm pitch)

Alternate connector when the user chooses to run **only one** fan. Functionally mirrors Fan 1.

| Pin | Net | Function |
|-----|-----|----------|
| 1 | `PWM_1` | Fan 1 PWM |
| 2 | `SPEED_1` | Fan 1 tachometer |
| 3 | `BAT+` | Battery voltage rail |
| 4 | `GND` | Ground |

> CN4 provides a standard 2.54 mm pitch option for a single fan. The firmware uses `SPEED_1` directly when only one fan is present, and the combined reading from both tach channels when two fans are connected via CN1.

---

## AIO Header — H1 (PZ254V-11-05P, 5-pin TH, 2.54 mm pitch)

All-in-one pinout combining SWD programming/debug with a reserved WS2812 extension passthrough.

| Pin | Net | Function |
|-----|-----|----------|
| 1 | `GND` | Ground |
| 2 | `+5V` | 5 V supply (from buck-boost, powers the MCU) |
| 3 | `SWD` | SWD data (SWIO) |
| 4 | `NRST` | Reset |
| 5 | `RGB_EXT` | WS2812 passthrough from LED4 DOUT (reserved, no firmware function implemented) |

> Standard WCH-LinkE / J-Link SWD header for programming and debugging CH32V003. Pin 5 carries the end of the on-board WS2812 daisy chain — available for a future external LED strip but currently unused in firmware.

---

## Battery Voltage Divider

```
BAT+ ── R4 (10 kΩ) ──┬── VDETECT (U1.6 / PD2 ADC)
                     R5 (4.7 kΩ)
                      │
                     GND
```

| Battery | Voltage Range | VDETECT Range | Notes |
|---------|--------------|---------------|-------|
| 1S LiPo | 3.0 – 4.2 V | 0.96 – 1.34 V | R5/R4+R5 = 0.320 |
| 2S LiPo | 6.0 – 8.4 V | 1.92 – 2.69 V | MCU auto-detects cell count |
| 3S LiPo | 9.0 – 12.6 V | 2.88 – 4.03 V | Max < 5 V ADC Vref |

> **Auto-detection logic:** Read ADC on PD2 at boot. If VDETECT > 2.7 V → 3S; 1.8–2.7 V → 2S; < 1.6 V → 1S.

---

## WS2812C RGB LED Daisy Chain

```
MCU (PC2) ── RGB ── LED1.DIN ── LED1.DOUT ── LED2.DIN ── LED2.DOUT ── LED3.DIN ── LED3.DOUT ── LED4.DIN ── LED4.DOUT ── RGB_EXT (H1.5)
```

| LED | Designator | Net (DIN) | Net (DOUT) | Indicator Function |
|-----|-----------|-----------|------------|-------------------|
| 1 | LED1 | `RGB` (U1.14) | `$1N41` (→ LED2) | Battery 5%–25% |
| 2 | LED2 | `$1N41` | `$1N42` (→ LED3) | Battery 25%–50% |
| 3 | LED3 | `$1N42` | `$1N43` (→ LED4) | Battery 50%–75% |
| 4 | LED4 | `$1N43` | `RGB_EXT` (H1.5) | Battery 75%–100% |

---

## Buck-Boost Converter — TPS63070RNMR (U5)

| Pin | Net | Function |
|-----|-----|----------|
| 1, 14 | `BUCK_EN` | Enable (pulled to BAT+ via R14 10 kΩ — always on when switch is closed) |
| 2 | `BUCK_PG` | Power Good (pulled to +5V via R9 100 kΩ) |
| 3 | `BUCK_VAUX` | Auxiliary output, decoupled with C19 100 nF to GND |
| 5 | `BUCK_FB` | Feedback node — R17 (680 kΩ) to +5V, R16 (130 kΩ) to GND → Vout = 5.0 V |
| 7, 8 | `+5V` | Regulated 5 V output |
| 9, 11 | `BUCK_Lx` | Switching nodes — inductor U11 (1.5 µH) |
| 12, 13 | `BAT+` | Battery input (2–16 V) |
| 4, 6, 10, 15 | `GND` | Power ground |

---

## Power Switching (Physical On/Off)

```
SW2 (SPDT slide switch):
  Common (1) ── BAT+
  NO (2)      ── SW_ON ── R19 (1 kΩ) ── MOS_G

Q1 (N-channel MOSFET, low-side):
  Gate  ── MOS_G (pulled down by R18 10 kΩ when switch OFF)
  Drain ── GND
  Source ── BAT-
```

- **Switch ON:** BAT+ flows through R19 to Q1 gate → Q1 turns on → BAT- connected to GND → circuit powered.
- **Switch OFF:** Gate pulled to GND via R18 → Q1 off → BAT- isolated → entire board off.

---

## Button — SW1 (KH-6X6X5H-STM)

| Pin | Net | Notes |
|-----|-----|-------|
| 1, 2 | `Btn` | Connected to U1.13 (PC1), use internal pull-up |
| 3, 4 | `GND` | Ground |

Pressed = low. Software debounce required.

| Gesture | Action |
|---------|--------|
| Single click | Cycle PWM duty: 0% → 25% → 50% → 75% |
| Double click | Jump to 100% duty |
| Long press | Quickly return to 0% duty |
