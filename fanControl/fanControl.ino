/*
 * Chasis Fan Control — Simple dual PWM fan controller with WS2812 speed gauge.
 *
 * MCU:  CH32V003F4P6 @ 48 MHz (TSSOP-20)
 * Fans: 2× PWM (PD5, PD6)
 * LEDs: 4× WS2812C via SPI1_MOSI (PC6) at 3 MHz
 * UI:   Single button on PC3 (INPUT_PULLUP):click=cycle;long-press=0%.
 */

// ---- Fan ----
#define PIN_FAN1_PWM PD5
#define PIN_FAN2_PWM PD6
// ---- Button ----
#define PIN_BTN PC3
#define BTN_DEBOUNCE_MS 30  // single click - switch level
#define BTN_LONG_MS 1000    // long press - level_low

// ---- Speed levels (5 levels, 0–100 %) ----
typedef enum {
  LEVEL_LOW = 0,
  LEVEL_1,
  LEVEL_2,
  LEVEL_3,
  LEVEL_4,
  LEVEL_COUNT
} level_t;
static level_t g_level = LEVEL_LOW;  // 0–4
static const uint8_t levelPctTable[LEVEL_COUNT] = { 0, 35, 50, 75, 100 };

// ---- Level colours (GRB order: [0]=G, [1]=R, [2]=B) ----
static const uint8_t levelColor[LEVEL_COUNT][3] = {
  { 0, 0, 2 },   //  LOW, blue
  { 1, 0, 1 },  // 35%, cyan
  { 2, 0, 0 },   // 50%, green
  { 1, 1, 0 },  // 75% — yellow
  { 0, 2, 0 },   // 100% — red
};

// ---- WS2812 via SPI ----
#define NUM_LEDS 4
#define SPI_BYTES_PER_LED 12  // 24 bits × 4 SPI-bits/bit / 8
// ---- WS2812 bit-expansion LUT (2 input bits → 1 SPI byte) ----
// 1→1110, 0→1000.  Upper nibble = bit1, lower nibble = bit0.
static const uint8_t wsLUT[4] = {
  0x88,  // 00
  0x8E,  // 01
  0xE8,  // 10
  0xEE,  // 11
};
// RGB LED buffers
static uint8_t g_leds[NUM_LEDS][3];  // GRB: [i][0]=G, [1]=R, [2]=B
static uint8_t g_spiBuf[NUM_LEDS * SPI_BYTES_PER_LED];
static void ws2812_begin(void) {
  // Enable SPI1 + GPIOC clocks
  RCC->APB2PCENR |= (1 << 12)    // SPI1
                    | (1 << 4);  // GPIOC

  // PC6 → alternate push-pull 50 MHz (SPI1_MOSI)
  uint32_t cfglr = GPIOC->CFGLR;
  cfglr &= ~(0xFU << 24);
  cfglr |= (0xBU << 24);  // CNF=10(af-pp), MODE=11(50 MHz)
  GPIOC->CFGLR = cfglr;

  // SPI1: master, mode 0, MSB first, BR=011=fPCLK/16=3 MHz
  SPI1->CTLR1 = (1 << 2)     // MSTR
                | (3 << 3);  // BR = 011 = /16
  SPI1->CTLR2 = 0;
  SPI1->CTLR1 |= (1 << 6);  // SPE
}


static void rgb_update(void) {
  for (uint8_t i = 0; i < NUM_LEDS; i++) {
    g_leds[i][0] = levelColor[g_level][0];  // G
    g_leds[i][1] = levelColor[g_level][1];  // R
    g_leds[i][2] = levelColor[g_level][2];  // B
  }
  // Expand g_leds[NUM_LEDS][3] (GRB) → g_spiBuf
  uint8_t *p = g_spiBuf;
  for (uint8_t i = 0; i < NUM_LEDS; i++) {
    for (uint8_t c = 0; c < 3; c++) {  // G, R, B
      uint8_t v = g_leds[i][c];
      *p++ = wsLUT[(v >> 6) & 3];
      *p++ = wsLUT[(v >> 4) & 3];
      *p++ = wsLUT[(v >> 2) & 3];
      *p++ = wsLUT[v & 3];
    }
  }

  // Transmit via hardware SPI
  for (uint16_t i = 0; i < sizeof(g_spiBuf); i++) {
    while (!(SPI1->STATR & (1 << 1)))
      ;  // wait TXE
    SPI1->DATAR = g_spiBuf[i];
  }
  while (SPI1->STATR & (1 << 7))
    ;  // wait BSY

  delayMicroseconds(60);  // RESET > 50 µs
}

static void fans_update(void) {
  uint8_t duty = map(levelPctTable[g_level], 0, 100, 0, 255);
  analogWrite(PIN_FAN1_PWM, duty);
  analogWrite(PIN_FAN2_PWM, duty);
  analogWrite(PD2, duty);
  analogWrite(PA1, duty);
}

static void level_apply(level_t lvl) {
  g_level = lvl;
  fans_update();
  rgb_update();
}

void setup(void) {
  pinMode(PIN_BTN, INPUT_PULLUP);

  pinMode(PIN_FAN1_PWM, OUTPUT);
  analogWrite(PIN_FAN1_PWM, 0);

  pinMode(PIN_FAN2_PWM, OUTPUT);
  analogWrite(PIN_FAN2_PWM, 0);

  ws2812_begin();
  level_apply(LEVEL_LOW);
}

void loop(void) {
  static uint32_t lastDebounce = 0;
  static uint8_t lastReading = HIGH;
  static uint8_t btnState = HIGH;
  static uint32_t pressStart = 0;
  static bool longFired = false;

  uint32_t now = millis();
  uint8_t reading = digitalRead(PIN_BTN);

  // ---- debounce ----
  if (reading != lastReading)
    lastDebounce = now;
  lastReading = reading;

  if ((now - lastDebounce) > BTN_DEBOUNCE_MS) {
    // ---- state change ----
    if (reading != btnState) {
      btnState = reading;

      if (btnState == LOW)  // press
      {
        pressStart = now;
        longFired = false;
      } else if (!longFired)  // release → short click
      {
        level_apply((level_t)((g_level + 1) % LEVEL_COUNT));
      }
    }

    // ---- long-press while held ----
    if (btnState == LOW && !longFired && (now - pressStart) >= BTN_LONG_MS) {
      longFired = true;
      level_apply(LEVEL_LOW);
    }
  }
}
