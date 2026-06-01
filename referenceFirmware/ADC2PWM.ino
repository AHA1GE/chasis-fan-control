/*
 * Unified ADC2PWM firmware for AVR, CH32 and ESP32.
 *
 * Behavior:
 * 1) Output PWM_BOOT_US during boot window with slow LED blink.
 * 2) Require ADC minimum once to unlock normal control.
 * 3) After unlock, map ADC to PWM range with asymmetric ramp limiting.
 */

#include "OLED.h"
#include "CH32V003_SERVO.h"

#define ENABLE_SCREEN 1

/*================================
 * Pins and Hardware
================================*/

#define VOLTAGE_REFERENCE_MILLIVOLTS 3400ULL  // should be 3v3, but TPS63060 here actually outputs 3.4V
#define PWM_PIN SERVO_PIN_PC4_TIM1_CH4
#define ADC_PIN PA2   // ADC0, potentiometer
#define BATT_PIN PD6  // ADC6, 4.7k/10k divider, 5v vRef -> `vBatt = adc*14.7/4.7*5/ADC_MAX_VALUE`
#define ADC_MAX_VALUE 1023
#define SDA_PIN PC1
#define SCL_PIN PC2

/*================================
 * Constants
================================*/

// ---------------- PWM Range ----------------
#define PWM_BOOT_US 890
#define PWM_MIN_US 1000
#define PWM_MAX_US 2000

// ---------------- PWM Ramp ----------------
#define RAMP_STEP_US_UP 2
#define RAMP_STEP_US_DOWN 4

// ---------------- Timing ----------------
#define BOOT_TIME_MS 2100
#define LOOP_DELAY_MS 10

// ---------------- Safety Latch ----------------
#define ADC_ARM_THRESHOLD 50

// ---------------- Battery Thresholds ----------------
#define BATTERY_THRESHOLD_1S_MIN 3000  // 3.0V (1s battery, critical low voltage)
#define BATTERY_THRESHOLD_1S_LOW 3300  // 3.3V (1s battery, warning low voltage)
#define BATTERY_THRESHOLD_1S_MAX 4500  // 4.5V (1s battery, 4.35V full charge (LiHv) + 0.15V margin)
#define BATTERY_THRESHOLD_2S_MIN 6000  // 6.0V (2s battery, critical low voltage)
#define BATTERY_THRESHOLD_2S_LOW 6500  // 6.5V (2s battery, warning low voltage)
#define BATTERY_THRESHOLD_2S_MAX 8900  // 8.9V (2s battery, 8.7V full charge (LiHv) + 0.2V margin)

// ---------------- OLED ----------------

/*================================
 * Global Variables and State Definitions
================================*/

static uint16_t pwmOutput = PWM_BOOT_US;
static uint16_t pwmTarget = PWM_BOOT_US;
static uint32_t vBattMv = 0;
static uint16_t throttleAdc = 0;
static char errorMessageBuffer[10] = { 0 };  // buffer for error message, max 9 chars + null terminator

enum BatteryState : uint8_t {
	BATTERY_STATE_NORMAL = 0,
	BATTERY_STATE_LOW,
	BATTERY_STATE_ERROR,
};
static BatteryState batteryState = BATTERY_STATE_NORMAL;
static BatteryState lastBatteryState = BATTERY_STATE_NORMAL;

static Servo esc;

static void copyErrorMessage(const char *msg) {
	strncpy(errorMessageBuffer, msg, sizeof(errorMessageBuffer) - 1);
	errorMessageBuffer[sizeof(errorMessageBuffer) - 1] = '\0';
}

/*================================
 * Helper Functions
 *
================================*/

static bool lowBatteryCheck() {
	// 1s-2s battery, divider: gnd_4.7k_battAdc_10k_vBatt, ref = VOLTAGE_REFERENCE_MILLIVOLTS
	const uint16_t rawBattAdc = analogRead(BATT_PIN);
	vBattMv = ((uint64_t)rawBattAdc * 147ULL * VOLTAGE_REFERENCE_MILLIVOLTS) / (47ULL * ADC_MAX_VALUE);

	batteryState = BATTERY_STATE_NORMAL;

	if (vBattMv <= BATTERY_THRESHOLD_1S_MIN) {
		// too low, error
		batteryState = BATTERY_STATE_ERROR;
		if (lastBatteryState != batteryState) {
			copyErrorMessage("BAT LOW");
		}
	} else if (vBattMv <= BATTERY_THRESHOLD_1S_LOW) {
		// 1s low battery, warning
		batteryState = BATTERY_STATE_LOW;
	} else if (vBattMv <= BATTERY_THRESHOLD_1S_MAX) {
		// 1s normal, do nothing
	} else if (vBattMv <= BATTERY_THRESHOLD_2S_MIN) {
		// not 1s nor 2s, error
		batteryState = BATTERY_STATE_ERROR;
		if (lastBatteryState != batteryState) {
			copyErrorMessage("BAT ERR");
		}
	} else if (vBattMv <= BATTERY_THRESHOLD_2S_LOW) {
		// 2s low battery, warning
		batteryState = BATTERY_STATE_LOW;
	} else if (vBattMv <= BATTERY_THRESHOLD_2S_MAX) {
		// 2s normal, do nothing
	} else {
		// above 2s max voltage, error
		batteryState = BATTERY_STATE_ERROR;
		if (lastBatteryState != batteryState) {
			copyErrorMessage("BAT HIGH");
		}
	}

	if (batteryState == BATTERY_STATE_NORMAL) {
		errorMessageBuffer[0] = '\0';
	}

	lastBatteryState = batteryState;
	return (batteryState == BATTERY_STATE_LOW);
}

static uint16_t adcToPulseUs(uint16_t adcValue) {
	return (uint16_t)map((long)adcValue, 0L, (long)ADC_MAX_VALUE, PWM_MIN_US, PWM_MAX_US);
}

#if ENABLE_SCREEN
/*================================
 * OLED Functions
 *
================================*/

static void screenInit(void) {
	OLED_Init();
	OLED_SetBrightness(50);
	OLED_ShowMixStringArea(0, 0, OLED_WIDTH, OLED_HEIGHT, 32, 20, "Init", OLED_FONT_8);  // OLED显示字符数组（字符串）
	OLED_Update();
}
static void screenClear(void) {
	OLED_Clear();
	OLED_Update();
}
static void screenShowDisarmed(void) {
	OLED_Clear();
	OLED_ShowMixStringArea(0, 0, OLED_WIDTH, OLED_HEIGHT, 20, 20, "DISARMED", OLED_FONT_8);  // OLED显示字符数组（字符串）
	OLED_Update();
}
static void screenPrintPrepare(void) {
	OLED_Clear();
	OLED_ShowMixStringArea(0, 0, OLED_WIDTH, OLED_HEIGHT, 0, 0, "Bat:", OLED_FONT_8);   // OLED显示字符数组（字符串）
	OLED_ShowMixStringArea(0, 0, OLED_WIDTH, OLED_HEIGHT, 0, 16, "Thr:", OLED_FONT_8);  // OLED显示字符数组（字符串）
	OLED_ShowMixStringArea(0, 0, OLED_WIDTH, OLED_HEIGHT, 0, 32, "PWM:", OLED_FONT_8);  // OLED显示字符数组（字符串）
	OLED_Update();
}
static void screenPrint(void) {
	char buffer[16];

	// format vBattMv to `x.xV` with 1 decimal place
	snprintf(buffer, sizeof(buffer), "%lu.%luV", (unsigned long)(vBattMv / 1000), (unsigned long)((vBattMv % 1000) / 100));
	// `Bat:` length 4, +1 for spacing
	OLED_ShowMixStringArea(0, 0, OLED_WIDTH, OLED_HEIGHT, 32, 0, buffer, OLED_FONT_8);  // OLED显示字符数组（字符串）

	snprintf(buffer, sizeof(buffer), "%u", throttleAdc);
	// `Thr:` length 4, +1 for spacing
	// Throttle need to be claered first since change from 0-1023
	OLED_ClearArea(32, 16, 32, 8);  //
	OLED_ShowMixStringArea(0, 0, OLED_WIDTH, OLED_HEIGHT, 32, 16, buffer, OLED_FONT_8);  // OLED显示字符数组（字符串）

	snprintf(buffer, sizeof(buffer), "%u", pwmOutput);
	// `PWM:` length 4, +1 for spacing
	OLED_ShowMixStringArea(0, 0, OLED_WIDTH, OLED_HEIGHT, 32, 32, buffer, OLED_FONT_8);  // OLED显示字符数组（字符串）

	OLED_Update();
}
static void screenLowBatteryWarning(void) {
	// blink low battery warning at the right corner
	if (millis() % 1000 < 500) {
		OLED_ShowMixStringArea(0, 0, OLED_WIDTH, OLED_HEIGHT, 64, 0, "!", OLED_FONT_8);  // OLED显示字符数组（字符串）
	} else {
		OLED_ShowMixStringArea(0, 0, OLED_WIDTH, OLED_HEIGHT, 64, 0, " ", OLED_FONT_8);  // OLED显示字符数组（字符串）
	}
}
static void screenShowError(const char *message, uint8_t messageLength) {
	// chack length, no more than 9 characters
	if (messageLength > 9) {
		// do nothing
	} else {
		OLED_Clear();
		OLED_ShowMixStringArea(0, 0, OLED_WIDTH, OLED_HEIGHT, 16, 16, "ERROR", OLED_FONT_8);  // OLED显示字符数组（字符串）
		OLED_ShowMixStringArea(0, 0, OLED_WIDTH, OLED_HEIGHT, 16, 32, message, OLED_FONT_8);  // OLED显示字符数组（字符串）
		OLED_Update();
	}
}
#endif

/*================================
 * Arduino Setup and Loop
 *
================================*/

void setup() {
	pinMode(ADC_PIN, INPUT);
	pinMode(BATT_PIN, INPUT);

	// init PWM before slow OLED so the ESC sees a valid boot pulse immediately.
	pwmOutput = PWM_BOOT_US;
	pwmTarget = PWM_BOOT_US;
	esc.attach(PWM_PIN);
	esc.writeMicroseconds(PWM_BOOT_US);
#if ENABLE_SCREEN
	// init screen
	screenInit();
#endif
	// ESC need PWM_BOOT_US for 2s
	delay(2000);

	// lock to low throttle
	pwmOutput = PWM_MIN_US;
	pwmTarget = PWM_MIN_US;
	esc.writeMicroseconds(pwmOutput);
#if ENABLE_SCREEN
	screenClear();
	screenShowDisarmed();
#endif
	while (analogRead(ADC_PIN) > ADC_ARM_THRESHOLD) {
		delay(LOOP_DELAY_MS);
	}

	// unlock, start normal operation
#if ENABLE_SCREEN
	screenPrintPrepare();
#endif
}
void loop() {
	bool isLowBattery = lowBatteryCheck();
	throttleAdc = analogRead(ADC_PIN);
	if (isLowBattery) {
		throttleAdc = throttleAdc / 2;
	}
	pwmTarget = adcToPulseUs(throttleAdc);
	if (pwmOutput < pwmTarget) {
		pwmOutput = (uint16_t)(pwmOutput + RAMP_STEP_US_UP);
		if (pwmOutput > pwmTarget) {
			pwmOutput = pwmTarget;
		}
	} else if (pwmOutput > pwmTarget) {
		pwmOutput = (uint16_t)(pwmOutput - RAMP_STEP_US_DOWN);
		if (pwmOutput < pwmTarget) {
			pwmOutput = pwmTarget;
		}
	}
	esc.writeMicroseconds(pwmOutput);

#if ENABLE_SCREEN
	if (isLowBattery) {
		screenLowBatteryWarning();
	} else {
		OLED_ShowMixStringArea(0, 0, OLED_WIDTH, OLED_HEIGHT, 64, 0, " ", OLED_FONT_8);
	}
	screenPrint();
#endif

	delay(LOOP_DELAY_MS);
}
