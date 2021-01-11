/*
 * clock.c
 *
 *  Created on: Dec 10, 2020
 *      Author: linhao
 */

#include "gpio.h"
#include "i2c.h"
#include "spi.h"
#include "adc.h"
#include "ds3231.h"
#include "max72xx.h"
#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "delay.h"

extern RTC_Data rtc;

extern osThreadId mainTaskHandle;

extern osTimerId getRTCTimerHandle;
extern osTimerId getSensorDataTimerHandle;
extern osTimerId togglePointTimerHandle;
extern osTimerId showDateTimerHandle;

extern void SystemClock_Config(void);
extern void MX_FREERTOS_Init(void);

static uint8_t need_flash_point = 1;
static uint8_t need_update_display = 1;

#define POINT_COL_NUM 11

static const uint8_t numbers_5x8[][8] = {
	{ 0x0e, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0e },
	{ 0x04, 0x06, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0e },
	{ 0x0e, 0x11, 0x10, 0x10, 0x08, 0x04, 0x02, 0x1f },
	{ 0x0e, 0x11, 0x10, 0x0c, 0x10, 0x10, 0x11, 0x0e },
	{ 0x10, 0x18, 0x14, 0x12, 0x11, 0x1f, 0x10, 0x10 },
	{ 0x1f, 0x01, 0x01, 0x0f, 0x10, 0x10, 0x11, 0x0e },
	{ 0x0e, 0x11, 0x01, 0x0f, 0x11, 0x11, 0x11, 0x0e },
	{ 0x1f, 0x10, 0x10, 0x08, 0x04, 0x02, 0x02, 0x02 },
	{ 0x0e, 0x11, 0x11, 0x0e, 0x11, 0x11, 0x11, 0x0e },
	{ 0x0e, 0x11, 0x11, 0x11, 0x1e, 0x10, 0x11, 0x0e },
	{ 0x00, 0x00, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00 },
	{ 0x00, 0x00, 0x00, 0x07, 0x04, 0x07, 0x01, 0x07 },
	{ 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01 },
	{ 0x00, 0x01, 0x01, 0x00, 0x00, 0x01, 0x01, 0x00 }
};

static const uint8_t numbers_3x5[10][8] = {
	{ 0x00, 0x00, 0x00, 0x07, 0x05, 0x05, 0x05, 0x07 },
	{ 0x00, 0x00, 0x00, 0x02, 0x02, 0x02, 0x02, 0x02 },
	{ 0x00, 0x00, 0x00, 0x07, 0x04, 0x07, 0x01, 0x07 },
	{ 0x00, 0x00, 0x00, 0x07, 0x04, 0x07, 0x04, 0x07 },
	{ 0x00, 0x00, 0x00, 0x05, 0x05, 0x07, 0x04, 0x04 },
	{ 0x00, 0x00, 0x00, 0x07, 0x01, 0x07, 0x04, 0x07 },
	{ 0x00, 0x00, 0x00, 0x07, 0x01, 0x07, 0x05, 0x07 },
	{ 0x00, 0x00, 0x00, 0x07, 0x04, 0x04, 0x04, 0x04 },
	{ 0x00, 0x00, 0x00, 0x07, 0x05, 0x07, 0x05, 0x07 },
	{ 0x00, 0x00, 0x00, 0x07, 0x05, 0x07, 0x04, 0x07 }
};

static const uint8_t signs[][8] = {
	{ 0x00, 0x80, 0x80, 0x00, 0x00, 0x80, 0x80, 0x00 },
	{ 0x3c, 0x42, 0xa5, 0x81, 0xa5, 0x99, 0x42, 0x3c },
	{ 0x3c, 0x42, 0xa5, 0x81, 0xbd, 0x81, 0x42, 0x3c },
	{ 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00 },
	{ 0x00, 0x06, 0x0c, 0x18, 0x30, 0x18, 0x0c, 0x06 }
};

static void SetDisplayFlag(void) {
	need_update_display = 1;
}

static bool TestDisplayFlag(void) {
	if (need_update_display) {
		need_update_display = 0;
		return true;
	}

	return false;
}

void led_display_time(uint8_t hour, uint8_t minute, uint8_t second) {
	const uint8_t hour_1st = hour / 10;
	const uint8_t hour_2nd = hour % 10;

	const uint8_t minute_1st = minute / 10;
	const uint8_t minute_2nd = minute % 10;

	const uint8_t second_1st = second / 10;
	const uint8_t second_2nd = second % 10;

	MAX72XX_ClearAll();

	for (uint8_t row = 0; row < 8; row++) {
		for (uint8_t dev = 0; dev < MAX_DEVICES; dev++) {
			uint8_t data = numbers_5x8[0][row];
			switch (dev) {
			case 0:
				data = (numbers_5x8[hour_1st][row] | numbers_5x8[hour_2nd][row] << 5);
				break;
			case 1:
				data = numbers_5x8[hour_2nd][row] >> 3 | ((numbers_5x8[minute_1st][row]) << 5);
				break;
			case 2:
				data = (numbers_5x8[minute_1st][row] >> 3) | (numbers_5x8[minute_2nd][row] << 2);
				break;
			case 3:
				data = (numbers_3x5[second_1st][row]) << 1 | ((numbers_3x5[second_2nd][row]) << 5);
				break;
			}

			MAX72XX_SetRowOne(dev, row, data);
		}
	}
}

void led_display_date(uint8_t month, uint8_t day_of_month, uint8_t dayofweek) {
	const uint8_t month_1st = month / 10;
	const uint8_t month_2nd = month % 10;

	const uint8_t day_1st = day_of_month / 10;
	const uint8_t day_2nd = day_of_month % 10;
	const uint8_t day_of_week = dayofweek;

	MAX72XX_ClearAll();

	for (uint8_t row = 0; row < 8; row++) {
		for (uint8_t dev = 0; dev < MAX_DEVICES; dev++) {
			uint8_t data = 0;
			switch (dev) {
			case 0:
				data = numbers_5x8[month_1st][row] << 2 | numbers_5x8[month_2nd][row] << 7;
				break;
			case 1:
				data = numbers_5x8[month_2nd][row] >> 1 | signs[3][row] << 5;
				break;
			case 2:
				data = numbers_5x8[day_1st][row] << 1 | numbers_5x8[day_2nd][row] << 6;
				break;
			case 3:
				data = numbers_5x8[day_2nd][row] >> 2 | (numbers_3x5[day_of_week == 1 ? 7 : day_of_week - 1][row]) << 5;
				break;

			}
			MAX72XX_SetRowOne(dev, row, data);
		}
	}
}

static bool Clock_UpdateRTC(void) {
	static uint8_t old_sec = 0;
	DS3231_GetTime(&rtc);

	if (rtc.Sec != old_sec) {
		old_sec = rtc.Sec;
		return true;
	}

	return false;
}

static void Clock_Init(void) {
	DS3231_Init();
	MAX72XX_Init();
}

static void Clock_FlashTimePoint() {
	MAX72XX_SetPoint(1, POINT_COL_NUM, need_flash_point);
	MAX72XX_SetPoint(2, POINT_COL_NUM, need_flash_point);
	MAX72XX_SetPoint(5, POINT_COL_NUM, need_flash_point);
	MAX72XX_SetPoint(6, POINT_COL_NUM, need_flash_point);
}

static void Clock_ShowTime() {
	led_display_time(rtc.Hour, rtc.Min, rtc.Sec);
	Clock_FlashTimePoint();
	SetDisplayFlag();
}

static void Clock_ShowDate(void) {
	led_display_date(rtc.Month, rtc.Day, rtc.DaysOfWeek);
	SetDisplayFlag();
}

static void Clock_SecondJumpUp(void) {
	Clock_FlashTimePoint();
	MAX72XX_TransformOne(3, TSU);
	SetDisplayFlag();
}

static void Clock_SecondJumpDown(void) {
	Clock_FlashTimePoint();
	MAX72XX_TransformOne(3, TSD);
	SetDisplayFlag();
}

static void Clock_UpdateDiplay() {
	if (TestDisplayFlag()) {
		MAX72XX_UpdateAll();
	}
}

enum {
	STATE_CLOCK_TIME = 0,
	STATE_CLOCK_DATE = 1,
	STATE_CLOCK_TEMP = 2,
};

enum {
	STATE_TIME_SHOW = 0,
	STATE_TIME_SEC_CHANGED = 1,
	STATE_TIME_SEC_JUMP_UP = 2,
	STATE_TIME_SEC_JUMP_DOWN = 3,
};

typedef struct {
	uint32_t state;
	int32_t repeat;
	uint32_t duration;
	void (*callback)(void);
} State;

static State clock_states[3] = {
	{
		.state = STATE_CLOCK_TIME,
		.repeat = -1,
		.duration = 50,
		.callback = Clock_ShowTime
	},
	{
		.state = STATE_CLOCK_DATE,
		.repeat = 1,
		.duration = 2000,
		.callback = Clock_ShowDate
	},
	{
		.state = STATE_CLOCK_TEMP,
		.repeat = 0,
		.duration = 0,
		.callback = 0
	}
};

static State time_states[4] = {
	{
		.state = STATE_TIME_SHOW,
		.repeat = -1,
		.duration = 10,
		.callback = Clock_ShowTime
	},
	{
		.state = STATE_TIME_SEC_CHANGED,
		.repeat = 0,
		.duration = 0,
		.callback = 0
	},
	{
		.state = STATE_TIME_SEC_JUMP_UP,
		.repeat = 2,
		.duration = 100,
		.callback = Clock_SecondJumpUp
	},
	{
		.state = STATE_TIME_SEC_JUMP_DOWN,
		.repeat = 2,
		.duration = 100,
		.callback = Clock_SecondJumpDown
	}
};

static State clock_s = { 0 };
static State time_s = { 0 };

static inline void ChangeClockState(int clock_state_n) {
	clock_s = clock_states[clock_state_n];
}

static inline void ChangeTimeState(int time_state_n) {
	time_s = time_states[time_state_n];
}

static inline bool TickState(State *s) {
	if (-1 != s->repeat && 0 == s->repeat) {
		return true;
	}

	if (s->callback) {
		s->callback();
	}

	if (-1 != s->repeat) {
		s->repeat--;
	}

	return false;

}

void CallbackGetRTC(void const *argument) {
	if (Clock_UpdateRTC()) {
		ChangeTimeState(STATE_TIME_SEC_CHANGED);
	}
}

void CallbackGetSensorData(void const *argument) {
}

void CallbackShowDate(void const *argument) {
	ChangeClockState(STATE_CLOCK_DATE);
}

void CallbackTogglePoint(void const *argument) {
	need_flash_point = !need_flash_point;
}

static void Clock_ShowWelcome(void) {
	MAX72XX_ControlAll(UPDATE, 1);
	MAX72XX_ClearAll();
	for (uint8_t row = 0; row < 8; row++) {
		MAX72XX_SetRowAll(row, 0xff);
	}

	vTaskDelay(100);

	MAX72XX_ClearAll();

	for (int i = 0; i < 32; i++) {
		MAX72XX_SetPixelColumn(i, 0xff);
		vTaskDelay(20);
		MAX72XX_SetPixelColumn(i, 0x00);
		vTaskDelay(20);
	}

	vTaskDelay(100);

	MAX72XX_ClearAll();
	for (int i = 0; i < 8; i++) {
		MAX72XX_SetRowAll(i, 0xff);
		vTaskDelay(20);
		MAX72XX_SetRowAll(i, 0x00);
		vTaskDelay(20);
	}

	vTaskDelay(100);

	MAX72XX_ClearAll();
	Clock_ShowTime();
	MAX72XX_ControlAll(UPDATE, 0);
}

void StartMainTask(void const *argument) {
	clock_s = clock_states[0];
	time_s = time_states[0];

	Clock_ShowWelcome();

	for (;;) {
		switch (clock_s.state) {
		case STATE_CLOCK_TIME: {
			switch (time_s.state) {
			case STATE_TIME_SHOW:
				TickState(&time_s);
				break;
			case STATE_TIME_SEC_CHANGED:
				if (TickState(&time_s)) {
					ChangeTimeState(STATE_TIME_SEC_JUMP_UP);
				}
				break;
			case STATE_TIME_SEC_JUMP_UP:
				if (TickState(&time_s)) {
					ChangeTimeState(STATE_TIME_SEC_JUMP_DOWN);
				}
				break;
			case STATE_TIME_SEC_JUMP_DOWN:
				if (TickState(&time_s)) {
					ChangeTimeState(STATE_TIME_SHOW);
				}
				break;
			}

			Clock_UpdateDiplay();

			vTaskDelay(time_s.duration);

			break;
		}
		case STATE_CLOCK_DATE:
			if (TickState(&clock_s)) {
				ChangeClockState(STATE_CLOCK_TIME);
				ChangeTimeState(STATE_TIME_SHOW);
				Clock_ShowTime();
			}

			Clock_UpdateDiplay();

			vTaskDelay(clock_s.duration);

			break;
		}

		DWT_DelayUs(10);
		DWT_DelayMs(1);
	}
}

int main(void) {
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_I2C1_Init();
	MX_SPI1_Init();
	MX_ADC1_Init();
	DWT_Init();
	Clock_Init();

	MX_FREERTOS_Init();

	osTimerStart(getRTCTimerHandle, 100);
	osTimerStart(getSensorDataTimerHandle, 60 * 1000);
	osTimerStart(togglePointTimerHandle, 500);
	osTimerStart(showDateTimerHandle, 60 * 1000);

	osKernelStart();

	while (1) {
	}
}
