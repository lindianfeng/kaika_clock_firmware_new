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
#include "dht11.h"
#include "cmsis_os.h"
#include "key.h"
#include "delay.h"
#include "utils.h"
#include "sht30.h"
#include "usbd_cdc_if.h"

extern ADC_HandleTypeDef hadc1;
extern RTC_Data rtc;

extern osThreadId mainTaskHandle;
extern osThreadId keyTaskHandle;

extern osTimerId getRTCTimerHandle;
extern osTimerId getSensorDataTimerHandle;
extern osTimerId togglePointTimerHandle;
extern osTimerId showDateTimerHandle;

extern uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
extern vcp_rx_t rx_data_ctr;

extern void SystemClock_Config(void);
extern void MX_FREERTOS_Init(void);
extern void MX_USB_DEVICE_Init(void);

static uint8_t temp_int = 0;
static uint8_t temp_deci = 0;
static uint8_t humi_int = 0;

uint32_t volatile clock_flag = 0;

#define POINT_COL_NUM 11

static const uint8_t numbers_5x8[][8] =
    {
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
    { 0x00, 0x06, 0x0c, 0x18, 0x30, 0x18, 0x0c, 0x06 },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01 }
};

enum
{
  CLOCK_FLAG_REFRESH_DISPLAY = 1,
  CLOCK_FLAG_TIME_SECOND_CHANGED = 2,
  CLOCK_FLAG_SHOW_DATE = 3,
  CLOCK_FLAG_SHOW_TEMP = 4,
  CLOCK_FLAG_FLASH_POINT = 5,
  CLOCK_FLAG_KEY1_PRESSED = 6,
  CLOCK_FLAG_KEY2_PRESSED = 7,
  CLOCK_FLAG_SET_RTC = 8,
};

static inline void SetClockFlag(uint32_t bit)
{
  bitSet(clock_flag, bit);
}

static inline void ClearClockFlag(uint32_t bit)
{
  bitClear(clock_flag, bit);
}

static inline void FlipClockFlag(uint32_t bit)
{
  bitFlip(clock_flag, bit);
}

static inline bool TestClockFlag(uint32_t bit)
{
  return bitRead(clock_flag, bit);
}

static bool TestAndClearFlag(uint32_t bit)
{
  if (bitRead(clock_flag, bit))
  {
    bitClear(clock_flag, bit);
    return true;
  }

  return false;
}

void led_display_time(uint8_t hour, uint8_t minute, uint8_t second)
{
  const uint8_t hour_1st = hour / 10;
  const uint8_t hour_2nd = hour % 10;

  const uint8_t minute_1st = minute / 10;
  const uint8_t minute_2nd = minute % 10;

  const uint8_t second_1st = second / 10;
  const uint8_t second_2nd = second % 10;

  MAX72XX_ClearAll();

  for (uint8_t row = 0; row < 8; row++)
  {
    for (uint8_t dev = 0; dev < MAX_DEVICES; dev++)
    {
      uint8_t data = numbers_5x8[0][row];
      switch (dev)
      {
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

void led_display_date(uint8_t month, uint8_t day_of_month, uint8_t dayofweek)
{
  const uint8_t month_1st = month / 10;
  const uint8_t month_2nd = month % 10;

  const uint8_t day_1st = day_of_month / 10;
  const uint8_t day_2nd = day_of_month % 10;
  const uint8_t day_of_week = dayofweek;

  MAX72XX_ClearAll();

  for (uint8_t row = 0; row < 8; row++)
  {
    for (uint8_t dev = 0; dev < MAX_DEVICES; dev++)
    {
      uint8_t data = 0;
      switch (dev)
      {
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

void led_display_temp_and_humi(uint8_t temp_int, uint8_t temp_decimals, uint8_t humidity)
{
  const uint8_t temp_int_1st = temp_int / 10;
  const uint8_t temp_int_2nd = temp_int % 10;

  const uint8_t temp_decimals_1st = temp_decimals & 0x0f;

  const uint8_t humidity_1st = humidity / 10;
  const uint8_t humidity_2nd = humidity % 10;

  MAX72XX_ClearAll();

  for (uint8_t row = 0; row < 8; row++)
  {
    for (uint8_t dev = 0; dev < MAX_DEVICES; dev++)
    {
      uint8_t data = 0;
      switch (dev)
      {
      case 0:
        data = numbers_5x8[temp_int_1st][row] | numbers_5x8[temp_int_2nd][row] << 5;
        break;
      case 1:
        data = numbers_5x8[temp_int_2nd][row] >> 3 | signs[5][row] << 3 | numbers_3x5[temp_decimals_1st][row] << 5;
        break;
      case 2:
        data = numbers_5x8[humidity_1st][row] << 6;
        break;
      case 3:
        data = (numbers_5x8[humidity_1st][row]) >> 2 | ((numbers_5x8[humidity_2nd][row]) << 3);
        break;

      }
      MAX72XX_SetRowOne(dev, row, data);
    }
  }
}

static void Clock_SetRunLed(bool b)
{
  HAL_GPIO_WritePin(RUN_LED_GPIO_Port, RUN_LED_Pin, b);
}

static void Clock_Init(void)
{
  DS3231_Init();
  MAX72XX_Init();
  sht30_init();
  Clock_SetRunLed(true);
}

static bool Clock_UpdateRTC(void)
{
  static uint8_t old_sec = 0;
  DS3231_GetTime(&rtc);

  if (rtc.Sec != old_sec)
  {
    old_sec = rtc.Sec;
    return true;
  }

  return false;
}

static void Clock_FlashTimePoint()
{
  const bool need_flash_point = TestClockFlag(CLOCK_FLAG_FLASH_POINT);
  MAX72XX_SetPoint(1, POINT_COL_NUM, need_flash_point);
  MAX72XX_SetPoint(2, POINT_COL_NUM, need_flash_point);
  MAX72XX_SetPoint(5, POINT_COL_NUM, need_flash_point);
  MAX72XX_SetPoint(6, POINT_COL_NUM, need_flash_point);
}

static void Clock_ShowTime()
{
  led_display_time(rtc.Hour, rtc.Min, rtc.Sec);
  Clock_FlashTimePoint();
  SetClockFlag(CLOCK_FLAG_REFRESH_DISPLAY);
}

static void Clock_ShowDate(void)
{
  led_display_date(rtc.Month, rtc.Day, rtc.DaysOfWeek);
  SetClockFlag(CLOCK_FLAG_REFRESH_DISPLAY);
}

static void Clock_ShowTemp(void)
{
  led_display_temp_and_humi(temp_int, temp_deci, humi_int);
  SetClockFlag(CLOCK_FLAG_REFRESH_DISPLAY);
}

static void Clock_SecondJumpUp(void)
{
  Clock_FlashTimePoint();
  MAX72XX_TransformOne(3, TSU);
  SetClockFlag(CLOCK_FLAG_REFRESH_DISPLAY);
}

static void Clock_SecondJumpDown(void)
{
  Clock_FlashTimePoint();
  MAX72XX_TransformOne(3, TSD);
  SetClockFlag(CLOCK_FLAG_REFRESH_DISPLAY);
}

static void Clock_UpdateDiplay()
{
  if (TestAndClearFlag(CLOCK_FLAG_REFRESH_DISPLAY))
  {
    MAX72XX_UpdateAll();
  }
}

static void Clock_ShowWelcome(void)
{
  MAX72XX_ControlAll(UPDATE, 1);
  MAX72XX_ClearAll();

  osDelay(200);

  for (uint8_t row = 0; row < 8; row++)
  {
    MAX72XX_SetRowAll(row, 0xff);
    osDelay(20);
  }

  osDelay(200);

  MAX72XX_ClearAll();

  for (int i = 0; i < 32; i++)
  {
    MAX72XX_SetPixelColumn(i, 0xff);
    vTaskDelay(15);
    MAX72XX_SetPixelColumn(i, 0x00);
    vTaskDelay(15);
  }

  osDelay(200);

  MAX72XX_ClearAll();
  for (int i = 0; i < 8; i++)
  {
    MAX72XX_SetRowAll(i, 0xff);
    vTaskDelay(20);
    MAX72XX_SetRowAll(i, 0x00);
    osDelay(20);
  }

  vTaskDelay(200);

  MAX72XX_ClearAll();
  Clock_ShowTime();
  MAX72XX_ControlAll(UPDATE, 0);
}

enum
{
  STATE_CLOCK_NONE = -1,
  STATE_CLOCK_TIME_SHOW = 0,
  STATE_CLOCK_TIME_SEC_CHANGED = 1,
  STATE_CLOCK_TIME_SEC_JUMP_UP = 2,
  STATE_CLOCK_TIME_SEC_JUMP_DOWN = 3,
  STATE_CLOCK_DATE = 4,
  STATE_CLOCK_TEMP = 5,
  STATE_CLOCK_KEY_INPUT0 = 6,
  STATE_CLOCK_KEY_INPUT1 = 7,
  STATE_CLOCK_KEY_INPUT2 = 8,
  STATE_CLOCK_KEY_INPUT3 = 9,
  STATE_CLOCK_KEY_INPUT4 = 10,
};

typedef struct
{
  uint32_t state;
  uint32_t next_state;
  int32_t repeat;
  uint32_t duration;
  void (*callback)(void);
  bool change_state;
} ClockState;

static ClockState clock_states[] = {
    {
        .state = STATE_CLOCK_TIME_SHOW,
        .next_state = STATE_CLOCK_NONE,
        .repeat = -1,
        .duration = 10,
        .callback = Clock_ShowTime,
        .change_state = 0
    },
    {
        .state = STATE_CLOCK_TIME_SEC_CHANGED,
        .next_state = STATE_CLOCK_TIME_SEC_JUMP_UP,
        .repeat = 1,
        .duration = 0,
        .callback = Clock_ShowTime,
        .change_state = 0
    },
    {
        .state = STATE_CLOCK_TIME_SEC_JUMP_UP,
        .next_state = STATE_CLOCK_TIME_SEC_JUMP_DOWN,
        .repeat = 2,
        .duration = 98,
        .callback = Clock_SecondJumpUp,
        .change_state = 0
    },
    {
        .state = STATE_CLOCK_TIME_SEC_JUMP_DOWN,
        .next_state = STATE_CLOCK_TIME_SHOW,
        .repeat = 2,
        .duration = 98,
        .callback = Clock_SecondJumpDown,
        .change_state = 0
    },
    {
        .state = STATE_CLOCK_DATE,
        .next_state = STATE_CLOCK_TEMP,
        .repeat = 1,
        .duration = 2000,
        .callback = Clock_ShowDate,
        .change_state = 0
    },
    {
        .state = STATE_CLOCK_TEMP,
        .next_state = STATE_CLOCK_TIME_SHOW,
        .repeat = 1,
        .duration = 2000,
        .callback = Clock_ShowTemp,
        .change_state = 0
    },
    {
        .state = STATE_CLOCK_KEY_INPUT0,
        .next_state = STATE_CLOCK_TIME_SHOW,
        .repeat = 1,
        .duration = 2000,
        .callback = Clock_ShowTemp,
        .change_state = 0
    },
    {
        .state = STATE_CLOCK_KEY_INPUT1,
        .next_state = STATE_CLOCK_TIME_SHOW,
        .repeat = 1,
        .duration = 2000,
        .callback = Clock_ShowTemp,
        .change_state = 0
    },
    {
        .state = STATE_CLOCK_KEY_INPUT2,
        .next_state = STATE_CLOCK_TIME_SHOW,
        .repeat = 1,
        .duration = 2000,
        .callback = Clock_ShowTemp,
        .change_state = 0
    },
    {
        .state = STATE_CLOCK_KEY_INPUT3,
        .next_state = STATE_CLOCK_TIME_SHOW,
        .repeat = 1,
        .duration = 2000,
        .callback = Clock_ShowTemp,
        .change_state = 0
    },
    {
        .state = STATE_CLOCK_KEY_INPUT4,
        .next_state = STATE_CLOCK_TIME_SHOW,
        .repeat = 1,
        .duration = 2000,
        .callback = Clock_ShowTemp,
        .change_state = 0
    }
};

static ClockState clock_s = { 0 };

static inline void ChangeClockState(ClockState *s, int clock_state_n)
{
  if (STATE_CLOCK_NONE == clock_state_n)
  {
    return;
  }

  *s = clock_states[clock_state_n];
}

static inline void TickState(ClockState *s)
{
  if (!s->repeat)
  {
    s->change_state = 1;
    return;
  }

  if (s->callback)
  {
    s->callback();
  }

  if (s->repeat != -1)
  {
    s->repeat--;

    if (!s->repeat)
    {
      s->change_state = 1;
    }
  }
}

void CallbackGetRTC(void const *argument)
{
  if (Clock_UpdateRTC())
  {
    SetClockFlag(CLOCK_FLAG_TIME_SECOND_CHANGED);
    usb_printf("sec changed!\r\n");
  }
}

void CallbackGetSensorData(void const *argument)
{
  DHT11_ReadData(&temp_int, &temp_deci, &humi_int);
  //SetClockFlag(CLOCK_FLAG_SHOW_TEMP);
  //usb_printf("r:%d,t:%d,h:%d \r\n",r,(int)t,(int)h);
}

void CallbackShowDate(void const *argument)
{
  SetClockFlag(CLOCK_FLAG_SHOW_DATE);
}

void CallbackTogglePoint(void const *argument)
{
  FlipClockFlag(CLOCK_FLAG_FLASH_POINT);
  //Clock_SetRunLed(TestClockFlag(CLOCK_FLAG_FLASH_POINT));
}

void StartCheckKeyTask(void const *argument)
{
  for (;;)
  {
    if (Key_Scan(KEY1_GPIO_Port, KEY1_Pin) == KEY_ON)
    {
      /* K1 被按下 */
      SetClockFlag(CLOCK_FLAG_KEY1_PRESSED);
    }

    if (Key_Scan(KEY2_GPIO_Port, KEY2_Pin) == KEY_ON)
    {
      /* K2 被按下 */
      SetClockFlag(CLOCK_FLAG_KEY2_PRESSED);
    }

    osDelay(100);
  }

}

void StartAdcTask(void const *argument)
{
  uint32_t ad_Value = 0;

  for (;;)
  {
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 50);

    if (HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC))
    {
      ad_Value = HAL_ADC_GetValue(&hadc1);

      const uint8_t cur_intensity = MAX72XX_GetIntensity();
      const uint8_t cal_intensity = ad_Value > 2999 ? 0 : 15 - ad_Value / 200;

      //usb_printf("ad_value:%d,cur_intensity:%d,cal_intensity:%d.\r\n", ad_Value, cur_intensity, cal_intensity);

      if (cal_intensity != cur_intensity)
      {
        MAX72XX_SetIntensity(cal_intensity > 5 ? 5 : cal_intensity);
      }
    }

    osDelay(500);
  }
}

void StartSetRTCTask(void const *argument)
{
  for (;;)
  {
    if (rx_data_ctr.flag)
    {
      rx_data_ctr.flag = 0;

      do
      {
        if (14 != rx_data_ctr.rxlen)
        {
          usb_printf("ERR,len:%d.\r\n", rx_data_ctr.rxlen);
          break;
        }

        usb_printf("OK,len:%d.\r\n", rx_data_ctr.rxlen);

        const uint8_t year = (UserRxBufferFS[0] - 48) * 10 + (UserRxBufferFS[1] - 48);
        const uint8_t month = (UserRxBufferFS[2] - 48) * 10 + (UserRxBufferFS[3] - 48);
        const uint8_t day = (UserRxBufferFS[4] - 48) * 10 + (UserRxBufferFS[5] - 48);
        const uint8_t hour = (UserRxBufferFS[6] - 48) * 10 + (UserRxBufferFS[7] - 48);
        const uint8_t min = (UserRxBufferFS[8] - 48) * 10 + (UserRxBufferFS[9] - 48);
        const uint8_t sec = (UserRxBufferFS[10] - 48) * 10 + (UserRxBufferFS[11] - 48);
        const uint8_t week = UserRxBufferFS[12] - 48 + 1;

        rtc.Year = 2000 + year;
        rtc.Month = month;
        rtc.Day = day;
        rtc.Hour = hour;
        rtc.Min = min;
        rtc.Sec = sec;
        rtc.DaysOfWeek = week;

        DS3231_SetTime(&rtc);

      } while (0);

      rx_data_ctr.rxlen = 0;
      memset(UserRxBufferFS, 0, APP_RX_DATA_SIZE);
    }

    osDelay(100);
  }
}

static inline bool IsNotInShowTimeState(ClockState *s)
{
  if (s->state == STATE_CLOCK_DATE || s->state == STATE_CLOCK_TEMP)
  {
    return true;
  }
  return false;
}

void StartMainTask(void const *argument)
{
  Clock_Init();

  Clock_ShowWelcome();

  DHT11_Init();

  Clock_SetRunLed(false);

  clock_s = clock_states[0];

  usb_printf("start main task\r\n");

  for (;;)
  {
    if (TestAndClearFlag(CLOCK_FLAG_KEY1_PRESSED))
    {
      ChangeClockState(&clock_s, STATE_CLOCK_DATE);
    } else if (TestAndClearFlag(CLOCK_FLAG_KEY2_PRESSED))
    {
      ChangeClockState(&clock_s, STATE_CLOCK_TEMP);
    } else if (!IsNotInShowTimeState(&clock_s) && TestAndClearFlag(CLOCK_FLAG_TIME_SECOND_CHANGED))
    {
      ChangeClockState(&clock_s, STATE_CLOCK_TIME_SEC_CHANGED);
    } else if (TestAndClearFlag(CLOCK_FLAG_SHOW_DATE))
    {
      ChangeClockState(&clock_s, STATE_CLOCK_DATE);
    }

    TickState(&clock_s);

    Clock_UpdateDiplay();

    osDelay(clock_s.duration);

    if (clock_s.change_state)
    {
      //usb_printf("clock_s cur_s:%d,next_s:%d.\r\n",clock_s.state,clock_s.next_state);
      ChangeClockState(&clock_s, clock_s.next_state);
    }

    osDelay(1);
  }
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_USB_DEVICE_Init();

  DWT_Init();

  MX_FREERTOS_Init();

  osTimerStart(getRTCTimerHandle, 100);
  osTimerStart(getSensorDataTimerHandle, 20 * 1000);
  osTimerStart(togglePointTimerHandle, 500);
  osTimerStart(showDateTimerHandle, 60 * 1000);

  osKernelStart();

  while (1)
  {
  }
}
