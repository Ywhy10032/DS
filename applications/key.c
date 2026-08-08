/**
  ******************************************************************************
  * @file           : key.c
  * @brief          : 四路按键消抖与事件检测
  ******************************************************************************
  */

#include "key.h"

typedef struct
{
  GPIO_TypeDef *port;
  uint16_t      pin;
} Key_Config;

static const Key_Config s_key_cfg[KEY_NUM] =
{
  [KEY1] = { key1_GPIO_Port, key1_Pin },   /* PC2 */
  [KEY2] = { key2_GPIO_Port, key2_Pin },   /* PC0 */
  [KEY3] = { key3_GPIO_Port, key3_Pin },   /* PF9 */
  [KEY4] = { key4_GPIO_Port, key4_Pin },   /* PF7 */
};

static uint8_t  s_stable[KEY_NUM]   = {0};  /* 消抖后的稳定状态，1=按下 */
static uint8_t  s_counter[KEY_NUM]  = {0};  /* 连续同电平计数 */
static uint8_t  s_pressed[KEY_NUM]  = {0};  /* 待取走的按下事件 */
static uint8_t  s_repeated[KEY_NUM] = {0};  /* 待取走的按下/连发事件 */
static uint16_t s_hold[KEY_NUM]     = {0};  /* 已按住多少次扫描 */
static uint8_t  s_clicked[KEY_NUM]   = {0}; /* 待取走的短按事件(抬手时才判定) */
static uint8_t  s_long[KEY_NUM]      = {0}; /* 待取走的长按事件 */
static uint8_t  s_long_done[KEY_NUM] = {0}; /* 本次按住已经报过长按了 */

void Key_Init(void)
{
  for (uint8_t i = 0; i < KEY_NUM; i++)
  {
    s_stable[i]    = 0;
    s_counter[i]   = 0;
    s_pressed[i]   = 0;
    s_repeated[i]  = 0;
    s_hold[i]      = 0;
    s_clicked[i]   = 0;
    s_long[i]      = 0;
    s_long_done[i] = 0;
  }
}

void Key_Scan(void)
{
  for (uint8_t i = 0; i < KEY_NUM; i++)
  {
    /* 上拉输入 + 按键接地：低电平 = 按下 */
    uint8_t raw = (HAL_GPIO_ReadPin(s_key_cfg[i].port, s_key_cfg[i].pin) == GPIO_PIN_RESET);

    if (raw == s_stable[i])
    {
      s_counter[i] = 0;                    /* 电平与稳定状态一致 */
    }
    else
    {
      /* 电平连续变化达到门限后更新稳定状态 */
      s_counter[i]++;
      if (s_counter[i] >= KEY_DEBOUNCE_COUNT)
      {
        s_counter[i] = 0;
        s_stable[i]  = raw;

        if (raw)
        {
          s_pressed[i]   = 1;              /* 记录下降沿 */
          s_repeated[i]  = 1;
          s_hold[i]      = 0;
          s_long_done[i] = 0;
        }
        else if (!s_long_done[i])
        {
          /* 未达到长按门限时，在按键释放后产生短按事件 */
          s_clicked[i] = 1;
        }
      }
    }

    /* 按住计时与连发 */
    if (!s_stable[i])
    {
      s_hold[i] = 0;
      continue;
    }

    if (s_hold[i] < 0xFFFFU)
    {
      s_hold[i]++;
    }

    /* 达到门限时立即产生一次长按事件 */
    if (!s_long_done[i] && (s_hold[i] >= KEY_LONG_PRESS_TICKS))
    {
      s_long_done[i] = 1;
      s_long[i]      = 1;
    }

    if ((s_hold[i] > KEY_REPEAT_DELAY_TICKS) &&
        (((s_hold[i] - KEY_REPEAT_DELAY_TICKS) % KEY_REPEAT_PERIOD_TICKS) == 0U))
    {
      s_repeated[i] = 1;
    }
  }
}

uint8_t Key_WasPressed(Key_ID id)
{
  uint8_t event;

  if (id >= KEY_NUM)
  {
    return 0;
  }

  event = s_pressed[id];
  s_pressed[id] = 0;                       /* 取走即清除，不会重复触发 */
  return event;
}

uint8_t Key_WasRepeated(Key_ID id)
{
  uint8_t event;

  if (id >= KEY_NUM)
  {
    return 0;
  }

  event = s_repeated[id];
  s_repeated[id] = 0;
  return event;
}

uint8_t Key_WasClicked(Key_ID id)
{
  uint8_t event;

  if (id >= KEY_NUM)
  {
    return 0;
  }

  event = s_clicked[id];
  s_clicked[id] = 0;
  return event;
}

uint8_t Key_WasLongPressed(Key_ID id)
{
  uint8_t event;

  if (id >= KEY_NUM)
  {
    return 0;
  }

  event = s_long[id];
  s_long[id] = 0;
  return event;
}

uint8_t Key_IsDown(Key_ID id)
{
  return (id < KEY_NUM) ? s_stable[id] : 0;
}
