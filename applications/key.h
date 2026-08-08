#ifndef __KEY_H
#define __KEY_H

#include "main.h"

/**
  ******************************************************************************
  * @brief  四路低电平有效按键的消抖与事件检测
  *
  * KEY1=PC2，KEY2=PC0，KEY3=PF9，KEY4=PF7。
  ******************************************************************************
  */

typedef enum
{
  KEY1 = 0,
  KEY2,
  KEY3,
  KEY4,
  KEY_NUM
} Key_ID;

/* 消抖计数以 Key_Scan() 的调用周期为单位，当前调用周期为 10ms */
#define KEY_DEBOUNCE_COUNT       3

void Key_Init(void);
void Key_Scan(void);

/* 连发和长按计数以 Key_Scan() 的调用周期为单位 */
#define KEY_REPEAT_DELAY_TICKS   40
#define KEY_REPEAT_PERIOD_TICKS  1
#define KEY_LONG_PRESS_TICKS     80

/* 读取并清除一次按下事件 */
uint8_t Key_WasPressed(Key_ID id);

/* 读取并清除一次按下或连发事件 */
uint8_t Key_WasRepeated(Key_ID id);

/* 短按在抬起时产生，长按在达到门限时产生，两者互斥 */
uint8_t Key_WasClicked(Key_ID id);
uint8_t Key_WasLongPressed(Key_ID id);

uint8_t Key_IsDown(Key_ID id);

#endif /* __KEY_H */
