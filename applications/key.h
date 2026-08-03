#ifndef __KEY_H
#define __KEY_H

#include "main.h"

/**
  ******************************************************************************
  * 4 路按键
  *
  *   KEY1 -> PC2        KEY2 -> PC0        KEY3 -> PF9        KEY4 -> PF7
  *
  * CubeMX 里已配成上拉输入(GPIO_PULLUP)，按键另一端接地，所以按下读到低电平。
  * 引脚与时钟由 MX_GPIO_Init() 初始化，本模块只做消抖与边沿检测。
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

/* 连续读到相同电平这么多次才认账。按 Key_Scan() 的调用周期算消抖时间，
   10ms 调用一次 x 3 = 30ms，足够滤掉机械抖动又不影响手感 */
#define KEY_DEBOUNCE_COUNT      3

/* 清空内部状态。可选，Key_Scan() 本身能自行收敛 */
void Key_Init(void);

/* 扫描一次按键。需周期性调用，建议 10ms */
void Key_Scan(void);

/* 按住自动连发：按下超过 DELAY 之后，每 PERIOD 次扫描产生一次事件。
   按 Key_Scan() 10ms 的调用周期算 = 按住 0.4s 后开始，每 10ms 连发一次 */
#define KEY_REPEAT_DELAY_TICKS  40
#define KEY_REPEAT_PERIOD_TICKS 1

/* 按住多少次扫描算"长按"。按 10ms 的扫描周期算 = 0.8s。
   别低于 0.5s：太短的话正常的一次按键很容易被误判成长按；也别太长，
   手指按着没反应会让人以为按键失灵(实际按满门槛的当下就有反应)。 */
#define KEY_LONG_PRESS_TICKS    80

/* 取走一次"按下"事件(下降沿)，读到后自动清除，不会重复触发 */
uint8_t Key_WasPressed(Key_ID id);

/**
  * @brief  取走一次"按下或连发"事件
  * @note   与 Key_WasPressed() 各自维护事件位，互不影响：
  *         切任务那种一次只该触发一次的用 WasPressed，
  *         调参那种按住要连续走的用本函数。
  */
uint8_t Key_WasRepeated(Key_ID id);

/**
  * @brief  取走一次"短按"事件 —— 按下并在 KEY_LONG_PRESS_TICKS 之内抬手
  * @note   与 Key_WasLongPressed() 配对使用，一次按键只会命中其中一个：
  *         按住够久 -> 长按事件(按满门槛的当下就发)，抬手时不再补短按；
  *         提前抬手 -> 短按事件。
  *
  *         代价是短按【要等抬手才生效】，比 Key_WasPressed() 晚一点。只有
  *         真的需要区分长短按的键才用它(如 KEY1：短按切任务、长按倒车)，
  *         其余的键继续用 Key_WasPressed() 按下即响应。
  *         三套事件位各自独立，同一个键混着读也互不影响。
  */
uint8_t Key_WasClicked(Key_ID id);

/* 取走一次"长按"事件，见上面 Key_WasClicked() 的说明 */
uint8_t Key_WasLongPressed(Key_ID id);

/* 当前是否按住 */
uint8_t Key_IsDown(Key_ID id);

#endif /* __KEY_H */
