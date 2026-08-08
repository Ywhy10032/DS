#ifndef __BUZZER_H
#define __BUZZER_H

#include "main.h"

/**
  ******************************************************************************
  * @brief  PF8（TIM13_CH1）无源蜂鸣器驱动
  *
  * TIM13 输出约 420Hz、50% 占空比的 PWM，Buzzer_Beep() 控制其启停。
  ******************************************************************************
  */

/* 开机提示音时长，ms */
#define BUZZER_BOOT_BEEP_MS     100

/* 启动/关闭 PWM 输出。须在 MX_TIM13_Init() 之后调用 */
void Buzzer_Init(void);

/* 阻塞发声指定时间，不应在控制环中调用 */
void Buzzer_Beep(uint16_t ms);

#endif /* __BUZZER_H */
