#ifndef __BUZZER_H
#define __BUZZER_H

#include "main.h"

/**
  ******************************************************************************
  * 无源蜂鸣器 —— PF8 (TIM13_CH1)
  *
  * 无源蜂鸣器本身不带振荡电路，靠外部方波驱动发声，所以不能像有源蜂鸣器那样
  * 拉高电平就响，必须用 PWM 喂一个音频频段的方波。CubeMX 已把 TIM13 配好：
  * PSC=83 -> 84MHz/84 = 1MHz 计数，ARR=2380 -> 约 420Hz、占空比 50%，
  * 直接落在蜂鸣器响度较高的频段上，Buzzer_Beep() 只管开关这路 PWM。
  ******************************************************************************
  */

/* 开机提示音时长(ms)，阻塞等待，只在 App_Init() 里用一次 */
#define BUZZER_BOOT_BEEP_MS     100

/* 启动/关闭 PWM 输出。须在 MX_TIM13_Init() 之后调用 */
void Buzzer_Init(void);

/* 阻塞响 ms 毫秒后关闭，期间占用 CPU(HAL_Delay)，不要放进控制环 */
void Buzzer_Beep(uint16_t ms);

#endif /* __BUZZER_H */
