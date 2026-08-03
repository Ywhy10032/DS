/**
  ******************************************************************************
  * @file           : buzzer.c
  * @brief          : 无源蜂鸣器驱动 (TIM13_CH1 / PF8)
  ******************************************************************************
  */

#include "buzzer.h"
#include "tim.h"

#define BUZZER_TIM_HANDLE     (&htim13)
#define BUZZER_TIM_CHANNEL    TIM_CHANNEL_1

void Buzzer_Init(void)
{
  /* 上电先确保 PWM 是关的，蜂鸣器保持静音，等 Buzzer_Beep() 主动响 */
  HAL_TIM_PWM_Stop(BUZZER_TIM_HANDLE, BUZZER_TIM_CHANNEL);
}

void Buzzer_Beep(uint16_t ms)
{
  HAL_TIM_PWM_Start(BUZZER_TIM_HANDLE, BUZZER_TIM_CHANNEL);
  HAL_Delay(ms);
  HAL_TIM_PWM_Stop(BUZZER_TIM_HANDLE, BUZZER_TIM_CHANNEL);
}
