/**
  ******************************************************************************
  * @file           : battery.c
  * @brief          : ADC1 电池电压采样与低通滤波
  ******************************************************************************
  */

#include "battery.h"
#include "adc.h"

#define BATTERY_ADC_VREF        3.3f
#define BATTERY_ADC_MAX         4095.0f
#define BATTERY_DIVIDER_RATIO   11.0f    /* (100k+10k)/10k */
#define BATTERY_LPF_ALPHA       0.1f     /* 一阶低通系数 */

/* 三节锂电池报警阈值 */
#define BATTERY_WARN_VOLTAGE    11.5f
#define BATTERY_LOW_VOLTAGE     11.3f

static float s_voltage = 0.0f;

static float Battery_ReadRaw(void)
{
  uint32_t raw;

  HAL_ADC_Start(&hadc1);
  if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK)
  {
    HAL_ADC_Stop(&hadc1);
    return s_voltage;                  /* 转换超时时保留上次结果 */
  }
  raw = HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);

  return (float)raw / BATTERY_ADC_MAX * BATTERY_ADC_VREF * BATTERY_DIVIDER_RATIO;
}

void Battery_Init(void)
{
  s_voltage = Battery_ReadRaw();
}

void Battery_Update(void)
{
  float sample = Battery_ReadRaw();

  s_voltage += BATTERY_LPF_ALPHA * (sample - s_voltage);
}

float Battery_GetVoltage(void)
{
  return s_voltage;
}

Battery_Level Battery_GetLevel(void)
{
  if (s_voltage < BATTERY_LOW_VOLTAGE)
  {
    return BATTERY_LEVEL_LOW;
  }
  if (s_voltage < BATTERY_WARN_VOLTAGE)
  {
    return BATTERY_LEVEL_WARN;
  }
  return BATTERY_LEVEL_NORMAL;
}
