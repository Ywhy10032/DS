#ifndef __BATTERY_H
#define __BATTERY_H

#include "main.h"

/**
  ******************************************************************************
  * 电源电压检测 —— PC4 (ADC1_IN14)
  *
  * 硬件分压：VIN --100k-- ADC --10k-- GND，即 Vadc = VIN * 10/(100+10)，
  * 反推 VIN = Vadc * 11。ADC1 本身由 CubeMX 生成的 MX_ADC1_Init() 初始化，
  * 在 main() 里跑在 App_Init() 之前，本模块只管转换与滤波。
  ******************************************************************************
  */

typedef enum
{
  BATTERY_LEVEL_NORMAL = 0,
  BATTERY_LEVEL_WARN,        /* 低于 11.5V，黄色 */
  BATTERY_LEVEL_LOW          /* 低于 11.3V，红色 */
} Battery_Level;

void          Battery_Init(void);        /* 采一次真实值，避免滤波器从 0 爬升 */
void          Battery_Update(void);      /* 采一次 ADC，一阶低通后更新电压，周期调用 */
float         Battery_GetVoltage(void);  /* 取滤波后的 VIN 电压，单位 V */
Battery_Level Battery_GetLevel(void);    /* 电压所处的报警等级 */

#endif /* __BATTERY_H */
