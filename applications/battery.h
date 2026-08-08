#ifndef __BATTERY_H
#define __BATTERY_H

#include "main.h"

/**
  ******************************************************************************
  * @brief  PC4（ADC1_IN14）电池电压检测
  *
  * 硬件分压：VIN --100k-- ADC --10k-- GND，即 Vadc = VIN * 10/(100+10)，
  * 反推 VIN = Vadc * 11。本模块负责 ADC 转换和低通滤波。
  ******************************************************************************
  */

typedef enum
{
  BATTERY_LEVEL_NORMAL = 0,
  BATTERY_LEVEL_WARN,        /* 低于 11.5V，黄色 */
  BATTERY_LEVEL_LOW          /* 低于 11.3V，红色 */
} Battery_Level;

void          Battery_Init(void);        /* 读取初始电压 */
void          Battery_Update(void);      /* 采样并更新滤波值 */
float         Battery_GetVoltage(void);  /* 返回滤波后的输入电压，V */
Battery_Level Battery_GetLevel(void);    /* 返回电压报警等级 */

#endif /* __BATTERY_H */
