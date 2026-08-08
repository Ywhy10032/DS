#ifndef __SERVO_H
#define __SERVO_H

#include "main.h"

/**
  ******************************************************************************
  * @brief  TIM4_CH4 舵机驱动
  *
  * TIM4 计数频率为 1MHz，周期为 20ms，比较值单位为微秒。
  ******************************************************************************
  */

/* 舵机电气量程，仅用于角度与脉宽换算 */
#define SERVO_MIN_PULSE_US      500
#define SERVO_MAX_PULSE_US      2500
#define SERVO_MAX_ANGLE_DEG     180.0f

/* 机构实测安全行程，所有输出接口均受此范围限制 */
#define SERVO_SAFE_MIN_US       1050
#define SERVO_SAFE_MAX_US       2400

/* 管道水平位置的实测脉宽 */
#define SERVO_LEVEL_US          1640

/* 启动 PWM，并将舵机置于水平位置 */
void Servo_Init(void);

/* 设置或读取舵机角度，设置值自动限制在安全行程内 */
void Servo_SetAngle(float deg);
float Servo_GetAngle(void);

/* 设置或读取脉宽，设置值自动限制在安全行程内 */
void Servo_SetPulseUs(uint16_t us);
uint16_t Servo_GetPulseUs(void);

/* 返回上一次设置是否触发安全限位 */
uint8_t Servo_IsAtLimit(void);

/* 手动标定的最小脉宽步进 */
#define SERVO_STEP_US           1

void Servo_StepUs(int16_t delta_us);

/* 自检时在安全行程内匀速往复摆动 */
#define SERVO_DEMO_HOLD_MS      1000
#define SERVO_DEMO_SPEED_UPS    600.0f

void Servo_DemoInit(void);
void Servo_DemoUpdate(void);

#endif /* __SERVO_H */
