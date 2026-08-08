#ifndef __PID_H
#define __PID_H

#include "main.h"

/**
  ******************************************************************************
  * @brief  位置式 PID 控制器
  *
  * 输出为 Kp*e + Ki*integral - Kd*d(measurement)/dt。
  * 微分作用于测量值，积分项带独立限幅和可选死区。
  ******************************************************************************
  */

typedef struct
{
  float   kp;
  float   ki;
  float   kd;
  float   dt;                 /* 控制周期，s */

  float   integral;           /* 误差积分 */
  float   prev_measurement;   /* 上一周期测量值 */

  float   out_min;
  float   out_max;
  float   integral_limit;     /* 积分项的绝对值上限 */
  float   integral_deadband;  /* 积分冻结区，0 表示关闭 */

  uint8_t first_run;          /* 首次更新时跳过微分 */
} PID_Controller;

/* 初始化控制器并清除内部状态 */
void  PID_Init(PID_Controller *pid, float kp, float ki, float kd, float dt);

void  PID_SetOutputLimits(PID_Controller *pid, float min, float max);
void  PID_SetIntegralLimit(PID_Controller *pid, float limit);
void  PID_SetIntegralDeadband(PID_Controller *pid, float deadband);

/* 修改增益，不清除已有积分 */
void  PID_SetTunings(PID_Controller *pid, float kp, float ki, float kd);

/* 按设定的 dt 更新一次控制器 */
float PID_Update(PID_Controller *pid, float setpoint, float measurement);

/* 清除积分和微分历史 */
void  PID_Reset(PID_Controller *pid);

/* 直接设置积分项输出，设置值受积分限幅约束 */
void  PID_PresetIntegral(PID_Controller *pid, float term);

#endif /* __PID_H */
