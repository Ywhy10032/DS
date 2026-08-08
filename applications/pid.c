/**
  ******************************************************************************
  * @file           : pid.c
  * @brief          : 位置式 PID 控制器（微分先行 + 抗积分饱和）
  ******************************************************************************
  */

#include "pid.h"

#include <math.h>

void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float dt)
{
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->dt = dt;

  pid->out_min           = -1000.0f;
  pid->out_max           =  1000.0f;
  pid->integral_limit    =  1000.0f;
  pid->integral_deadband =     0.0f;   /* 默认关闭 */

  PID_Reset(pid);
}

void PID_SetOutputLimits(PID_Controller *pid, float min, float max)
{
  pid->out_min = min;
  pid->out_max = max;
}

void PID_SetIntegralLimit(PID_Controller *pid, float limit)
{
  pid->integral_limit = limit;
}

void PID_SetIntegralDeadband(PID_Controller *pid, float deadband)
{
  pid->integral_deadband = deadband;
}

void PID_SetTunings(PID_Controller *pid, float kp, float ki, float kd)
{
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
}

void PID_Reset(PID_Controller *pid)
{
  pid->integral         = 0.0f;
  pid->prev_measurement = 0.0f;
  pid->first_run        = 1;
}

void PID_PresetIntegral(PID_Controller *pid, float term)
{
  if (pid->ki <= 1e-6f)
  {
    return;                    /* Ki 为零时积分项无输出 */
  }

  /* 对积分项输出限幅，与 PID_Update() 的口径一致 */
  if (term > pid->integral_limit)
  {
    term = pid->integral_limit;
  }
  else if (term < -pid->integral_limit)
  {
    term = -pid->integral_limit;
  }

  pid->integral = term / pid->ki;
}

float PID_Update(PID_Controller *pid, float setpoint, float measurement)
{
  float error = setpoint - measurement;
  float p_term;
  float i_term;
  float d_term = 0.0f;
  float out;

  /* 比例项 */
  p_term = pid->kp * error;

  /* 积分项，带死区和抗饱和限幅 */
  if (pid->ki > 1e-6f)
  {
    /* 死区内冻结积分，保留已有的稳态补偿 */
    if (fabsf(error) >= pid->integral_deadband)
    {
      float integral_max;

      pid->integral += error * pid->dt;

      /* 对 ki*integral 限幅，使限幅含义不随 Ki 改变 */
      integral_max = pid->integral_limit / pid->ki;
      if (pid->integral > integral_max)
      {
        pid->integral = integral_max;
      }
      else if (pid->integral < -integral_max)
      {
        pid->integral = -integral_max;
      }
    }
  }
  else
  {
    pid->integral = 0.0f;      /* Ki 关闭时清除积分 */
  }
  i_term = pid->ki * pid->integral;

  /* 微分项作用于测量值 */
  if (pid->first_run)
  {
    pid->first_run = 0;        /* 首次更新没有历史测量值 */
  }
  else
  {
    d_term = -pid->kd * (measurement - pid->prev_measurement) / pid->dt;
  }
  pid->prev_measurement = measurement;

  /* 求和并执行输出限幅 */
  out = p_term + i_term + d_term;

  if (out > pid->out_max)
  {
    out = pid->out_max;
  }
  else if (out < pid->out_min)
  {
    out = pid->out_min;
  }

  return out;
}
