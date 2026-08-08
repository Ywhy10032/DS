#ifndef __BALL_H
#define __BALL_H

#include "main.h"

/**
 * @file ball.h
 * @brief 基于视觉反馈的钢球位置控制。
 *
 * 控制器采用串级结构：位置环输出目标球速，速度环输出相对水平位置的
 * 舵机脉宽。两级控制器只在收到新的有效视觉帧时更新。
 */

/* 默认目标位置，单位 cm。当前坐标系中的摆杆中心为 12.5 cm。 */
#define BALL_TARGET_CM          12.5f

/* 速度环参数：球速误差 -> 舵机脉宽偏移。 */
#define BALL_VEL_KP             33.0f
#define BALL_VEL_KI             70.0f
#define BALL_VEL_KD             0.0f

/* 速度环积分项限幅，单位 us。 */
#define BALL_VEL_I_LIMIT_US     500.0f

/* 速度误差小于该值时冻结积分，单位 cm/s。 */
#define BALL_VEL_I_DEADBAND_CMS 1.0f

/* 位置环参数：位置误差 -> 目标球速。 */
#define BALL_POS_KP             1.6f
#define BALL_POS_KI             0.2f
#define BALL_POS_KD             0.022f

/* 位置环积分项和输出球速的限幅。 */
#define BALL_POS_I_LIMIT_CMS    1.5f
#define BALL_VEL_LIMIT_CMS      12.0f

/**
 * 按剩余距离限制目标球速时使用的减速度，单位 cm/s^2。
 * 速度上限为 sqrt(2 * a * 剩余距离)，设为 0 时关闭。任务三会加载自己的值。
 */
#define BALL_POS_BRAKE_CMS2     0.0f

/* 速度环输出相对水平脉宽的最大偏移，单位 us。 */
#define BALL_OUTPUT_LIMIT_US    560.0f

/* 舵机输出方向。摄像头或机构方向改变后需重新确认。 */
#define BALL_OUTPUT_SIGN        (+1.0f)

/* 由位置差分得到球速后使用的一阶低通系数。 */
#define BALL_VEL_LPF            0.30f

/* 视觉帧有效性参数。 */
#define BALL_MIN_CONFIDENCE     0.50f
#define BALL_TIMEOUT_MS         200
#define BALL_DT_MIN_S           0.005f
#define BALL_DT_MAX_S           0.150f

/**
 * 车辆纵向加速度前馈。
 * 增益单位为 us/(rpm/s)，运行时可通过 Ball_SetAccelFFGain() 调整。
 */
#define BALL_FF_ENABLE          1
#define BALL_FF_US_PER_RPMS     5.0f

/* 过弯前馈当前关闭；保留增益供试验使用。 */
#define BALL_FF_CURVE_ENABLE    0
#define BALL_FF_CURVE_GAIN      0.04f

/* 可按任务切换的球控参数。 */
typedef struct
{
  float pos_kp, pos_ki, pos_kd;
  float pos_i_limit_cms;
  float vel_limit_cms;
  float pos_brake_cms2;

  float vel_kp, vel_ki, vel_kd;
  float vel_i_limit_us;
  float vel_i_deadband_cms;

  float out_limit_us;
} Ball_Tune;

#define BALL_TUNE_DEFAULT_INIT                                  \
{                                                               \
  BALL_POS_KP, BALL_POS_KI, BALL_POS_KD,                        \
  BALL_POS_I_LIMIT_CMS, BALL_VEL_LIMIT_CMS, BALL_POS_BRAKE_CMS2, \
  BALL_VEL_KP, BALL_VEL_KI, BALL_VEL_KD,                        \
  BALL_VEL_I_LIMIT_US, BALL_VEL_I_DEADBAND_CMS,                 \
  BALL_OUTPUT_LIMIT_US                                          \
}

/* 参数组接口。 */
void Ball_SetTune(const Ball_Tune *tune);
void Ball_ResetTune(void);
Ball_Tune Ball_GetTune(void);

/* 初始化并周期更新球控状态。Ball_Update() 只处理新的视觉帧。 */
void Ball_Init(void);
void Ball_Update(void);

/* 使能或关闭闭环。关闭时摆杆回到水平位置并复位控制器。 */
void Ball_Enable(uint8_t on);
uint8_t Ball_IsEnabled(void);

/* 目标位置接口，单位 cm。 */
void Ball_SetTarget(float cm);
float Ball_GetTarget(void);

/* 车辆运动前馈输入。 */
void Ball_SetAccelFF(float rpm_per_s);
void Ball_SetCurveFF(float v_times_diff);

/* 纵向加速度前馈增益接口。该增益不随 Ball_Tune 重置。 */
void  Ball_SetAccelFFGain(float us_per_rpms);
float Ball_GetAccelFFGain(void);

/* 诊断数据。 */
float   Ball_GetPosCm(void);
float   Ball_GetVelCmS(void);
float   Ball_GetVelSetCmS(void);
float   Ball_GetOutputUs(void);
uint8_t Ball_IsTracking(void);

#endif /* __BALL_H */
