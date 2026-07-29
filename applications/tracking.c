/**
  ******************************************************************************
  * @file           : tracking.c
  * @brief          : 循迹外环 —— 加权质心求偏差，转向 PID 输出左右差速
  ******************************************************************************
  */

#include "tracking.h"
#include "grayscale.h"
#include "pid.h"

/* 8 路探头的安装坐标，下标 0~7 从左到右 */
static const float s_position_mm[GRAY_CHANNEL_NUM] = TRACK_SENSOR_POSITIONS;

/* ---------------- 运行时状态 ---------------- */
static PID_Controller s_steer_pid;

static uint8_t  s_raw[GRAY_CHANNEL_NUM] = {0};
static float    s_offset      = 0.0f;      /* 最近一次的有效偏差 */
static uint8_t  s_lost        = 0;
static uint16_t s_lost_ticks  = 0;

static float    s_base_rpm    = TRACK_BASE_RPM;
static float    s_target[2]   = {0.0f, 0.0f};   /* [0]=左 [1]=右 */

static HAL_StatusTypeDef s_status = HAL_ERROR;  /* 最近一次灰度读取的结果 */

/**
  * @brief  加权质心：把每一路的"黑度"当权重，求这堆权重的重心
  * @param  offset 算出的偏差(mm)，仅在返回 1 时有效
  * @retval 1 = 成功；0 = 丢线(权重总和过小)
  *
  * @note   归一化后白场 255、黑场 0。白底黑线时权重取 255-读数，读数越黑
  *         权重越大；黑底白线则反过来直接用读数。
  *         低于噪声阈值的通道整个清零 —— 白场读数不会正好是极值，不掐掉
  *         的话八路的零星波动会把质心往中间拽偏。
  */
static uint8_t Track_Centroid(const uint8_t *values, float *offset)
{
  float sum_weight   = 0.0f;
  float sum_weighted = 0.0f;

  for (uint8_t i = 0; i < GRAY_CHANNEL_NUM; i++)
  {
#if TRACK_LINE_IS_BLACK
    int32_t weight = 255 - (int32_t)values[i];
#else
    int32_t weight = (int32_t)values[i];
#endif

    if (weight < TRACK_WEIGHT_NOISE_TH)
    {
      weight = 0;
    }

    sum_weight   += (float)weight;
    sum_weighted += (float)weight * s_position_mm[i];
  }

  if (sum_weight < (float)TRACK_LOST_TH)
  {
    return 0;                       /* 八路都没看到线 */
  }

  *offset = sum_weighted / sum_weight;
  return 1;
}

void Track_Init(void)
{
  PID_Init(&s_steer_pid, TRACK_STEER_KP, TRACK_STEER_KI, TRACK_STEER_KD,
           TRACK_PERIOD_S);
  PID_SetOutputLimits(&s_steer_pid, -TRACK_STEER_LIMIT_RPM, TRACK_STEER_LIMIT_RPM);
  PID_SetIntegralLimit(&s_steer_pid, TRACK_STEER_LIMIT_RPM / 2.0f);

  s_offset     = 0.0f;
  s_lost       = 0;
  s_lost_ticks = 0;
  s_base_rpm   = TRACK_BASE_RPM;
  s_target[0]  = 0.0f;
  s_target[1]  = 0.0f;
}

static float Track_ClampRpm(float rpm)
{
  if (rpm > TRACK_MAX_RPM)
  {
    return TRACK_MAX_RPM;
  }
  if (rpm < -TRACK_MAX_RPM)
  {
    return -TRACK_MAX_RPM;
  }
  return rpm;
}

void Track_Update(void)
{
  uint8_t values[GRAY_CHANNEL_NUM];
  float   offset;
  float   steer;

  /* ---------- 1. 读灰度 ---------- */
  /* I2C 偶发失败时沿用上一次的偏差，不让通讯抖动传进控制环 */
  s_status = Gray_ReadAll(values);
  if (s_status == HAL_OK)
  {
    for (uint8_t i = 0; i < GRAY_CHANNEL_NUM; i++)
    {
      s_raw[i] = values[i];
    }

    /* ---------- 2. 加权质心 ---------- */
    if (Track_Centroid(values, &offset))
    {
      s_offset     = offset;
      s_lost       = 0;
      s_lost_ticks = 0;
    }
    else
    {
      /* 丢线：保持上一次的偏差不变。这样车会继续按丢线前的方向修正，
         多数情况下能自己拐回线上；实在找不回来就靠下面的超时停车兜底 */
      s_lost = 1;
      if (s_lost_ticks < 0xFFFFU)
      {
        s_lost_ticks++;
      }
    }
  }

  /* ---------- 3. 丢线超时保护 ---------- */
  if ((uint32_t)s_lost_ticks * TRACK_PERIOD_MS >= TRACK_LOST_STOP_MS)
  {
    s_target[0] = 0.0f;
    s_target[1] = 0.0f;
    return;
  }

  /* ---------- 4. 转向 PID ---------- */
  /* 目标偏差恒为 0(线压在车体中线上)。offset > 0 表示线在右侧，
     PID 输出为负，于是左轮 base-steer 变快、右轮 base+steer 变慢，
     车头向右修正 —— 正好把线拉回中间 */
  steer = PID_Update(&s_steer_pid, 0.0f, s_offset);

  s_target[0] = Track_ClampRpm(s_base_rpm - steer);
  s_target[1] = Track_ClampRpm(s_base_rpm + steer);
}

void Track_GetTargets(float *left_rpm, float *right_rpm)
{
  if (left_rpm != NULL)
  {
    *left_rpm = s_target[0];
  }
  if (right_rpm != NULL)
  {
    *right_rpm = s_target[1];
  }
}

float Track_GetOffset(void)
{
  return s_offset;
}

uint8_t Track_IsLost(void)
{
  return s_lost;
}

const uint8_t *Track_GetRaw(void)
{
  return s_raw;
}

HAL_StatusTypeDef Track_GetStatus(void)
{
  return s_status;
}

void Track_SetBaseSpeed(float rpm)
{
  s_base_rpm = rpm;
}

void Track_SetTunings(float kp, float ki, float kd)
{
  PID_SetTunings(&s_steer_pid, kp, ki, kd);
}

void Track_Stop(void)
{
  s_base_rpm  = 0.0f;
  s_target[0] = 0.0f;
  s_target[1] = 0.0f;
  PID_Reset(&s_steer_pid);
}

/**
  ******************************************************************************
  * 转向 PID 整定
  *
  *   Kp  车压线上时把它拉回中线的力度。
  *       太小 -> 过弯跟不上，冲出赛道
  *       太大 -> 直道上左右画蛇(S 形摆动)
  *       先 Ki=Kd=0，只调 Kp 到"直道基本走直、弯道能跟上"为止。
  *
  *   Kd  抑制画蛇的阻尼，靠偏差的变化率提前反打方向。
  *       Kp 调好后如果还有轻微摆动，加一点 Kd 会明显改善。
  *       太大会对灰度噪声过敏，表现为高频抖动。
  *
  *   Ki  循迹环一般保持 0。转向没有"稳态误差"需要消除，加了反而会在
  *       连续弯道后残留积分，让车拐过头。除非车有明显的机械偏心
  *       (松手直行会往一边偏)，才给一点点 Ki。
  *
  *   基准速度和 Kp 是耦合的：TRACK_BASE_RPM 提高后，同样的 Kp 会显得偏软，
  *   通常需要同步加大。建议先在低速(40~60rpm)把形状调对，再逐步提速。
  ******************************************************************************
  */
