/**
  ******************************************************************************
  * @file           : tracking.c
  * @brief          : 八路灰度循迹外环
  ******************************************************************************
  */

#include "tracking.h"
#include "grayscale.h"
#include "pid.h"

#include <math.h>

/* 8 路探头的安装坐标，下标 0~7 从左到右 */
static const float s_position_mm[GRAY_CHANNEL_NUM] = TRACK_SENSOR_POSITIONS;

/* ---------------- 运行时状态 ---------------- */
static PID_Controller s_steer_pid;

static uint8_t  s_raw[GRAY_CHANNEL_NUM] = {0};
static float    s_offset      = 0.0f;      /* 最近一次的有效偏差 */
static uint8_t  s_lost        = 0;
static uint16_t s_lost_ticks  = 0;
static uint8_t  s_dark_count  = 0;         /* 有多少路探头看到黑色 */
static uint8_t  s_dark_left   = 0;         /* 车体中线左侧黑色通道数 */
static uint8_t  s_dark_right  = 0;         /* 车体中线右侧黑色通道数 */
static float    s_line_weight = 0.0f;      /* 八路权重总和(总黑度) */

/* 当前横线门限；进入终点窗口后可由任务层调整。 */
static uint8_t  s_cross_min_ch   = TRACK_CROSS_MIN_CH;
static float    s_cross_max_off  = TRACK_CROSS_MAX_OFFSET_MM;

/* 终点窗口内的通道数和总黑度峰值。 */
static uint8_t  s_dark_peak   = 0;
static float    s_weight_peak = 0.0f;

static float    s_base_rpm    = TRACK_BASE_RPM;
static float    s_curve_slow  = TRACK_CURVE_SLOWDOWN;
static float    s_target[2]   = {0.0f, 0.0f};   /* [0]=左 [1]=右 */

static HAL_StatusTypeDef s_status = HAL_ERROR;  /* 最近一次灰度读取的结果 */

/**
  * @brief  加权质心：把每一路的"黑度"当权重，求这堆权重的重心
  * @param  offset 算出的偏差(mm)，仅在返回 1 时有效
  * @retval 1 = 成功；0 = 丢线(权重总和过小)
  *
  * @note   白底黑线时使用 255-读数作为权重；低于噪声门限的权重清零。
  */
static uint8_t Track_Centroid(const uint8_t *values, float *offset)
{
  float   sum_weight   = 0.0f;
  float   sum_weighted = 0.0f;
  uint8_t dark_count   = 0;
  uint8_t dark_left    = 0;
  uint8_t dark_right   = 0;

  for (uint8_t i = 0; i < GRAY_CHANNEL_NUM; i++)
  {
#if TRACK_LINE_IS_BLACK
    int32_t weight = 255 - (int32_t)values[i];
#else
    int32_t weight = (int32_t)values[i];
#endif

    if (weight >= TRACK_CROSS_WEIGHT_TH)
    {
      /* 统计黑色通道数量及其在车体中线两侧的分布。 */
      dark_count++;

      if (s_position_mm[i] < 0.0f)
      {
        dark_left++;
      }
      else
      {
        dark_right++;
      }
    }

    if (weight < TRACK_WEIGHT_NOISE_TH)
    {
      weight = 0;
    }

    sum_weight   += (float)weight;
    sum_weighted += (float)weight * s_position_mm[i];
  }

  s_dark_count  = dark_count;
  s_dark_left   = dark_left;
  s_dark_right  = dark_right;
  s_line_weight = sum_weight;

  /* 保留终点检测所需的短时峰值。 */
  if (dark_count > s_dark_peak)
  {
    s_dark_peak = dark_count;
  }
  if (sum_weight > s_weight_peak)
  {
    s_weight_peak = sum_weight;
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
  s_curve_slow = TRACK_CURVE_SLOWDOWN;
  s_target[0]  = 0.0f;
  s_target[1]  = 0.0f;

  /* 每次初始化恢复默认横线门限。 */
  s_cross_min_ch  = TRACK_CROSS_MIN_CH;
  s_cross_max_off = TRACK_CROSS_MAX_OFFSET_MM;

  Track_ResetCrossPeak();
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
  float   base;
  float   floor_rpm;

  /* ---------- 1. 读灰度 ---------- */
  /* I2C 读取失败时保留上一拍控制状态。 */
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
      /* 对质心偏差进行一阶低通。 */
      s_offset    += TRACK_OFFSET_LPF * (offset - s_offset);
      s_lost       = 0;
      s_lost_ticks = 0;
    }
    else
    {
      /* 丢线后沿原偏差方向使用阵列外侧虚拟位置，以输出最大修正。 */
      s_lost = 1;
      s_offset = (s_offset >= 0.0f) ? TRACK_LOST_OFFSET_MM : -TRACK_LOST_OFFSET_MM;

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
  /* 目标偏差为 0；正偏差表示黑线位于车体右侧。 */
  steer = PID_Update(&s_steer_pid, 0.0f, s_offset);

  /* ---------- 5. 弯道减速 ---------- */
  /* 按偏差幅值降低基准转速。 */
  base = s_base_rpm - s_curve_slow * fabsf(s_offset);

  /* 基准速度为 0 时减速下限也必须为 0。 */
  floor_rpm = (s_base_rpm < TRACK_MIN_RPM) ? s_base_rpm : TRACK_MIN_RPM;
  if (base < floor_rpm)
  {
    base = floor_rpm;
  }

  s_target[0] = Track_ClampRpm(base - steer);
  s_target[1] = Track_ClampRpm(base + steer);
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

uint8_t Track_IsCrossLine(void)
{
  uint8_t hit;

  /* 横线的黑色通道必须同时分布在车体中线两侧。 */
  if ((s_dark_left == 0U) || (s_dark_right == 0U))
  {
    return 0;
  }

  /* 使用偏差门限排除姿态偏斜过大的情况。 */
  if (fabsf(s_offset) > s_cross_max_off)
  {
    return 0;
  }

  /* 判据一：黑色通道数达到门限。 */
  hit = (s_dark_count >= s_cross_min_ch);

  /* 判据二：总黑度达到门限。门限为 0 时关闭该判据。 */
  if ((TRACK_CROSS_SUM_TH > 0.0f) && (s_line_weight >= TRACK_CROSS_SUM_TH))
  {
    hit = 1;
  }

  return hit;
}

uint8_t Track_GetDarkCount(void)
{
  return s_dark_count;
}

float Track_GetLineWeight(void)
{
  return s_line_weight;
}

void Track_SetCrossGate(uint8_t min_ch, float max_offset_mm)
{
  s_cross_min_ch  = min_ch;
  s_cross_max_off = max_offset_mm;
}

void Track_ResetCrossPeak(void)
{
  /* 以当前测量值作为峰值起点，保留进入窗口当拍的数据。 */
  s_dark_peak   = s_dark_count;
  s_weight_peak = s_line_weight;
}

uint8_t Track_GetDarkPeak(void)
{
  return s_dark_peak;
}

float Track_GetWeightPeak(void)
{
  return s_weight_peak;
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

void Track_SetCurveSlowdown(float rpm_per_mm)
{
  s_curve_slow = rpm_per_mm;
}

void Track_SetSteerLimit(float rpm)
{
  PID_SetOutputLimits(&s_steer_pid, -rpm, rpm);
  PID_SetIntegralLimit(&s_steer_pid, rpm / 2.0f);
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
