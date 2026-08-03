/**
  ******************************************************************************
  * @file           : tracking.c
  * @brief          : 循迹外环 —— 加权质心求偏差，转向 PID 输出左右差速
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
static uint8_t  s_dark_left   = 0;         /* 其中车体中线【左边】有几路 */
static uint8_t  s_dark_right  = 0;         /* 中线【右边】有几路 */
static float    s_line_weight = 0.0f;      /* 八路权重总和(总黑度) */

/* 横线判据的当前门槛。默认是严判据，终点窗口里由任务层放宽 */
static uint8_t  s_cross_min_ch   = TRACK_CROSS_MIN_CH;
static float    s_cross_max_off  = TRACK_CROSS_MAX_OFFSET_MM;

/* 峰值保持。车过 A 点只有几拍，实时值根本看不清，靠这两个数事后复盘 */
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
  * @note   归一化后白场 255、黑场 0。白底黑线时权重取 255-读数，读数越黑
  *         权重越大；黑底白线则反过来直接用读数。
  *         低于噪声阈值的通道整个清零 —— 白场读数不会正好是极值，不掐掉
  *         的话八路的零星波动会把质心往中间拽偏。
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
      /* 顺手统计有多少路是黑的、分布在中线哪一侧，两个都用于横线判定 */
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

  /* 峰值保持：横线只被扫过几拍，实时值肉眼看不住，留下最大值供事后复盘 */
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

  /* 横线判据恢复默认的严门槛。放宽是任务层在终点窗口里临时干的事，
     每次启动都必须先收回来，否则上一趟放宽的门槛会带到下一趟的全程 */
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
      /* 一阶低通，磨掉宽线造成的死区跳变(见 tracking.h 里 TRACK_OFFSET_LPF 的说明) */
      s_offset    += TRACK_OFFSET_LPF * (offset - s_offset);
      s_lost       = 0;
      s_lost_ticks = 0;
    }
    else
    {
      /* 丢线 = 线已经跑出探头阵列。把偏差钉到阵列边缘之外、方向沿用丢线前，
         让转向环直接给出最大修正。
         注意不能只是"保持上一次的偏差"：线是从边缘滑出去的，滑出瞬间那个
         偏差往往还不到满量程，照着它修正力度远远不够，车会几乎直着冲出弯道 */
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
  /* 目标偏差恒为 0(线压在车体中线上)。offset > 0 表示线在右侧，
     PID 输出为负，于是左轮 base-steer 变快、右轮 base+steer 变慢，
     车头向右修正 —— 正好把线拉回中间 */
  steer = PID_Update(&s_steer_pid, 0.0f, s_offset);

  /* ---------- 5. 弯道减速 ---------- */
  /* 偏差越大弯越急，按比例压低基准速度。转向力度有物理上限，速度高到一定
     程度就只能靠减速来换转向半径 —— 这是高速循迹能过弯的关键 */
  base = s_base_rpm - s_curve_slow * fabsf(s_offset);

  /* 减速下限。注意要跟着 s_base_rpm 走：外部把基准设成 0(停车)时，
     下限也必须是 0，否则这里反而会把车重新推起来 */
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

  /* ---------- 一票否决：黑簇必须【跨过车体中线】 ----------
     这是把 A 点横线和弯道斜穿区分开的关键，而且是几何上必然成立的：
     横线骑在纵线上，车又在跟着纵线走，所以它盖住的那一片必然横跨中线；
     而弯道里线是甩到一侧去的，黑簇整片偏在左边或右边。

     实测(停车瞬间那一拍的原始数据，权重 = 255-读数，门限 120)：
       A 点横线    ch1/ch2/ch3 在左 + ch4 在右   -> 跨中线  ✓
       弯道误判 1  ch5/ch6/ch7 全在右侧          -> 不跨    ✗
       弯道误判 2  ch4/ch5/ch6 全在右侧          -> 不跨    ✗
     光看"几路黑"是分不开的(弯道也能凑到 3 路)，看"在哪一侧"一分就开。 */
  if ((s_dark_left == 0U) || (s_dark_right == 0U))
  {
    return 0;
  }

  /* 再卡姿态：压在横线上时质心不会太远，而急弯里线明显偏向一侧 */
  if (fabsf(s_offset) > s_cross_max_off)
  {
    return 0;
  }

  /* 判据一：够多路同时黑。姿态正的时候最干净 */
  hit = (s_dark_count >= s_cross_min_ch);

  /* 判据二：总黑度够大。车带着横摆角压上横线时，各路是先后进入胶带的，
     同一拍里数不满路数，但"半黑"的探头仍然按比例贡献权重，总和照样上得去
     —— 这条是给出弯口那种歪着过 A 的姿态兜底的(见 tracking.h 的说明) */
  /* 阈值填 0 就是关掉这条判据。写成运行时判断而不是 #if —— 预处理器不认
     浮点比较，编译器自己会把这个常量条件折叠掉，不占运行开销 */
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
  /* 用【本拍的实测值】做起点而不是清 0：任务层是在进终点窗口那一拍调用本
     函数的，而那一拍的灰度数据在此之前就已经算完了(app.c 里 Track_Update()
     排在 Task_Update() 前面)。清 0 的话，万一就在这一拍判到终点停了车，
     屏幕上的峰值会显示成 0，反而看不到当时到底是什么数据触发的 */
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
