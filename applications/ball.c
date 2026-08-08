/**
  ******************************************************************************
  * @file           : ball.c
  * @brief          : 钢球位置与速度串级控制
  ******************************************************************************
  * @note  控制器只在新视觉帧到达时更新。速度环积分补偿恒定阻力；位置环
  *        微分直接使用低通后的球速估计。
  ******************************************************************************
  */

#include "ball.h"
#include "vision.h"
#include "servo.h"
#include "pid.h"

#include <math.h>
#include <stddef.h>

/* ---------------- 运行时状态 ---------------- */

/* 外环：位置误差 -> 速度指令。内环：速度误差 -> 舵机倾角 */
static PID_Controller s_pos_pid;
static PID_Controller s_vel_pid;

/* 当前生效的参数组，可由任务层切换。 */
static Ball_Tune s_tune = BALL_TUNE_DEFAULT_INIT;

static float    s_target_cm  = BALL_TARGET_CM;
static float    s_pos_cm     = BALL_TARGET_CM;
static float    s_vel_cm_s   = 0.0f;    /* 滤波后的球速估计 */
static float    s_vel_set    = 0.0f;    /* 位置环下达的速度指令 */
static float    s_output_us  = 0.0f;    /* 速度环输出(相对水平点) */

static float    s_accel_ff   = 0.0f;    /* 小车当前加速度(rpm/秒)，任务层告知 */
static float    s_curve_ff   = 0.0f;    /* v_avg x 轮速差，用于过弯前馈 */

/* 纵向加速度前馈增益是机构参数，不随任务参数组重置。 */
static float    s_ff_gain    = BALL_FF_US_PER_RPMS;

static uint32_t s_last_frames  = 0;     /* 上次处理到第几帧 */
static uint32_t s_last_good_ms = 0;     /* 最近一次采纳帧的时刻 */
static uint8_t  s_enabled      = 0;
static uint8_t  s_tracking     = 0;
static uint8_t  s_have_prev    = 0;     /* 是否已有上一帧可用来求速度 */
static float    s_prev_pos_cm  = 0.0f;

/**
  * @brief  把两级 PID 的限幅写进控制器
  * @note   参数组变化后需要同步更新两个 PID 实例中的限幅。
  */
static void Ball_ApplyLimits(void)
{
  PID_SetOutputLimits(&s_pos_pid, -s_tune.vel_limit_cms, s_tune.vel_limit_cms);
  PID_SetIntegralLimit(&s_pos_pid, s_tune.pos_i_limit_cms);

  PID_SetOutputLimits(&s_vel_pid, -s_tune.out_limit_us, s_tune.out_limit_us);
  PID_SetIntegralLimit(&s_vel_pid, s_tune.vel_i_limit_us);
  PID_SetIntegralDeadband(&s_vel_pid, s_tune.vel_i_deadband_cms);
}

/**
  * @brief  杆放平，并清掉两级控制器的历史
  * @note   视觉断链或闭环关闭时，摆杆回到水平位置并清除控制器历史。
  */
static void Ball_GoLevel(void)
{
  Servo_SetPulseUs(SERVO_LEVEL_US);

  PID_Reset(&s_pos_pid);
  PID_Reset(&s_vel_pid);

  s_vel_set   = 0.0f;
  s_output_us = 0.0f;
  s_vel_cm_s  = 0.0f;
  s_have_prev = 0;
  s_tracking  = 0;
}

void Ball_Init(void)
{
  /* dt 在有效视觉帧到达时按实际帧间隔更新。 */
  PID_Init(&s_pos_pid, s_tune.pos_kp, s_tune.pos_ki, 0.0f, BALL_DT_MAX_S);
  PID_Init(&s_vel_pid, s_tune.vel_kp, s_tune.vel_ki, s_tune.vel_kd, BALL_DT_MAX_S);

  Ball_ApplyLimits();

  s_target_cm    = BALL_TARGET_CM;
  s_pos_cm       = BALL_TARGET_CM;
  s_last_frames  = Vision_GetFrameCount();
  s_last_good_ms = HAL_GetTick();
  s_enabled      = 0;

  Ball_GoLevel();
}

/**
  * @note   位置和速度估计始终更新；仅 PID 解算及舵机输出受使能状态控制。
  */
void Ball_Update(void)
{
  uint32_t now    = HAL_GetTick();
  uint32_t frames = Vision_GetFrameCount();

  /* ---------- 只在收到新帧时更新位置/速度估计 ---------- */
  if (frames != s_last_frames)
  {
    const Vision_Ball *b = Vision_GetBall();

    s_last_frames = frames;

    if ((b->valid != 0U) && (b->confidence >= BALL_MIN_CONFIDENCE))
    {
      float dt = (float)(now - s_last_good_ms) / 1000.0f;

      /* 限制异常帧间隔，避免速度估计突变 */
      if (dt < BALL_DT_MIN_S)
      {
        dt = BALL_DT_MIN_S;
      }
      else if (dt > BALL_DT_MAX_S)
      {
        dt = BALL_DT_MAX_S;
      }

      s_pos_cm       = b->x_cm;
      s_last_good_ms = now;

      /* 使用位置差分估算球速，不采用视觉端上传的像素速度。 */
      if (s_have_prev)
      {
        float raw_vel = (s_pos_cm - s_prev_pos_cm) / dt;

        s_vel_cm_s += BALL_VEL_LPF * (raw_vel - s_vel_cm_s);
      }
      else
      {
        s_vel_cm_s  = 0.0f;
        s_have_prev = 1;
      }
      s_prev_pos_cm = s_pos_cm;

      /* ---------- 闭环控制：只在使能时才算、才写舵机 ---------- */
      if (s_enabled)
      {
        float out;

        /* 外环微分项使用滤波球速，避免再次对原始位置做差分。 */
        PID_SetTunings(&s_pos_pid, s_tune.pos_kp, s_tune.pos_ki, 0.0f);
        s_pos_pid.dt = dt;
        s_vel_set = PID_Update(&s_pos_pid, s_target_cm, s_pos_cm)
                    - (s_tune.pos_kd * s_vel_cm_s);

        /* 手工加入微分项后重新执行目标球速限幅。 */
        if (s_vel_set > s_tune.vel_limit_cms)
        {
          s_vel_set = s_tune.vel_limit_cms;
        }
        else if (s_vel_set < -s_tune.vel_limit_cms)
        {
          s_vel_set = -s_tune.vel_limit_cms;
        }

        /* 按 v_cap = sqrt(2 * a * 剩余距离) 限制接近目标时的球速。 */
        if (s_tune.pos_brake_cms2 > 0.0f)
        {
          float dist  = fabsf(s_target_cm - s_pos_cm);
          float v_cap = sqrtf(2.0f * s_tune.pos_brake_cms2 * dist);

          if (s_vel_set > v_cap)
          {
            s_vel_set = v_cap;
          }
          else if (s_vel_set < -v_cap)
          {
            s_vel_set = -v_cap;
          }
        }

        /* 速度环积分用于补偿摩擦、摆杆坡度和水平点误差。 */
        PID_SetTunings(&s_vel_pid, s_tune.vel_kp, s_tune.vel_ki, s_tune.vel_kd);
        s_vel_pid.dt = dt;
        s_output_us = PID_Update(&s_vel_pid, s_vel_set, s_vel_cm_s);

        /* 叠加车辆运动前馈。 */
        out = s_output_us;

#if BALL_FF_ENABLE
        /* 正向加速时使用负向补偿；s_output_us 只保留反馈控制器输出。 */
        out -= s_ff_gain * s_accel_ff;
#endif
#if BALL_FF_CURVE_ENABLE
        out -= BALL_FF_CURVE_GAIN * s_curve_ff;
#endif

        Servo_SetPulseUs((uint16_t)(SERVO_LEVEL_US +
                                    (int16_t)(BALL_OUTPUT_SIGN * out)));
        s_tracking = 1;
      }
    }
  }

  /* 闭环使能期间超过超时阈值未收到有效帧，摆杆回平。 */
  if (s_enabled && ((now - s_last_good_ms) > BALL_TIMEOUT_MS))
  {
    Ball_GoLevel();
  }
}

void Ball_SetTune(const Ball_Tune *tune)
{
  if (tune == NULL)
  {
    return;
  }

  s_tune = *tune;
  Ball_ApplyLimits();
}

void Ball_ResetTune(void)
{
  const Ball_Tune def = BALL_TUNE_DEFAULT_INIT;

  Ball_SetTune(&def);
}

Ball_Tune Ball_GetTune(void)
{
  return s_tune;
}

void Ball_Enable(uint8_t on)
{
  if (on && !s_enabled)
  {
    /* 重新使能时清除上一轮控制历史。 */
    s_last_frames  = Vision_GetFrameCount();
    s_last_good_ms = HAL_GetTick();
    Ball_GoLevel();
  }

  s_enabled = (on != 0U);

  if (!s_enabled)
  {
    Ball_GoLevel();
  }
}

uint8_t Ball_IsEnabled(void)
{
  return s_enabled;
}

void Ball_SetTarget(float cm)
{
  s_target_cm = cm;
}

float Ball_GetTarget(void)
{
  return s_target_cm;
}

void Ball_SetAccelFF(float rpm_per_s)
{
  s_accel_ff = rpm_per_s;
}

void Ball_SetCurveFF(float v_times_diff)
{
  s_curve_ff = v_times_diff;
}

void Ball_SetAccelFFGain(float us_per_rpms)
{
  s_ff_gain = us_per_rpms;
}

float Ball_GetAccelFFGain(void)
{
  return s_ff_gain;
}

float Ball_GetPosCm(void)
{
  return s_pos_cm;
}

float Ball_GetVelCmS(void)
{
  return s_vel_cm_s;
}

float Ball_GetVelSetCmS(void)
{
  return s_vel_set;
}

float Ball_GetOutputUs(void)
{
  return s_output_us;
}

uint8_t Ball_IsTracking(void)
{
  return s_tracking;
}
