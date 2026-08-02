/**
  ******************************************************************************
  * @file           : ball.c
  * @brief          : 球杆闭环 —— 串级：位置环 -> 速度环 -> 舵机倾角
  ******************************************************************************
  * @note  结构与整定顺序见 ball.h 顶部。这里只强调实现上的三个要点：
  *
  *        1. 两级都只在【收到新视觉帧】时推进。视觉是唯一的反馈来源，
  *           两帧之间位置没有新信息，照常跑一遍只会让微分项在"没变"的拍上
  *           读到 0、在"变了"的拍上读到尖峰，等于给控制器喂噪声。
  *
  *        2. 速度环的【积分】是这一版的核心。摩擦、管子下垂、水平点残差
  *           这些恒定阻力全部由它自动累积补偿 —— 上一版那套摩擦前馈、
  *           静摩擦补偿状态机、下垂前馈、瞄准偏置因此全部删除。
  *
  *        3. 位置环的微分【不用】PID 内部的差分，而是直接用本文件里滤波过的
  *           速度估计。理由见 Ball_Update() 里的注释。
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

/* 当前生效的参数组，任务层用 Ball_SetTune() 切换 */
static Ball_Tune s_tune = BALL_TUNE_DEFAULT_INIT;

static float    s_target_cm  = BALL_TARGET_CM;
static float    s_pos_cm     = BALL_TARGET_CM;
static float    s_vel_cm_s   = 0.0f;    /* 滤波后的球速估计 */
static float    s_vel_set    = 0.0f;    /* 位置环下达的速度指令 */
static float    s_output_us  = 0.0f;    /* 速度环输出(相对水平点) */

static float    s_accel_ff   = 0.0f;    /* 小车当前加速度(rpm/秒)，任务层告知 */
static float    s_curve_ff   = 0.0f;    /* v_avg x 轮速差，用于过弯前馈 */

static uint32_t s_last_frames  = 0;     /* 上次处理到第几帧 */
static uint32_t s_last_good_ms = 0;     /* 最近一次采纳帧的时刻 */
static uint8_t  s_enabled      = 0;
static uint8_t  s_tracking     = 0;
static uint8_t  s_have_prev    = 0;     /* 是否已有上一帧可用来求速度 */
static float    s_prev_pos_cm  = 0.0f;

/**
  * @brief  把两级 PID 的限幅写进控制器
  * @note   增益每帧都会重设，但限幅存在 PID 内部，改参数组时必须同步。
  *         位置环的输出就是速度指令，所以它的输出限幅 = vel_limit_cms。
  */
static void Ball_ApplyLimits(void)
{
  PID_SetOutputLimits(&s_pos_pid, -s_tune.vel_limit_cms, s_tune.vel_limit_cms);
  PID_SetIntegralLimit(&s_pos_pid, s_tune.pos_i_limit_cms);

  PID_SetOutputLimits(&s_vel_pid, -s_tune.out_limit_us, s_tune.out_limit_us);
  PID_SetIntegralLimit(&s_vel_pid, s_tune.vel_i_limit_us);
}

/**
  * @brief  杆放平，并清掉两级控制器的历史
  * @note   视觉断链时【必须】回平，不能保持上一次的输出 —— 斜着的杆会让球
  *         一直加速，几百毫秒就冲出去了。放平至少让它匀速滑行。
  *         两级的积分都要清：速度环的积分可能正憋着几百 us 去顶摩擦，
  *         留着它等于断链后还按住油门。
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
  /* dt 每帧都会按实际帧间隔更新，这里给的初值只是占位 */
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

void Ball_Update(void)
{
  uint32_t now = HAL_GetTick();
  uint32_t frames;

  if (!s_enabled)
  {
    return;
  }

  /* ---------- 只在收到新帧时跑控制 ---------- */
  frames = Vision_GetFrameCount();
  if (frames != s_last_frames)
  {
    const Vision_Ball *b = Vision_GetBall();

    s_last_frames = frames;

    if ((b->valid != 0U) && (b->confidence >= BALL_MIN_CONFIDENCE))
    {
      float dt = (float)(now - s_last_good_ms) / 1000.0f;
      float out;

      /* 视觉端卡顿或刚上电时 dt 会异常，不夹住微分项会算出天文数字 */
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

      /* ---------- 速度估计 ---------- */
      /* 自己按 x_cm 差分，不用视觉发来的 vx_pixel_s —— 后者由 alpha-beta
         滤波器的 beta 支路给出，而 beta 在球接近静止时降到 0.01，
         时间常数长达约 1.7 秒，完全没法喂给速度环。详见 ball.h */
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

      /* ---------- 外环：位置误差 -> 速度指令 ---------- */
      /* Kd 传 0，微分项在下面手工加 —— PID 内部是对测量值做原始差分，
         而位置的原始差分正是噪声最大的那个量；我们手上已经有滤波过的
         速度估计 s_vel_cm_s，直接用它做微分项，信号质量好得多。
         位置环微分 = -Kd x d(位置)/dt，而 d(位置)/dt 就是球速，
         所以这一项就是 -pos_kd * s_vel_cm_s。 */
      PID_SetTunings(&s_pos_pid, s_tune.pos_kp, s_tune.pos_ki, 0.0f);
      s_pos_pid.dt = dt;
      s_vel_set = PID_Update(&s_pos_pid, s_target_cm, s_pos_cm)
                  - (s_tune.pos_kd * s_vel_cm_s);

      /* 手工加完微分要重新限幅：PID_Update() 内部那次限幅管不到这一项。
         这个限幅同时是串级的带宽闸门 —— 外环不许要求内环做到它做不到的
         速度，见 ball.h 的 BALL_VEL_LIMIT_CMS */
      if (s_vel_set > s_tune.vel_limit_cms)
      {
        s_vel_set = s_tune.vel_limit_cms;
      }
      else if (s_vel_set < -s_tune.vel_limit_cms)
      {
        s_vel_set = -s_tune.vel_limit_cms;
      }

      /* ---------- 内环：速度误差 -> 舵机倾角 ---------- */
      /* 这一级的积分项承担了上一版整套外挂补偿的职责：摩擦、管子下垂、
         水平点残差造成的恒定阻力，全部由它自动累积出对应的常驻倾角 */
      PID_SetTunings(&s_vel_pid, s_tune.vel_kp, s_tune.vel_ki, s_tune.vel_kd);
      s_vel_pid.dt = dt;
      s_output_us = PID_Update(&s_vel_pid, s_vel_set, s_vel_cm_s);

      /* ---------- 小车运动的前馈 ---------- */
      out = s_output_us;

#if BALL_FF_ENABLE
      /* 车往前加速时球相对摆杆向【后】滑(x 增大)，所以要往 x 减小的方向
         预先倾杆，符号取负。前馈不进 s_output_us —— 那个值留给显示，
         代表反馈控制器自己的意图 */
      out -= BALL_FF_US_PER_RPMS * s_accel_ff;
#endif
#if BALL_FF_CURVE_ENABLE
      out -= BALL_FF_CURVE_GAIN * s_curve_ff;
#endif

      Servo_SetPulseUs((uint16_t)(SERVO_LEVEL_US +
                                  (int16_t)(BALL_OUTPUT_SIGN * out)));
      s_tracking = 1;
    }
  }

  /* ---------- 掉帧保护 ---------- */
  if ((now - s_last_good_ms) > BALL_TIMEOUT_MS)
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
    /* 重新使能时把历史清干净，否则会带着上一轮的积分猛推一下 */
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
