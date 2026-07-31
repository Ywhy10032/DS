/**
  ******************************************************************************
  * @file           : ball.c
  * @brief          : 球杆闭环 —— 视觉测位置，舵机调倾角
  *
  * ★ 控制律待重写 ★
  * 本文件目前只有【框架】：取帧、算 dt、估速度、掉帧保护、使能管理、诊断接口。
  * 真正的控制律留空（见 Ball_Update() 里的 "控制律" 段落），
  * 此时 s_output_us 恒为 0，杆保持水平 —— 这是安全的初始状态。
  *
  * 重写前先读 ball_notes.md。框架部分的六条不变量见该文件第六节，
  * 那些是踩出来的，改动前请确认理由。
  ******************************************************************************
  */

#include "ball.h"
#include "vision.h"
#include "servo.h"
#include "pid.h"

#include <math.h>

/* ---------------- 运行时状态 ---------------- */
static PID_Controller s_pid;

static float    s_target_cm   = BALL_TARGET_CM;
static float    s_pos_cm      = BALL_TARGET_CM;
static float    s_vel_cm_s    = 0.0f;
static float    s_output_us   = 0.0f;   /* 反馈控制器的输出，相对水平点的 us 偏移 */
static float    s_accel_ff    = 0.0f;   /* 小车当前加速度(rpm/秒)，由任务层告知 */
static float    s_curve_ff    = 0.0f;   /* v_avg x 轮速差，用于过弯前馈 */

static uint32_t s_last_frames  = 0;     /* 上次处理到第几帧 */
static uint32_t s_last_good_ms = 0;     /* 最近一次采纳帧的时刻 */
static uint8_t  s_enabled      = 0;
static uint8_t  s_tracking     = 0;
static uint8_t  s_have_prev    = 0;     /* 是否已有上一帧可用来求速度 */
static float    s_prev_pos_cm  = 0.0f;

/**
  * @brief  杆放平，并清掉控制器的历史
  * @note   视觉断链时【必须】回平，不能保持上一次的输出 —— 斜着的杆会让球
  *         一直加速，几百毫秒就冲出去了。放平至少让它匀速滑行。
  */
static void Ball_GoLevel(void)
{
  Servo_SetPulseUs(SERVO_LEVEL_US);
  PID_Reset(&s_pid);

  s_output_us = 0.0f;
  s_vel_cm_s  = 0.0f;
  s_have_prev = 0;
  s_tracking  = 0;
}

void Ball_Init(void)
{
  PID_Init(&s_pid, BALL_KP, BALL_KI, BALL_KD, BALL_DT_MAX_S);
  PID_SetOutputLimits(&s_pid, -BALL_OUTPUT_LIMIT_US, BALL_OUTPUT_LIMIT_US);
  PID_SetIntegralLimit(&s_pid, BALL_INTEGRAL_LIMIT_US);

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
  /* 不按固定周期跑：帧是异步来的，两帧之间位置根本没变，照样跑一遍的话
     微分项会在"没变"的拍上读到 0、在"变了"的拍上读到一个尖峰，
     等于给控制器喂了一串噪声 */
  frames = Vision_GetFrameCount();
  if (frames != s_last_frames)
  {
    const Vision_Ball *b = Vision_GetBall();

    s_last_frames = frames;

    if ((b->valid != 0U) && (b->confidence >= BALL_MIN_CONFIDENCE))
    {
      float dt = (float)(now - s_last_good_ms) / 1000.0f;

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
      /* 速度自己按 x_cm 求差分，而不是用视觉发来的 vx_pixel_s：
         后者单位是像素/秒，与 cm 之间的比例、正负方向都取决于摄像头安装，
         用错方向的微分等于把阻尼变成正反馈。等标定出比例和符号之后
         再换成它会更平滑(源头是滤波器输出，噪声小)。 */
      if (s_have_prev)
      {
        /* 差分把位置噪声放大了 1/dt(约 30) 倍，必须低通一下再用 */
        float raw_vel = (s_pos_cm - s_prev_pos_cm) / dt;

        s_vel_cm_s += BALL_VEL_LPF * (raw_vel - s_vel_cm_s);
      }
      else
      {
        s_vel_cm_s  = 0.0f;
        s_have_prev = 1;
      }
      s_prev_pos_cm = s_pos_cm;

      /* ================================================================
         控制律 —— 待重写

         可用的输入：
           s_target_cm  目标位置(cm)
           s_pos_cm     当前位置(cm)，噪声约 0.1cm
           s_vel_cm_s   已低通的球速(cm/s)，噪声约 1.3cm/s，滞后约 75ms
           dt           本帧的时间间隔(秒)，已夹在 BALL_DT_* 之间

         要求写出的输出：
           s_output_us  相对 SERVO_LEVEL_US 的脉宽偏移，
                        必须自己夹在 ±BALL_OUTPUT_LIMIT_US 内

         用现成的 PID 模块的话（微分作用在测量值上，正好就是"按球速阻尼"）：
           s_pid.dt    = dt;    // dt 每帧都在变，用固定 dt 会让微分项
                                // 的幅度随帧率漂移，所以每帧都要更新
           s_output_us = PID_Update(&s_pid, s_target_cm, s_pos_cm);

         整定顺序和各机制的坑见 ball_notes.md。
         现在恒为 0 —— 杆保持水平，舵机不动，是安全的初始状态。
         ================================================================ */
      s_output_us = 0.0f;

      /* ---------- 车体运动前馈 ---------- */
      {
        float out = s_output_us;

#if BALL_FF_ENABLE
        /* 加速度前馈。车往前加速时球相对摆杆向【后】滑(x 增大)，
           所以要往 x 减小的方向预先倾杆，符号取负。
           前馈不进 s_output_us，那个值留给显示，代表反馈控制器的意图 */
        out -= BALL_FF_US_PER_RPMS * s_accel_ff;
#endif
#if BALL_FF_CURVE_ENABLE
        /* 过弯前馈，同理 —— 向心加速度也是我们自己造出来的，可以提前抵消 */
        out -= BALL_FF_CURVE_GAIN * s_curve_ff;
#endif

        /* Servo_SetPulseUs() 内部夹在 SERVO_SAFE_* 里，是唯一的保护手段 */
        Servo_SetPulseUs((uint16_t)(SERVO_LEVEL_US +
                                    (int16_t)(BALL_OUTPUT_SIGN * out)));
      }
      s_tracking = 1;
    }
  }

  /* ---------- 掉帧保护 ---------- */
  if ((now - s_last_good_ms) > BALL_TIMEOUT_MS)
  {
    Ball_GoLevel();
  }
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

void Ball_SetAccelFF(float rpm_per_s)
{
  s_accel_ff = rpm_per_s;
}

void Ball_SetCurveFF(float v_times_diff)
{
  s_curve_ff = v_times_diff;
}

float Ball_GetTarget(void)
{
  return s_target_cm;
}

float Ball_GetPosCm(void)
{
  return s_pos_cm;
}

float Ball_GetVelCmS(void)
{
  return s_vel_cm_s;
}

float Ball_GetOutputUs(void)
{
  return s_output_us;
}

uint8_t Ball_IsTracking(void)
{
  return s_tracking;
}
