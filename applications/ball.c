/**
  ******************************************************************************
  * @file           : ball.c
  * @brief          : 球杆闭环 —— 视觉测位置，PD 控制舵机倾角
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
static float    s_output_us   = 0.0f;
static float    s_accel_ff    = 0.0f;   /* 小车当前加速度(rpm/秒)，由任务层告知 */
static float    s_curve_ff    = 0.0f;   /* v_avg x 轮速差，用于过弯前馈 */
static float    s_stiction_us = 0.0f;   /* 静摩擦补偿的当前爬升值 */
static uint8_t  s_stick_armed = 0;      /* 补偿是否已武装(球停稳且仍有偏差) */
#if BALL_STICTION_ENABLE
static float    s_stick_pos0  = 0.0f;   /* 武装那一刻的球位置，用来判断是否起步 */
#endif

static uint32_t s_last_frames = 0;      /* 上次处理到第几帧 */
static uint32_t s_last_good_ms = 0;     /* 最近一次采纳帧的时刻 */
static uint8_t  s_enabled     = 0;
static uint8_t  s_tracking    = 0;
static uint8_t  s_have_prev   = 0;      /* 是否已有上一帧可用来求速度 */
static float    s_prev_pos_cm = 0.0f;

/**
  * @brief  杆放平，并清掉控制器的历史
  * @note   视觉断链时【必须】回平，不能保持上一次的输出 —— 斜着的杆会让球
  *         一直加速，几百毫秒就冲出去了。放平至少让它匀速滑行。
  */
static void Ball_GoLevel(void)
{
  Servo_SetPulseUs(SERVO_LEVEL_US);
  PID_Reset(&s_pid);

  s_output_us   = 0.0f;
  s_stiction_us = 0.0f;
  s_stick_armed = 0;
  s_vel_cm_s    = 0.0f;
  s_have_prev   = 0;
  s_tracking    = 0;
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

      /* 偏差落进细调区没有？增益、摩擦前馈、补偿爬升速率【三样都】各有一套，
         统一用这一个判据切换：赶路段要冲劲，细调段要每一脚都轻，
         否则一脚就把球顶过目标 */
#if (BALL_GAIN_SCHEDULE || BALL_STICTION_ENABLE || BALL_FRICTION_FF_ENABLE)
      uint8_t in_fine = (fabsf(s_target_cm - s_pos_cm) <= BALL_COARSE_ERR_CM);
#endif

#if BALL_GAIN_SCHEDULE
      /* 偏差大就换激进参数直接顶过静摩擦门槛，进细调区再切回温柔的那套。
         限幅不动 —— 它同时管着刹车权限，中途缩水会导致冲过头 */
      if (!in_fine)
      {
        PID_SetTunings(&s_pid, BALL_COARSE_KP, 0.0f, BALL_COARSE_KD);
      }
      else
      {
        PID_SetTunings(&s_pid, BALL_KP, BALL_KI, BALL_KD);
      }
#endif

      /* PID 内部的微分作用在【测量值】上，正好就是我们要的"按球速阻尼"。
         dt 每帧都在变，所以每次都要更新 —— 用固定 dt 会让微分项的
         幅度随帧率漂移 */
      s_pid.dt    = dt;
      s_output_us = PID_Update(&s_pid, s_target_cm, s_pos_cm);

#if BALL_FRICTION_FF_ENABLE
      /* ---------- 摩擦前馈 ---------- */
      /* 把库仑摩擦从对象里"减掉"，PID 的输出就重新代表净推力，
         不必再自己先挣出 160us 才开始干活。少了这一项，球必然停在
         偏差 = 160/Kp 处开始爬行 —— 详见 ball.h */
      {
        float ff_err = s_target_cm - s_pos_cm;

        /* 死区内不补：那是"已经到位、彻底撒手"的区域，
           在这里叠推力就会把球顶出去，形成目标两侧的摆荡 */
        if (fabsf(ff_err) > BALL_STICTION_ERR_CM)
        {
          /* 球跑起来就线性退出。补偿的职责只有帮球挣脱，
             球在动时 D 项已经主导，再同向叠推力是跟刹车对着干 */
          float fade = 1.0f - fabsf(s_vel_cm_s) / BALL_FRICTION_FF_FADE_CMS;

          /* 细调区用更小的前馈：挣脱之后补偿撤销、前馈却留着，粗调区那
             130us 加上比例项高于动摩擦，球会继续加速 —— 赶路段正需要，
             到了目标附近就成了"微调幅度太大" */
          float ff_us = in_fine ? BALL_FINE_FRICTION_FF_US : BALL_FRICTION_FF_US;

          if (fade > 0.0f)
          {
            s_output_us += (ff_err > 0.0f) ? (ff_us * fade)
                                           : (-ff_us * fade);

            /* 叠加之后要重新限幅：PID_Update() 内部的限幅管不到这一项 */
            if (s_output_us > BALL_OUTPUT_LIMIT_US)
            {
              s_output_us = BALL_OUTPUT_LIMIT_US;
            }
            else if (s_output_us < -BALL_OUTPUT_LIMIT_US)
            {
              s_output_us = -BALL_OUTPUT_LIMIT_US;
            }
          }
        }
      }
#endif

#if BALL_STICTION_ENABLE
      /* 静摩擦补偿：球几乎静止却仍有偏差时，逐步加大倾角直到它起步。
         只在静止时介入 —— 球一动起来就归零、交还给正常 PID，
         所以不会在目标附近反复推 */
      {
        float err = s_target_cm - s_pos_cm;

        if (fabsf(err) <= BALL_STICTION_ERR_CM)
        {
          /* 已经到位，不折腾 */
          s_stick_armed = 0;
          s_stiction_us = 0.0f;
        }
        else if (!s_stick_armed)
        {
          /* closing > 0 表示球正在朝目标靠近(偏差在缩小)。
             err>0 时目标在 x 增大方向，此时 vel>0 就是在靠近；err<0 反之 */
          float closing = (err > 0.0f) ? s_vel_cm_s : -s_vel_cm_s;

          /* 两道判据缺一不可：
             1) 速度够低 —— 球真的停了才谈得上"卡住"
             2) 没在朝目标滑 —— 补偿的职责是"卡住了推一把"，
                不是给正在滑向目标的球加油

             只有第 1 条会形成【棘轮】：撤销补偿的判据是"挪动 0.15cm"，
             而挪完这 0.15cm 球速通常只有 2~3cm/s，仍低于 4cm/s 的门槛，
             于是立刻重新武装、又给一次预载，如此反复 —— 等效于全程踩着
             油门把球一路推过目标，表现就是在目标两侧来回摆荡。
             门槛本身没法再往下压：它必须高于速度估计的噪声底(约 1.3cm/s)。
             所以只能靠第 2 条从方向上把这个循环断掉。 */
          if ((fabsf(s_vel_cm_s) < BALL_STICTION_VEL_CMS) &&
              (closing < BALL_STICTION_APPROACH_CMS))
          {
            s_stick_armed = 1;
            s_stick_pos0  = s_pos_cm;

            /* 从预载值起爬而不是从 0：0~预载 这一段球必然不动，爬过它纯属
               干等，而这段死时间正是最后 1cm 走得慢的原因。
               "慢速逼近真实门槛"的性质由剩下那一段保留着。
               预载是绝对值，不跟着天花板 BALL_STICTION_US 走 —— 两者的
               取值方向相反，理由见 ball.h */
            s_stiction_us = BALL_STICTION_PRELOAD_US;
          }
        }
        else if (fabsf(s_pos_cm - s_stick_pos0) > BALL_STICTION_MOVE_CM)
        {
          /* 球挣脱了。立刻(而不是缓降)撤掉大倾角，交还给 PID —— 慢一步就是过冲 */
          s_stick_armed = 0;
          s_stiction_us = 0.0f;
        }
        else
        {
          /* 细调区爬得更慢：爬升速率决定挣脱那一刻的倾角比真实门槛高出多少
             (每帧涨 速率 x dt)，那点超出量就是"一脚踢多远"的主要来源 */
          s_stiction_us += (in_fine ? BALL_FINE_RAMP_UPS
                                    : BALL_STICTION_RAMP_UPS) * dt;
          if (s_stiction_us > BALL_STICTION_US)
          {
            s_stiction_us = BALL_STICTION_US;
          }

          /* 只在 PID 自己给不出这么大幅度时才顶上去 */
          if (fabsf(s_output_us) < s_stiction_us)
          {
            /* 用偏差的符号而不是输出的符号：球静止时微分项为 0，
               输出可能因为积分尚未累积而接近 0，符号不可靠 */
            s_output_us = (err > 0.0f) ? s_stiction_us : -s_stiction_us;
          }
        }
      }
#endif

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

/**
  ******************************************************************************
  * 整定步骤
  *
  * 【第 0 步：确认符号，这一步做错后面全白搭】
  *   把 BALL_KP、BALL_KI、BALL_KD 全设为 0，使能闭环。此时输出恒为 0、
  *   杆保持水平，舵机不会动，是安全的。
  *   然后只把 BALL_KP 设成一个很小的值(比如 2)，用手把球放在【大于 12.5cm】
  *   的一侧，观察杆往哪边倾：
  *     - 杆倾斜的方向让球【滚回中心】 -> 符号对
  *     - 球被推得更远 -> 把 BALL_OUTPUT_SIGN 改成 -1
  *   符号错了是正反馈，球会瞬间冲到杆的一端，别在高增益下试。
  *
  * 【第 1 步：先加 Kd，不是先加 Kp】
  *   这和电机速度环的习惯相反。双积分器没有任何自然阻尼，Kp 单独作用时
  *   系统是临界稳定的 —— 球会绕着中点等幅振荡，永远停不下来，
  *   这时候再怎么调 Kp 都没用。
  *   保持 Kp=10，把 Kd 从 5 开始往上加。判据是：用手把球推开后松手，
  *   球回中的过程【不再来回振荡】、最多过冲一次就停住。
  *
  * 【第 2 步：再调 Kp】
  *   Kd 够了之后 Kp 决定回中的快慢。
  *     太小 -> 球慢吞吞往回挪，或者停在偏离中心的地方不动
  *     太大 -> 又开始振荡(这时要继续加 Kd，两者是配套往上走的)
  *
  * 【第 3 步：Ki 通常不需要】
  *   只有当球总是稳定停在偏离目标几毫米的固定位置时才加，说明 SERVO_LEVEL_US
  *   不是真正的水平点。更好的做法是直接微调 SERVO_LEVEL_US，
  *   比用积分去补要干净 —— 积分在这种不稳定对象上很容易攒过头。
  *   真要加就从 0.5 开始，并注意 BALL_INTEGRAL_LIMIT_US 已经限制在 40us。
  *
  * 【关于噪声】
  *   Kd 直接放大视觉位置的抖动。如果舵机出现高频颤动而球其实是静止的，
  *   说明 Kd 对当前的视觉噪声来说太大了，要么降 Kd，要么让视觉端把
  *   position_filter 的滤波调狠一点 —— 后者更治本，因为降 Kd 会牺牲阻尼。
  ******************************************************************************
  */
