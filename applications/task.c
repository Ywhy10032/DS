/**
  ******************************************************************************
  * @file           : task.c
  * @brief          : 比赛任务状态机、里程计时与车辆运动规划
  ******************************************************************************
  */

#include "task.h"
#include "key.h"
#include "tracking.h"
#include "encoder.h"
#include "motor.h"
#include "ball.h"
#include "servo.h"

#include <math.h>
#include <stddef.h>

#define TASK_PI                 3.14159265f

/* 一圈的行驶距离(mm) = 轮子周长 */
#define TASK_WHEEL_CIRC_MM      (TASK_PI * TASK_WHEEL_DIAMETER_MM)

/* ---------------- 运行时状态 ---------------- */
static Task_ID    s_task      = TASK_2;      /* 开机默认停在任务二 */
static Task_State s_state     = TASK_STATE_IDLE;

static uint32_t s_start_tick   = 0;
static uint32_t s_elapsed_ms   = 0;          /* 评分计时；任务五/六过 A 后立即定格 */
static int32_t  s_start_count  = 0;          /* 启动时的编码器基准 */
static float    s_distance_m   = 0.0f;
static float    s_ramp_rpm     = 0.0f;       /* 载球任务的速度斜坡当前值 */
static float    s_ramp_accel   = 0.0f;       /* 本拍施加的加速度(rpm/秒)，带符号。
                                                它自己也走斜坡(限 jerk)，见
                                                Task_RampBase() 与 task.h 的
                                                TASK_ACCEL_RISE_S */
static uint32_t s_split_ms     = 0;          /* 到达评分点的用时，0 = 还没到 */
static uint8_t  s_lap_done     = 0;          /* 任务五/六：整圈已跑完 */
static uint32_t s_lap_done_ms  = 0;          /* 通过 A 的时刻，用来算滑行时长 */
static uint8_t  s_cross_armed  = 0;          /* 已进入终点窗口、横线判据已放宽 */

/* 任务三运行阶段。减速由球控模块的剩余距离限速完成。 */
typedef enum
{
  TASK3_GO_PLUS = 0,      /* 目标 +5cm */
  TASK3_GO_MINUS,         /* 到过 +5cm 了，折返，目标 -5cm */
  TASK3_SETTLE            /* 够到 -5cm 了，等它稳住 */
} Task3_Phase;

static Task3_Phase s_t3_phase    = TASK3_GO_PLUS;
static uint32_t    s_t3_settle_t0 = 0;       /* 进入容差圈的时刻，用来算稳定时长 */

/* 任务一开始前的球控使能状态，结束后恢复。 */
static uint8_t s_t1_ball_en = 0;

/**
  * @brief  取左右轮的平均累计计数
  * @note   Encoder_GetCount() 与速度环的 Encoder_GetRPM() 各自维护标记，
  *         同时调用互不干扰
  */
static int32_t Task_AvgCount(void)
{
  int32_t left  = Encoder_GetCount(ENCODER_LEFT);
  int32_t right = Encoder_GetCount(ENCODER_RIGHT);

  return (left + right) / 2;
}

/**
  * @brief  刷新本次任务已行驶的距离
  */
static void Task_UpdateOdometry(void)
{
  s_distance_m = (float)(Task_AvgCount() - s_start_count)
                 / (float)ENCODER_COUNTS_PER_REV
                 * TASK_WHEEL_CIRC_MM / 1000.0f;
}

/**
  * @brief  把基准速度按限定的加减速率往目标值靠
  * @note   速度和加速度均受限，形成 S 形速度曲线。当前加速度同时传给
  *         球控模块，供纵向加速度前馈使用。
  */
static void Task_RampBase(float target_rpm, float accel, float decel)
{
  float gap   = target_rpm - s_ramp_rpm;
  float a_lim = (gap >= 0.0f) ? accel : decel;    /* 本方向允许的加速度幅值 */
  float jerk;
  float a_cap;
  float want;
  float step;

  /* 限制加加速度，使加速度在 TASK_ACCEL_RISE_S 内平滑建立。 */
  jerk = a_lim / TASK_ACCEL_RISE_S;

  /* 根据剩余速度差计算允许加速度，保证加速度能在目标速度处回到 0。 */
  a_cap = sqrtf(2.0f * jerk * fabsf(gap));
  want  = (a_lim < a_cap) ? a_lim : a_cap;
  if (gap < 0.0f)
  {
    want = -want;
  }

  step = jerk * TRACK_PERIOD_S;
  if (s_ramp_accel < want)
  {
    s_ramp_accel += step;
    if (s_ramp_accel > want) { s_ramp_accel = want; }
  }
  else if (s_ramp_accel > want)
  {
    s_ramp_accel -= step;
    if (s_ramp_accel < want) { s_ramp_accel = want; }
  }

  s_ramp_rpm += s_ramp_accel * TRACK_PERIOD_S;

  /* 处理浮点步进越界，并同步清除前馈加速度。 */
  if (((gap >= 0.0f) && (s_ramp_rpm > target_rpm)) ||
      ((gap <  0.0f) && (s_ramp_rpm < target_rpm)))
  {
    s_ramp_rpm   = target_rpm;
    s_ramp_accel = 0.0f;
  }

  Track_SetBaseSpeed(s_ramp_rpm);
  Ball_SetAccelFF(s_ramp_accel);
}

/**
  * @brief  把过弯产生的向心加速度前馈给球杆
  * @note   使用平均轮速与轮速差的乘积近似向心加速度。比例换算由
  *         BALL_FF_CURVE_GAIN 统一承担。
  */
static void Task_UpdateCurveFF(void)
{
  float left  = 0.0f;
  float right = 0.0f;

  Track_GetTargets(&left, &right);
  Ball_SetCurveFF(0.5f * (left + right) * (right - left));
}

static void Task_Start(void)
{
  s_start_tick  = HAL_GetTick();
  s_elapsed_ms  = 0;
  s_start_count = Task_AvgCount();
  s_distance_m  = 0.0f;
  s_ramp_rpm    = 0.0f;
  s_ramp_accel  = 0.0f;
  s_split_ms    = 0;
  s_lap_done    = 0;
  s_lap_done_ms = 0;
  s_cross_armed = 0;
  s_state       = TASK_STATE_RUN;

  /* 清掉上一轮残留的转向积分与微分历史，并把基准速度恢复成 TRACK_BASE_RPM */
  Track_Init();

  /* 清除上一任务留下的纵向加速度前馈。 */
  Ball_SetAccelFF(0.0f);

  /* 先恢复默认球控参数，再由具体任务覆盖。 */
  Ball_ResetTune();

  /* 任务一仅运行舵机演示。演示期间关闭球控，避免两个模块同时写脉宽。 */
  if (s_task == TASK_1)
  {
    s_t1_ball_en = Ball_IsEnabled();
    Ball_Enable(0);

    Track_SetBaseSpeed(0.0f);
    Servo_DemoInit();
  }
  /* 任务三使用专用球控参数，车轮保持静止。 */
  else if (s_task == TASK_3)
  {
    /* 装载任务三的大位移定位参数。 */
    Ball_Tune tune = BALL_TUNE_DEFAULT_INIT;

    tune.pos_kp = TASK3_POS_KP;
    tune.pos_ki = TASK3_POS_KI;
    tune.pos_kd = TASK3_POS_KD;
    tune.vel_kp = TASK3_VEL_KP;
    tune.vel_ki = TASK3_VEL_KI;
    tune.vel_kd = TASK3_VEL_KD;

    tune.vel_limit_cms = TASK3_VEL_LIMIT_CMS;

    /* 任务三启用按剩余距离计算的球速上限。 */
    tune.pos_brake_cms2 = TASK3_BRAKE_ACCEL_CMS2;

    Ball_SetTune(&tune);

    /* 确保任务启动时球控闭环处于使能状态。 */
    Ball_Enable(1);

    s_t3_phase     = TASK3_GO_PLUS;
    s_t3_settle_t0 = HAL_GetTick();

    Track_SetBaseSpeed(0.0f);
    Ball_SetTarget(TASK3_PLUS_CM);
  }
  /* 任务四使用直线段专用的平缓循迹参数。 */
  else if (s_task == TASK_4)
  {
    Track_SetTunings(TASK4_STEER_KP, TASK4_STEER_KI, TASK4_STEER_KD);
    Track_SetCurveSlowdown(TASK4_CURVE_SLOWDOWN);

    /* Track_Init() 会恢复默认差速上限，因此每次启动都需重新设置。 */
    Track_SetSteerLimit(TASK4_STEER_LIMIT_RPM);

    Track_SetBaseSpeed(0.0f);
  }
  /* 任务五和任务六共用整圈平稳行驶参数。 */
  else if ((s_task == TASK_5) || (s_task == TASK_6))
  {
    Track_SetTunings(TASK56_STEER_KP, TASK56_STEER_KI, TASK56_STEER_KD);
    Track_SetCurveSlowdown(TASK56_CURVE_SLOWDOWN);
    Track_SetSteerLimit(TASK56_STEER_LIMIT_RPM);
    Track_SetBaseSpeed(0.0f);
  }
  /* 任务七直接生成倒车轮速，不使用循迹外环。 */
  else if (s_task == TASK_7)
  {
    /* 直线倒车不需要弯道前馈。 */
    Ball_SetCurveFF(0.0f);

    Track_SetBaseSpeed(0.0f);
  }
}

static void Task_Finish(Task_State end_state)
{
  /* 任务五/六在通过 A 的瞬间已经掐表，后面的匀速滑行和缓停不计入成绩。
     这里不能再用最终停车时刻覆盖；其他任务仍在结束时正常定格。 */
  if (((s_task != TASK_5) && (s_task != TASK_6)) || !s_lap_done)
  {
    s_elapsed_ms = HAL_GetTick() - s_start_tick;
  }
  s_state      = end_state;

  /* 任务一结束时摆杆回水平位置，并恢复原球控状态。 */
  if (s_task == TASK_1)
  {
    Servo_SetPulseUs(SERVO_LEVEL_US);
    Ball_Enable(s_t1_ball_en);
  }

  /* 任务三非正常完成时将目标恢复到中心；正常完成后继续保持 -5 cm。 */
  if ((s_task == TASK_3) && (end_state != TASK_STATE_DONE))
  {
    Ball_SetTarget(TASK3_CENTER_CM);
  }

  Track_Stop();

  /* 先进入短路刹车，随后由 App_Idle() 完成主动反拖和驻车。 */
  Motor_BrakeAll();
}

/**
  * @brief  任务一：舵机在 SERVO_SAFE_MIN_US ~ SERVO_SAFE_MAX_US 之间往复摆动
  * @note   车全程不动(见 Task_UsesVehicle())。摆动本身由 Servo_DemoUpdate()
  *         按 tick 匀速推进，与本函数被调用的频率无关，所以 20ms 一拍够用。
  *
  *         不设时间兜底：这是个用来看机构的演示，跑到按 KEY2 为止。
  */
static void Task1_Run(void)
{
  Servo_DemoUpdate();
}

/**
  * @brief  进入终点窗口时把横线判据放宽一次
  * @retval 1 = 已经在窗口内(可以开始找横线了)
  * @note   窗口外使用默认门限；进入终点窗口后加载任务层门限。
  */
static uint8_t Task_ArmFinishGate(void)
{
  if (s_distance_m < TASK_FINISH_WINDOW_M)
  {
    return 0;
  }

  if (!s_cross_armed)
  {
    s_cross_armed = 1;
    Track_SetCrossGate(TASK_CROSS_MIN_CH, TASK_CROSS_MAX_OFFSET_MM);

    /* 重置峰值，后续记录仅覆盖终点窗口。 */
    Track_ResetCrossPeak();
  }

  return 1;
}

/**
  * @brief  任务二：巡线一圈，回到 A 点横线处停车
  */
static void Task2_Run(void)
{
  uint8_t in_window;

  Task_UpdateOdometry();

  in_window = Task_ArmFinishGate();

  /* ---------- 终点前减速(可选) ---------- */
#if TASK2_CREEP_ENABLE
  if (s_distance_m >= TASK2_CREEP_START_M)
  {
    Track_SetBaseSpeed(TASK2_CREEP_RPM);
  }
#endif

  /* ---------- 终点判定 ---------- */
  /* 仅在终点窗口内检测横线，避免起点横线导致立即完成。 */
  if (in_window && Track_IsCrossLine())
  {
    Task_Finish(TASK_STATE_DONE);
    return;
  }

  /* ---------- 里程兜底 ---------- */
  /* 横线漏检时按里程结束，避免进入第二圈。 */
#if TASK2_DIST_STOP_ENABLE
  if (s_distance_m >= TASK2_DIST_STOP_M)
  {
    Task_Finish(TASK_STATE_DONE);
  }
#endif
}

/**
  * @brief  任务三：全程闭环，把球从中心送到 +5cm、折返到 -5cm 并稳住
  * @note   本状态机只负责到位判断和目标切换，减速由球控模块完成。
  */
static void Task3_Run(void)
{
  switch (s_t3_phase)
  {
    case TASK3_GO_PLUS:
      /* 进入 +5 cm 容差后立即切换到 -5 cm 目标。 */
      if (fabsf(Ball_GetPosCm() - TASK3_PLUS_CM) <= TASK3_ARRIVE_CM)
      {
        s_t3_phase = TASK3_GO_MINUS;
        Ball_SetTarget(TASK3_MINUS_CM);
      }
      break;

    case TASK3_GO_MINUS:
      if (fabsf(Ball_GetPosCm() - TASK3_MINUS_CM) <= TASK3_ARRIVE_CM)
      {
        s_t3_phase     = TASK3_SETTLE;
        s_t3_settle_t0 = HAL_GetTick();

        /* 记录首次到达 -5 cm 的时间。 */
        s_split_ms = s_elapsed_ms;
      }
      break;

    default:    /* TASK3_SETTLE：等它在容差内连续待够 */
      if (fabsf(Ball_GetPosCm() - TASK3_MINUS_CM) > TASK3_ARRIVE_CM)
      {
        s_t3_settle_t0 = HAL_GetTick();     /* 又跑出去了，重新计时 */
      }
      else if ((HAL_GetTick() - s_t3_settle_t0) >= TASK3_SETTLE_MS)
      {
        /* 完成后保持球控闭环，使钢球继续稳定在 -5 cm。 */
        Task_Finish(TASK_STATE_DONE);
        return;
      }
      break;
  }

  /* ---------- 时间兜底 ---------- */
  if (s_elapsed_ms >= TASK3_RUN_TIME_MS)
  {
    Task_Finish(TASK_STATE_DONE);
  }
}

/**
  * @brief  任务四：慢速巡线通过 B 位置，到时停车
  * @note   使用较低车速和 S 形速度曲线，减小纵向及横向加速度扰动。
  */
static void Task4_Run(void)
{
  float target_rpm;

  Task_UpdateOdometry();

  /* 首次到达 B 点里程时记录 A 到 B 的时间。 */
  if ((s_split_ms == 0U) && (s_distance_m >= TASK4_B_DISTANCE_M))
  {
    s_split_ms = s_elapsed_ms;
  }

  /* 通过 B 后按里程进入减速段。 */
  target_rpm = (s_distance_m >= TASK4_STOP_DIST_M) ? 0.0f : TASK4_BASE_RPM;

  Task_RampBase(target_rpm, TASK4_ACCEL_RPM_PER_S, TASK4_DECEL_RPM_PER_S);
  Task_UpdateCurveFF();

  /* 目标速度平滑降到 0 后结束任务。 */
  if ((target_rpm <= 0.0f) && (s_ramp_rpm <= 0.0f))
  {
    Task_Finish(TASK_STATE_DONE);
    return;
  }

  /* ---------- 时间兜底 ---------- */
  if (s_elapsed_ms >= TASK4_RUN_TIME_MS)
  {
    Task_Finish(TASK_STATE_DONE);
  }
}

/**
  * @brief  任务五/六：慢速匀速跑完整圈，通过 A 位置后平缓停车
  * @note   两个任务共用低速、匀速和受限差速的行车参数，仅钢球目标不同。
  */
static void TaskBallLap_Run(void)
{
  float target_rpm;

  Task_UpdateOdometry();

  /* ---------- 整圈完成判定 ---------- */
  /* 进入终点窗口后才检测 A 点横线。 */
  if (!s_lap_done)
  {
    if ((Task_ArmFinishGate() && Track_IsCrossLine()) ||
        (s_distance_m >= TASK56_DIST_STOP_M))
    {
      s_lap_done    = 1;
      s_lap_done_ms = HAL_GetTick();
      s_split_ms    = s_elapsed_ms;  /* 通过 A 的时间 */
      s_elapsed_ms  = s_split_ms;    /* 正式掐表；滑行和缓停期间保持不变 */
    }
  }

  /* ---------- 速度斜坡 ---------- */
  /* 通过 A 后保持匀速 TASK56_COAST_MS，再按 S 形曲线减速。 */
  target_rpm = TASK56_BASE_RPM;
  if (s_lap_done && ((HAL_GetTick() - s_lap_done_ms) >= TASK56_COAST_MS))
  {
    target_rpm = 0.0f;
  }
  Task_RampBase(target_rpm, TASK56_ACCEL_RPM_PER_S, TASK56_DECEL_RPM_PER_S);
  Task_UpdateCurveFF();

  if (s_lap_done && (s_ramp_rpm <= 0.0f))
  {
    Task_Finish(TASK_STATE_DONE);
    return;
  }

  /* ---------- 时间兜底 ---------- */
  /* 过 A 后 s_elapsed_ms 已冻结，安全超时必须继续使用实际运行时长。 */
  if ((HAL_GetTick() - s_start_tick) >= TASK56_RUN_TIME_MS)
  {
    Task_Finish(TASK_STATE_DONE);
  }
}

/**
  * @brief  任务七(隐藏)：直着往后倒一段，到里程就平缓停住
  * @note   左右轮使用相同负转速，并以 S 形速度曲线起停。倒车时不运行循迹。
  */
static void Task7_Run(void)
{
  float target_rpm;
  float dist;

  Task_UpdateOdometry();

  /* 倒车里程为负值，距离判断使用绝对值。 */
  dist = fabsf(s_distance_m);

  /* 在目标距离前预留减速行程。 */
  target_rpm = (dist >= (TASK7_DIST_M - TASK7_BRAKE_M)) ? 0.0f : -TASK7_BASE_RPM;

  Task_RampBase(target_rpm, TASK7_ACCEL_RPM_PER_S, TASK7_DECEL_RPM_PER_S);

  /* 倒车目标速度从负值平滑回到 0 后结束。 */
  if ((target_rpm >= 0.0f) && (s_ramp_rpm >= 0.0f))
  {
    Task_Finish(TASK_STATE_DONE);
  }

  /* 本任务只按里程结束，可通过 KEY2 或 S 指令中止。 */
}

/**
  * @brief  短按 KEY1 时切到的下一个任务
  * @note   任务七不参与短按循环，只能通过长按 KEY1 或 N7 指令进入。
  */
static Task_ID Task_NextId(void)
{
  Task_ID next = (Task_ID)(s_task + 1);

  return (next >= TASK_7) ? TASK_1 : next;
}

void Task_Init(void)
{
  Key_Init();

  s_task       = TASK_2;
  s_state      = TASK_STATE_IDLE;
  s_elapsed_ms = 0;
  s_distance_m = 0.0f;
}

void Task_Update(void)
{
  /* KEY1 短按切换任务，长按直接启动任务七。 */
  if (Key_WasLongPressed(KEY1))
  {
    Task_SetId(TASK_7);
    Task_Go();
  }
  else if (Key_WasClicked(KEY1))
  {
    Task_SetId(Task_NextId());
  }

  /* ---------- KEY2：启动 / 中途停止 ---------- */
  if (Key_WasPressed(KEY2))
  {
    if (s_state == TASK_STATE_RUN)
    {
      Task_Stop();
    }
    else
    {
      Task_Go();
    }
  }

  /* ---------- 运行中的任务体 ---------- */
  if (s_state != TASK_STATE_RUN)
  {
    return;
  }

  /* 任务五/六通过 A 后继续平缓停车，但评分计时必须停在过线时刻。 */
  if (!(((s_task == TASK_5) || (s_task == TASK_6)) && s_lap_done))
  {
    s_elapsed_ms = HAL_GetTick() - s_start_tick;
  }

  switch (s_task)
  {
    case TASK_1:
      Task1_Run();
      break;

    case TASK_2:
      Task2_Run();
      break;

    case TASK_3:
      Task3_Run();
      break;

    case TASK_4:
      Task4_Run();
      break;

    /* 任务五和任务六共用行车逻辑。 */
    case TASK_5:
    case TASK_6:
      TaskBallLap_Run();
      break;

    case TASK_7:
      Task7_Run();
      break;

    default:
      Task_UpdateOdometry();
      break;
  }
}

Task_ID Task_GetId(void)
{
  return s_task;
}

Task_State Task_GetState(void)
{
  return s_state;
}

uint8_t Task_IsRunning(void)
{
  return (s_state == TASK_STATE_RUN);
}

void Task_SetId(Task_ID id)
{
  /* 运行期间禁止切换任务。 */
  if ((s_state == TASK_STATE_RUN) || (id >= TASK_NUM))
  {
    return;
  }

  s_task       = id;
  s_state      = TASK_STATE_IDLE;
  s_elapsed_ms = 0;
  s_distance_m = 0.0f;

  /* 切换到载球任务时确保球控闭环已使能。 */
  if ((s_task == TASK_3) || (s_task == TASK_4) ||
      (s_task == TASK_5) || (s_task == TASK_6))
  {
    Ball_Enable(1);
  }

  /* 任务三、四、五切换后自动回中心；任务六保留当前目标位置。 */
  if ((s_task == TASK_3) || (s_task == TASK_4) || (s_task == TASK_5))
  {
    Ball_SetTarget(TASK3_CENTER_CM);
  }
}

void Task_Go(void)
{
  if (s_state == TASK_STATE_RUN)
  {
    return;
  }

  Task_Start();
}

void Task_Stop(void)
{
  if (s_state == TASK_STATE_RUN)
  {
    Task_Finish(TASK_STATE_IDLE);       /* 手动叫停，计时定格 */
  }
}

uint8_t Task_UsesVehicle(void)
{
  /* 任务一和任务三不需要车轮运动。 */
  return ((s_task != TASK_1) && (s_task != TASK_3));
}

uint8_t Task_GetDriveTargets(float *left_rpm, float *right_rpm)
{
  /* 仅运行中的任务七直接接管轮速。 */
  if ((s_task != TASK_7) || (s_state != TASK_STATE_RUN))
  {
    return 0;
  }

  /* 左右轮使用相同目标转速，任务层不附加转向修正。 */
  if (left_rpm != NULL)
  {
    *left_rpm = s_ramp_rpm;
  }
  if (right_rpm != NULL)
  {
    *right_rpm = s_ramp_rpm;
  }

  return 1;
}

uint32_t Task_GetElapsedMs(void)
{
  return s_elapsed_ms;
}

uint32_t Task_GetSplitMs(void)
{
  return s_split_ms;
}

float Task_GetDistanceM(void)
{
  return s_distance_m;
}
