/**
  ******************************************************************************
  * @file           : task.c
  * @brief          : 任务调度 —— 按键切换/启停，任务二的一圈计时与终点停车
  ******************************************************************************
  */

#include "task.h"
#include "key.h"
#include "tracking.h"
#include "encoder.h"
#include "motor.h"

#define TASK_PI                 3.14159265f

/* 一圈的行驶距离(mm) = 轮子周长 */
#define TASK_WHEEL_CIRC_MM      (TASK_PI * TASK_WHEEL_DIAMETER_MM)

/* ---------------- 运行时状态 ---------------- */
static Task_ID    s_task      = TASK_2;      /* 开机默认停在任务二 */
static Task_State s_state     = TASK_STATE_IDLE;

static uint32_t s_start_tick   = 0;
static uint32_t s_elapsed_ms   = 0;          /* 完成/停止后定格 */
static int32_t  s_start_count  = 0;          /* 启动时的编码器基准 */
static float    s_distance_m   = 0.0f;

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

static void Task_Start(void)
{
  s_start_tick  = HAL_GetTick();
  s_elapsed_ms  = 0;
  s_start_count = Task_AvgCount();
  s_distance_m  = 0.0f;
  s_state       = TASK_STATE_RUN;

  /* 清掉上一轮残留的转向积分与微分历史，并把基准速度恢复成 TRACK_BASE_RPM */
  Track_Init();
}

static void Task_Finish(Task_State end_state)
{
  s_elapsed_ms = HAL_GetTick() - s_start_tick;
  s_state      = end_state;

  Track_Stop();
  Motor_BrakeAll();       /* 短路刹车，比滑行停得干脆，停车偏差才压得住 */
}

/**
  * @brief  任务二：巡线一圈，回到 A 点横线处停车
  */
static void Task2_Run(void)
{
  /* ---------- 里程 ---------- */
  s_distance_m = (float)(Task_AvgCount() - s_start_count)
                 / (float)ENCODER_COUNTS_PER_REV
                 * TASK_WHEEL_CIRC_MM / 1000.0f;

  /* ---------- 终点前减速(可选) ---------- */
#if TASK2_CREEP_ENABLE
  if (s_distance_m >= TASK2_CREEP_START_M)
  {
    Track_SetBaseSpeed(TASK2_CREEP_RPM);
  }
#endif

  /* ---------- 终点判定 ---------- */
  /* 起跑时车就压在 A 点横线上，所以必须先跑出去一段才开始检测，
     否则按下启动的瞬间就会判定为"已完成一圈" */
  if ((s_distance_m >= TASK2_MIN_LAP_M) && Track_IsCrossLine())
  {
    Task_Finish(TASK_STATE_DONE);
    return;
  }

  /* ---------- 里程兜底 ---------- */
  /* 横线没识别到时的最后一道保险，免得车一路跑第二圈 */
#if TASK2_DIST_STOP_ENABLE
  if (s_distance_m >= TASK2_DIST_STOP_M)
  {
    Task_Finish(TASK_STATE_DONE);
  }
#endif
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
  /* ---------- KEY1：切换任务 ---------- */
  /* 运行中屏蔽，避免跑着跑着被误触切走 */
  if (Key_WasPressed(KEY1))
  {
    if (s_state != TASK_STATE_RUN)
    {
      s_task  = (Task_ID)((s_task + 1) % TASK_NUM);
      s_state = TASK_STATE_IDLE;
      s_elapsed_ms = 0;
      s_distance_m = 0.0f;
    }
  }

  /* ---------- KEY2：启动 / 中途停止 ---------- */
  if (Key_WasPressed(KEY2))
  {
    if (s_state == TASK_STATE_RUN)
    {
      Task_Finish(TASK_STATE_IDLE);       /* 手动叫停，计时定格 */
    }
    else
    {
      Task_Start();
    }
  }

  /* ---------- 运行中的任务体 ---------- */
  if (s_state != TASK_STATE_RUN)
  {
    return;
  }

  s_elapsed_ms = HAL_GetTick() - s_start_tick;

  switch (s_task)
  {
    case TASK_2:
      Task2_Run();
      break;

    /* 任务一/三/四待实现，目前只是普通巡线，按 KEY2 停 */
    case TASK_1:
    case TASK_3:
    case TASK_4:
    default:
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

uint32_t Task_GetElapsedMs(void)
{
  return s_elapsed_ms;
}

float Task_GetDistanceM(void)
{
  return s_distance_m;
}
