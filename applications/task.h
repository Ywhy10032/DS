#ifndef __TASK_H
#define __TASK_H

#include "main.h"

/**
  ******************************************************************************
  * 任务调度
  *
  *   KEY1  切换任务(仅在未运行时有效，避免跑着跑着被误切)
  *   KEY2  开始当前任务；运行中再按一次立即停止并刹车
  *
  * 任务二：小车置于 A 点，按键启动后沿黑线顺时针行驶一圈并停到 A 点，
  *         计时停止并显示行驶总时间。要求 ≤20s、停车偏差 ≤2cm。
  *         A 点有一道横着的黑胶带，用它作为终点判据。
  ******************************************************************************
  */

/* ================= 车体参数 ================= */

/* 轮子直径(mm)，实测值。用于把编码器计数换算成行驶距离 */
#define TASK_WHEEL_DIAMETER_MM      67.0f

/* ================= 任务二参数 ================= */

/* 赛道一圈的实测长度(m) */
#define TASK2_LAP_LENGTH_M          6.14f

/* 起跑时车就压在 A 点横线上，必须先跑出这个距离才开始检测终点横线，
   否则一启动就会立刻"到达终点"。同时也能挡掉赛道中途的干扰。
   取一圈长度的一半，既躲开起点又不会错过终点 */
#define TASK2_MIN_LAP_M             (TASK2_LAP_LENGTH_M * 0.5f)

/* ---------------- 终点减速(可选) ----------------
   150rpm 约 0.5m/s，光靠"看到横线再刹车"很难压进 2cm：
   外环 20ms 的检测延迟本身就是 10mm，再加上刹车距离必然超标。
   跑到"一圈长度减 0.4m"时降到蠕行速度，这样看到横线几乎能立刻停住。

   !! 这个功能完全依赖里程的准确性，而里程又依赖上面的 TASK_WHEEL_DIAMETER_MM !!
   轮子直径填错会导致减速点跑偏：填大了会提前减速(只是慢一点，无害)，
   填小了会来不及减速甚至冲过终点。务必先量准轮径再依赖它。 */
#define TASK2_CREEP_ENABLE          1
#define TASK2_CREEP_START_M         (TASK2_LAP_LENGTH_M - 0.4f)
#define TASK2_CREEP_RPM             50.0f

/* ---------------- 里程兜底停车 ----------------
   万一横线没被识别到(标记太窄、被磨损、车压偏)，光靠视觉判据车会一直跑下去。
   跑过一圈长度再加这点余量还没看到横线，就按里程直接停 —— 停在 A 点附近，
   总比冲第二圈强。里程有轮滑误差，所以只作兜底，正常应该由横线先触发。 */
#define TASK2_DIST_STOP_ENABLE      1
#define TASK2_DIST_STOP_M           (TASK2_LAP_LENGTH_M + 0.15f)

/* ================= 类型 ================= */

typedef enum
{
  TASK_1 = 0,
  TASK_2,
  TASK_3,
  TASK_4,
  TASK_NUM
} Task_ID;

typedef enum
{
  TASK_STATE_IDLE = 0,    /* 待命，等 KEY2 */
  TASK_STATE_RUN,         /* 运行中 */
  TASK_STATE_DONE         /* 已完成，计时定格 */
} Task_State;

/* ================= 接口 ================= */

/* 初始化。须在 Track_Init() / Encoder_Init() 之后调用 */
void Task_Init(void);

/* 跑一拍任务调度。须每 TRACK_PERIOD_MS 调用一次(与循迹外环同拍) */
void Task_Update(void);

Task_ID    Task_GetId(void);
Task_State Task_GetState(void);
uint8_t    Task_IsRunning(void);

/* 行驶总时间(ms)。运行中实时累加，完成或停止后定格 */
uint32_t Task_GetElapsedMs(void);

/* 本次任务已行驶的距离(m) */
float Task_GetDistanceM(void);

#endif /* __TASK_H */
