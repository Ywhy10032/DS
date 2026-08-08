#ifndef __TASK_H
#define __TASK_H

#include "main.h"

/**
 * @file task.h
 * @brief 比赛任务状态机及任务参数。
 *
 * KEY1 短按在任务一至任务六之间切换，长按直接启动隐藏的倒车任务；
 * KEY2 启动或停止当前任务。运行期间禁止切换任务。
 */

/* 车轮直径，用于将编码器计数换算为里程。 */
#define TASK_WHEEL_DIAMETER_MM      67.0f

/* 速度斜坡中加速度从 0 建立到限值所需的时间。 */
#define TASK_ACCEL_RISE_S           0.25f

/* 标定后的一圈里程。 */
#define TASK_LAP_LENGTH_M           6.00f

/* 进入该里程后才允许识别 A 点横线。 */
#define TASK_FINISH_WINDOW_M        (TASK_LAP_LENGTH_M - 0.35f)

/* 终点窗口内使用的横线检测门限。 */
#define TASK_CROSS_MIN_CH           4
#define TASK_CROSS_MAX_OFFSET_MM    35.0f

/* 任务二：巡线一圈，在 A 点停车。 */
#define TASK2_CREEP_ENABLE          1
#define TASK2_CREEP_START_M         (TASK_LAP_LENGTH_M - 0.35f)
#define TASK2_CREEP_RPM             40.0f
#define TASK2_DIST_STOP_ENABLE      1
#define TASK2_DIST_STOP_M           (TASK_LAP_LENGTH_M + 0.30f)

/* 任务三：中心 -> +5 cm -> -5 cm，并在终点保持稳定。 */
#define TASK3_CENTER_CM             12.5f
#define TASK3_PLUS_CM               (TASK3_CENTER_CM + 5.0f)
#define TASK3_MINUS_CM              (TASK3_CENTER_CM - 5.0f)

/* 任务三的位置刹车曲线和球速上限。 */
#define TASK3_BRAKE_ACCEL_CMS2      3.0f
#define TASK3_VEL_LIMIT_CMS         12.0f

/* 到位容差和连续稳定时间。 */
#define TASK3_ARRIVE_CM             0.6f
#define TASK3_SETTLE_MS             350

/* 任务三专用球控参数。 */
#define TASK3_POS_KP                1.25f
#define TASK3_POS_KI                0.0f
#define TASK3_POS_KD                0.022f
#define TASK3_VEL_KP                33.0f
#define TASK3_VEL_KI                70.0f
#define TASK3_VEL_KD                0.0f

/* 任务三异常退出时间。 */
#define TASK3_RUN_TIME_MS           15000

/* 任务四：从 A 行驶至 B，球保持在中心位置。 */
#define TASK4_B_DISTANCE_M          1.6f
#define TASK4_BASE_RPM              90.0f
#define TASK4_STOP_DIST_M           (TASK4_B_DISTANCE_M + 0.25f)
#define TASK4_ACCEL_RPM_PER_S       40.0f
#define TASK4_DECEL_RPM_PER_S       40.0f
#define TASK4_STEER_LIMIT_RPM       50.0f
#define TASK4_RUN_TIME_MS           14000

/* 任务四使用较软的直线循迹参数，并关闭弯道减速。 */
#define TASK4_STEER_KP              1.0f
#define TASK4_STEER_KI              0.0f
#define TASK4_STEER_KD              0.15f
#define TASK4_CURVE_SLOWDOWN        0.0f

/* 任务五和任务六共用行车参数，差别仅在钢球目标位置。 */
#define TASK56_BASE_RPM             80.0f
#define TASK56_ACCEL_RPM_PER_S      30.0f
#define TASK56_DECEL_RPM_PER_S      30.0f
#define TASK56_STEER_KP             1.5f
#define TASK56_STEER_KI             0.0f
#define TASK56_STEER_KD             0.20f
#define TASK56_CURVE_SLOWDOWN       0.0f
#define TASK56_STEER_LIMIT_RPM      50.0f

/* 横线漏检时使用里程完成一圈判定。 */
#define TASK56_DIST_STOP_M          (TASK_LAP_LENGTH_M + 0.15f)

/* 通过 A 后保持匀速的时间，随后再平缓停车。 */
#define TASK56_COAST_MS             2000

/* 任务五和任务六的安全退出时间。 */
#define TASK56_RUN_TIME_MS           35000

/* 隐藏任务七：不使用循迹，按相同负转速直线倒车。 */
#define TASK7_BASE_RPM              80.0f
#define TASK7_DIST_M                0.50f
#define TASK7_BRAKE_M               0.22f
#define TASK7_ACCEL_RPM_PER_S       60.0f
#define TASK7_DECEL_RPM_PER_S       60.0f

typedef enum
{
  TASK_1 = 0,
  TASK_2,
  TASK_3,
  TASK_4,
  TASK_5,
  TASK_6,
  TASK_7,
  TASK_NUM
} Task_ID;

typedef enum
{
  TASK_STATE_IDLE = 0,
  TASK_STATE_RUN,
  TASK_STATE_DONE
} Task_State;

/* 初始化和周期更新。Task_Update() 的调用周期为 TRACK_PERIOD_MS。 */
void Task_Init(void);
void Task_Update(void);

Task_ID    Task_GetId(void);
Task_State Task_GetState(void);
uint8_t    Task_IsRunning(void);

/* 任务选择和启停接口。运行期间 Task_SetId() 不生效。 */
void Task_SetId(Task_ID id);
void Task_Go(void);
void Task_Stop(void);

/* 返回当前任务是否需要车轮运动。 */
uint8_t Task_UsesVehicle(void);

/**
 * 获取任务层直接生成的左右轮目标转速。
 * 仅运行中的任务七返回 1；其他任务返回 0，由循迹模块提供目标转速。
 */
uint8_t Task_GetDriveTargets(float *left_rpm, float *right_rpm);

/**
 * 获取任务计时。
 * 一般在任务结束时定格；任务五和任务六通过 A 点时立即定格。
 */
uint32_t Task_GetElapsedMs(void);

/* 获取评分点时间：任务三为到达 -5 cm，任务四为到达 B，任务五/六为通过 A。 */
uint32_t Task_GetSplitMs(void);

/* 获取本次任务的平均轮里程，单位 m。 */
float Task_GetDistanceM(void);

#endif /* __TASK_H */
