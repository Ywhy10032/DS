#ifndef __TRACKING_H
#define __TRACKING_H

#include "main.h"

/**
  ******************************************************************************
  * 循迹（巡线）—— 加权质心求横向偏差 + 转向 PID 输出差速
  *
  * 串级结构：
  *   外环(本模块, 20ms)  灰度8路 -> 加权质心 -> 偏差mm -> 转向PID -> 左右目标转速
  *   内环(app.c,   10ms)  目标转速 -> 速度PID -> PWM
  *
  * 内环必须比外环快，否则外环发出的指令还没执行完就被改了，整个系统会震荡。
  ******************************************************************************
  */

/* ================= 传感器几何 ================= */

/* !! 按实测修改 !!
   8 路探头从左到右的安装坐标，单位 mm，车体中线为 0、左负右正。
   下面按 10mm 等间距对称排布填入。
   注意：只要间距是均匀的，这里填的绝对数值只相当于换算单位，
   填错了不影响能不能跑，只会让 Kp 需要按比例缩放。 */
#define TRACK_SENSOR_POSITIONS  { -35.0f, -25.0f, -15.0f, -5.0f, \
                                    5.0f,  15.0f,  25.0f,  35.0f }

/* 赛道是白底黑线填 1；黑底白线填 0 */
#define TRACK_LINE_IS_BLACK     1

/* 单路权重低于此值视为噪声，该路不参与质心计算。
   白场读数不会正好是极值，不掐掉的话八路的零星波动会把质心往中间拽 */
#define TRACK_WEIGHT_NOISE_TH   20

/* 8 路权重总和低于此值判定为丢线 */
#define TRACK_LOST_TH           30

/* ================= 运行参数 ================= */

/* 外环周期，必须与 app.c 里调用 Track_Update() 的实际间隔一致 */
#define TRACK_PERIOD_MS         20
#define TRACK_PERIOD_S          (TRACK_PERIOD_MS / 1000.0f)

/* 直道基准速度(输出轴 rpm)，左右轮在此基础上加减差速 */
#define TRACK_BASE_RPM          150.0f

/* 转向 PID。整定方法见 tracking.c 末尾 */
#define TRACK_STEER_KP          1.2f
#define TRACK_STEER_KI          0.0f
#define TRACK_STEER_KD          0.05f

/* 差速上限(rpm)。限制住转向环最多能让两轮差多少，防止过弯直接原地打转 */
#define TRACK_STEER_LIMIT_RPM   60.0f

/* 单轮目标转速的上限，防止基准速度 + 差速超出电机能力 */
#define TRACK_MAX_RPM           200.0f

/* 连续丢线超过这个时间就停车，避免车失控冲出赛道 */
#define TRACK_LOST_STOP_MS      1000

/* ================= 接口 ================= */

/* 初始化转向 PID。须在 Gray_Init() 之后调用 */
void Track_Init(void);

/* 跑一拍外环：读灰度 -> 算质心 -> 转向 PID -> 更新左右目标转速。
   必须每 TRACK_PERIOD_MS 稳定调用一次 */
void Track_Update(void);

/* 取出给内环速度环用的左右目标转速 */
void Track_GetTargets(float *left_rpm, float *right_rpm);

/* 最近一次算出的横向偏差(mm)，正数表示线在车体右侧 */
float Track_GetOffset(void);

/* 当前是否丢线 */
uint8_t Track_IsLost(void);

/* 最近一次读到的 8 路原始值，供显示/调试 */
const uint8_t *Track_GetRaw(void);

/* 最近一次灰度 I2C 传输的结果，供状态显示用 */
HAL_StatusTypeDef Track_GetStatus(void);

/* 改基准速度。设为 0 即停车但仍保持循迹计算 */
void Track_SetBaseSpeed(float rpm);

/* 在线改转向 PID 参数，方便用调试器整定 */
void Track_SetTunings(float kp, float ki, float kd);

/* 立即停车并清空转向 PID 的积分与微分历史 */
void Track_Stop(void);

#endif /* __TRACKING_H */
