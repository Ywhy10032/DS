#ifndef __TRACKING_H
#define __TRACKING_H

#include "main.h"

/**
 * @file tracking.h
 * @brief 八路灰度循迹外环。
 *
 * 本模块每 20 ms 读取一次灰度值，通过加权质心计算横向偏差，再由转向
 * PID 生成左右轮目标转速。车轮速度闭环由 app.c 以 10 ms 周期执行。
 */

/* 八路探头从左到右的位置，单位 mm，车体中线为 0。 */
#define TRACK_SENSOR_POSITIONS  { -42.0f, -30.0f, -18.0f, -6.0f, \
                                    6.0f,  18.0f,  30.0f,  42.0f }

/* 偏差一阶低通系数，1.0 表示不滤波。 */
#define TRACK_OFFSET_LPF        0.8f

/* 1：白底黑线；0：黑底白线。 */
#define TRACK_LINE_IS_BLACK     1

/* 低于该值的单路黑度不参与质心计算。 */
#define TRACK_WEIGHT_NOISE_TH   50

/* 八路总权重低于该值时判定丢线。 */
#define TRACK_LOST_TH           30

/* 横线检测参数。单路黑度达到阈值后才计入黑色通道数。 */
#define TRACK_CROSS_WEIGHT_TH       120
#define TRACK_CROSS_MIN_CH          4
#define TRACK_CROSS_MAX_OFFSET_MM   20.0f

/* 总黑度判据，设为 0 可关闭。 */
#define TRACK_CROSS_SUM_TH      700.0f

/* 外环调用周期，必须与 app.c 的实际调度周期一致。 */
#define TRACK_PERIOD_MS         20
#define TRACK_PERIOD_S          (TRACK_PERIOD_MS / 1000.0f)

/* 默认行驶速度与转向 PID 参数。 */
#define TRACK_BASE_RPM          150.0f
#define TRACK_STEER_KP          2.0f
#define TRACK_STEER_KI          0.0f
#define TRACK_STEER_KD          0.25f
#define TRACK_STEER_LIMIT_RPM   100.0f

/* 偏差每增加 1 mm 时降低的基准转速，以及弯道减速下限。 */
#define TRACK_CURVE_SLOWDOWN    1.2f
#define TRACK_MIN_RPM           50.0f

/* 丢线后使用阵列外侧的虚拟偏差，以输出最大方向修正。 */
#define TRACK_LOST_OFFSET_MM    70.0f

/* 单轮目标转速限幅。 */
#define TRACK_MAX_RPM           250.0f

/* 连续丢线达到该时间后将左右目标转速置零。 */
#define TRACK_LOST_STOP_MS      1000

/* 初始化循迹状态和转向 PID。 */
void Track_Init(void);

/* 执行一次灰度采样、偏差计算和目标转速更新。 */
void Track_Update(void);

/* 获取左右轮目标转速。 */
void Track_GetTargets(float *left_rpm, float *right_rpm);

/* 获取最近一次有效横向偏差，正值表示黑线位于车体右侧。 */
float Track_GetOffset(void);

/* 返回当前是否丢线。 */
uint8_t Track_IsLost(void);

/**
 * 判断当前是否检测到横线。
 *
 * 黑色通道必须分布在车体中线两侧，偏差必须小于当前门限；满足通道数
 * 或总黑度任一判据后返回 1。
 */
uint8_t Track_IsCrossLine(void);

/* 获取当前黑色通道数和八路总黑度。 */
uint8_t Track_GetDarkCount(void);
float Track_GetLineWeight(void);

/* 设置横线检测的通道数和偏差门限。 */
void Track_SetCrossGate(uint8_t min_ch, float max_offset_mm);

/* 横线数据峰值用于保留车辆经过标记线时的短时测量结果。 */
void    Track_ResetCrossPeak(void);
uint8_t Track_GetDarkPeak(void);
float   Track_GetWeightPeak(void);

/* 获取最近一次灰度原始值和 I2C 状态。 */
const uint8_t *Track_GetRaw(void);
HAL_StatusTypeDef Track_GetStatus(void);

/* 运行时参数接口。 */
void Track_SetBaseSpeed(float rpm);
void Track_SetTunings(float kp, float ki, float kd);
void Track_SetCurveSlowdown(float rpm_per_mm);
void Track_SetSteerLimit(float rpm);

/* 将目标转速置零并复位转向 PID。 */
void Track_Stop(void);

#endif /* __TRACKING_H */
