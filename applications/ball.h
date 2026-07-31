#ifndef __BALL_H
#define __BALL_H

#include "main.h"

/**
  ******************************************************************************
  * 球杆闭环 —— 视觉测位置，舵机调倾角，把钢球稳在摆杆上的指定点
  *
  * 被控对象是【二阶不稳定】的：倾角决定的是球的加速度而不是位置，
  *     x'' = (5/7) * g * sin(theta)
  * 也就是一个双积分器。这带来两个必须记住的结论：
  *
  *   1. 纯 P 控制一定等幅振荡。双积分器本身没有任何阻尼，比例项只能提供
  *      "回中的力"，提供不了"刹车的力"，球会一次次冲过中点。
  *      D 项不是可选项，是稳定性的必要条件。
  *
  *   2. 杆放平不等于球会停 —— 只是不再加速。要让运动的球停下来，
  *      必须先把杆往【反方向】倾斜给它减速，这正是 D 项在做的事。
  ******************************************************************************
  */

/* 目标位置(cm)。摆杆中心 O 实测在 12.5cm 处 */
#define BALL_TARGET_CM          12.5f

/* ---------------- 控制参数 ----------------
   输出单位是"相对水平点的舵机脉宽偏移(us)"。
   整定顺序见 ball.c 末尾，要点是先把 Kd 加够再谈 Kp。 */
#define BALL_KP                 10.0f   /* us 每 cm 偏差 */
#define BALL_KI                 0.0f    /* 先留 0，见 ball.c 说明 */
#define BALL_KD                 10.0f   /* us 每 (cm/s) 速度 */

/* 输出限幅(us)。相对 SERVO_LEVEL_US 的最大偏移。
   机构的对称可用行程有 ±450us，这里只放开 ±150us：
   倾角越大球加速越猛，超过某个角度就根本来不及刹住。宁可先小后大。 */
#define BALL_OUTPUT_LIMIT_US    150.0f
#define BALL_INTEGRAL_LIMIT_US  60.0f

/**
  * !! 上电第一次务必确认这个符号 !!
  *
  * +1 表示"球偏向 x 增大的方向时，舵机脉宽应该增大"。这取决于摄像头的
  * 安装朝向和连杆的抬升方向，两者任一装反都要改成 -1。
  *
  * 符号错了就是正反馈：球会被越推越远，瞬间冲到杆的一端。
  * 安全的验证方法见 ball.c 末尾 —— 先别开闭环，用手把球放在偏离中心的位置，
  * 看程序算出的 OUT 方向对不对。
  */
#define BALL_OUTPUT_SIGN        (+1.0f)

/* 置信度低于此值的帧直接丢弃，不参与控制 */
#define BALL_MIN_CONFIDENCE     0.50f

/* 超过这么久没有可用帧就把杆放平并清空积分 */
#define BALL_TIMEOUT_MS         200

/* 帧间隔的合理范围(秒)。视觉端卡顿或刚上电时 dt 会异常，
   不夹住的话微分项会算出天文数字，舵机瞬间打死 */
#define BALL_DT_MIN_S           0.010f
#define BALL_DT_MAX_S           0.150f

/* ================= 接口 ================= */

/* 初始化，须在 Servo_Init() / Vision_Init() 之后调用 */
void Ball_Init(void);

/* 主循环周期调用。内部只在【收到新帧】时才跑一拍控制 */
void Ball_Update(void);

/* 使能/停用。停用时舵机回到水平点 */
void Ball_Enable(uint8_t on);
uint8_t Ball_IsEnabled(void);

/* 改目标位置，用于任务六的"任意指定位置" */
void Ball_SetTarget(float cm);
float Ball_GetTarget(void);

/* 诊断用 */
float   Ball_GetPosCm(void);        /* 最近一次采纳的球位置 */
float   Ball_GetVelCmS(void);       /* 估算的球速度 */
float   Ball_GetOutputUs(void);     /* 相对水平点的输出偏移 */
uint8_t Ball_IsTracking(void);      /* 视觉可用且正在闭环 */

#endif /* __BALL_H */
