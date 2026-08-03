#ifndef __PID_H
#define __PID_H

#include "main.h"

/**
  ******************************************************************************
  * 位置式 PID 控制器
  *
  *   out = Kp*e + Ki*∫e dt - Kd*d(measurement)/dt
  *
  * 两个工程上的细节：
  *   1. 微分作用在【测量值】而不是误差上。目标值阶跃变化时，误差的微分会产生
  *      一个巨大尖峰(derivative kick)，作用在测量值上就没有这个问题，而在目标
  *      值不变时两者完全等价。
  *   2. 积分带独立限幅(抗积分饱和)。电机堵转或目标超出能力范围时，积分项会一直
  *      累加到天文数字，等负载恢复后需要很久才能"退饱和"，表现为长时间失控。
  ******************************************************************************
  */

typedef struct
{
  float   kp;
  float   ki;
  float   kd;
  float   dt;                 /* 控制周期，单位：秒 */

  float   integral;           /* 误差积分，单位：误差 x 秒 */
  float   prev_measurement;   /* 上一次的测量值，用于求微分 */

  float   out_min;
  float   out_max;
  float   integral_limit;     /* 积分项(ki*integral)的幅值上限 */

  /**
    * 积分死区：|error| 小于这个值时，积分【冻结】——本拍不再累加，
    * 也不清零，就停在上一次的值上。
    *
    * 用途：对象带静摩擦这类死区非线性时，纯积分会在目标附近产生一个
    * 缓慢的极限环——噪声让 error 在 0 附近正负跳动，积分本该正负抵消，
    * 但只要有一点点系统性偏置(残余坡度、水平点误差)，积分就会朝一个
    * 方向缓慢"充电"，充到越过静摩擦门槛就把对象猛地推动一下，冲过头，
    * 反向修正，重新稳住，然后再次缓慢充电……表现为"稳一会儿、抖一下、
    * 又稳住"周期性发作。加了死区之后，噪声量级的误差不再喂给积分，
    * 它就稳稳停在"刚好顶住恒定阻力"的那个值上不再乱走；真正的扰动
    * (幅度明显大于噪声)照常被积分吸收，不受影响。
    *
    * 0 = 关闭(默认)，不影响原有任何行为。
    */
  float   integral_deadband;

  uint8_t first_run;          /* 首次运行时跳过微分，避免开机尖峰 */
} PID_Controller;

/* 初始化并清零内部状态，dt 为固定的控制周期(秒) */
void  PID_Init(PID_Controller *pid, float kp, float ki, float kd, float dt);

/* 输出限幅，应与执行器量程一致 */
void  PID_SetOutputLimits(PID_Controller *pid, float min, float max);

/* 积分项限幅，建议取输出量程的 50%~70%，给 P 留出响应余量 */
void  PID_SetIntegralLimit(PID_Controller *pid, float limit);

/* 积分死区，见结构体字段注释。0 = 关闭 */
void  PID_SetIntegralDeadband(PID_Controller *pid, float deadband);

/* 在线改参数，不影响已累积的积分 */
void  PID_SetTunings(PID_Controller *pid, float kp, float ki, float kd);

/* 跑一拍闭环，必须以 dt 为周期稳定调用 */
float PID_Update(PID_Controller *pid, float setpoint, float measurement);

/* 清空积分与微分历史，目标大幅跳变或重新使能执行器时调用 */
void  PID_Reset(PID_Controller *pid);

/**
  * @brief  把积分【项】(ki*integral)直接预置成指定值，用于无扰切换
  * @param  term  期望的积分项输出，量纲与控制器输出一致
  *
  * @note   开环/手动控制切到闭环的那一刻，如果积分从 0 起步，控制器要花
  *         时间重新累积出"顶住恒定阻力所需的那份输出"，这段时间里对象会
  *         先滑走一截(bump)。已经知道稳态需要多大输出时(比如事先标定好的
  *         静态平衡角)，用这个函数把积分直接顶到位，切换瞬间输出就等于
  *         那个已知值，不产生跳变。
  *
  *         Ki 为 0 时不做任何事 —— 积分项恒为 0，没有可预置的量。
  *         预置值同样受 integral_limit 约束。
  */
void  PID_PresetIntegral(PID_Controller *pid, float term);

#endif /* __PID_H */
