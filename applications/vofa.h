#ifndef __VOFA_H
#define __VOFA_H

#include "main.h"

/**
  ******************************************************************************
  * VOFA+ 上位机通信 (USART1 / PA9-TX、PA10-RX，115200 8N1)
  ******************************************************************************
  * @note  PA9/PA10 与 USART1 的复用由 CubeMX 生成的 MX_USART1_UART_Init()
  *        完成，因此 Vofa_Init() 必须在它之后调用。
  *
  *        接收走中断，做法与 vision.c 一致(单字节中断 + 环形缓冲 + 主循环
  *        解析)。但 HAL_UART_RxCpltCallback/HAL_UART_ErrorCallback 全工程
  *        只能有一份定义，vision.c 已经占用了这两个名字 —— 所以真正的 HAL
  *        回调被集中放到 usart.c(USER CODE 段) 里按 huart->Instance 分发，
  *        本模块只对外暴露 Vofa_UART_*Callback() 这几个非 HAL 名字的处理函数。
  *
  * ---------------- 上行：FireWater（板 -> 上位机，用于画图） ----------------
  * 每帧一行文本，格式 "ch0,ch1,...,ch10\n"，11 个通道、顺序固定，见下面的
  * 列表 —— 这是 VOFA+ "FireWater" 数据源的标准格式，遇到 '\n' 就切一帧。
  *
  *   ch0  目标位置 cm(SET)
  *   ch1  实测位置 cm(POS) —— 不受球杆闭环使能状态影响，任务三开环运行期间
  *        (E0)照常更新，标定三段动作时间就靠看这条曲线，见 ball.c 的说明
  *   ch2  偏差 cm，ch0-ch1(ERR)
  *   ch3  实测速度 cm/s(VEL) —— 同 ch1，闭环关掉时也照常更新
  *   ch4  位置环下达的速度指令 cm/s(VSET) —— 串级调试的关键中间量，只有闭环
  *        使能时才更新，任务三开环运行期间是上一次闭环时的旧值
  *   ch5  速度环输出，相对水平点的 us 偏移(OUT)，同 ch4 只在闭环使能时更新
  *   ch6  实际下发的舵机脉宽 us(US) —— 任务三开环运行期间这条就是三段动作
  *        直接写的脉宽，能看出当前在哪一段
  *   ch7  是否在闭环跟踪，0/1(TRACKING) —— 只反映闭环状态，任务三开环运行期间
  *        恒为 0(哪怕球在动)，不代表视觉丢了
  *   ch8  当前任务号，1~6 对应任务一~任务六(TASK_ID+1) —— 跟下面 N 指令
  *        "发几就是任务几"用的是同一套编号，ch8 显示几就发 N 几能切回来
  *   ch9  当前任务是否在运行，0/1(TASK_RUN)
  *   ch10 当前任务已耗时，秒(TASK_ELAPSED_S) —— 完成/停止后定格，
  *        与 app.c 主界面秒表显示的是同一个数
  *   ch11 电池电压 V(BATTERY_V)，见 battery.c 的一阶低通滤波值
  *
  * 用 snprintf 拼文本：CMakeLists.txt 里已经为 LCD_DisplayDecimals() 链了
  * -u _printf_float，浮点版 printf 族在这个工程里本来就可用，不必像
  * vision.c 的接收侧那样绕开(那边绕开的是 newlib-nano 默认阉割的浮点
  * scanf，跟这里的 printf 是两码事)。
  *
  * ---------------- 下行：文本指令（上位机 -> 板，设定目标/改 PID/控任务） ----------------
  * 一行一条指令，以 '\n' 结束('\r' 忽略)。在 VOFA+ 的"发送"面板(或自定义
  * 控件绑定的发送字符串)按下面的格式发 ASCII 文本即可：
  *
  *   T<cm>              设定小球目标位置，例如 T17.5
  *   O<kp>,<ki>,<kd>    改位置环(外环)三个增益，其余字段(限幅等)沿用当前值
  *   P<kp>,<ki>,<kd>    改速度环(内环)三个增益
  *   D<cms>             改速度环积分死区(cm/s)，例如 D1.0 —— 治"稳一会儿、
  *                       抖一下、又稳住"周期性发作，见 ball.h 对应常量注释
  *   L<us>              改舵机输出总限幅，例如 L560 —— 球卡在这个值推不动
  *                       就调它(摩擦增大/灰尘时会先撞到这个天花板)
  *   I<us>              改速度环积分限幅，例如 I500 —— 和 L 是一对，
  *                       I 必须比 L 留出比例项的余量(见 ball.h 说明)
  *   E<0|1>             球杆闭环 使能/关闭，例如 E1
  *   R                  恢复 ball.h 里 #define 的默认参数组
  *   W<us>              任务三开环三段共用的倾角偏移量，例如 W300，
  *                       见 task.h 的 Task3_OL_Params.tilt_us
  *   H<us>              任务三 -5cm 处标定出的静态稳定角，例如 H1640，
  *                       见 task.h 的 Task3_OL_Params.hold_minus_us
  *   1<ms> 2<ms> 3<ms>  任务三开环阶段一/二/三的时长，例如 1300、2500、3300，
  *                       见 task.h 的 Task3_OL_Params.t1_ms/t2_ms/t3_ms
  *   J<cm>              任务三交接窗口的【下限】，例如 J3.0 —— 球很慢时兜底
  *                       用，见 task.h 的 TASK3_HANDOFF_CM
  *   B<cms2>            任务三估停车距离用的减速度，例如 B15 —— 交接窗口
  *                       = v²/(2B)，随球速自适应。调过冲的主旋钮：还冲过头
  *                       就【调小】(窗口变宽、提前交接)，见 task.h 的
  *                       TASK3_BRAKE_ACCEL_CMS2
  *   N<1~6>             切换到指定任务，发几就是任务几(1=任务一...6=任务六，
  *                       没有任务 0)，等价于按 KEY1 循环切到该任务；运行中
  *                       发送会被忽略，规则与物理按键一致(见 task.h)
  *   G                  启动当前任务，等价于空闲/已完成状态下按一次 KEY2；
  *                       运行中发送无效果
  *   S                  停止当前任务，等价于运行中按一次 KEY2(计时定格、
  *                       车刹停)；未运行时发送无效果
  *
  * 解析用 strtof/strtoul 而不是 sscanf("%f")，原因同 vision.c。
  ******************************************************************************
  */

/* 接收环形缓冲区大小，必须是 2 的幂。指令是人手动发的，远比视觉帧稀疏，
   128 字节足够存好几行不被主循环及时处理的指令 */
#define VOFA_RX_RING_SIZE     128

/* 单条指令最大长度，超长直接丢弃，防止一个坏字节把解析器带偏 */
#define VOFA_LINE_MAX         64

/* 上行发送周期(ms)，跟 app.c 外环/task 层同拍，没必要发得比控制还快 */
#define VOFA_TX_PERIOD_MS     20

/* 通道数，见上面 FireWater 格式说明的 ch0~ch11 列表 */
#define VOFA_TX_CH_NUM        12

/* 一行文本的最大长度："-1234.56," 每通道最多约 10 字节，12 通道 + 换行 +
   结尾 '\0' 留足余量。snprintf 万一算出更长也会在这里截断，不会越界 */
#define VOFA_TX_LINE_MAX      156

/* 初始化。须在 MX_USART1_UART_Init() 之后调用 */
void Vofa_Init(void);

/* 主循环周期调用：按 VOFA_TX_PERIOD_MS 打包一帧数据发出去，
   同时解析已经收到的指令。非阻塞 */
void Vofa_Update(void);

/* ---------------- 供 usart.c 的共享 HAL 回调分发 ---------------- */
void Vofa_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void Vofa_UART_ErrorCallback(UART_HandleTypeDef *huart);
void Vofa_UART_TxCpltCallback(UART_HandleTypeDef *huart);

/* 累计发送帧数 / 成功执行的指令数 / 解析失败(含串口错误)次数，诊断用 */
uint32_t Vofa_GetTxCount(void);
uint32_t Vofa_GetRxCmdCount(void);
uint32_t Vofa_GetErrorCount(void);

#endif /* __VOFA_H */
