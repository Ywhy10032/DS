#ifndef __VISION_H
#define __VISION_H

#include "main.h"

/**
  ******************************************************************************
  * 视觉模块串口接收 (UART4 / PA0-TX、PA1-RX、115200 8N1)
  *
  * 视觉端每识别成功一次就发一帧：
  *     $BALL,<valid>,<x_cm>,<vx_pixel_s>,<confidence>,<frame_time_ms>[,<target_cm>]*\n
  * 例(带目标)：
  *     $BALL,1,12.34,-5.6,0.87,123456,12.50*
  * 例(不带目标，老格式)：
  *     $BALL,1,12.34,-5.6,0.87,123456*
  *
  *   valid          1 = 本帧钢珠坐标有效
  *   x_cm           沿摆杆轴线的位置，厘米
  *   vx_pixel_s     估算的横向像素速度，像素/秒
  *   confidence     YOLO 检测置信度
  *   frame_time_ms  视觉模块【自己】的时间戳，与本机 HAL_GetTick() 无关，
  *                  只能用来判断视觉端是否卡帧，不能拿来和本机时间做差
  *   target_cm      【可选】视觉端设定的小球目标位置，厘米。不想指定目标的帧
  *                  可以整个省掉这个字段(连逗号一起)，解析器不会因为少这
  *                  一项就把前面 valid/x_cm/confidence 这些数据也丢掉 ——
  *                  是否带了这个字段见 Vision_Ball.has_target。
  *                  app.c 只在【没有任务在运行】且 has_target 为真时才会把
  *                  它写进 Ball_SetTarget()；任务运行中的目标由 task.c 的
  *                  状态机(如任务三的 +5cm/-5cm 折返)或 vofa.c 的 T 指令
  *                  管理，让路给它们，避免每帧被这里覆盖打断
  ******************************************************************************
  */

/* 接收环形缓冲区大小，必须是 2 的幂(用掩码取模)。
   115200 下 10ms 能收 115 字节，缓冲要能扛住主循环偶尔被刷屏阻塞的时间 */
#define VISION_RX_RING_SIZE     512

/* 单帧最大长度，超长直接丢弃，防止一个坏字节把解析器带偏 */
#define VISION_LINE_MAX         64

/* 超过这个时间没收到有效帧就判定视觉断链。
   后面接球杆闭环时必须检查这个 —— 拿着过期坐标去控制，球早就不在那了 */
#define VISION_TIMEOUT_MS       200

/**
  * 中断向量与 NVIC 使能由谁负责。
  *
  * 现在 CubeMX 的 NVIC 页已经勾选了 UART4 global interrupt，
  * stm32f4xx_it.c 会生成 UART4_IRQHandler、usart.c 里也会调
  * HAL_NVIC_EnableIRQ() —— 所以本模块不能再定义一份，否则链接冲突。
  *
  * 只有在 CubeMX 里【没有】勾选该中断时才需要把它设回 1。
  */
#define VISION_OWN_IRQ_HANDLER  0

typedef struct
{
  uint8_t  valid;
  float    x_cm;
  float    vx_pixel_s;
  float    confidence;
  uint32_t frame_time_ms;
  float    target_cm;
  uint8_t  has_target;   /* 本帧是否带了 target_cm 字段 */
} Vision_Ball;

/* 启动中断接收。须在 MX_UART4_Init() 之后调用 */
void Vision_Init(void);

/**
  * ---------------- 供 usart.c 的共享 HAL 回调分发 ----------------
  * HAL_UART_RxCpltCallback / HAL_UART_ErrorCallback 全工程只能有一份定义，
  * USART1 那路(vofa.c，见其头文件说明)也要用同一个回调名。真正的 HAL 回调
  * 集中放在 usart.c 里按 huart->Instance 分发，本模块只暴露自己的处理函数。
  */
void Vision_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void Vision_UART_ErrorCallback(UART_HandleTypeDef *huart);

/* 解析缓冲区里已收到的数据。主循环周期调用，非阻塞 */
void Vision_Update(void);

/* 最近一帧的解析结果。断链时内容保持为上一帧，用 Vision_IsFresh() 判断可用性 */
const Vision_Ball *Vision_GetBall(void);

/* 数据是否还新鲜(距上一帧 < VISION_TIMEOUT_MS) */
uint8_t Vision_IsFresh(void);

/* 距最近一帧过去了多久(ms)，用于诊断视觉端帧率 */
uint32_t Vision_GetAgeMs(void);

/* 累计成功解析的帧数 / 解析失败与串口错误的次数 */
uint32_t Vision_GetFrameCount(void);
uint32_t Vision_GetErrorCount(void);

#endif /* __VISION_H */
