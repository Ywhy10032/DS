#ifndef __VISION_H
#define __VISION_H

#include "main.h"

/**
  ******************************************************************************
  * @brief  视觉模块串口接收，UART4，115200 8N1
  *
  * 帧格式：
  * $BALL,<valid>,<x_cm>,<vx_pixel_s>,<confidence>,<frame_time_ms>[,<target_cm>]*
  * target_cm 为可选字段；frame_time_ms 是视觉模块自身的时间戳。
  ******************************************************************************
  */

/* 环形缓冲区大小必须为 2 的幂 */
#define VISION_RX_RING_SIZE     512
#define VISION_LINE_MAX         64
#define VISION_TIMEOUT_MS       200

/* UART4 中断入口由 CubeMX 生成的中断文件提供 */
#define VISION_OWN_IRQ_HANDLER  0

typedef struct
{
  uint8_t  valid;
  float    x_cm;
  float    vx_pixel_s;
  float    confidence;
  uint32_t frame_time_ms;
  float    target_cm;
  uint8_t  has_target;   /* 本帧是否包含 target_cm */
} Vision_Ball;

void Vision_Init(void);

/* 由 usart.c 中的共享 HAL 回调按串口实例分发 */
void Vision_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void Vision_UART_ErrorCallback(UART_HandleTypeDef *huart);

/* 在主循环中解析接收缓冲区 */
void Vision_Update(void);

/* 返回最近一次成功解析的数据；使用前应检查数据是否超时 */
const Vision_Ball *Vision_GetBall(void);
uint8_t Vision_IsFresh(void);
uint32_t Vision_GetAgeMs(void);

uint32_t Vision_GetFrameCount(void);
uint32_t Vision_GetErrorCount(void);

#endif /* __VISION_H */
