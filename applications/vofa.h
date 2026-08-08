#ifndef __VOFA_H
#define __VOFA_H

#include "main.h"

/**
  ******************************************************************************
  * @brief  VOFA+ 上位机通信，USART1，115200 8N1
  *
  * 上行采用 FireWater 文本格式，共 12 个通道：
  * SET、POS、ERR、VEL、VSET、OUT、US、TRACKING、TASK_ID、TASK_RUN、
  * TASK_ELAPSED_S、BATTERY_V。任务五、六再次经过 A 点后，计时通道保持
  * 锁存值，车辆继续完成停车过程。
  *
  * 下行指令以换行结束：
  * T<cm>             设置滚球目标位置
  * O<kp>,<ki>,<kd>   设置位置环增益
  * P<kp>,<ki>,<kd>   设置速度环增益
  * D<cm/s>           设置速度环积分死区
  * L<us>             设置舵机输出限幅
  * I<us>             设置速度环积分限幅
  * E<0|1>            关闭或使能滚球闭环
  * R                  恢复滚球控制默认参数
  * B<cm/s2>          设置位置环刹车减速度
  * V<cm/s>           设置速度指令上限
  * F<us/(rpm/s)>     设置车辆加速度前馈增益
  * N<1~7>            选择任务
  * G                  启动当前任务
  * S                  停止当前任务
  ******************************************************************************
  */

/* 环形缓冲区大小必须为 2 的幂 */
#define VOFA_RX_RING_SIZE     128
#define VOFA_LINE_MAX         64

#define VOFA_TX_PERIOD_MS     20
#define VOFA_TX_CH_NUM        12
#define VOFA_TX_LINE_MAX      156

void Vofa_Init(void);
void Vofa_Update(void);

/* 由 usart.c 中的共享 HAL 回调按串口实例分发 */
void Vofa_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void Vofa_UART_ErrorCallback(UART_HandleTypeDef *huart);
void Vofa_UART_TxCpltCallback(UART_HandleTypeDef *huart);

uint32_t Vofa_GetTxCount(void);
uint32_t Vofa_GetRxCmdCount(void);
uint32_t Vofa_GetErrorCount(void);

#endif /* __VOFA_H */
