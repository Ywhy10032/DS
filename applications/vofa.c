/**
  ******************************************************************************
  * @file           : vofa.c
  * @brief          : VOFA+ FireWater 遥测与文本指令通信
  ******************************************************************************
  * @note  HAL 串口回调由 usart.c 按串口实例分发。
  ******************************************************************************
  */

#include "vofa.h"
#include "usart.h"
#include "ball.h"
#include "servo.h"
#include "task.h"
#include "battery.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#define VOFA_UART            (&huart1)
#define VOFA_RING_MASK       (VOFA_RX_RING_SIZE - 1U)

/* ---------------- 中断侧：接收 ---------------- */
static volatile uint8_t  s_ring[VOFA_RX_RING_SIZE];
static volatile uint16_t s_head = 0;        /* 中断写 */
static volatile uint16_t s_tail = 0;        /* 主循环读 */
static uint8_t           s_rx_byte = 0;

/* ---------------- 解析侧 ---------------- */
static char     s_line[VOFA_LINE_MAX];
static uint16_t s_line_len = 0;

/* ---------------- 发送侧 ---------------- */
static char              s_tx_line[VOFA_TX_LINE_MAX];
static volatile uint8_t  s_tx_busy = 0;     /* 上一帧还没发完就跳过本拍 */
static uint32_t          s_last_tx_tick = 0;

static uint32_t s_tx_count  = 0;
static uint32_t s_rx_count  = 0;
static uint32_t s_err_count = 0;

/* ================= 指令解析 ================= */

/**
  * @brief  跳过一个逗号分隔符
  * @retval 下一字段的起点；当前位置不是逗号则返回 NULL
  */
static const char *Vofa_NextField(const char *p)
{
  return (*p == ',') ? (p + 1) : NULL;
}

/**
  * @brief  解析并立即执行一行指令，格式见 vofa.h
  * @retval 1 成功，0 格式错误(整行丢弃，不改变任何状态)
  */
static uint8_t Vofa_ParseLine(const char *line)
{
  const char *p = line + 1;   /* line[0] 是指令类型 */
  char       *end;

  switch (line[0])
  {
    case 'T':
    {
      float cm = strtof(p, &end);

      if (end == p) { return 0; }
      Ball_SetTarget(cm);
      return 1;
    }

    case 'O':
    case 'P':
    {
      /* 保留当前限幅等参数，只更新指令指定的三个增益 */
      Ball_Tune tune = Ball_GetTune();
      float     kp, ki, kd;

      kp = strtof(p, &end);
      if (end == p) { return 0; }
      p = Vofa_NextField(end);
      if (p == NULL) { return 0; }

      ki = strtof(p, &end);
      if (end == p) { return 0; }
      p = Vofa_NextField(end);
      if (p == NULL) { return 0; }

      kd = strtof(p, &end);
      if (end == p) { return 0; }

      if (line[0] == 'O')
      {
        tune.pos_kp = kp; tune.pos_ki = ki; tune.pos_kd = kd;
      }
      else
      {
        tune.vel_kp = kp; tune.vel_ki = ki; tune.vel_kd = kd;
      }
      Ball_SetTune(&tune);
      return 1;
    }

    case 'E':
    {
      uint8_t on = (uint8_t)strtoul(p, &end, 10);

      if (end == p) { return 0; }
      Ball_Enable(on);
      return 1;
    }

    case 'D':
    {
      /* 设置速度环积分死区，单位为 cm/s */
      Ball_Tune tune = Ball_GetTune();
      float     cms  = strtof(p, &end);

      if (end == p) { return 0; }
      tune.vel_i_deadband_cms = cms;
      Ball_SetTune(&tune);
      return 1;
    }

    case 'L':
    {
      /* 设置舵机输出限幅，单位为 us */
      Ball_Tune tune  = Ball_GetTune();
      float     limit = strtof(p, &end);

      if (end == p) { return 0; }
      tune.out_limit_us = limit;
      Ball_SetTune(&tune);
      return 1;
    }

    case 'I':
    {
      /* 设置速度环积分项限幅，单位为 us */
      Ball_Tune tune  = Ball_GetTune();
      float     limit = strtof(p, &end);

      if (end == p) { return 0; }
      tune.vel_i_limit_us = limit;
      Ball_SetTune(&tune);
      return 1;
    }

    case 'R':
      Ball_ResetTune();
      return 1;

    case 'B':
    {
      /* 设置位置环刹车曲线减速度，0 表示关闭 */
      Ball_Tune tune = Ball_GetTune();
      float     a    = strtof(p, &end);

      if ((end == p) || (a < 0.0f)) { return 0; }
      tune.pos_brake_cms2 = a;
      Ball_SetTune(&tune);
      return 1;
    }

    case 'V':
    {
      /* 设置位置环输出的速度上限，单位为 cm/s */
      Ball_Tune tune  = Ball_GetTune();
      float     limit = strtof(p, &end);

      if ((end == p) || (limit <= 0.0f)) { return 0; }
      tune.vel_limit_cms = limit;
      Ball_SetTune(&tune);
      return 1;
    }

    case 'F':
    {
      /* 设置车辆加速度前馈增益，该参数不属于 Ball_Tune */
      float g = strtof(p, &end);

      if ((end == p) || (g < 0.0f)) { return 0; }
      Ball_SetAccelFFGain(g);
      return 1;
    }

    case 'N':
    {
      /* 上位机任务编号为 1~7，内部枚举从 0 开始 */
      uint32_t n = strtoul(p, &end, 10);

      if ((end == p) || (n < 1U) || (n > (uint32_t)TASK_NUM))
      {
        return 0;
      }
      Task_SetId((Task_ID)(n - 1U));       /* 运行时由 Task_SetId 拒绝切换 */
      return 1;
    }

    case 'G':
      Task_Go();
      return 1;

    case 'S':
      Task_Stop();
      return 1;

    default:
      return 0;
  }
}

/* ================= 中断 ================= */

void Vofa_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  uint16_t next = (uint16_t)((s_head + 1U) & VOFA_RING_MASK);

  if (next != s_tail)          /* 满了就丢弃本字节，不覆盖未读数据 */
  {
    s_ring[s_head] = s_rx_byte;
    s_head = next;
  }

  HAL_UART_Receive_IT(huart, &s_rx_byte, 1);
}

void Vofa_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  /* 清除串口错误并重新启动接收 */
  __HAL_UART_CLEAR_OREFLAG(huart);
  s_err_count++;

  HAL_UART_Receive_IT(huart, &s_rx_byte, 1);
}

void Vofa_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  (void)huart;
  s_tx_busy = 0;
}

/* ================= 对外接口 ================= */

void Vofa_Init(void)
{
  s_head      = 0;
  s_tail      = 0;
  s_line_len  = 0;
  s_tx_busy   = 0;
  s_tx_count  = 0;
  s_rx_count  = 0;
  s_err_count = 0;

  s_last_tx_tick = HAL_GetTick();

  HAL_UART_Receive_IT(VOFA_UART, &s_rx_byte, 1);
}

/**
  * @brief  解析环形缓冲区里已收到的字节，按行拆出指令
  */
static void Vofa_ParseRx(void)
{
  while (s_tail != s_head)
  {
    char c = (char)s_ring[s_tail];

    s_tail = (uint16_t)((s_tail + 1U) & VOFA_RING_MASK);

    if (c == '\r')               /* VOFA+ 发送面板常带 \r\n，\r 直接忽略 */
    {
      continue;
    }

    if (c == '\n')
    {
      if (s_line_len > 0U)
      {
        s_line[s_line_len] = '\0';
        if (Vofa_ParseLine(s_line)) { s_rx_count++; }
        else                        { s_err_count++; }
        s_line_len = 0;
      }
      continue;
    }

    if (s_line_len < (VOFA_LINE_MAX - 1U))
    {
      s_line[s_line_len++] = c;
    }
    else
    {
      s_line_len = 0;           /* 超长，整行丢弃等下一个换行重新开始 */
      s_err_count++;
    }
  }
}

/**
  * @brief  生成并异步发送一帧 FireWater 文本
  * @note   通道顺序必须与 vofa.h 中的定义一致。
  */
static void Vofa_SendFrame(void)
{
  int len;

  if (s_tx_busy)                 /* 上一帧未完成时跳过本次发送 */
  {
    return;
  }

  len = snprintf(s_tx_line, sizeof(s_tx_line),
                 "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                 (double)Ball_GetTarget(),
                 (double)Ball_GetPosCm(),
                 (double)(Ball_GetTarget() - Ball_GetPosCm()),
                 (double)Ball_GetVelCmS(),
                 (double)Ball_GetVelSetCmS(),
                 (double)Ball_GetOutputUs(),
                 (double)Servo_GetPulseUs(),
                 Ball_IsTracking() ? 1.0 : 0.0,
                 (double)(Task_GetId() + 1),   /* 转换为上位机任务编号 */
                 Task_IsRunning() ? 1.0 : 0.0,
                 (double)Task_GetElapsedMs() / 1000.0,
                 (double)Battery_GetVoltage());

  if (len <= 0)                  /* 文本编码失败 */
  {
    return;
  }
  if ((size_t)len >= sizeof(s_tx_line))
  {
    len = (int)sizeof(s_tx_line) - 1;   /* 限制为发送缓冲区长度 */
  }

  s_tx_busy = 1;
  if (HAL_UART_Transmit_IT(VOFA_UART, (uint8_t *)s_tx_line, (uint16_t)len) != HAL_OK)
  {
    s_tx_busy = 0;              /* 发送启动失败，下一周期重试 */
    return;
  }
  s_tx_count++;
}

void Vofa_Update(void)
{
  uint32_t now = HAL_GetTick();

  Vofa_ParseRx();

  if ((now - s_last_tx_tick) >= VOFA_TX_PERIOD_MS)
  {
    s_last_tx_tick = now;
    Vofa_SendFrame();
  }
}

uint32_t Vofa_GetTxCount(void)    { return s_tx_count; }
uint32_t Vofa_GetRxCmdCount(void) { return s_rx_count; }
uint32_t Vofa_GetErrorCount(void) { return s_err_count; }
