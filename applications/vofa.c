/**
  ******************************************************************************
  * @file           : vofa.c
  * @brief          : VOFA+ 上位机通信 (USART1) —— 上行 FireWater 画图，
  *                    下行文本指令改目标位置 / PID
  ******************************************************************************
  * @note  协议细节见 vofa.h。中断接收与解析走的是和 vision.c 一样的套路
  *        (单字节中断 + 环形缓冲 + 主循环里按行解析)，HAL_UART_RxCpltCallback
  *        等真正的 HAL 回调集中放在 usart.c 里分发，本文件只实现自己那一路。
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
      /* 先取当前生效的整组参数，只改这条指令覆盖的三个增益 —— 限幅等
         字段不用每次都在指令里重复输入 */
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
      /* 速度环积分死区(cm/s)，见 ball.h 的 BALL_VEL_I_DEADBAND_CMS ——
         专治"稳一会儿、抖一下、又稳住"周期性发作 */
      Ball_Tune tune = Ball_GetTune();
      float     cms  = strtof(p, &end);

      if (end == p) { return 0; }
      tune.vel_i_deadband_cms = cms;
      Ball_SetTune(&tune);
      return 1;
    }

    case 'L':
    {
      /* 舵机输出总限幅(us)，见 ball.h 的 BALL_OUTPUT_LIMIT_US ——
         摩擦增大(灰尘)时球卡在这个天花板推不动，就调它 */
      Ball_Tune tune  = Ball_GetTune();
      float     limit = strtof(p, &end);

      if (end == p) { return 0; }
      tune.out_limit_us = limit;
      Ball_SetTune(&tune);
      return 1;
    }

    case 'I':
    {
      /* 速度环积分限幅(us)，见 ball.h 的 BALL_VEL_I_LIMIT_US ——
         和 L 是一对：I 必须比 L 留出比例项的余量，否则积分总被比例项
         挤没，摩擦一增大就先卡死 */
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

    case 'W':
    {
      /* 任务三开环三段共用的倾角偏移量(us)，例如 W300。
         见 task.h 的 Task3_OL_Params.tilt_us / TASK3_OL_TILT_US 的说明 */
      Task3_OL_Params ol = Task3_GetOLParams();
      float           us = strtof(p, &end);

      if (end == p) { return 0; }
      ol.tilt_us = (uint16_t)us;
      Task3_SetOLParams(&ol);
      return 1;
    }

    case 'H':
    {
      /* 任务三 -5cm 处标定出的静态稳定角(us)，例如 H1640。
         见 task.h 的 Task3_OL_Params.hold_minus_us / TASK3_OL_HOLD_MINUS_US */
      Task3_OL_Params ol = Task3_GetOLParams();
      float           us = strtof(p, &end);

      if (end == p) { return 0; }
      ol.hold_minus_us = (uint16_t)us;
      Task3_SetOLParams(&ol);
      return 1;
    }

    case 'J':
    {
      /* 任务三开环->闭环的交接窗口【下限】(cm)，例如 J3.0。
         见 task.h 的 Task3_OL_Params.handoff_cm / TASK3_HANDOFF_CM */
      Task3_OL_Params ol = Task3_GetOLParams();
      float           cm = strtof(p, &end);

      if (end == p) { return 0; }
      ol.handoff_cm = cm;
      Task3_SetOLParams(&ol);
      return 1;
    }

    case 'B':
    {
      /* 任务三估停车距离用的减速度(cm/s²)，例如 B15 —— 交接窗口 = v²/(2B)。
         调过冲的主旋钮：还冲过头就【调小】(窗口变宽，提前交接留够刹车距离)。
         见 task.h 的 Task3_OL_Params.brake_accel_cms2 */
      Task3_OL_Params ol   = Task3_GetOLParams();
      float           a    = strtof(p, &end);

      if ((end == p) || (a <= 0.0f)) { return 0; }   /* 0 会除零，挡掉 */
      ol.brake_accel_cms2 = a;
      Task3_SetOLParams(&ol);
      return 1;
    }

    case '1':
    case '2':
    case '3':
    {
      /* 任务三开环阶段一/二/三的时长(ms)，例如 1300、2500、3300。
         见 task.h 的 Task3_OL_Params.t1_ms/t2_ms/t3_ms */
      Task3_OL_Params ol = Task3_GetOLParams();
      uint32_t        ms = strtoul(p, &end, 10);

      if (end == p) { return 0; }

      switch (line[0])
      {
        case '1': ol.t1_ms = ms; break;
        case '2': ol.t2_ms = ms; break;
        default:  ol.t3_ms = ms; break;
      }
      Task3_SetOLParams(&ol);
      return 1;
    }

    case 'N':
    {
      uint32_t n = strtoul(p, &end, 10);   /* 发几就是任务几，1~6，没有任务 0 */

      if ((end == p) || (n < 1U) || (n > (uint32_t)TASK_NUM))
      {
        return 0;
      }
      Task_SetId((Task_ID)(n - 1U));       /* 运行中会被 Task_SetId 自己挡掉 */
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
  /* 溢出/帧错误会让 HAL 把接收状态置回 READY 并停止接收 —— 不重新挂上的话
     通信会就此永久中断。这类错误在对端没开、波特率不匹配时很常见 */
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
  * @brief  拼一行 FireWater 文本("ch0,ch1,...,ch10\n")并非阻塞发出
  * @note   顺序必须和 vofa.h 头注释里的 ch0~ch10 列表一致 —— 上位机波形图
  *         按位置号对应通道，改顺序这里和文档要一起改。
  */
static void Vofa_SendFrame(void)
{
  int len;

  if (s_tx_busy)                 /* 上一帧还没发完，宁可少发一帧也不打断控制节拍 */
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
                 (double)(Task_GetId() + 1),   /* 发几就是任务几，跟 N 指令同一套编号 */
                 Task_IsRunning() ? 1.0 : 0.0,
                 (double)Task_GetElapsedMs() / 1000.0,
                 (double)Battery_GetVoltage());

  if (len <= 0)                  /* 编码失败，没什么好发的 */
  {
    return;
  }
  if ((size_t)len >= sizeof(s_tx_line))
  {
    len = (int)sizeof(s_tx_line) - 1;   /* 被截断：宁可发一行不完整的，也别越界 */
  }

  s_tx_busy = 1;
  if (HAL_UART_Transmit_IT(VOFA_UART, (uint8_t *)s_tx_line, (uint16_t)len) != HAL_OK)
  {
    s_tx_busy = 0;              /* 发起失败(如串口正忙于收字节)，下一拍再试 */
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
