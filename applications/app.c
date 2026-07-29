/**
  ******************************************************************************
  * @file           : app.c
  * @brief          : 应用层 —— 任务调度 + 循迹串级控制 + 显示
  ******************************************************************************
  * 调度层次：
  *   按键  10ms  Key_Scan()
  *   任务  20ms  Task_Update()  按键事件 / 计时 / 里程 / 终点判定
  *   外环  20ms  Track_Update() 灰度8路 -> 加权质心 -> 转向PID -> 左右目标转速
  *   内环  10ms  速度PID -> PWM
  ******************************************************************************
  */

#include "app.h"
#include "lcd_spi_200.h"
#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include "grayscale.h"
#include "tracking.h"
#include "key.h"
#include "task.h"

/* ================= 控制参数 ================= */

/* 速度环 PID */
#define APP_PID_KP          1.5f
#define APP_PID_KI          20.0f
#define APP_PID_KD          0.0f
#define APP_PID_I_LIMIT     600.0f

/* 内环周期 10ms */
#define APP_CTRL_MS         10
#define APP_CTRL_DT         (APP_CTRL_MS / 1000.0f)

/* 外环与任务层每 2 拍跑一次 = 20ms，须与 TRACK_PERIOD_MS 一致 */
#define APP_OUTER_EVERY     (TRACK_PERIOD_MS / APP_CTRL_MS)

/* ================= 显示参数 ================= */
#define APP_FONT            ASCII_Font24       /* 字模 24(高) x 12(宽) */
#define APP_FONT_W          12

#define APP_TASK_Y          8
#define APP_STATE_Y         40
#define APP_TIME_Y          72
#define APP_GRAY_Y0         112                /* 灰度第一行 */
#define APP_GRAY_DY         28
#define APP_OFF_Y           228
#define APP_RPM_Y           258
#define APP_I2C_Y           288

#define APP_LABEL_X         14
#define APP_VALUE_X         (APP_LABEL_X + 7 * APP_FONT_W)
#define APP_VALUE_LEN       7

#define APP_GRAY_L_LABEL_X  20                 /* 左列 CH0~CH3 */
#define APP_GRAY_L_VALUE_X  (APP_GRAY_L_LABEL_X + 2 * APP_FONT_W)
#define APP_GRAY_R_LABEL_X  130                /* 右列 CH4~CH7 */
#define APP_GRAY_R_VALUE_X  (APP_GRAY_R_LABEL_X + 2 * APP_FONT_W)
#define APP_GRAY_VALUE_LEN  3

#define APP_RPM_L_X         14
#define APP_RPM_R_X         126
#define APP_RPM_VALUE_LEN   4

/* 偏差与"几路黑"共用一行，DK 用来现场标定横线阈值 */
#define APP_OFF_VALUE_LEN   6
#define APP_DK_LABEL_X      174
#define APP_DK_VALUE_X      (APP_DK_LABEL_X + 3 * APP_FONT_W)
#define APP_DK_VALUE_LEN    2

/* 刷屏走 SPI 是毫秒级阻塞操作，摊开成每次只画一个字段才不会挤占控制节拍。
   8 路灰度 + 偏差 + 黑路数 + 2 个转速 + 里程 = 13 个字段 */
#define APP_DRAW_EVERY      2
#define APP_FIELD_NUM       (GRAY_CHANNEL_NUM + 5)
#define APP_FIELD_OFFSET    (GRAY_CHANNEL_NUM)
#define APP_FIELD_DARK      (GRAY_CHANNEL_NUM + 1)
#define APP_FIELD_LRPM      (GRAY_CHANNEL_NUM + 2)
#define APP_FIELD_RRPM      (GRAY_CHANNEL_NUM + 3)
#define APP_FIELD_DIST      (GRAY_CHANNEL_NUM + 4)

/* 里程显示，用来校准轮径：跑完一圈应该读到赛道实测长度 */
#define APP_DIST_LABEL_X    140
#define APP_DIST_VALUE_X    (APP_DIST_LABEL_X + 2 * APP_FONT_W)
#define APP_DIST_VALUE_LEN  5

/* 计时单独按 100ms 刷新，不跟着字段轮转，否则秒表跳得太慢不像话 */
#define APP_TIME_EVERY      10

/* 显示用的一阶低通系数 */
#define APP_DISP_ALPHA      0.15f

/* ================= 运行时状态 ================= */
static PID_Controller s_pid[MOTOR_NUM];
static float          s_rpm[MOTOR_NUM]      = {0.0f, 0.0f};
static float          s_out[MOTOR_NUM]      = {0.0f, 0.0f};
static float          s_target[MOTOR_NUM]   = {0.0f, 0.0f};
static float          s_rpm_disp[MOTOR_NUM] = {0.0f, 0.0f};

static HAL_StatusTypeDef s_gray_status = HAL_ERROR;
static Task_ID           s_shown_task  = TASK_NUM;      /* 强制首次重画 */
static Task_State        s_shown_state = TASK_STATE_DONE;

static uint32_t s_last_tick  = 0;
static uint8_t  s_outer_cnt  = 0;
static uint8_t  s_draw_cnt   = 0;
static uint8_t  s_draw_field = 0;
static uint8_t  s_time_cnt   = 0;

/**
  * @brief  画出不会变化的部分
  */
static void App_DrawStaticLayout(void)
{
  char label[4] = {'0', ':', '\0', '\0'};

  LCD_SetAsciiFont(&APP_FONT);
  LCD_SetColor(LCD_WHITE);

  LCD_DisplayString(APP_LABEL_X, APP_TIME_Y, "TIME:");

  for (uint8_t i = 0; i < GRAY_CHANNEL_NUM; i++)
  {
    uint16_t x = (i < 4) ? APP_GRAY_L_LABEL_X : APP_GRAY_R_LABEL_X;
    uint16_t y = APP_GRAY_Y0 + (i % 4) * APP_GRAY_DY;

    label[0] = (char)('0' + i);
    LCD_DisplayString(x, y, label);
  }

  LCD_DisplayString(APP_LABEL_X, APP_OFF_Y, "OFF:");
  LCD_DisplayString(APP_DK_LABEL_X, APP_OFF_Y, "DK:");
  LCD_DisplayString(APP_RPM_L_X, APP_RPM_Y, "L:");
  LCD_DisplayString(APP_RPM_R_X, APP_RPM_Y, "R:");
  LCD_DisplayString(APP_LABEL_X, APP_I2C_Y, "I2C:");
  LCD_DisplayString(APP_DIST_LABEL_X, APP_I2C_Y, "D:");
}

/**
  * @brief  任务号与状态，只在变化时重画
  */
static void App_DrawTaskLine(void)
{
  char title[8] = "TASK 1";

  LCD_SetAsciiFont(&APP_FONT);
  LCD_SetColor(LCD_WHITE);

  title[5] = (char)('1' + Task_GetId());
  LCD_DisplayString((LCD_Width - 6 * APP_FONT_W) / 2, APP_TASK_Y, title);
}

static void App_DrawStateLine(void)
{
  LCD_SetAsciiFont(&APP_FONT);

  switch (Task_GetState())
  {
    case TASK_STATE_RUN:
      LCD_SetColor(LCD_GREEN);
      LCD_DisplayString(APP_LABEL_X, APP_STATE_Y, "RUNNING ");
      break;

    case TASK_STATE_DONE:
      LCD_SetColor(LCD_YELLOW);
      LCD_DisplayString(APP_LABEL_X, APP_STATE_Y, "FINISHED");
      break;

    default:
      LCD_SetColor(LCD_CYAN);
      LCD_DisplayString(APP_LABEL_X, APP_STATE_Y, "READY   ");
      break;
  }
}

static void App_DrawStatus(void)
{
  LCD_SetAsciiFont(&APP_FONT);

  if (s_gray_status == HAL_OK)
  {
    LCD_SetColor(LCD_GREEN);
    LCD_DisplayString(APP_LABEL_X + 5 * APP_FONT_W, APP_I2C_Y, "OK  ");
  }
  else
  {
    LCD_SetColor(LCD_RED);
    LCD_DisplayString(APP_LABEL_X + 5 * APP_FONT_W, APP_I2C_Y, "FAIL");
  }
}

/**
  * @brief  行驶总时间，秒 + 两位小数
  */
static void App_DrawTime(void)
{
  LCD_SetAsciiFont(&APP_FONT);
  LCD_SetColor(Task_IsRunning() ? LCD_WHITE : LCD_YELLOW);
  LCD_DisplayDecimals(APP_VALUE_X, APP_TIME_Y,
                      (double)Task_GetElapsedMs() / 1000.0, APP_VALUE_LEN, 2);
}

/**
  * @brief  重画一个数值字段
  * @note   数值右对齐补空格，驱动会连背景一起重绘，不用先擦除
  */
static void App_DrawField(uint8_t field)
{
  LCD_SetAsciiFont(&APP_FONT);

  if (field < GRAY_CHANNEL_NUM)
  {
    uint16_t x = (field < 4) ? APP_GRAY_L_VALUE_X : APP_GRAY_R_VALUE_X;
    uint16_t y = APP_GRAY_Y0 + (field % 4) * APP_GRAY_DY;

    LCD_SetColor(LCD_YELLOW);
    LCD_DisplayNumber(x, y, (int32_t)Track_GetRaw()[field], APP_GRAY_VALUE_LEN);
  }
  else if (field == APP_FIELD_OFFSET)
  {
    /* 压在横线上标绿、丢线标红，一眼能看出传感器当前的处境 */
    if (Track_IsCrossLine())
    {
      LCD_SetColor(LCD_GREEN);
    }
    else
    {
      LCD_SetColor(Track_IsLost() ? LCD_RED : LCD_CYAN);
    }
    LCD_DisplayDecimals(APP_VALUE_X, APP_OFF_Y, Track_GetOffset(), APP_OFF_VALUE_LEN, 1);
  }
  else if (field == APP_FIELD_DARK)
  {
    /* 有几路探头看到黑色。推着车过 A 点，看这里的峰值就能定横线阈值 */
    LCD_SetColor(Track_IsCrossLine() ? LCD_GREEN : LCD_WHITE);
    LCD_DisplayNumber(APP_DK_VALUE_X, APP_OFF_Y,
                      (int32_t)Track_GetDarkCount(), APP_DK_VALUE_LEN);
  }
  else if (field == APP_FIELD_LRPM)
  {
    LCD_SetColor(LCD_GREEN);
    LCD_DisplayNumber(APP_RPM_L_X + 2 * APP_FONT_W, APP_RPM_Y,
                      (int32_t)s_rpm_disp[MOTOR_LEFT], APP_RPM_VALUE_LEN);
  }
  else if (field == APP_FIELD_RRPM)
  {
    LCD_SetColor(LCD_GREEN);
    LCD_DisplayNumber(APP_RPM_R_X + 2 * APP_FONT_W, APP_RPM_Y,
                      (int32_t)s_rpm_disp[MOTOR_RIGHT], APP_RPM_VALUE_LEN);
  }
  else
  {
    LCD_SetColor(LCD_WHITE);
    LCD_DisplayDecimals(APP_DIST_VALUE_X, APP_I2C_Y,
                        Task_GetDistanceM(), APP_DIST_VALUE_LEN, 2);
  }
}

/**
  * @brief  跑一拍速度环(内环)
  */
static void App_SpeedLoop(void)
{
  for (uint8_t i = 0; i < MOTOR_NUM; i++)
  {
    Encoder_ID enc = (i == MOTOR_LEFT) ? ENCODER_LEFT : ENCODER_RIGHT;

    s_rpm[i] = Encoder_GetRPM(enc, APP_CTRL_MS);
    s_out[i] = PID_Update(&s_pid[i], s_target[i], s_rpm[i]);

    Motor_SetSpeed((Motor_ID)i, (int16_t)s_out[i]);

    s_rpm_disp[i] += APP_DISP_ALPHA * (s_rpm[i] - s_rpm_disp[i]);
  }
}

/**
  * @brief  任务未运行时的处理：刹车并把速度环清干净
  * @note   必须清积分，否则下次启动时会带着上一轮的残留猛冲一下
  */
static void App_Idle(void)
{
  for (uint8_t i = 0; i < MOTOR_NUM; i++)
  {
    Encoder_ID enc = (i == MOTOR_LEFT) ? ENCODER_LEFT : ENCODER_RIGHT;

    s_rpm[i]    = Encoder_GetRPM(enc, APP_CTRL_MS);
    s_target[i] = 0.0f;
    s_out[i]    = 0.0f;
    PID_Reset(&s_pid[i]);

    s_rpm_disp[i] += APP_DISP_ALPHA * (s_rpm[i] - s_rpm_disp[i]);
  }

  Motor_BrakeAll();
}

void App_Init(void)
{
  /* ---------- LCD ---------- */
  SPI_LCD_Init();
  LCD_SetDirection(Direction_V);        /* 竖屏 240x320 */
  LCD_SetBackColor(LCD_BLACK);
  LCD_SetColor(LCD_WHITE);
  LCD_Clear();
  LCD_ShowNumMode(Fill_Space);

  App_DrawStaticLayout();

  /* ---------- 灰度传感器 ---------- */
  s_gray_status = Gray_Init();
  App_DrawStatus();

  /* ---------- 电机与编码器 ---------- */
  Motor_Init();
  Encoder_Init();

  /* ---------- 内环：两路速度 PID ---------- */
  for (uint8_t i = 0; i < MOTOR_NUM; i++)
  {
    PID_Init(&s_pid[i], APP_PID_KP, APP_PID_KI, APP_PID_KD, APP_CTRL_DT);
    PID_SetOutputLimits(&s_pid[i], -(float)MOTOR_SPEED_MAX, (float)MOTOR_SPEED_MAX);
    PID_SetIntegralLimit(&s_pid[i], APP_PID_I_LIMIT);
  }

  /* ---------- 外环与任务层 ---------- */
  Track_Init();
  Task_Init();

  App_DrawTaskLine();
  App_DrawStateLine();
  App_DrawTime();
  s_shown_task  = Task_GetId();
  s_shown_state = Task_GetState();

  Motor_Enable();                       /* 解除待机，但任务未启动时是刹车状态 */

  s_last_tick = HAL_GetTick();
}

void App_Run(void)
{
  uint32_t now = HAL_GetTick();

  if ((now - s_last_tick) < APP_CTRL_MS)
  {
    return;
  }
  s_last_tick = now;

  /* ---------- 按键：每拍扫描 ---------- */
  Key_Scan();

  /* ---------- 任务层 + 循迹外环：每 2 拍 ---------- */
  s_outer_cnt++;
  if (s_outer_cnt >= APP_OUTER_EVERY)
  {
    s_outer_cnt = 0;

    /* 灰度没通就不许跑，否则会拿着全 0 的数据一头冲出去 */
    if (Task_IsRunning() && (s_gray_status == HAL_OK))
    {
      HAL_StatusTypeDef status;

      Track_Update();
      Track_GetTargets(&s_target[MOTOR_LEFT], &s_target[MOTOR_RIGHT]);

      status = Track_GetStatus();
      if (status != s_gray_status)
      {
        s_gray_status = status;
        App_DrawStatus();
      }
    }

    /* Task_Update 放在 Track_Update 之后，这样终点判定用的是本拍的新数据 */
    Task_Update();
  }

  /* ---------- 内环：每拍 ---------- */
  if (Task_IsRunning())
  {
    App_SpeedLoop();
  }
  else
  {
    App_Idle();
  }

  /* ---------- 显示 ---------- */
  if (Task_GetId() != s_shown_task)
  {
    s_shown_task = Task_GetId();
    App_DrawTaskLine();
  }
  if (Task_GetState() != s_shown_state)
  {
    s_shown_state = Task_GetState();
    App_DrawStateLine();
  }

  s_time_cnt++;
  if (s_time_cnt >= APP_TIME_EVERY)
  {
    s_time_cnt = 0;
    App_DrawTime();
  }

  s_draw_cnt++;
  if (s_draw_cnt >= APP_DRAW_EVERY)
  {
    s_draw_cnt = 0;
    App_DrawField(s_draw_field);
    s_draw_field = (uint8_t)((s_draw_field + 1) % APP_FIELD_NUM);
  }
}
