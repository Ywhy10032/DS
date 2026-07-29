/**
  ******************************************************************************
  * @file           : app.c
  * @brief          : 应用层 —— 8 路灰度采集显示 + 双电机转速闭环
  ******************************************************************************
  */

#include "app.h"
#include "lcd_spi_200.h"
#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include "grayscale.h"

/* ================= 控制参数：调 PID 只改这一段 ================= */

/* 目标转速(输出轴 rpm)。调传感器期间设为 0，电机被闭环按住不动 */
#define APP_TARGET_RPM      0.0f

/* PID 参数。整定步骤：先 P 后 I，Kd 一般留 0 */
#define APP_PID_KP          1.5f
#define APP_PID_KI          20.0f
#define APP_PID_KD          0.0f

/* 积分项上限，取输出量程的 60%，剩下的留给比例项 */
#define APP_PID_I_LIMIT     600.0f

/* 控制周期 10ms */
#define APP_CTRL_MS         10
#define APP_CTRL_DT         (APP_CTRL_MS / 1000.0f)

/* 灰度采样周期 50ms。8 字节 I2C 读在 100kHz 下约 1ms，不会挤占控制节拍 */
#define APP_GRAY_EVERY      5

/* ================= 显示参数 ================= */
#define APP_FONT            ASCII_Font24       /* 字模 24(高) x 12(宽) */
#define APP_FONT_W          12

#define APP_TITLE_Y         10
#define APP_GRAY_Y0         50                 /* 灰度第一行 */
#define APP_GRAY_DY         32                 /* 灰度行距 */
#define APP_LRPM_Y          190
#define APP_RRPM_Y          222
#define APP_STATUS_Y        265

#define APP_GRAY_L_LABEL_X  20                 /* 左列 CH0~CH3 */
#define APP_GRAY_L_VALUE_X  (APP_GRAY_L_LABEL_X + 2 * APP_FONT_W)
#define APP_GRAY_R_LABEL_X  130                /* 右列 CH4~CH7 */
#define APP_GRAY_R_VALUE_X  (APP_GRAY_R_LABEL_X + 2 * APP_FONT_W)
#define APP_GRAY_VALUE_LEN  3

#define APP_RPM_LABEL_X     20
#define APP_RPM_VALUE_X     (APP_RPM_LABEL_X + 7 * APP_FONT_W)
#define APP_RPM_VALUE_LEN   7

/* 刷屏走 SPI 是毫秒级阻塞操作，摊开成每次只画一个字段才不会挤占控制节拍。
   共 8 路灰度 + 2 个转速 = 10 个字段，每 2 个控制周期画一个 -> 200ms 刷完一轮 */
#define APP_DRAW_EVERY      2
#define APP_FIELD_NUM       (GRAY_CHANNEL_NUM + 2)

/* 显示用的一阶低通系数：单次 10ms 采样有量化抖动，平滑后读数才稳 */
#define APP_DISP_ALPHA      0.15f

/* ================= 运行时状态 ================= */
static PID_Controller s_pid[MOTOR_NUM];
static float          s_rpm[MOTOR_NUM]      = {0.0f, 0.0f};   /* 实测转速 */
static float          s_out[MOTOR_NUM]      = {0.0f, 0.0f};   /* PID 输出 */
static float          s_rpm_disp[MOTOR_NUM] = {0.0f, 0.0f};   /* 平滑后的显示值 */

static uint8_t        s_gray[GRAY_CHANNEL_NUM] = {0};
static HAL_StatusTypeDef s_gray_status = HAL_ERROR;

static uint32_t s_last_tick  = 0;
static uint8_t  s_draw_cnt   = 0;
static uint8_t  s_draw_field = 0;
static uint8_t  s_gray_cnt   = 0;

/**
  * @brief  画出不会变化的部分，之后只刷新数值
  */
static void App_DrawStaticLayout(void)
{
  char label[4] = {'0', ':', '\0', '\0'};

  LCD_SetAsciiFont(&APP_FONT);
  LCD_SetColor(LCD_WHITE);

  LCD_DisplayString((LCD_Width - 11 * APP_FONT_W) / 2, APP_TITLE_Y, "GRAY SENSOR");

  /* 左列 CH0~CH3，右列 CH4~CH7 */
  for (uint8_t i = 0; i < GRAY_CHANNEL_NUM; i++)
  {
    uint16_t x = (i < 4) ? APP_GRAY_L_LABEL_X : APP_GRAY_R_LABEL_X;
    uint16_t y = APP_GRAY_Y0 + (i % 4) * APP_GRAY_DY;

    label[0] = (char)('0' + i);
    LCD_DisplayString(x, y, label);
  }

  LCD_DisplayString(APP_RPM_LABEL_X, APP_LRPM_Y, "L RPM:");
  LCD_DisplayString(APP_RPM_LABEL_X, APP_RRPM_Y, "R RPM:");
  LCD_DisplayString(APP_RPM_LABEL_X, APP_STATUS_Y, "I2C:");
}

/**
  * @brief  刷新 I2C 状态行，只在初始化和状态变化时调用
  */
static void App_DrawStatus(void)
{
  LCD_SetAsciiFont(&APP_FONT);

  if (s_gray_status == HAL_OK)
  {
    LCD_SetColor(LCD_GREEN);
    LCD_DisplayString(APP_RPM_LABEL_X + 5 * APP_FONT_W, APP_STATUS_Y, "OK  ");
  }
  else
  {
    LCD_SetColor(LCD_RED);
    LCD_DisplayString(APP_RPM_LABEL_X + 5 * APP_FONT_W, APP_STATUS_Y, "FAIL");
  }
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
    LCD_DisplayNumber(x, y, (int32_t)s_gray[field], APP_GRAY_VALUE_LEN);
  }
  else if (field == GRAY_CHANNEL_NUM)
  {
    LCD_SetColor(LCD_GREEN);
    LCD_DisplayDecimals(APP_RPM_VALUE_X, APP_LRPM_Y,
                        s_rpm_disp[MOTOR_LEFT], APP_RPM_VALUE_LEN, 1);
  }
  else
  {
    LCD_SetColor(LCD_GREEN);
    LCD_DisplayDecimals(APP_RPM_VALUE_X, APP_RRPM_Y,
                        s_rpm_disp[MOTOR_RIGHT], APP_RPM_VALUE_LEN, 1);
  }
}

/**
  * @brief  跑一拍速度环
  */
static void App_SpeedLoop(void)
{
  for (uint8_t i = 0; i < MOTOR_NUM; i++)
  {
    Encoder_ID enc = (i == MOTOR_LEFT) ? ENCODER_LEFT : ENCODER_RIGHT;

    s_rpm[i] = Encoder_GetRPM(enc, APP_CTRL_MS);
    s_out[i] = PID_Update(&s_pid[i], APP_TARGET_RPM, s_rpm[i]);

    Motor_SetSpeed((Motor_ID)i, (int16_t)s_out[i]);

    /* 一阶低通，只用于显示，不参与闭环 */
    s_rpm_disp[i] += APP_DISP_ALPHA * (s_rpm[i] - s_rpm_disp[i]);
  }
}

void App_Init(void)
{
  /* ---------- LCD 初始化 ---------- */
  SPI_LCD_Init();
  LCD_SetDirection(Direction_V);        /* 竖屏 240x320 */
  LCD_SetBackColor(LCD_BLACK);
  LCD_SetColor(LCD_WHITE);
  LCD_Clear();
  LCD_ShowNumMode(Fill_Space);          /* 数值右对齐补空格，位数变化不留残影 */

  App_DrawStaticLayout();

  /* ---------- 灰度传感器 ---------- */
  /* 内部会阻塞轮询 ping，等传感器上电就绪，最长 3 秒 */
  s_gray_status = Gray_Init();
  App_DrawStatus();

  /* ---------- 电机与编码器 ---------- */
  Motor_Init();                         /* 完成后 STBY=0、速度为 0 */
  Encoder_Init();

  /* ---------- 两路速度环 ---------- */
  for (uint8_t i = 0; i < MOTOR_NUM; i++)
  {
    PID_Init(&s_pid[i], APP_PID_KP, APP_PID_KI, APP_PID_KD, APP_CTRL_DT);
    PID_SetOutputLimits(&s_pid[i], -(float)MOTOR_SPEED_MAX, (float)MOTOR_SPEED_MAX);
    PID_SetIntegralLimit(&s_pid[i], APP_PID_I_LIMIT);
  }

  Motor_Enable();                       /* 解除待机，闭环开始接管 */

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

  App_SpeedLoop();

  /* ---------- 每 APP_GRAY_EVERY 拍读一次灰度 ---------- */
  s_gray_cnt++;
  if (s_gray_cnt >= APP_GRAY_EVERY)
  {
    HAL_StatusTypeDef status;

    s_gray_cnt = 0;
    status = Gray_ReadAll(s_gray);

    if (status != s_gray_status)
    {
      s_gray_status = status;
      App_DrawStatus();
    }
  }

  /* ---------- 刷屏摊到不同的控制周期上，每次只画一个字段 ---------- */
  s_draw_cnt++;
  if (s_draw_cnt >= APP_DRAW_EVERY)
  {
    s_draw_cnt = 0;
    App_DrawField(s_draw_field);
    s_draw_field = (uint8_t)((s_draw_field + 1) % APP_FIELD_NUM);
  }
}
