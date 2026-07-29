/**
  ******************************************************************************
  * @file           : app.c
  * @brief          : 应用层 —— 双电机 20% PWM 开环运行，屏幕实时显示转速
  ******************************************************************************
  */

#include "app.h"
#include "lcd_spi_200.h"
#include "motor.h"
#include "encoder.h"

/* ---------------- 显示参数 ---------------- */
#define APP_FONT            ASCII_Font24       /* 字模 24(高) x 12(宽) */
#define APP_FONT_W          12
#define APP_FONT_H          24

#define APP_TITLE_Y         20
#define APP_PWM_Y           60
#define APP_LEFT_Y          120
#define APP_RIGHT_Y         160
#define APP_LABEL_X         20
#define APP_VALUE_X         (APP_LABEL_X + 7 * APP_FONT_W)   /* "L RPM: " 之后 */
#define APP_VALUE_LEN       7                  /* 含符号与小数点的总宽度 */
#define APP_VALUE_DECS      1                  /* 小数位数 */

/* ---------------- 运行参数 ---------------- */
#define APP_PWM_PERCENT     20
#define APP_MOTOR_SPEED     (MOTOR_SPEED_MAX * APP_PWM_PERCENT / 100)   /* 200 */

/* 采样周期取 50ms：即便电机满速(约 345000 计数/秒)也远小于 16 位计数器
   32767 计数的回绕窗口(95ms)，不会丢圈 */
#define APP_SAMPLE_MS       50
/* 每 4 次采样刷新一屏，取平均可以抹掉单次采样的抖动 */
#define APP_SAMPLES_PER_DRAW 4

/* ---------------- 运行时状态 ---------------- */
static uint32_t s_last_tick = 0;
static float    s_rpm_acc[ENCODER_NUM] = {0.0f, 0.0f};
static uint8_t  s_sample_cnt = 0;

/**
  * @brief  画出不会变化的部分，之后只刷新数值区域
  */
static void App_DrawStaticLayout(void)
{
  LCD_SetColor(LCD_WHITE);
  LCD_SetAsciiFont(&APP_FONT);

  LCD_DisplayString((LCD_Width - 10 * APP_FONT_W) / 2, APP_TITLE_Y, "MOTOR TEST");

  LCD_DisplayString(APP_LABEL_X, APP_PWM_Y,    "PWM:  20%");
  LCD_DisplayString(APP_LABEL_X, APP_LEFT_Y,   "L RPM:");
  LCD_DisplayString(APP_LABEL_X, APP_RIGHT_Y,  "R RPM:");
}

/**
  * @brief  刷新左右两个转速数值
  * @note   数值右对齐补空格，且驱动会连背景一起重绘，无需先擦除
  */
static void App_DrawSpeeds(float left_rpm, float right_rpm)
{
  LCD_SetColor(LCD_GREEN);
  LCD_SetAsciiFont(&APP_FONT);

  LCD_DisplayDecimals(APP_VALUE_X, APP_LEFT_Y,  left_rpm,  APP_VALUE_LEN, APP_VALUE_DECS);
  LCD_DisplayDecimals(APP_VALUE_X, APP_RIGHT_Y, right_rpm, APP_VALUE_LEN, APP_VALUE_DECS);
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

  /* ---------- 电机与编码器初始化 ---------- */
  Motor_Init();                         /* 完成后 STBY=0、速度为 0 */
  Encoder_Init();

  /* ---------- 双电机 20% PWM 正转 ---------- */
  Motor_SetSpeeds(APP_MOTOR_SPEED, APP_MOTOR_SPEED);
  Motor_Enable();                       /* 解除待机，电机开始转动 */

  s_last_tick = HAL_GetTick();
}

void App_Run(void)
{
  uint32_t now = HAL_GetTick();

  if ((now - s_last_tick) < APP_SAMPLE_MS)
  {
    return;
  }
  s_last_tick = now;

  /* 每 APP_SAMPLE_MS 采一次，保证编码器计数器不会回绕过头 */
  s_rpm_acc[ENCODER_LEFT]  += Encoder_GetRPM(ENCODER_LEFT,  APP_SAMPLE_MS);
  s_rpm_acc[ENCODER_RIGHT] += Encoder_GetRPM(ENCODER_RIGHT, APP_SAMPLE_MS);
  s_sample_cnt++;

  if (s_sample_cnt >= APP_SAMPLES_PER_DRAW)
  {
    App_DrawSpeeds(s_rpm_acc[ENCODER_LEFT]  / APP_SAMPLES_PER_DRAW,
                   s_rpm_acc[ENCODER_RIGHT] / APP_SAMPLES_PER_DRAW);

    s_rpm_acc[ENCODER_LEFT]  = 0.0f;
    s_rpm_acc[ENCODER_RIGHT] = 0.0f;
    s_sample_cnt = 0;
  }
}
