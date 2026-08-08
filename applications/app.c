/**
  ******************************************************************************
  * @file           : app.c
  * @brief          : 应用层调度、车轮速度闭环与界面刷新
  ******************************************************************************
  * 调度周期：
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
#include "servo.h"
#include "vision.h"
#include "ball.h"
#include "vofa.h"
#include "battery.h"
#include "buzzer.h"

#include <math.h>

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

/* 电压角标在所有页面显示。屏幕翻转后，逻辑左上角对应物理右下角。 */
#define APP_BATT_FONT       ASCII_Font16
#define APP_BATT_FONT_W     8
#define APP_BATT_X          4
#define APP_BATT_Y          4
#define APP_BATT_VALUE_X    (APP_BATT_X + 4 * APP_BATT_FONT_W)
#define APP_BATT_VALUE_LEN  5

/* 偏差与黑色通道数共用一行，DK 用于标定横线阈值 */
#define APP_OFF_VALUE_LEN   6
#define APP_DK_LABEL_X      174
#define APP_DK_VALUE_X      (APP_DK_LABEL_X + 3 * APP_FONT_W)
#define APP_DK_VALUE_LEN    2

/**
  * 舵机工作模式，四种模式互斥。
  * OFF：关闭 PWM；DEMO：行程自检；MANUAL：按键标定；BALL：滚球闭环。
  */
#define APP_SERVO_OFF       0
#define APP_SERVO_DEMO      1
#define APP_SERVO_MANUAL    2
#define APP_SERVO_BALL      3

#define APP_SERVO_MODE      APP_SERVO_BALL

#define APP_SERVO_ENABLE    (APP_SERVO_MODE != APP_SERVO_OFF)

/* KEY3 循环切换页面；手动标定模式占用 KEY3/KEY4，不执行翻页。 */
typedef enum
{
  APP_PAGE_MAIN = 0,      /* 循迹主界面 */
  APP_PAGE_PID,           /* 球杆闭环当前生效的六个 PID 增益 */
  APP_PAGE_BALL,          /* 球杆闭环 */
  APP_PAGE_NUM
} App_Page;

/* 每次只刷新一个字段，减少 SPI 阻塞对控制周期的影响。 */
#define APP_DRAW_EVERY      2
#if APP_SERVO_ENABLE
#define APP_MAIN_FIELD_NUM  (GRAY_CHANNEL_NUM + 6)
#else
#define APP_MAIN_FIELD_NUM  (GRAY_CHANNEL_NUM + 5)   /* 不显示舵机脉宽 */
#endif
#define APP_FIELD_OFFSET    (GRAY_CHANNEL_NUM)
#define APP_FIELD_DARK      (GRAY_CHANNEL_NUM + 1)
#define APP_FIELD_LRPM      (GRAY_CHANNEL_NUM + 2)
#define APP_FIELD_RRPM      (GRAY_CHANNEL_NUM + 3)
#define APP_FIELD_DIST      (GRAY_CHANNEL_NUM + 4)
#define APP_FIELD_SERVO     (GRAY_CHANNEL_NUM + 5)

/* ---------------- PID 页 ---------------- */
/* PID 页面沿用原视觉页面的坐标常量 */
#define APP_VIS_Y0          52
#define APP_VIS_DY          32
#define APP_VIS_VALUE_X     (APP_LABEL_X + 6 * APP_FONT_W)
#define APP_VIS_VALUE_LEN   8

/* 显示 Ball_Tune 当前生效的六个增益，与 VOFA+ 调参数据一致。 */
enum
{
  APP_PID_POS_KP = 0,     /* 位置环(外环) Kp */
  APP_PID_POS_KI,         /* 位置环(外环) Ki */
  APP_PID_POS_KD,         /* 位置环(外环) Kd */
  APP_PID_VEL_KP,         /* 速度环(内环) Kp */
  APP_PID_VEL_KI,         /* 速度环(内环) Ki */
  APP_PID_VEL_KD,         /* 速度环(内环) Kd */
  APP_PID_FIELD_NUM
};

/* ---------------- 球杆页 ---------------- */
enum
{
  APP_BALL_SET = 0,       /* 目标位置 cm */
  APP_BALL_POS,           /* 实测位置 cm */
  APP_BALL_ERR,           /* 偏差 cm */
  APP_BALL_VEL,           /* 球速 cm/s(实测) */
  APP_BALL_VSET,          /* 位置环输出的速度指令，cm/s */
  APP_BALL_OUT,           /* 速度环输出，相对水平点的 us 偏移 */
  APP_BALL_US,            /* 实际下发的舵机脉宽 */
  APP_BALL_STATE,         /* 是否正在闭环 */
  APP_BALL_FIELD_NUM
};

/* ERR 超过这个值就标红。取任务五/六要求的 1cm */
#define APP_BALL_ERR_LIMIT_CM   1.0f

/* 舵机脉宽，标定机构行程时直接读这个数 */
#define APP_SERVO_LABEL_X   120
#define APP_SERVO_VALUE_X   (APP_SERVO_LABEL_X + 3 * APP_FONT_W)
#define APP_SERVO_VALUE_LEN 4

/* 里程显示，用来校准轮径：跑完一圈应该读到赛道实测长度 */
#define APP_DIST_LABEL_X    140
#define APP_DIST_VALUE_X    (APP_DIST_LABEL_X + 2 * APP_FONT_W)
#define APP_DIST_VALUE_LEN  5

/* 计时字段每 100ms 刷新一次 */
#define APP_TIME_EVERY      10

/* 显示用的一阶低通系数 */
#define APP_DISP_ALPHA      0.15f

/* 实测转速低于此值(rpm)就认为车停稳了，可以从主动反拖切换成短路刹车驻车 */
#define APP_STOP_RPM_TH     3.0f

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
/* 默认显示滚球闭环页面 */
static App_Page s_page       = APP_PAGE_BALL;

/**
  * @brief  当前页有几个数值字段
  */
static uint8_t App_FieldCount(void)
{
  switch (s_page)
  {
    case APP_PAGE_PID:    return APP_PID_FIELD_NUM;
    case APP_PAGE_BALL:   return APP_BALL_FIELD_NUM;
    default:              return APP_MAIN_FIELD_NUM;
  }
}

/**
  * @brief  画主界面里不会变化的部分
  */
static void App_DrawStaticLayout(void)
{
  char label[4] = {'0', ':', '\0', '\0'};

  LCD_SetAsciiFont(&APP_FONT);
  LCD_SetColor(LCD_WHITE);

  LCD_DisplayString(APP_LABEL_X, APP_TIME_Y, "TIME:");
#if APP_SERVO_ENABLE
  LCD_DisplayString(APP_SERVO_LABEL_X, APP_STATE_Y, "US:");
#endif

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
  * @brief  画 PID 页里不会变化的部分
  */
static void App_ShowPage(void);

static void App_DrawPidLayout(void)
{
  static const char *labels[APP_PID_FIELD_NUM] =
  {
    "PKP:", "PKI:", "PKD:", "VKP:", "VKI:", "VKD:"
  };

  LCD_SetAsciiFont(&APP_FONT);
  LCD_SetColor(LCD_WHITE);

  LCD_DisplayString((LCD_Width - 3 * APP_FONT_W) / 2, APP_TASK_Y, "PID");

  for (uint8_t i = 0; i < APP_PID_FIELD_NUM; i++)
  {
    LCD_DisplayString(APP_LABEL_X, APP_VIS_Y0 + i * APP_VIS_DY, (char *)labels[i]);
  }
}

/**
  * @brief  画球杆页里不会变化的部分
  */
static void App_DrawBallLayout(void)
{
  static const char *labels[APP_BALL_FIELD_NUM] =
  {
    "SET:", "POS:", "ERR:", "VEL:", "VSET:", "OUT:", "US:", "RUN:"
  };

  LCD_SetAsciiFont(&APP_FONT);
  LCD_SetColor(LCD_WHITE);

  LCD_DisplayString((LCD_Width - 4 * APP_FONT_W) / 2, APP_TASK_Y, "BALL");

  for (uint8_t i = 0; i < APP_BALL_FIELD_NUM; i++)
  {
    LCD_DisplayString(APP_LABEL_X, APP_VIS_Y0 + i * APP_VIS_DY, (char *)labels[i]);
  }
}

/**
  * @brief  重画球杆页的一个数值字段
  */
static void App_DrawBallField(uint8_t field)
{
  uint16_t y = APP_VIS_Y0 + field * APP_VIS_DY;

  LCD_SetAsciiFont(&APP_FONT);
  LCD_SetColor(Ball_IsTracking() ? LCD_GREEN : LCD_RED);

  switch (field)
  {
    case APP_BALL_SET:
      LCD_SetColor(LCD_WHITE);
      LCD_DisplayDecimals(APP_VIS_VALUE_X, y, Ball_GetTarget(), APP_VIS_VALUE_LEN, 2);
      break;

    case APP_BALL_POS:
      LCD_DisplayDecimals(APP_VIS_VALUE_X, y, Ball_GetPosCm(), APP_VIS_VALUE_LEN, 2);
      break;

    case APP_BALL_ERR:
    {
      float err = Ball_GetTarget() - Ball_GetPosCm();

      /* 超过任务五、六的 1cm 误差门限时标红 */
      LCD_SetColor((fabsf(err) > APP_BALL_ERR_LIMIT_CM) ? LCD_RED : LCD_GREEN);
      LCD_DisplayDecimals(APP_VIS_VALUE_X, y, err, APP_VIS_VALUE_LEN, 2);
      break;
    }

    case APP_BALL_VEL:
      LCD_DisplayDecimals(APP_VIS_VALUE_X, y, Ball_GetVelCmS(), APP_VIS_VALUE_LEN, 1);
      break;

    case APP_BALL_VSET:
      /* 显示位置环输出的速度指令 */
      LCD_SetColor(LCD_CYAN);
      LCD_DisplayDecimals(APP_VIS_VALUE_X, y, Ball_GetVelSetCmS(), APP_VIS_VALUE_LEN, 1);
      break;

    case APP_BALL_OUT:
      LCD_SetColor(LCD_CYAN);
      LCD_DisplayNumber(APP_VIS_VALUE_X, y, (int32_t)Ball_GetOutputUs(), APP_VIS_VALUE_LEN);
      break;

    case APP_BALL_US:
      LCD_SetColor(LCD_YELLOW);
      LCD_DisplayNumber(APP_VIS_VALUE_X, y, (int32_t)Servo_GetPulseUs(), APP_VIS_VALUE_LEN);
      break;

    default:    /* APP_BALL_STATE */
      LCD_DisplayString(APP_VIS_VALUE_X, y,
                        Ball_IsEnabled() ? (Ball_IsTracking() ? " TRACK " : " LOST  ")
                                         : "  OFF  ");
      break;
  }
}

/**
  * @brief  任务号与状态，只在变化时重画
  */
static void App_DrawTaskLine(void)
{
  /* 主界面专用字段 */
  if (s_page != APP_PAGE_MAIN)
  {
    return;
  }

  char title[8] = "TASK 1";

  LCD_SetAsciiFont(&APP_FONT);
  LCD_SetColor(LCD_WHITE);

  title[5] = (char)('1' + Task_GetId());
  LCD_DisplayString((LCD_Width - 6 * APP_FONT_W) / 2, APP_TASK_Y, title);
}

static void App_DrawStateLine(void)
{
  /* 主界面专用字段 */
  if (s_page != APP_PAGE_MAIN)
  {
    return;
  }

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
  /* 主界面专用字段 */
  if (s_page != APP_PAGE_MAIN)
  {
    return;
  }

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
  * @brief  电压角标：标签只需画一次，翻页清屏后跟着页面布局重画
  */
static void App_DrawBatteryLabel(void)
{
  LCD_SetAsciiFont(&APP_BATT_FONT);
  LCD_SetColor(LCD_WHITE);
  LCD_DisplayString(APP_BATT_X, APP_BATT_Y, "BAT:");
}

/**
  * @brief  电压角标：数值部分，跟 App_DrawTime() 同一个 100ms 节拍刷新
  */
static void App_DrawBatteryValue(void)
{
  uint32_t color;

  switch (Battery_GetLevel())
  {
    case BATTERY_LEVEL_LOW:   color = LCD_RED;    break;
    case BATTERY_LEVEL_WARN:  color = LCD_YELLOW; break;
    default:                  color = LCD_WHITE;  break;
  }

  LCD_SetAsciiFont(&APP_BATT_FONT);
  LCD_SetColor(color);
  LCD_DisplayDecimals(APP_BATT_VALUE_X, APP_BATT_Y,
                      (double)Battery_GetVoltage(), APP_BATT_VALUE_LEN, 2);
}

/**
  * @brief  显示任务时间，单位为秒
  * @note   任务四在 B 点锁存时间；任务五、六在再次经过 A 点时锁存时间。
  */
static void App_DrawTime(void)
{
  /* 主界面专用字段 */
  if (s_page != APP_PAGE_MAIN)
  {
    return;
  }

  uint32_t split_ms = Task_GetSplitMs();

  LCD_SetAsciiFont(&APP_FONT);

  if (split_ms != 0U)
  {
    LCD_SetColor(LCD_CYAN);
    LCD_DisplayDecimals(APP_VALUE_X, APP_TIME_Y,
                        (double)split_ms / 1000.0, APP_VALUE_LEN, 2);
    return;
  }

  LCD_SetColor(Task_IsRunning() ? LCD_WHITE : LCD_YELLOW);
  LCD_DisplayDecimals(APP_VALUE_X, APP_TIME_Y,
                      (double)Task_GetElapsedMs() / 1000.0, APP_VALUE_LEN, 2);
}

/**
  * @brief  重画 PID 页的一个数值字段
  */
static void App_DrawPidField(uint8_t field)
{
  Ball_Tune tune = Ball_GetTune();
  uint16_t  y    = APP_VIS_Y0 + field * APP_VIS_DY;

  LCD_SetAsciiFont(&APP_FONT);
  LCD_SetColor(LCD_CYAN);

  switch (field)
  {
    case APP_PID_POS_KP:
      LCD_DisplayDecimals(APP_VIS_VALUE_X, y, tune.pos_kp, APP_VIS_VALUE_LEN, 2);
      break;

    case APP_PID_POS_KI:
      LCD_DisplayDecimals(APP_VIS_VALUE_X, y, tune.pos_ki, APP_VIS_VALUE_LEN, 2);
      break;

    case APP_PID_POS_KD:
      LCD_DisplayDecimals(APP_VIS_VALUE_X, y, tune.pos_kd, APP_VIS_VALUE_LEN, 2);
      break;

    case APP_PID_VEL_KP:
      LCD_DisplayDecimals(APP_VIS_VALUE_X, y, tune.vel_kp, APP_VIS_VALUE_LEN, 2);
      break;

    case APP_PID_VEL_KI:
      LCD_DisplayDecimals(APP_VIS_VALUE_X, y, tune.vel_ki, APP_VIS_VALUE_LEN, 2);
      break;

    default:    /* APP_PID_VEL_KD */
      LCD_DisplayDecimals(APP_VIS_VALUE_X, y, tune.vel_kd, APP_VIS_VALUE_LEN, 2);
      break;
  }
}

/**
  * @brief  重画一个数值字段
  * @note   数值右对齐补空格，驱动会连背景一起重绘，不用先擦除
  */
static void App_DrawField(uint8_t field)
{
  if (s_page == APP_PAGE_PID)
  {
    App_DrawPidField(field);
    return;
  }
  if (s_page == APP_PAGE_BALL)
  {
    App_DrawBallField(field);
    return;
  }

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
    /* 横线标绿，丢线标红 */
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
    /* 运行时显示黑色通道数，停止后显示终点检测窗口内的峰值。 */
    uint8_t dk = Task_IsRunning() ? Track_GetDarkCount() : Track_GetDarkPeak();

    LCD_SetColor(Track_IsCrossLine() ? LCD_GREEN : LCD_WHITE);
    LCD_DisplayNumber(APP_DK_VALUE_X, APP_OFF_Y, (int32_t)dk, APP_DK_VALUE_LEN);
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
  else if (field == APP_FIELD_DIST)
  {
    LCD_SetColor(LCD_WHITE);
    LCD_DisplayDecimals(APP_DIST_VALUE_X, APP_I2C_Y,
                        Task_GetDistanceM(), APP_DIST_VALUE_LEN, 2);
  }
#if APP_SERVO_ENABLE
  else
  {
    LCD_SetColor(LCD_YELLOW);
    LCD_DisplayNumber(APP_SERVO_VALUE_X, APP_STATE_Y,
                      (int32_t)Servo_GetPulseUs(), APP_SERVO_VALUE_LEN);
  }
#endif
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
  * @brief  未运行时先由速度环减速，停稳后切换为短路刹车
  */
static void App_Idle(void)
{
  uint8_t moving = 0;

  for (uint8_t i = 0; i < MOTOR_NUM; i++)
  {
    Encoder_ID enc = (i == MOTOR_LEFT) ? ENCODER_LEFT : ENCODER_RIGHT;

    s_rpm[i]    = Encoder_GetRPM(enc, APP_CTRL_MS);
    s_target[i] = 0.0f;

    if (fabsf(s_rpm[i]) > APP_STOP_RPM_TH)
    {
      moving = 1;
    }

    s_rpm_disp[i] += APP_DISP_ALPHA * (s_rpm[i] - s_rpm_disp[i]);
  }

  if (moving)
  {
    /* 车辆仍在运动，由速度环主动减速 */
    for (uint8_t i = 0; i < MOTOR_NUM; i++)
    {
      s_out[i] = PID_Update(&s_pid[i], 0.0f, s_rpm[i]);
      Motor_SetSpeed((Motor_ID)i, (int16_t)s_out[i]);
    }
    return;
  }

  /* 停稳后清除积分并短路刹车 */
  for (uint8_t i = 0; i < MOTOR_NUM; i++)
  {
    s_out[i] = 0.0f;
    PID_Reset(&s_pid[i]);
  }

  Motor_BrakeAll();
}

/**
  * @brief  清屏并绘制当前页面
  */
static void App_ShowPage(void)
{
  LCD_Clear();
  s_draw_field = 0;
  s_draw_cnt   = 0;

  App_DrawBatteryLabel();
  App_DrawBatteryValue();

  if (s_page == APP_PAGE_PID)
  {
    App_DrawPidLayout();
    return;
  }
  if (s_page == APP_PAGE_BALL)
  {
    App_DrawBallLayout();
    return;
  }

  App_DrawStaticLayout();
  App_DrawTaskLine();
  App_DrawStateLine();
  App_DrawTime();
  App_DrawStatus();
}

void App_Init(void)
{
  /* 开机提示音 */
  Buzzer_Init();
  Buzzer_Beep(BUZZER_BOOT_BEEP_MS);

  /* 显示屏 */
  SPI_LCD_Init();
  LCD_SetDirection(Direction_V_Flip);   /* 竖屏显示，旋转 180 度 */
  LCD_SetBackColor(LCD_BLACK);
  LCD_SetColor(LCD_WHITE);
  LCD_Clear();
  LCD_ShowNumMode(Fill_Space);

  App_DrawStaticLayout();

  /* 视觉通信 */
  Vision_Init();

  /* VOFA+ 通信 */
  Vofa_Init();

  /* 灰度传感器 */
  s_gray_status = Gray_Init();
  App_DrawStatus();

  /* 电池电压 */
  Battery_Init();

  /* 电机与编码器 */
  Motor_Init();
  Encoder_Init();

  /* 两路车轮速度环 */
  for (uint8_t i = 0; i < MOTOR_NUM; i++)
  {
    PID_Init(&s_pid[i], APP_PID_KP, APP_PID_KI, APP_PID_KD, APP_CTRL_DT);
    PID_SetOutputLimits(&s_pid[i], -(float)MOTOR_SPEED_MAX, (float)MOTOR_SPEED_MAX);
    PID_SetIntegralLimit(&s_pid[i], APP_PID_I_LIMIT);
  }

  /* 循迹与任务层 */
  Track_Init();
  Task_Init();

  /* 舵机及滚球控制 */
#if APP_SERVO_ENABLE
  Servo_Init();                         /* 启动 PWM 并回到水平位置 */
#if (APP_SERVO_MODE == APP_SERVO_DEMO)
  Servo_DemoInit();
#elif (APP_SERVO_MODE == APP_SERVO_BALL)
  Ball_Init();
  Ball_Enable(1);
#endif
#endif

  /* 初始化完成后绘制当前页面 */
  App_ShowPage();
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

  /* 按键扫描 */
  Key_Scan();

  /* 解析视觉数据 */
  Vision_Update();

  /* 处理 VOFA+ 指令并发送遥测数据 */
  Vofa_Update();

#if (APP_SERVO_MODE == APP_SERVO_MANUAL)
  /* 舵机标定模式征用了 KEY3/KEY4，此时不翻页 */
#else
  /* KEY3：切换显示页面 */
  if (Key_WasPressed(KEY3))
  {
    s_page = (App_Page)((s_page + 1) % APP_PAGE_NUM);
    App_ShowPage();
  }
#endif

#if (APP_SERVO_MODE == APP_SERVO_DEMO)
  Servo_DemoUpdate();                   /* 在安全行程内往复摆动 */
#elif (APP_SERVO_MODE == APP_SERVO_BALL)
  /* 空闲时允许视觉帧更新目标。任务三完成后继续保持任务设定值。 */
  if (!Task_IsRunning() && Vision_IsFresh() &&
      !((Task_GetId() == TASK_3) && (Task_GetState() == TASK_STATE_DONE)))
  {
    const Vision_Ball *vb = Vision_GetBall();

    if (vb->has_target)
    {
      Ball_SetTarget(vb->target_cm);
    }
  }
  Ball_Update();                        /* 有新视觉帧时更新滚球闭环 */
#elif (APP_SERVO_MODE == APP_SERVO_MANUAL)
  /* 手动模式下，KEY3/KEY4 以最小步进连续调整舵机脉宽。 */
  if (Key_WasRepeated(KEY3))
  {
    Servo_StepUs(+SERVO_STEP_US);
  }
  if (Key_WasRepeated(KEY4))
  {
    Servo_StepUs(-SERVO_STEP_US);
  }
#endif

  /* 任务层与循迹外环 */
  s_outer_cnt++;
  if (s_outer_cnt >= APP_OUTER_EVERY)
  {
    s_outer_cnt = 0;

    /* 任务提供轮速目标时跳过循迹外环。 */
    if (Task_IsRunning() &&
        !Task_GetDriveTargets(&s_target[MOTOR_LEFT], &s_target[MOTOR_RIGHT]))
    {
      /* 只有车辆任务且灰度通信正常时才运行循迹外环。 */
      if (Task_UsesVehicle() && (s_gray_status == HAL_OK))
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
      else
      {
        /* 静止任务保持车轮目标转速为零 */
        s_target[MOTOR_LEFT]  = 0.0f;
        s_target[MOTOR_RIGHT] = 0.0f;
      }
    }

    /* 任务状态机使用本周期更新后的循迹数据 */
    Task_Update();
  }

  /* 车轮速度内环 */
  if (Task_IsRunning())
  {
    App_SpeedLoop();
  }
  else
  {
    App_Idle();
  }

  /* 显示刷新 */
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
    Battery_Update();
    App_DrawBatteryValue();
  }

  s_draw_cnt++;
  if (s_draw_cnt >= APP_DRAW_EVERY)
  {
    s_draw_cnt = 0;
    App_DrawField(s_draw_field);
    s_draw_field = (uint8_t)((s_draw_field + 1) % App_FieldCount());
  }
}
