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

/* 电压显示：常驻角标，不属于任何一页，翻页清屏后也要留着。
   App_Init() 里用 Direction_V_Flip 把整屏转了 180 度，逻辑坐标原点
   (左上角)转出来正是用户看到的物理右下角，所以就近画在逻辑 (0,0) 附近 */
#define APP_BATT_FONT       ASCII_Font16
#define APP_BATT_FONT_W     8
#define APP_BATT_X          4
#define APP_BATT_Y          4
#define APP_BATT_VALUE_X    (APP_BATT_X + 4 * APP_BATT_FONT_W)
#define APP_BATT_VALUE_LEN  5

/* 偏差与"几路黑"共用一行，DK 用来现场标定横线阈值 */
#define APP_OFF_VALUE_LEN   6
#define APP_DK_LABEL_X      174
#define APP_DK_VALUE_X      (APP_DK_LABEL_X + 3 * APP_FONT_W)
#define APP_DK_VALUE_LEN    2

/**
  * 舵机工作模式。四种互斥 —— 谁都在每拍写脉宽，同时开就会互相覆盖。
  *
  *   OFF     压根不启动 TIM4_CH4 的 PWM。PD15 保持低电平，舵机收不到任何
  *           脉冲，既不动作也不锁角度(失力状态)。注意这与"输出 0 度的脉冲"
  *           完全不同：后者舵机会用力顶在 0 度上。
  *   DEMO    在安全行程内自动往复摆动，用来验证机构行程与方向。
  *   MANUAL  KEY3/KEY4 以 1us 步进手动微调，用来标定 SERVO_SAFE_* 与水平点。
  *           此模式会征用 KEY3，翻页功能失效。
  *   BALL    球杆闭环，由视觉数据驱动。
  */
#define APP_SERVO_OFF       0
#define APP_SERVO_DEMO      1
#define APP_SERVO_MANUAL    2
#define APP_SERVO_BALL      3

#define APP_SERVO_MODE      APP_SERVO_BALL

#define APP_SERVO_ENABLE    (APP_SERVO_MODE != APP_SERVO_OFF)

/* ---------------- 分页 ----------------
   KEY3 循环切换。切页时整屏清空重画，之后照旧只刷新数值字段。

   注意：APP_SERVO_DEMO 设为 0(舵机手动标定)时，KEY3/KEY4 会被征用为脉宽
   微调，此时无法翻页 —— 标定是临时模式，两者不会同时用。 */
typedef enum
{
  APP_PAGE_MAIN = 0,      /* 循迹主界面 */
  APP_PAGE_PID,           /* 球杆闭环当前生效的六个 PID 增益 */
  APP_PAGE_BALL,          /* 球杆闭环 */
  APP_PAGE_NUM
} App_Page;

/* 刷屏走 SPI 是毫秒级阻塞操作，摊开成每次只画一个字段才不会挤占控制节拍。
   8 路灰度 + 偏差 + 黑路数 + 2 个转速 + 里程 (+ 舵机脉宽) */
#define APP_DRAW_EVERY      2
#if APP_SERVO_ENABLE
#define APP_MAIN_FIELD_NUM  (GRAY_CHANNEL_NUM + 6)
#else
#define APP_MAIN_FIELD_NUM  (GRAY_CHANNEL_NUM + 5)   /* 少一个舵机脉宽字段 */
#endif
#define APP_FIELD_OFFSET    (GRAY_CHANNEL_NUM)
#define APP_FIELD_DARK      (GRAY_CHANNEL_NUM + 1)
#define APP_FIELD_LRPM      (GRAY_CHANNEL_NUM + 2)
#define APP_FIELD_RRPM      (GRAY_CHANNEL_NUM + 3)
#define APP_FIELD_DIST      (GRAY_CHANNEL_NUM + 4)
#define APP_FIELD_SERVO     (GRAY_CHANNEL_NUM + 5)

/* ---------------- PID 页 ---------------- */
/* 坐标常量沿用原视觉页的布局位置，改名字太费事、纯几何数值与"视觉"无关 */
#define APP_VIS_Y0          52
#define APP_VIS_DY          32
#define APP_VIS_VALUE_X     (APP_LABEL_X + 6 * APP_FONT_W)
#define APP_VIS_VALUE_LEN   8

/* 显示当前生效的 Ball_Tune 六个增益 —— 与 vofa.c 的 O/P 指令改的是同一组数，
   方便现场核对远程调参是否真的生效 */
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
  APP_BALL_VSET,          /* 位置环下达的速度指令 cm/s —— 串级调试看这个 */
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

/* 计时单独按 100ms 刷新，不跟着字段轮转，否则秒表跳得太慢不像话 */
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
/* 开机默认停在球杆页 —— 现在主要在调球杆闭环，开机就要看的是 SET/POS/ERR，
   省得每次上电先按一下 KEY3 翻页 */
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

      /* 1cm 就是任务五/六的评分门限，超了标红 —— 不用心算，扫一眼就知道达没达标 */
      LCD_SetColor((fabsf(err) > APP_BALL_ERR_LIMIT_CM) ? LCD_RED : LCD_GREEN);
      LCD_DisplayDecimals(APP_VIS_VALUE_X, y, err, APP_VIS_VALUE_LEN, 2);
      break;
    }

    case APP_BALL_VEL:
      LCD_DisplayDecimals(APP_VIS_VALUE_X, y, Ball_GetVelCmS(), APP_VIS_VALUE_LEN, 1);
      break;

    case APP_BALL_VSET:
      /* 位置环下达的速度指令 —— 串级调试时最该看的中间量。
         VEL 迟迟追不上 VSET 就是速度环 Kp 不够；VSET 本身抖得厉害
         就是位置环 Kd 不够或 Kp 太大 */
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
  /* 这几行只属于主界面；在别的页上调用会画花屏幕 */
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
  /* 这几行只属于主界面；在别的页上调用会画花屏幕 */
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
  /* 这几行只属于主界面；在别的页上调用会画花屏幕 */
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
  * @brief  计时显示，秒 + 两位小数
  * @note   任务四/五到达评分点后改显示锁存的分段时间并标青色 ——
  *         任务四是 A->B(≤8s)、任务五是整圈到 A(≤30s)。行驶总时间还包含
  *         之后的减速滑停段，比评分时间多一两秒，不能拿来对照。
  */
static void App_DrawTime(void)
{
  /* 这几行只属于主界面；在别的页上调用会画花屏幕 */
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
  * @note   每次都重新 Ball_GetTune() 取一份 —— 这一页刷新不快(APP_DRAW_EVERY)，
  *         没必要为了省这一次结构体拷贝去额外维护缓存
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
    /* 有几路探头看到黑色。
       运行中显示实时值；停下来显示【峰值】—— 车扫过 A 点只有几拍，实时值
       根本来不及看，而峰值是任务层在进终点窗口时清零的(见 task.c 的
       Task_ArmFinishGate)，所以跑完一趟读到的就是"过 A 那一下最多数到几路"。
       它没到 TASK_CROSS_MIN_CH 就是没停下来的原因，据此调 tracking.h 的
       TRACK_CROSS_WEIGHT_TH / task.h 的 TASK_CROSS_MIN_CH */
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
  * @brief  任务未运行时的处理：先主动刹到零，停稳后再短路刹车驻车
  *
  * @note   TB6612 的短路刹车靠电机反电动势产生制动力矩，而反电动势正比于转速
  *         —— 蠕行速度(40rpm)下反电动势很小，制动力矩弱得可怜，车会滑出一截。
  *         所以这里先让速度环以 0 为目标继续工作，它会输出反向 PWM 把车【拽】停，
  *         制动力矩不再依赖车速。等真正停稳了再切成短路刹车驻车、并清空积分。
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
    /* 还在滑行：速度环主动反拖 */
    for (uint8_t i = 0; i < MOTOR_NUM; i++)
    {
      s_out[i] = PID_Update(&s_pid[i], 0.0f, s_rpm[i]);
      Motor_SetSpeed((Motor_ID)i, (int16_t)s_out[i]);
    }
    return;
  }

  /* 已经停稳：短路刹车驻车。必须清积分，否则下次启动会带着残留猛冲一下 */
  for (uint8_t i = 0; i < MOTOR_NUM; i++)
  {
    s_out[i] = 0.0f;
    PID_Reset(&s_pid[i]);
  }

  Motor_BrakeAll();
}

/**
  * @brief  整屏切换到当前页
  * @note   页与页的静态文字位置不同，必须整屏清空重画；之后回到"每拍只刷一个
  *         数值字段"的常规节奏，不会持续占用 SPI
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
  /* ---------- 蜂鸣器：开机响一声，提示上电自检开始 ---------- */
  Buzzer_Init();
  Buzzer_Beep(BUZZER_BOOT_BEEP_MS);

  /* ---------- LCD ---------- */
  SPI_LCD_Init();
  LCD_SetDirection(Direction_V_Flip);   /* 竖屏 240x320，整屏旋转 180 度 */
  LCD_SetBackColor(LCD_BLACK);
  LCD_SetColor(LCD_WHITE);
  LCD_Clear();
  LCD_ShowNumMode(Fill_Space);

  App_DrawStaticLayout();

  /* ---------- 视觉模块 ---------- */
  Vision_Init();

  /* ---------- VOFA+ 上位机 ---------- */
  Vofa_Init();

  /* ---------- 灰度传感器 ---------- */
  s_gray_status = Gray_Init();
  App_DrawStatus();

  /* ---------- 电压检测 ---------- */
  Battery_Init();

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

  /* ---------- 舵机 ---------- */
#if APP_SERVO_ENABLE
  Servo_Init();                         /* 启动 PWM，回到水平点 1500us */
#if (APP_SERVO_MODE == APP_SERVO_DEMO)
  Servo_DemoInit();
#elif (APP_SERVO_MODE == APP_SERVO_BALL)
  Ball_Init();
  Ball_Enable(1);
#endif
#endif

  /* 所有模块都初始化完了，整屏重画一次当前页 —— 与 KEY3 翻页走同一条路径，
     免得两处各画各的、以后加字段时漏掉一边 */
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

  /* ---------- 按键：每拍扫描 ---------- */
  Key_Scan();

  /* ---------- 视觉：解析中断收进来的字节 ---------- */
  Vision_Update();

  /* ---------- VOFA+：解析上位机指令 + 周期发一帧画图数据 ---------- */
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
  Servo_DemoUpdate();                   /* 在 1050~2400us 之间往复摆动 */
#elif (APP_SERVO_MODE == APP_SERVO_BALL)
  /* 视觉端新增的 target_cm 只在【没有任务在运行】且【本帧确实带了这个字段】
     时才采用 —— 任务运行中的目标由 task.c 的状态机(如任务三的 -5cm 保持)
     或 vofa.c 的 T 指令管理，每帧都用视觉值覆盖会把它们的目标切换打断；
     视觉端没发目标的帧(has_target==0)自然也不该拿 0 去瞎设。
     断链(Vision_IsFresh()==0)时同样不采用，避免拿着陈旧值瞎跑。

     任务三【已完成】的状态要额外排除：它跑完之后计时虽然停了，但闭环还在
     按着球稳定在 -5cm(规则要求"稳定在该点附近"，见 task.h)，这时候被视觉
     的目标一覆盖，球就被拽走了，等于把刚拿到的分数丢掉。中途叫停(IDLE)
     不在此列 —— 那种情况球已经被归位到中心，让视觉接管没问题 */
  if (!Task_IsRunning() && Vision_IsFresh() &&
      !((Task_GetId() == TASK_3) && (Task_GetState() == TASK_STATE_DONE)))
  {
    const Vision_Ball *vb = Vision_GetBall();

    if (vb->has_target)
    {
      Ball_SetTarget(vb->target_cm);
    }
  }
  Ball_Update();                        /* 球杆闭环，内部只在新帧到达时动作 */
#elif (APP_SERVO_MODE == APP_SERVO_MANUAL)
  /* KEY3/KEY4：舵机以最小步进(1us)增减脉宽，用来标定机构行程。
     用 Key_WasRepeated() 而不是 Key_WasPressed()：按住会连发，
     否则 1us 一步走完整个行程要按上千下。
     KEY1/KEY2 由 Task_Update() 用 Key_WasPressed() 消费，两套事件位互不影响 */
  if (Key_WasRepeated(KEY3))
  {
    Servo_StepUs(+SERVO_STEP_US);
  }
  if (Key_WasRepeated(KEY4))
  {
    Servo_StepUs(-SERVO_STEP_US);
  }
#endif

  /* ---------- 任务层 + 循迹外环：每 2 拍 ---------- */
  s_outer_cnt++;
  if (s_outer_cnt >= APP_OUTER_EVERY)
  {
    s_outer_cnt = 0;

    /* 灰度没通就不许跑，否则会拿着全 0 的数据一头冲出去。
       静止任务(如任务三)也不能跑外环 —— 否则车会自己沿线开走 */
    if (Task_IsRunning() && Task_UsesVehicle() && (s_gray_status == HAL_OK))
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
    else if (Task_IsRunning())
    {
      /* 静止任务：目标转速钉死为 0，速度环会主动把轮子按住不动 */
      s_target[MOTOR_LEFT]  = 0.0f;
      s_target[MOTOR_RIGHT] = 0.0f;
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
