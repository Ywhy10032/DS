#ifndef __GRAYSCALE_H
#define __GRAYSCALE_H

#include "main.h"

/**
  ******************************************************************************
  * @brief  感为科技八路灰度传感器 I2C 驱动
  *
  * 传感器连接 I2C1：SCL=PB8，SDA=PB9。AD1/AD0 均为 0，
  * 七位地址为 0x4C；HAL 接口使用左移一位后的地址。
  ******************************************************************************
  */

#define GRAY_I2C_ADDR           (0x4C << 1)
#define GRAY_CHANNEL_NUM        8

/* 传感器通信命令 */
#define GRAY_CMD_PING           0xAA    /* 同步，正常返回 0x66 */
#define GRAY_PING_REPLY         0x66
#define GRAY_CMD_CH_ENABLE      0xCE    /* 通道使能 */
#define GRAY_CMD_NORMALIZE      0xCF    /* 归一化使能 */
#define GRAY_CMD_READ_ANALOG    0xB0    /* 连续读取 CH0~CH7 */

/* 同步、使能八路通道并开启归一化 */
HAL_StatusTypeDef Gray_Init(void);

/* 读取八路归一化数据，values 缓冲区长度不得小于 GRAY_CHANNEL_NUM */
HAL_StatusTypeDef Gray_ReadAll(uint8_t *values);

/* 初始化阶段等待同步响应的最长时间 */
#define GRAY_PING_TIMEOUT_MS    3000

#endif /* __GRAYSCALE_H */
