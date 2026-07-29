/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_CAN1_Init(void);

/* USER CODE BEGIN Prototypes */

/* CAN application layer (see can.c / docs/CAN_Protocol.md) */
uint8_t           CAN1_SelfTest(void);
HAL_StatusTypeDef CAN1_Start(void);
void              CAN1_PollReceive(void);
HAL_StatusTypeDef CAN1_SendColor(uint8_t color);

extern volatile uint8_t  g_can_light;    /* 0=off, 1=red, 2=blue */
extern volatile uint32_t g_can_vib;      /* vibration counter    */
extern volatile uint8_t  g_can_online;   /* first frame seen     */
extern volatile uint8_t  g_can_updated;  /* redraw pending flag  */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

