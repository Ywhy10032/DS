/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* ---------------------------------------------------------------------------
 *  CAN application layer  (protocol: docs/CAN_Protocol.md)
 *    0x100  MCU -> host : data[0]=light state, data[1..4]=vibration count (LE)
 *    0x200  host -> MCU : data[0]=target color (0=off / 1=red / 2=blue)
 * ------------------------------------------------------------------------- */

/* Latest device status, updated by CAN1_PollReceive() from the main loop */
volatile uint8_t  g_can_light   = 0;   /* 0=off, 1=red, 2=blue           */
volatile uint32_t g_can_vib     = 0;   /* vibration counter              */
volatile uint8_t  g_can_online  = 0;   /* set once first 0x100 arrives   */
volatile uint8_t  g_can_updated = 0;   /* new data pending redraw        */

/* Configure the RX filter (accept-all std frames) and start the peripheral.
   Returns HAL_OK only if the peripheral actually reached NORMAL mode. */
HAL_StatusTypeDef CAN1_Start(void)
{
  CAN_FilterTypeDef f = {0};
  f.FilterBank           = 0;
  f.FilterMode           = CAN_FILTERMODE_IDMASK;
  f.FilterScale          = CAN_FILTERSCALE_32BIT;
  f.FilterIdHigh         = 0x0000;
  f.FilterIdLow          = 0x0000;
  f.FilterMaskIdHigh     = 0x0000;      /* mask 0 -> let every ID through */
  f.FilterMaskIdLow      = 0x0000;
  f.FilterFIFOAssignment = CAN_RX_FIFO0;
  f.FilterActivation     = ENABLE;
  f.SlaveStartFilterBank = 14;
  if (HAL_CAN_ConfigFilter(&hcan1, &f) != HAL_OK)
    return HAL_ERROR;

  return HAL_CAN_Start(&hcan1);
}

/* Internal loopback self-test. Puts CAN1 into LOOPBACK mode and checks that a
   frame we transmit comes back on RX FIFO0. This needs NO bus, transceiver,
   second node or ACK - so it isolates a host-side fault from a bus fault.
   Returns 1 = PASS, 0 = FAIL. Leaves the peripheral back in NORMAL (READY). */
uint8_t CAN1_SelfTest(void)
{
  CAN_FilterTypeDef   f  = {0};
  CAN_TxHeaderTypeDef tx = {0};
  CAN_RxHeaderTypeDef rx;
  uint8_t  td[1] = { 0xA5 };
  uint8_t  rd[8];
  uint32_t mailbox, start;
  uint8_t  pass = 0;

  HAL_CAN_Stop(&hcan1);
  hcan1.Init.Mode = CAN_MODE_LOOPBACK;
  if (HAL_CAN_Init(&hcan1) != HAL_OK) goto restore;

  f.FilterBank           = 0;
  f.FilterMode           = CAN_FILTERMODE_IDMASK;
  f.FilterScale          = CAN_FILTERSCALE_32BIT;
  f.FilterFIFOAssignment = CAN_RX_FIFO0;
  f.FilterActivation     = ENABLE;
  f.SlaveStartFilterBank = 14;
  if (HAL_CAN_ConfigFilter(&hcan1, &f) != HAL_OK) goto restore;
  if (HAL_CAN_Start(&hcan1) != HAL_OK) goto restore;

  tx.StdId = 0x123;
  tx.IDE   = CAN_ID_STD;
  tx.RTR   = CAN_RTR_DATA;
  tx.DLC   = 1;
  if (HAL_CAN_AddTxMessage(&hcan1, &tx, td, &mailbox) != HAL_OK) goto restore;

  start = HAL_GetTick();
  while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) == 0U)
  {
    if (HAL_GetTick() - start > 50U) goto restore;   /* timeout */
  }
  if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx, rd) == HAL_OK)
  {
    if (rx.StdId == 0x123 && rd[0] == 0xA5) pass = 1;
  }

restore:
  HAL_CAN_Stop(&hcan1);
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  HAL_CAN_Init(&hcan1);
  return pass;
}

/* Non-blocking: drain RX FIFO0 and decode any 0x100 status frames */
void CAN1_PollReceive(void)
{
  CAN_RxHeaderTypeDef rx;
  uint8_t d[8];

  while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0)
  {
    if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx, d) != HAL_OK)
      break;

    if (rx.IDE == CAN_ID_STD && rx.StdId == 0x100 && rx.DLC >= 5)
    {
      g_can_light = d[0];
      g_can_vib   = (uint32_t)d[1]        | ((uint32_t)d[2] << 8) |
                    ((uint32_t)d[3] << 16) | ((uint32_t)d[4] << 24);
      g_can_online  = 1;
      g_can_updated = 1;
    }
  }
}

/* Send a 0x200 color-control frame. color: 0=off / 1=red / 2=blue */
HAL_StatusTypeDef CAN1_SendColor(uint8_t color)
{
  CAN_TxHeaderTypeDef tx = {0};
  uint8_t  d[1] = { color };
  uint32_t mailbox;

  tx.StdId = 0x200;
  tx.IDE   = CAN_ID_STD;
  tx.RTR   = CAN_RTR_DATA;
  tx.DLC   = 1;

  return HAL_CAN_AddTxMessage(&hcan1, &tx, d, &mailbox);
}

/* USER CODE END 1 */
