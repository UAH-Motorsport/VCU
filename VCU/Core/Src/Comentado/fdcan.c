/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    fdcan.c
 * @brief   This file provides code for the configuration
 *          of the FDCAN instances.
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
#include "fdcan.h"                                                   // Includes declarations from fdcan.h required by this source file.

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

FDCAN_HandleTypeDef hfdcan1;                                         // Declares hfdcan1 as a FDCAN_HandleTypeDef variable used by this module.

/* FDCAN1 init function */
void MX_FDCAN1_Init(void)                                            // Defines the initialization routine for the FDCAN1 peripheral.
{                                                                    // Opens the current code block.

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;                                         // Associates hfdcan1 with the FDCAN1 hardware peripheral instance.
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;                      // Sets the ClockDivider initialization parameter of hfdcan1 to FDCAN_CLOCK_DIV1.
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;                    // Sets the FrameFormat initialization parameter of hfdcan1 to FDCAN_FRAME_CLASSIC.
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;                             // Sets the Mode initialization parameter of hfdcan1 to FDCAN_MODE_NORMAL.
  hfdcan1.Init.AutoRetransmission = ENABLE;                          // Sets the AutoRetransmission initialization parameter of hfdcan1 to ENABLE.
  hfdcan1.Init.TransmitPause = DISABLE;                              // Sets the TransmitPause initialization parameter of hfdcan1 to DISABLE.
  hfdcan1.Init.ProtocolException = DISABLE;                          // Sets the ProtocolException initialization parameter of hfdcan1 to DISABLE.
  hfdcan1.Init.NominalPrescaler = 17;                                // Sets the NominalPrescaler initialization parameter of hfdcan1 to 17.
  hfdcan1.Init.NominalSyncJumpWidth = 1;                             // Sets the NominalSyncJumpWidth initialization parameter of hfdcan1 to 1.
  hfdcan1.Init.NominalTimeSeg1 = 12;                                 // Sets the NominalTimeSeg1 initialization parameter of hfdcan1 to 12.
  hfdcan1.Init.NominalTimeSeg2 = 7;                                  // Sets the NominalTimeSeg2 initialization parameter of hfdcan1 to 7.
  hfdcan1.Init.DataPrescaler = 17;                                   // Sets the DataPrescaler initialization parameter of hfdcan1 to 17.
  hfdcan1.Init.DataSyncJumpWidth = 1;                                // Sets the DataSyncJumpWidth initialization parameter of hfdcan1 to 1.
  hfdcan1.Init.DataTimeSeg1 = 12;                                    // Sets the DataTimeSeg1 initialization parameter of hfdcan1 to 12.
  hfdcan1.Init.DataTimeSeg2 = 7;                                     // Sets the DataTimeSeg2 initialization parameter of hfdcan1 to 7.
  hfdcan1.Init.StdFiltersNbr = 0;                                    // Sets the StdFiltersNbr initialization parameter of hfdcan1 to 0.
  hfdcan1.Init.ExtFiltersNbr = 0;                                    // Sets the ExtFiltersNbr initialization parameter of hfdcan1 to 0.
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;            // Sets the TxFifoQueueMode initialization parameter of hfdcan1 to FDCAN_TX_FIFO_OPERATION.
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)                            // Checks whether HAL_FDCAN_Init(&hfdcan1) != HAL_OK before executing the conditional block.
  {                                                                  // Opens the current code block.
    Error_Handler();                                                 // Transfers execution to the application error handler.
  }                                                                  // Closes the current code block.
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}                                                                    // Closes the current code block.

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* fdcanHandle)             // Defines the low-level FDCAN clock, GPIO and interrupt initialization callback.
{                                                                    // Opens the current code block.

  GPIO_InitTypeDef GPIO_InitStruct = {0};                            // Declares GPIO_InitStruct and initializes its configuration structure to zero.
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};                      // Declares PeriphClkInit and initializes its configuration structure to zero.
  if(fdcanHandle->Instance==FDCAN1)                                  // Checks whether fdcanHandle->Instance==FDCAN1 before executing the conditional block.
  {                                                                  // Opens the current code block.
  /* USER CODE BEGIN FDCAN1_MspInit 0 */

  /* USER CODE END FDCAN1_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;        // Sets the peripheral-clock PeriphClockSelection selection to RCC_PERIPHCLK_FDCAN.
    PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;    // Sets the peripheral-clock FdcanClockSelection selection to RCC_FDCANCLKSOURCE_PCLK1.
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)         // Checks whether HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK before executing the conditional block.
    {                                                                // Opens the current code block.
      Error_Handler();                                               // Transfers execution to the application error handler.
    }                                                                // Closes the current code block.

    /* FDCAN1 clock enable */
    __HAL_RCC_FDCAN_CLK_ENABLE();                                    // Enables the FDCAN peripheral clock.

    __HAL_RCC_GPIOA_CLK_ENABLE();                                    // Enables the GPIOA peripheral clock.
    /**FDCAN1 GPIO Configuration
    PA11     ------> FDCAN1_RX
    PA12     ------> FDCAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;                   // Sets the GPIO Pin configuration field to GPIO_PIN_11|GPIO_PIN_12.
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;                          // Sets the GPIO Mode configuration field to GPIO_MODE_AF_PP.
    GPIO_InitStruct.Pull = GPIO_NOPULL;                              // Sets the GPIO Pull configuration field to GPIO_NOPULL.
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;                     // Sets the GPIO Speed configuration field to GPIO_SPEED_FREQ_LOW.
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;                     // Sets the GPIO Alternate configuration field to GPIO_AF9_FDCAN1.
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);                          // Applies the prepared GPIO configuration to the selected port.

    /* FDCAN1 interrupt Init */
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 0, 0);                     // Sets the priority of the selected interrupt in the NVIC.
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);                             // Enables the selected interrupt request in the NVIC.
    HAL_NVIC_SetPriority(FDCAN1_IT1_IRQn, 0, 0);                     // Sets the priority of the selected interrupt in the NVIC.
    HAL_NVIC_EnableIRQ(FDCAN1_IT1_IRQn);                             // Enables the selected interrupt request in the NVIC.
  /* USER CODE BEGIN FDCAN1_MspInit 1 */

  /* USER CODE END FDCAN1_MspInit 1 */
  }                                                                  // Closes the current code block.
}                                                                    // Closes the current code block.

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* fdcanHandle)           // Defines the low-level FDCAN deinitialization callback.
{                                                                    // Opens the current code block.

  if(fdcanHandle->Instance==FDCAN1)                                  // Checks whether fdcanHandle->Instance==FDCAN1 before executing the conditional block.
  {                                                                  // Opens the current code block.
  /* USER CODE BEGIN FDCAN1_MspDeInit 0 */

  /* USER CODE END FDCAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_FDCAN_CLK_DISABLE();                                   // Disables the FDCAN peripheral clock.

    /**FDCAN1 GPIO Configuration
    PA11     ------> FDCAN1_RX
    PA12     ------> FDCAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);                 // Returns the selected GPIO pins to their reset state.

    /* FDCAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);                            // Disables the selected interrupt request in the NVIC.
    HAL_NVIC_DisableIRQ(FDCAN1_IT1_IRQn);                            // Disables the selected interrupt request in the NVIC.
  /* USER CODE BEGIN FDCAN1_MspDeInit 1 */

  /* USER CODE END FDCAN1_MspDeInit 1 */
  }                                                                  // Closes the current code block.
}                                                                    // Closes the current code block.

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

