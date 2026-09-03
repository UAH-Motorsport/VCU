/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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
#include "gpio.h"                                                     // Includes declarations from gpio.h required by this source file.

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)                                               // Defines the GPIO initialization routine for the application.
{                                                                     // Opens the current code block.

  GPIO_InitTypeDef GPIO_InitStruct = {0};                             // Declares GPIO_InitStruct and initializes its configuration structure to zero.

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();                                       // Enables the GPIOF peripheral clock.
  __HAL_RCC_GPIOC_CLK_ENABLE();                                       // Enables the GPIOC peripheral clock.
  __HAL_RCC_GPIOA_CLK_ENABLE();                                       // Enables the GPIOA peripheral clock.
  __HAL_RCC_GPIOB_CLK_ENABLE();                                       // Enables the GPIOB peripheral clock.

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);    // Writes the requested logic level to the selected GPIO pin.

  /*Configure GPIO pins : INPUT_SC_Pin ARRANQUE_Pin */
  GPIO_InitStruct.Pin = INPUT_SC_Pin|ARRANQUE_Pin;                    // Sets the GPIO Pin configuration field to INPUT_SC_Pin|ARRANQUE_Pin.
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;                             // Sets the GPIO Mode configuration field to GPIO_MODE_INPUT.
  GPIO_InitStruct.Pull = GPIO_NOPULL;                                 // Sets the GPIO Pull configuration field to GPIO_NOPULL.
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);                             // Applies the prepared GPIO configuration to the selected port.

  /*Configure GPIO pin : BUZZER_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin;                                   // Sets the GPIO Pin configuration field to BUZZER_Pin.
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;                         // Sets the GPIO Mode configuration field to GPIO_MODE_OUTPUT_PP.
  GPIO_InitStruct.Pull = GPIO_NOPULL;                                 // Sets the GPIO Pull configuration field to GPIO_NOPULL.
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;                        // Sets the GPIO Speed configuration field to GPIO_SPEED_FREQ_LOW.
  HAL_GPIO_Init(BUZZER_GPIO_Port, &GPIO_InitStruct);                  // Applies the prepared GPIO configuration to the selected port.

}                                                                     // Closes the current code block.

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
