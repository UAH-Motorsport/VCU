/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "usart.h"                                                           // Includes declarations from usart.h required by this source file.

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

UART_HandleTypeDef huart2;                                                   // Declares huart2 as a UART_HandleTypeDef variable used by this module.

/* USART2 init function */

void MX_USART2_UART_Init(void)                                               // Defines the USART2 initialization routine used for serial communication.
{                                                                            // Opens the current code block.
  huart2.Instance = USART2;                                                  // Associates huart2 with the USART2 hardware peripheral instance.
  huart2.Init.BaudRate = 115200;                                             // Sets the BaudRate initialization parameter of huart2 to 115200.
  huart2.Init.WordLength = UART_WORDLENGTH_8B;                               // Sets the WordLength initialization parameter of huart2 to UART_WORDLENGTH_8B.
  huart2.Init.StopBits = UART_STOPBITS_1;                                    // Sets the StopBits initialization parameter of huart2 to UART_STOPBITS_1.
  huart2.Init.Parity = UART_PARITY_NONE;                                     // Sets the Parity initialization parameter of huart2 to UART_PARITY_NONE.
  huart2.Init.Mode = UART_MODE_TX_RX;                                        // Sets the Mode initialization parameter of huart2 to UART_MODE_TX_RX.
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;                               // Sets the HwFlowCtl initialization parameter of huart2 to UART_HWCONTROL_NONE.
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;                           // Sets the OverSampling initialization parameter of huart2 to UART_OVERSAMPLING_16.
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;                  // Sets the OneBitSampling initialization parameter of huart2 to UART_ONE_BIT_SAMPLE_DISABLE.
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;                          // Sets the ClockPrescaler initialization parameter of huart2 to UART_PRESCALER_DIV1.
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;              // Sets the USART2 AdvancedInit.AdvFeatureInit configuration field to UART_ADVFEATURE_NO_INIT.

  if (HAL_UART_Init(&huart2) != HAL_OK)                                      // Checks whether HAL_UART_Init(&huart2) != HAL_OK before executing the conditional block.
  {                                                                          // Opens the current code block.
    Error_Handler();                                                         // Transfers execution to the application error handler.
  }                                                                          // Closes the current code block.

  if (HAL_UARTEx_SetTxFifoThreshold(&huart2,                                 // Checks whether HAL_UARTEx_SetTxFifoThreshold(&huart2, before executing the conditional block.
                                    UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)    // Continues the current multi-line expression or function call.
  {                                                                          // Opens the current code block.
    Error_Handler();                                                         // Transfers execution to the application error handler.
  }                                                                          // Closes the current code block.

  if (HAL_UARTEx_SetRxFifoThreshold(&huart2,                                 // Checks whether HAL_UARTEx_SetRxFifoThreshold(&huart2, before executing the conditional block.
                                    UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)    // Continues the current multi-line expression or function call.
  {                                                                          // Opens the current code block.
    Error_Handler();                                                         // Transfers execution to the application error handler.
  }                                                                          // Closes the current code block.

  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)                         // Checks whether HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK before executing the conditional block.
  {                                                                          // Opens the current code block.
    Error_Handler();                                                         // Transfers execution to the application error handler.
  }                                                                          // Closes the current code block.
}                                                                            // Closes the current code block.

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)                        // Defines the low-level UART clock and GPIO initialization callback.
{                                                                            // Opens the current code block.
  GPIO_InitTypeDef GPIO_InitStruct = {0};                                    // Declares GPIO_InitStruct and initializes its configuration structure to zero.

  if(uartHandle->Instance == USART2)                                         // Checks whether uartHandle->Instance == USART2 before executing the conditional block.
  {                                                                          // Opens the current code block.
    __HAL_RCC_USART2_CLK_ENABLE();                                           // Enables the USART2 peripheral clock.
    __HAL_RCC_GPIOA_CLK_ENABLE();                                            // Enables the GPIOA peripheral clock.

    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;                           // Sets the GPIO Pin configuration field to GPIO_PIN_2 | GPIO_PIN_3.
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;                                  // Sets the GPIO Mode configuration field to GPIO_MODE_AF_PP.
    GPIO_InitStruct.Pull = GPIO_NOPULL;                                      // Sets the GPIO Pull configuration field to GPIO_NOPULL.
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;                             // Sets the GPIO Speed configuration field to GPIO_SPEED_FREQ_LOW.
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;                             // Sets the GPIO Alternate configuration field to GPIO_AF7_USART2.

    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);                                  // Applies the prepared GPIO configuration to the selected port.
  }                                                                          // Closes the current code block.
}                                                                            // Closes the current code block.

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)                      // Defines the low-level UART deinitialization callback.
{                                                                            // Opens the current code block.
  if(uartHandle->Instance == USART2)                                         // Checks whether uartHandle->Instance == USART2 before executing the conditional block.
  {                                                                          // Opens the current code block.
    __HAL_RCC_USART2_CLK_DISABLE();                                          // Disables the USART2 peripheral clock.
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2 | GPIO_PIN_3);                         // Returns the selected GPIO pins to their reset state.
  }                                                                          // Closes the current code block.
}                                                                            // Closes the current code block.

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
