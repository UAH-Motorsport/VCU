/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    adc.c
 * @brief   This file provides code for the configuration
 *          of the ADC instances.
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
#include "adc.h"                                                         // Includes declarations from adc.h required by this source file.

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;                                                 // Declares hadc1 as a ADC_HandleTypeDef variable used by this module.
ADC_HandleTypeDef hadc2;                                                 // Declares hadc2 as a ADC_HandleTypeDef variable used by this module.
ADC_HandleTypeDef hadc3;                                                 // Declares hadc3 as a ADC_HandleTypeDef variable used by this module.

/* ADC1 init function */
void MX_ADC1_Init(void)                                                  // Defines the initialization routine for the ADC1 peripheral.
{                                                                        // Opens the current code block.

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};                                  // Declares multimode and initializes its configuration structure to zero.
  ADC_ChannelConfTypeDef sConfig = {0};                                  // Declares sConfig and initializes its configuration structure to zero.

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;                                                 // Associates hadc1 with the ADC1 hardware peripheral instance.
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;                  // Sets the ClockPrescaler initialization parameter of hadc1 to ADC_CLOCK_SYNC_PCLK_DIV4.
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;                            // Sets the Resolution initialization parameter of hadc1 to ADC_RESOLUTION_12B.
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;                            // Sets the DataAlign initialization parameter of hadc1 to ADC_DATAALIGN_RIGHT.
  hadc1.Init.GainCompensation = 0;                                       // Sets the GainCompensation initialization parameter of hadc1 to 0.
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;                             // Sets the ScanConvMode initialization parameter of hadc1 to ADC_SCAN_ENABLE.
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;                            // Sets the EOCSelection initialization parameter of hadc1 to ADC_EOC_SEQ_CONV.
  hadc1.Init.LowPowerAutoWait = DISABLE;                                 // Sets the LowPowerAutoWait initialization parameter of hadc1 to DISABLE.
  hadc1.Init.ContinuousConvMode = ENABLE;                                // Sets the ContinuousConvMode initialization parameter of hadc1 to ENABLE.
  hadc1.Init.NbrOfConversion = 2;                                        // Sets the NbrOfConversion initialization parameter of hadc1 to 2.
  hadc1.Init.DiscontinuousConvMode = DISABLE;                            // Sets the DiscontinuousConvMode initialization parameter of hadc1 to DISABLE.
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;                      // Sets the ExternalTrigConv initialization parameter of hadc1 to ADC_SOFTWARE_START.
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;       // Sets the ExternalTrigConvEdge initialization parameter of hadc1 to ADC_EXTERNALTRIGCONVEDGE_NONE.
  hadc1.Init.DMAContinuousRequests = DISABLE;                            // Sets the DMAContinuousRequests initialization parameter of hadc1 to DISABLE.
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;                         // Sets the Overrun initialization parameter of hadc1 to ADC_OVR_DATA_OVERWRITTEN.
  hadc1.Init.OversamplingMode = DISABLE;                                 // Sets the OversamplingMode initialization parameter of hadc1 to DISABLE.
  if (HAL_ADC_Init(&hadc1) != HAL_OK)                                    // Checks whether HAL_ADC_Init(&hadc1) != HAL_OK before executing the conditional block.
  {                                                                      // Opens the current code block.
    Error_Handler();                                                     // Transfers execution to the application error handler.
  }                                                                      // Closes the current code block.

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;                                 // Sets multimode.Mode to ADC_MODE_INDEPENDENT.
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)    // Checks whether HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK before executing the conditional block.
  {                                                                      // Opens the current code block.
    Error_Handler();                                                     // Transfers execution to the application error handler.
  }                                                                      // Closes the current code block.

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;                                       // Sets the ADC channel Channel configuration field to ADC_CHANNEL_1.
  sConfig.Rank = ADC_REGULAR_RANK_1;                                     // Sets the ADC channel Rank configuration field to ADC_REGULAR_RANK_1.
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;                      // Sets the ADC channel SamplingTime configuration field to ADC_SAMPLETIME_47CYCLES_5.
  sConfig.SingleDiff = ADC_SINGLE_ENDED;                                 // Sets the ADC channel SingleDiff configuration field to ADC_SINGLE_ENDED.
  sConfig.OffsetNumber = ADC_OFFSET_NONE;                                // Sets the ADC channel OffsetNumber configuration field to ADC_OFFSET_NONE.
  sConfig.Offset = 0;                                                    // Sets the ADC channel Offset configuration field to 0.
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)                 // Checks whether HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK before executing the conditional block.
  {                                                                      // Opens the current code block.
    Error_Handler();                                                     // Transfers execution to the application error handler.
  }                                                                      // Closes the current code block.

  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_2;                                     // Sets the ADC channel Rank configuration field to ADC_REGULAR_RANK_2.
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)                 // Checks whether HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK before executing the conditional block.
  {                                                                      // Opens the current code block.
    Error_Handler();                                                     // Transfers execution to the application error handler.
  }                                                                      // Closes the current code block.
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}                                                                        // Closes the current code block.
/* ADC2 init function */
void MX_ADC2_Init(void)                                                  // Defines the initialization routine for the ADC2 peripheral.
{                                                                        // Opens the current code block.

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};                                  // Declares sConfig and initializes its configuration structure to zero.

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;                                                 // Associates hadc2 with the ADC2 hardware peripheral instance.
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;                  // Sets the ClockPrescaler initialization parameter of hadc2 to ADC_CLOCK_SYNC_PCLK_DIV4.
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;                            // Sets the Resolution initialization parameter of hadc2 to ADC_RESOLUTION_12B.
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;                            // Sets the DataAlign initialization parameter of hadc2 to ADC_DATAALIGN_RIGHT.
  hadc2.Init.GainCompensation = 0;                                       // Sets the GainCompensation initialization parameter of hadc2 to 0.
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;                            // Sets the ScanConvMode initialization parameter of hadc2 to ADC_SCAN_DISABLE.
  hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;                            // Sets the EOCSelection initialization parameter of hadc2 to ADC_EOC_SEQ_CONV.
  hadc2.Init.LowPowerAutoWait = DISABLE;                                 // Sets the LowPowerAutoWait initialization parameter of hadc2 to DISABLE.
  hadc2.Init.ContinuousConvMode = ENABLE;                                // Sets the ContinuousConvMode initialization parameter of hadc2 to ENABLE.
  hadc2.Init.NbrOfConversion = 1;                                        // Sets the NbrOfConversion initialization parameter of hadc2 to 1.
  hadc2.Init.DiscontinuousConvMode = DISABLE;                            // Sets the DiscontinuousConvMode initialization parameter of hadc2 to DISABLE.
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;                      // Sets the ExternalTrigConv initialization parameter of hadc2 to ADC_SOFTWARE_START.
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;       // Sets the ExternalTrigConvEdge initialization parameter of hadc2 to ADC_EXTERNALTRIGCONVEDGE_NONE.
  hadc2.Init.DMAContinuousRequests = DISABLE;                            // Sets the DMAContinuousRequests initialization parameter of hadc2 to DISABLE.
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;                         // Sets the Overrun initialization parameter of hadc2 to ADC_OVR_DATA_OVERWRITTEN.
  hadc2.Init.OversamplingMode = DISABLE;                                 // Sets the OversamplingMode initialization parameter of hadc2 to DISABLE.
  if (HAL_ADC_Init(&hadc2) != HAL_OK)                                    // Checks whether HAL_ADC_Init(&hadc2) != HAL_OK before executing the conditional block.
  {                                                                      // Opens the current code block.
    Error_Handler();                                                     // Transfers execution to the application error handler.
  }                                                                      // Closes the current code block.

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_12;                                      // Sets the ADC channel Channel configuration field to ADC_CHANNEL_12.
  sConfig.Rank = ADC_REGULAR_RANK_1;                                     // Sets the ADC channel Rank configuration field to ADC_REGULAR_RANK_1.
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;                     // Sets the ADC channel SamplingTime configuration field to ADC_SAMPLETIME_640CYCLES_5.
  sConfig.SingleDiff = ADC_SINGLE_ENDED;                                 // Sets the ADC channel SingleDiff configuration field to ADC_SINGLE_ENDED.
  sConfig.OffsetNumber = ADC_OFFSET_NONE;                                // Sets the ADC channel OffsetNumber configuration field to ADC_OFFSET_NONE.
  sConfig.Offset = 0;                                                    // Sets the ADC channel Offset configuration field to 0.
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)                 // Checks whether HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK before executing the conditional block.
  {                                                                      // Opens the current code block.
    Error_Handler();                                                     // Transfers execution to the application error handler.
  }                                                                      // Closes the current code block.
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}                                                                        // Closes the current code block.
/* ADC3 init function */
void MX_ADC3_Init(void)                                                  // Defines the initialization routine for the ADC3 peripheral.
{                                                                        // Opens the current code block.

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};                                  // Declares multimode and initializes its configuration structure to zero.
  ADC_ChannelConfTypeDef sConfig = {0};                                  // Declares sConfig and initializes its configuration structure to zero.

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;                                                 // Associates hadc3 with the ADC3 hardware peripheral instance.
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;                  // Sets the ClockPrescaler initialization parameter of hadc3 to ADC_CLOCK_SYNC_PCLK_DIV4.
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;                            // Sets the Resolution initialization parameter of hadc3 to ADC_RESOLUTION_12B.
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;                            // Sets the DataAlign initialization parameter of hadc3 to ADC_DATAALIGN_RIGHT.
  hadc3.Init.GainCompensation = 0;                                       // Sets the GainCompensation initialization parameter of hadc3 to 0.
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;                            // Sets the ScanConvMode initialization parameter of hadc3 to ADC_SCAN_DISABLE.
  hadc3.Init.EOCSelection = ADC_EOC_SEQ_CONV;                            // Sets the EOCSelection initialization parameter of hadc3 to ADC_EOC_SEQ_CONV.
  hadc3.Init.LowPowerAutoWait = DISABLE;                                 // Sets the LowPowerAutoWait initialization parameter of hadc3 to DISABLE.
  hadc3.Init.ContinuousConvMode = ENABLE;                                // Sets the ContinuousConvMode initialization parameter of hadc3 to ENABLE.
  hadc3.Init.NbrOfConversion = 1;                                        // Sets the NbrOfConversion initialization parameter of hadc3 to 1.
  hadc3.Init.DiscontinuousConvMode = DISABLE;                            // Sets the DiscontinuousConvMode initialization parameter of hadc3 to DISABLE.
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;                      // Sets the ExternalTrigConv initialization parameter of hadc3 to ADC_SOFTWARE_START.
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;       // Sets the ExternalTrigConvEdge initialization parameter of hadc3 to ADC_EXTERNALTRIGCONVEDGE_NONE.
  hadc3.Init.DMAContinuousRequests = DISABLE;                            // Sets the DMAContinuousRequests initialization parameter of hadc3 to DISABLE.
  hadc3.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;                         // Sets the Overrun initialization parameter of hadc3 to ADC_OVR_DATA_OVERWRITTEN.
  hadc3.Init.OversamplingMode = DISABLE;                                 // Sets the OversamplingMode initialization parameter of hadc3 to DISABLE.
  if (HAL_ADC_Init(&hadc3) != HAL_OK)                                    // Checks whether HAL_ADC_Init(&hadc3) != HAL_OK before executing the conditional block.
  {                                                                      // Opens the current code block.
    Error_Handler();                                                     // Transfers execution to the application error handler.
  }                                                                      // Closes the current code block.

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;                                 // Sets multimode.Mode to ADC_MODE_INDEPENDENT.
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc3, &multimode) != HAL_OK)    // Checks whether HAL_ADCEx_MultiModeConfigChannel(&hadc3, &multimode) != HAL_OK before executing the conditional block.
  {                                                                      // Opens the current code block.
    Error_Handler();                                                     // Transfers execution to the application error handler.
  }                                                                      // Closes the current code block.

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;                                       // Sets the ADC channel Channel configuration field to ADC_CHANNEL_1.
  sConfig.Rank = ADC_REGULAR_RANK_1;                                     // Sets the ADC channel Rank configuration field to ADC_REGULAR_RANK_1.
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;                       // Sets the ADC channel SamplingTime configuration field to ADC_SAMPLETIME_2CYCLES_5.
  sConfig.SingleDiff = ADC_SINGLE_ENDED;                                 // Sets the ADC channel SingleDiff configuration field to ADC_SINGLE_ENDED.
  sConfig.OffsetNumber = ADC_OFFSET_NONE;                                // Sets the ADC channel OffsetNumber configuration field to ADC_OFFSET_NONE.
  sConfig.Offset = 0;                                                    // Sets the ADC channel Offset configuration field to 0.
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)                 // Checks whether HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK before executing the conditional block.
  {                                                                      // Opens the current code block.
    Error_Handler();                                                     // Transfers execution to the application error handler.
  }                                                                      // Closes the current code block.
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}                                                                        // Closes the current code block.

static uint32_t HAL_RCC_ADC12_CLK_ENABLED=0;                             // Declares HAL_RCC_ADC12_CLK_ENABLED and initializes it with 0.

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)                       // Defines the low-level ADC clock, GPIO and interrupt initialization callback.
{                                                                        // Opens the current code block.

  GPIO_InitTypeDef GPIO_InitStruct = {0};                                // Declares GPIO_InitStruct and initializes its configuration structure to zero.
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};                          // Declares PeriphClkInit and initializes its configuration structure to zero.
  if(adcHandle->Instance==ADC1)                                          // Checks whether adcHandle->Instance==ADC1 before executing the conditional block.
  {                                                                      // Opens the current code block.
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;            // Sets the peripheral-clock PeriphClockSelection selection to RCC_PERIPHCLK_ADC12.
    PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;       // Sets the peripheral-clock Adc12ClockSelection selection to RCC_ADC12CLKSOURCE_SYSCLK.
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)             // Checks whether HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK before executing the conditional block.
    {                                                                    // Opens the current code block.
      Error_Handler();                                                   // Transfers execution to the application error handler.
    }                                                                    // Closes the current code block.

    /* ADC1 clock enable */
    HAL_RCC_ADC12_CLK_ENABLED++;                                         // Increments HAL_RCC_ADC12_CLK_ENABLED by one.
    if(HAL_RCC_ADC12_CLK_ENABLED==1){                                    // Checks whether HAL_RCC_ADC12_CLK_ENABLED==1 before executing the conditional block.
      __HAL_RCC_ADC12_CLK_ENABLE();                                      // Enables the shared ADC1/ADC2 peripheral clock.
    }                                                                    // Closes the current code block.

    __HAL_RCC_GPIOA_CLK_ENABLE();                                        // Enables the GPIOA peripheral clock.
    /**ADC1 GPIO Configuration
    PA0     ------> ADC1_IN1
    PA1     ------> ADC1_IN2
    */
    GPIO_InitStruct.Pin = APPS_1_Pin|APPS_2_Pin;                         // Sets the GPIO Pin configuration field to APPS_1_Pin|APPS_2_Pin.
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;                             // Sets the GPIO Mode configuration field to GPIO_MODE_ANALOG.
    GPIO_InitStruct.Pull = GPIO_NOPULL;                                  // Sets the GPIO Pull configuration field to GPIO_NOPULL.
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);                              // Applies the prepared GPIO configuration to the selected port.

    /* ADC1 interrupt Init */
    HAL_NVIC_SetPriority(ADC1_2_IRQn, 0, 0);                             // Sets the priority of the selected interrupt in the NVIC.
    HAL_NVIC_EnableIRQ(ADC1_2_IRQn);                                     // Enables the selected interrupt request in the NVIC.
  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }                                                                      // Closes the current code block.
  else if(adcHandle->Instance==ADC2)                                     // Checks the alternative condition adcHandle->Instance==ADC2.
  {                                                                      // Opens the current code block.
  /* USER CODE BEGIN ADC2_MspInit 0 */

  /* USER CODE END ADC2_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;            // Sets the peripheral-clock PeriphClockSelection selection to RCC_PERIPHCLK_ADC12.
    PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;       // Sets the peripheral-clock Adc12ClockSelection selection to RCC_ADC12CLKSOURCE_SYSCLK.
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)             // Checks whether HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK before executing the conditional block.
    {                                                                    // Opens the current code block.
      Error_Handler();                                                   // Transfers execution to the application error handler.
    }                                                                    // Closes the current code block.

    /* ADC2 clock enable */
    HAL_RCC_ADC12_CLK_ENABLED++;                                         // Increments HAL_RCC_ADC12_CLK_ENABLED by one.
    if(HAL_RCC_ADC12_CLK_ENABLED==1){                                    // Checks whether HAL_RCC_ADC12_CLK_ENABLED==1 before executing the conditional block.
      __HAL_RCC_ADC12_CLK_ENABLE();                                      // Enables the shared ADC1/ADC2 peripheral clock.
    }                                                                    // Closes the current code block.

    __HAL_RCC_GPIOB_CLK_ENABLE();                                        // Enables the GPIOB peripheral clock.
    /**ADC2 GPIO Configuration
    PB2     ------> ADC2_IN12
    */
    GPIO_InitStruct.Pin = FRENO_Pin;                                     // Sets the GPIO Pin configuration field to FRENO_Pin.
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;                             // Sets the GPIO Mode configuration field to GPIO_MODE_ANALOG.
    GPIO_InitStruct.Pull = GPIO_NOPULL;                                  // Sets the GPIO Pull configuration field to GPIO_NOPULL.
    HAL_GPIO_Init(FRENO_GPIO_Port, &GPIO_InitStruct);                    // Applies the prepared GPIO configuration to the selected port.

    /* ADC2 interrupt Init */
    HAL_NVIC_SetPriority(ADC1_2_IRQn, 0, 0);                             // Sets the priority of the selected interrupt in the NVIC.
    HAL_NVIC_EnableIRQ(ADC1_2_IRQn);                                     // Enables the selected interrupt request in the NVIC.
  /* USER CODE BEGIN ADC2_MspInit 1 */

  /* USER CODE END ADC2_MspInit 1 */
  }                                                                      // Closes the current code block.
  else if(adcHandle->Instance==ADC3)                                     // Checks the alternative condition adcHandle->Instance==ADC3.
  {                                                                      // Opens the current code block.
  /* USER CODE BEGIN ADC3_MspInit 0 */

  /* USER CODE END ADC3_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC345;           // Sets the peripheral-clock PeriphClockSelection selection to RCC_PERIPHCLK_ADC345.
    PeriphClkInit.Adc345ClockSelection = RCC_ADC345CLKSOURCE_SYSCLK;     // Sets the peripheral-clock Adc345ClockSelection selection to RCC_ADC345CLKSOURCE_SYSCLK.
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)             // Checks whether HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK before executing the conditional block.
    {                                                                    // Opens the current code block.
      Error_Handler();                                                   // Transfers execution to the application error handler.
    }                                                                    // Closes the current code block.

    /* ADC3 clock enable */
    __HAL_RCC_ADC345_CLK_ENABLE();                                       // Enables the ADC3/ADC4/ADC5 peripheral clock domain.

    __HAL_RCC_GPIOB_CLK_ENABLE();                                        // Enables the GPIOB peripheral clock.
    /**ADC3 GPIO Configuration
    PB1     ------> ADC3_IN1
    */
    GPIO_InitStruct.Pin = FRENO_2_Pin;                                   // Sets the GPIO Pin configuration field to FRENO_2_Pin.
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;                             // Sets the GPIO Mode configuration field to GPIO_MODE_ANALOG.
    GPIO_InitStruct.Pull = GPIO_NOPULL;                                  // Sets the GPIO Pull configuration field to GPIO_NOPULL.
    HAL_GPIO_Init(FRENO_2_GPIO_Port, &GPIO_InitStruct);                  // Applies the prepared GPIO configuration to the selected port.

  /* USER CODE BEGIN ADC3_MspInit 1 */

  /* USER CODE END ADC3_MspInit 1 */
  }                                                                      // Closes the current code block.
}                                                                        // Closes the current code block.

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)                     // Defines the low-level ADC deinitialization callback.
{                                                                        // Opens the current code block.

  if(adcHandle->Instance==ADC1)                                          // Checks whether adcHandle->Instance==ADC1 before executing the conditional block.
  {                                                                      // Opens the current code block.
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_ADC12_CLK_ENABLED--;                                         // Decrements HAL_RCC_ADC12_CLK_ENABLED by one.
    if(HAL_RCC_ADC12_CLK_ENABLED==0){                                    // Checks whether HAL_RCC_ADC12_CLK_ENABLED==0 before executing the conditional block.
      __HAL_RCC_ADC12_CLK_DISABLE();                                     // Disables the shared ADC1/ADC2 peripheral clock when no longer required.
    }                                                                    // Closes the current code block.

    /**ADC1 GPIO Configuration
    PA0     ------> ADC1_IN1
    PA1     ------> ADC1_IN2
    */
    HAL_GPIO_DeInit(GPIOA, APPS_1_Pin|APPS_2_Pin);                       // Returns the selected GPIO pins to their reset state.

    /* ADC1 interrupt Deinit */
  /* USER CODE BEGIN ADC1:ADC1_2_IRQn disable */
    /**
     * Uncomment the line below to disable the "ADC1_2_IRQn" interrupt
     * Be aware, disabling shared interrupt may affect other IPs
     */
    /* HAL_NVIC_DisableIRQ(ADC1_2_IRQn); */
  /* USER CODE END ADC1:ADC1_2_IRQn disable */

  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }                                                                      // Closes the current code block.
  else if(adcHandle->Instance==ADC2)                                     // Checks the alternative condition adcHandle->Instance==ADC2.
  {                                                                      // Opens the current code block.
  /* USER CODE BEGIN ADC2_MspDeInit 0 */

  /* USER CODE END ADC2_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_ADC12_CLK_ENABLED--;                                         // Decrements HAL_RCC_ADC12_CLK_ENABLED by one.
    if(HAL_RCC_ADC12_CLK_ENABLED==0){                                    // Checks whether HAL_RCC_ADC12_CLK_ENABLED==0 before executing the conditional block.
      __HAL_RCC_ADC12_CLK_DISABLE();                                     // Disables the shared ADC1/ADC2 peripheral clock when no longer required.
    }                                                                    // Closes the current code block.

    /**ADC2 GPIO Configuration
    PB2     ------> ADC2_IN12
    */
    HAL_GPIO_DeInit(FRENO_GPIO_Port, FRENO_Pin);                         // Returns the selected GPIO pins to their reset state.

    /* ADC2 interrupt Deinit */
  /* USER CODE BEGIN ADC2:ADC1_2_IRQn disable */
    /**
     * Uncomment the line below to disable the "ADC1_2_IRQn" interrupt
     * Be aware, disabling shared interrupt may affect other IPs
     */
    /* HAL_NVIC_DisableIRQ(ADC1_2_IRQn); */
  /* USER CODE END ADC2:ADC1_2_IRQn disable */

  /* USER CODE BEGIN ADC2_MspDeInit 1 */

  /* USER CODE END ADC2_MspDeInit 1 */
  }                                                                      // Closes the current code block.
  else if(adcHandle->Instance==ADC3)                                     // Checks the alternative condition adcHandle->Instance==ADC3.
  {                                                                      // Opens the current code block.
  /* USER CODE BEGIN ADC3_MspDeInit 0 */

  /* USER CODE END ADC3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC345_CLK_DISABLE();                                      // Disables the ADC3/ADC4/ADC5 peripheral clock domain.

    /**ADC3 GPIO Configuration
    PB1     ------> ADC3_IN1
    */
    HAL_GPIO_DeInit(FRENO_2_GPIO_Port, FRENO_2_Pin);                     // Returns the selected GPIO pins to their reset state.

  /* USER CODE BEGIN ADC3_MspDeInit 1 */

  /* USER CODE END ADC3_MspDeInit 1 */
  }                                                                      // Closes the current code block.
}                                                                        // Closes the current code block.

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

