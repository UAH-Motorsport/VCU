/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "main.h"                      // Includes declarations from main.h required by this source file.
#include "stm32g4xx_it.h"              // Includes declarations from stm32g4xx_it.h required by this source file.
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;        // Declares hadc1 as a ADC_HandleTypeDef variable used by this module.
extern ADC_HandleTypeDef hadc2;        // Declares hadc2 as a ADC_HandleTypeDef variable used by this module.
extern FDCAN_HandleTypeDef hfdcan1;    // Declares hfdcan1 as a FDCAN_HandleTypeDef variable used by this module.
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)                 // Defines the Non-Maskable Interrupt exception handler.
{                                      // Opens the current code block.
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)                           // Repeats the enclosed block while 1 remains true.
  {                                    // Opens the current code block.
  }                                    // Closes the current code block.
  /* USER CODE END NonMaskableInt_IRQn 1 */
}                                      // Closes the current code block.

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)           // Defines the HardFault exception handler.
{                                      // Opens the current code block.
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)                            // Repeats the enclosed block while 1 remains true.
  {                                    // Opens the current code block.
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }                                    // Closes the current code block.
}                                      // Closes the current code block.

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)           // Defines the memory-management fault exception handler.
{                                      // Opens the current code block.
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)                            // Repeats the enclosed block while 1 remains true.
  {                                    // Opens the current code block.
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }                                    // Closes the current code block.
}                                      // Closes the current code block.

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)            // Defines the bus-fault exception handler.
{                                      // Opens the current code block.
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)                            // Repeats the enclosed block while 1 remains true.
  {                                    // Opens the current code block.
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }                                    // Closes the current code block.
}                                      // Closes the current code block.

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)          // Defines the usage-fault exception handler.
{                                      // Opens the current code block.
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)                            // Repeats the enclosed block while 1 remains true.
  {                                    // Opens the current code block.
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }                                    // Closes the current code block.
}                                      // Closes the current code block.

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)                 // Defines the supervisor-call exception handler.
{                                      // Opens the current code block.
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}                                      // Closes the current code block.

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)            // Defines the debug-monitor exception handler.
{                                      // Opens the current code block.
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}                                      // Closes the current code block.

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)              // Defines the PendSV exception handler.
{                                      // Opens the current code block.
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}                                      // Closes the current code block.

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)             // Defines the SysTick exception handler.
{                                      // Opens the current code block.
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();                       // Increments the HAL millisecond time base.
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}                                      // Closes the current code block.

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles ADC1 and ADC2 global interrupt.
  */
void ADC1_2_IRQHandler(void)           // Defines the interrupt service routine shared by ADC1 and ADC2.
{                                      // Opens the current code block.
  /* USER CODE BEGIN ADC1_2_IRQn 0 */

  /* USER CODE END ADC1_2_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);          // Passes the ADC interrupt to the STM32 HAL interrupt handler.
  HAL_ADC_IRQHandler(&hadc2);          // Passes the ADC interrupt to the STM32 HAL interrupt handler.
  /* USER CODE BEGIN ADC1_2_IRQn 1 */

  /* USER CODE END ADC1_2_IRQn 1 */
}                                      // Closes the current code block.

/**
  * @brief This function handles FDCAN1 interrupt 0.
  */
void FDCAN1_IT0_IRQHandler(void)       // Defines the FDCAN1 interrupt-line 0 service routine.
{                                      // Opens the current code block.
  /* USER CODE BEGIN FDCAN1_IT0_IRQn 0 */

  /* USER CODE END FDCAN1_IT0_IRQn 0 */
  HAL_FDCAN_IRQHandler(&hfdcan1);      // Passes the FDCAN interrupt to the STM32 HAL interrupt handler.
  /* USER CODE BEGIN FDCAN1_IT0_IRQn 1 */

  /* USER CODE END FDCAN1_IT0_IRQn 1 */
}                                      // Closes the current code block.

/**
  * @brief This function handles FDCAN1 interrupt 1.
  */
void FDCAN1_IT1_IRQHandler(void)       // Defines the FDCAN1 interrupt-line 1 service routine.
{                                      // Opens the current code block.
  /* USER CODE BEGIN FDCAN1_IT1_IRQn 0 */

  /* USER CODE END FDCAN1_IT1_IRQn 0 */
  HAL_FDCAN_IRQHandler(&hfdcan1);      // Passes the FDCAN interrupt to the STM32 HAL interrupt handler.
  /* USER CODE BEGIN FDCAN1_IT1_IRQn 1 */

  /* USER CODE END FDCAN1_IT1_IRQn 1 */
}                                      // Closes the current code block.

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
