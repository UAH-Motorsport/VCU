/* USER CODE BEGIN Header */
/**
 *****************************************************************************
 * @file           : main.c
 * @author         : David Hernández Rentero
 * @project        : main
 * @date           : 2026-07-30
 * @editor         : Visual Studio Code
 *
 * @brief          : Main program body
 * @brief          : Implements Relative Current Control (0-100%)
 * @brief          : Sets Safety Limits: Max AC = 15A, Max Brake = 0A
 * @brief          : Implements Plausibility Check between APPS1 and APPS2
 * @brief          : Uses a single 4-20 mA brake input for the R2D activation condition
 * @brief          : Implements Drive Enable/Disable via Button Press
 * @brief          : FIXED: Node ID reverted to 0x1B
 ******************************************************************************
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

#pragma region includes                                                                                                   // Starts the IDE folding region for includes.

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "main.h"                                                                                                         // Main header file for the project
#include "adc.h"                                                                                                          // Header file for ADC (Analog-to-Digital Converter) configuration
#include "fdcan.h"                                                                                                        // Header file for FDCAN (Flexible Data-Rate Controller Area Network) configuration
#include "gpio.h"                                                                                                         // Header file for GPIO (General-Purpose Input/Output) configuration

#include <string.h>                                                                                                       // For memset
#include <stdbool.h>                                                                                                      // For boolean data type
#include <math.h>                                                                                                         // For mathematical functions like abs()
#include <stdio.h>                                                                                                        // Debug logs through SWV/ITM

/* USER CODE END Includes */

#pragma endregion                                                                                                         // Ends the current IDE folding region.

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/

#pragma region defines                                                                                                    // Starts the IDE folding region for defines.

/* USER CODE BEGIN PD */

/* --- Control Limits ---
*  Potentiometer (0-4095) will map to Relative Current (0-100%)
*  CAN manual 2.4 says scale is 10, so 100.0% = 1000*/

#define REL_CURRENT_MIN                 0                                                                                 // Minimum Relative Current (0%)
#define REL_CURRENT_MAX                 100 * 10                                                                          // Maximum Relative Current (100.0% scaled by 10)

// --- Safety Limits ---
// Max AC Current = 15A. Scale 10 -> 150

#define LIMIT_MAX_AC_CURRENT            15 * 10                                                                           // Maximum AC Current Limit (15.0A scaled by 10)

// Max Brake Current = 0A. Scale 10 -> 0

#define LIMIT_MAX_BRAKE_CURRENT         0 * 10                                                                            // Maximum Brake Current Limit (0A scaled by 10)

// --- Node ID ---
// CORRECTED: Reverted to 0x1B (Decimal 27) from the specifications of DTI manual.

#define NODE_ID                          0x1B                                                                             // Defines NODE_ID with the value or expression 0x1B.

// --- Packet IDs (From DTI Manual V2.4) ---
#define PACKET_ID_SET_REL_CURRENT        0x05                                                                             // Table 9, Page 17
#define PACKET_ID_SET_MAX_AC             0x08                                                                             // Table 9, Page 17
#define PACKET_ID_SET_MAX_BRAKE          0x09                                                                             // Table 9, Page 17
#define PACKET_ID_DRIVE_ENABLE           0x0C                                                                             // Table 9, Page 17

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define R2D_HOLD_TIME_MS                 2000U                                                                            // Time to hold the button pressed to enable drive (2 seconds)
#define R2D_BUZZER_TIME_MS               2000U                                                                            // Time the buzzer must be active
#define R2D_CHECK_PERIOD_MS              10U                                                                              // Period to check the button state during the hold time (10 ms)
#define Delay_10_MS()                    HAL_Delay(10)                                                                    // Macro for a 10 ms delay
#define Delay_100_MS()                   HAL_Delay(100)                                                                   // Macro for a 100 ms delay
#define CAN_ID_R2D                       0x0000301BU                                                                      // R2D message identifier from the PDU (Extended ID: 0x301B)
#define CAN_ID_AMS_TEMPERATURES          0x0000501BU                                                                      // AMS temperatures message identifier (Extended ID: 0x501B)
#define CAN_ID_BRAKE_LIGHTS              0x0000601BU                                                                      // Brake lights message identifier (Extended ID: 0x601B)
#define CAN_ID_AMS_BATTERY_MEASUREMENTS  0x0000701BU                                                                      // AMS current and voltage message identifier (Extended ID: 0x701B)
#define R2D_ACTIVATION_BYTE              4U                                                                               // Byte index in the CAN message where the activation bit is located (Byte 4, zero-indexed)
#define R2D_ACTIVATION_MASK              0x10U                                                                            // Start bit 36, length 1
#define R2D_STATUS_ACTIVE                0x0004U                                                                          // R2D Status Active definied in  DBC
#define R2D_ACTIVE_TIMEOUT_MS            5000U                                                                            // Timeout for R2D status message reception (5 seconds)
#define R2D_STATUS_CHECK_PERIOD_MS       10U                                                                              // Period to check the R2D status message reception (10 ms)
#define APPS1_ADC_MAX                    3723U                                                                            // Maximum calibrated ADC value used for APPS1 scaling (3.0 V equivalent)
#define APPS2_ADC_MAX                    4095U                                                                            // Maximum 12-bit ADC value used for APPS2 scaling
#define APPS_MAX_DIFFERENCE              100.0f                                                                           // Maximum allowed difference between APPS1 and APPS2 for plausibility check (10.0% of full scale)
#define APPS_ERROR_TIME_MS               100U                                                                             // Time the APPS discrepancy must remain present before declaring a fault
#define MAIN_LOOP_PERIOD_MS              10U                                                                              // Main control-loop period used for APPS supervision
#define CAN_TX_TIMEOUT_MS                100U                                                                             // Maximum time allowed while waiting for space in the CAN Tx FIFO

// --- R2D startup debug -------------------------------------------------------
#define R2D_DEBUG_LOGS                   1U                                                                                // Set to 0 to disable R2D startup logs
#define R2D_LOG_PERIOD_MS                250U                                                                              // Periodic input-status log while waiting for ARRANQUE + brake
#define R2D_PDU_LOG_PERIOD_MS            250U                                                                              // Periodic PDU-status log while waiting for R2D ACTIVE

#if R2D_DEBUG_LOGS                                                                                                        // Compiles the following block only when this preprocessor condition is true.
#define R2D_LOG(...)                     printf(__VA_ARGS__)                                                              // Defines the R2D debug logging macro for the selected build configuration.
#else                                                                                                                     // Selects the alternative branch of the current compile-time condition.
#define R2D_LOG(...)                     ((void)0)                                                                        // Defines the R2D debug logging macro for the selected build configuration.
#endif                                                                                                                    // Ends the current conditional-compilation block.

/** --- Brake 4-20 mA input --------------------------------------------------
 * Se emplea un único canal de freno en PB2 / ADC2_IN12.
 *
 * Hipótesis de hardware empleada en estos valores:
 *   - Resistencia shunt: 165 ohm
 *   - Referencia ADC (VDDA): 3.3 V
 *   - ADC: 12 bits (0-4095)
 *
 * 4 mA  -> 0.66 V -> ADC ~= 819
 * 20 mA -> 3.30 V -> ADC = 4095
 *
 * AVISO: con 165 ohm, 20 mA producen idealmente 3.30 V, exactamente el fondo
 * de escala del ADC. Por tanto, no existe margen superior para detectar sobrecorriente
 * pero en teoría los bucles 4-20 no deben sobre pasar los 20 mA pq están preparados para ello
 *  y tolerancias del sensor, shunt o VDDA pueden provocar saturación.
 * El diagnóstico de fallo inferior sí puede mantenerse y deberá calibrarse en placa. */

#define BRAKE_SHUNT_RESISTOR_OHM          165.0f                                                                           // Shunt resistor used to convert the 4-20 mA signal into voltage
#define BRAKE_ADC_REFERENCE_V             3.3f                                                                             // ADC analogue reference voltage
#define BRAKE_ADC_FULL_SCALE              4095.0f                                                                          // 12-bit ADC full-scale value
#define BRAKE_ADC_MIN                     819U                                                                             // Expected ADC value at 4 mA
#define BRAKE_ADC_MAX                     4095U                                                                            // Expected ADC value at 20 mA
#define BRAKE_ADC_FAULT_LOW               700U                                                                             // Values below this threshold are considered an invalid 4-20 mA signal
#define BRAKE_ADC_FAULT_HIGH              4095U                                                                            // Values above this threshold are considered an invalid 4-20 mA signal
#define BRAKE_ACTIVE_THRESHOLD_ADC        2457U                                                                            // Approx. 12 mA / 50% of the 4-20 mA measurement span


/* ARRANQUE está configurado como entrada digital en PC6.
 * Cambiar a GPIO_PIN_SET si el pulsador es activo a 
 * nivel alto si cambiamos la config del pin. */

#define ARRANQUE_ACTIVE_STATE       GPIO_PIN_RESET                                                                          // Active state of the ARRANQUE button (GPIO_PIN_SET for active high, GPIO_PIN_RESET for active low)

/* USER CODE END PM */

#pragma endregion                                                                                                         // Ends the current IDE folding region.

/* Private variables ---------------------------------------------------------*/

#pragma region private variables                                                                                          // Starts the IDE folding region for private variables.

/* hadc1, hadc2 and hfdcan1 are already declared in adc.h and fdcan.h. */

/* COM1 is the NUCLEO Virtual COM Port routed through the ST-LINK USB interface. */
COM_InitTypeDef BspCOMInit;                                                                                               // Declares BspCOMInit as a COM_InitTypeDef variable used by this module.

/* USER CODE BEGIN PV */
FDCAN_TxHeaderTypeDef TxHeader;                                                                                           // CAN Tx header (same for all messages, only ID and data change)
FDCAN_RxHeaderTypeDef RxHeader;                                                                                           // CAN Rx header (used for receiving messages, to read ID and data length)
uint8_t               TxData[8];                                                                                          // Generic buffer for all CAN messages
uint8_t               RxData[8];                                                                                          // Buffer for receiving CAN messages
/* Signals received from the PDU: R2D message, ID 0x301B. */
volatile uint16_t     R2D_status = 0U;                                                                                    // R2D main status received in bytes 0-1
volatile uint16_t     R2D_internal_status = 0U;                                                                           // R2D internal phase received in bytes 2-3
volatile uint16_t     R2D_error_flags = 0U;                                                                               // R2D error flags received in bytes 6-7
volatile bool         R2D_air_positive_active = false;                                                                    // AIR positive state received in byte 4, bit 0
volatile bool         R2D_air_negative_active = false;                                                                    // AIR negative state received in byte 4, bit 1
volatile bool         R2D_precharge_active = false;                                                                       // Precharge relay state received in byte 4, bit 2
volatile bool         R2D_discharge_active = false;                                                                       // Discharge relay state received in byte 4, bit 3
volatile bool         R2D_activation_sequence_vcu = false;                                                                // VCU activation request received in byte 4, bit 4

/* Signals received from the AMS. */
volatile float        AMS_temperature_max = 0.0f;                                                                         // Maximum battery temperature in degrees Celsius
volatile float        AMS_temperature_min = 0.0f;                                                                         // Minimum battery temperature in degrees Celsius
volatile float        AMS_battery_current = 0.0f;                                                                         // Battery current in amperes
volatile float        AMS_battery_voltage = 0.0f;                                                                         // Battery voltage in volts

/* Additional CAN signal and reception supervision variables. */
volatile bool         brake_lights_active = false;                                                                        // Brake lights state received in message 0x601B
volatile bool         CAN_reception_error = false;                                                                        // Set if a message cannot be extracted from RX FIFO0
volatile uint32_t     PDU_last_R2D_tick = 0U;                                                                             // Tick of the last correctly decoded R2D message
volatile uint32_t     AMS_last_temperature_tick = 0U;                                                                     // Tick of the last correctly decoded temperature message
volatile uint32_t     AMS_last_measurement_tick = 0U;                                                                     // Tick of the last correctly decoded battery measurement message
volatile uint32_t     brake_lights_last_tick = 0U;                                                                        // Tick of the last correctly decoded brake lights message

uint16_t              adc_value = 0;                                                                                      // Variable to store raw ADC value (0-4095)
uint16_t              brake_adc_value = 0;                                                                                // Raw brake ADC value from PB2 / ADC2_IN12
uint16_t              BRAKE = 0;                                                                                          // Brake signal scaled from the valid 4-20 mA range to 0-1000
float                 brake_current_mA = 0.0f;                                                                            // Brake-loop current reconstructed from the ADC measurement
bool                  brake_signal_valid = false;                                                                         // Indicates whether the measured current is inside the valid 4-20 mA diagnostic window
uint16_t              APPS1 = 0;                                                                                          // Variable to store mapped APPS1 value (0-1000, representing 0-100.0% Relative Current)
uint16_t              APPS2 = 0;                                                                                          // Variable to store mapped APPS2 value (0-1000, representing 0-100.0% Relative Current)
float                 differential = 0.0f;                                                                                // Variable to store the difference between APPS1 and APPS2 for plausibility check
uint16_t              target_rel_current = 0;                                                                             // Variable to store the target relative current (0-1000, representing 0-100.0%) that will be sent to the motor controller after mapping from the potentiometer
GPIO_PinState         button_prev = GPIO_PIN_SET;                                                                         // Previous button state reserved for future edge detection
bool                  APPS_implausibility_error = false;                                                                  // Persistent APPS plausibility-fault flag
bool                  APPS_error_timer_started = false;                                                                   // Indicates whether the APPS discrepancy timer is running
uint32_t              APPS_error_start_tick = 0U;                                                                         // Tick at which the APPS discrepancy first appeared

/* USER CODE END PV */

#pragma endregion                                                                                                         // Ends the current IDE folding region.

/* Private function prototypes -----------------------------------------------*/

#pragma region private function prototypes                                                                                // Starts the IDE folding region for private function prototypes.

/* USER CODE BEGIN PFP */

void SystemClock_Config(void);                                                                                            // Function to configure the system clock
void Error_Handler(void);                                                                                                 // Function to handle errors (e.g., HAL failures)
void Send_CAN_Message(uint32_t id, uint8_t *data, uint32_t len);                                                          // Function to send a CAN message with specified ID, data, and length
void Wait_And_Send_R2D_Activation(void);                                                                                  // Waits for the ARRANQUE button to be held for R2D_HOLD_TIME_MS and sends the R2D activation message
void Send_R2D_Activation(void);                                                                                           // Sends the R2D activation message over CAN
bool R2D_Inputs_Are_Active(uint16_t brake_adc);                                                                           // Checks ARRANQUE and the valid single 4-20 mA brake channel
static bool Read_Brake_Channel(void);                                                                                     // Reads the brake signal from ADC2
void Sound_R2D_Buzzer(void);                                                                                              // Activates the R2D buzzer for the configured time
static uint16_t Read_BE_U16(const uint8_t *data);                                                                         // Reads an unsigned 16-bit Motorola/Big-Endian value
static uint32_t Read_BE_U32(const uint8_t *data);                                                                         // Reads an unsigned 32-bit Motorola/Big-Endian value
static void Process_Received_CAN_Message(const FDCAN_RxHeaderTypeDef *header, const uint8_t *data);                       // Decodes received PDU and AMS messages according to the DBC
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);                                         // HAL callback called when a new message enters RX FIFO0
long map(long x, long in_min, long in_max, long out_min, long out_max);                                                   // Maps a value from one range to another (used for mapping ADC values to relative current)

/* USER CODE END PFP */

#pragma endregion                                                                                                         // Ends the current IDE folding region.

/* Private user code ---------------------------------------------------------*/

#pragma region private user code                                                                                          // Starts the IDE folding region for private user code.

/* USER CODE BEGIN 0 */

/* We use the fuction map() to convert the ADC value (0-4095) to a relative 
 * current value (0-1000) for the motor controller. This allows us to control
 * the motor speed based on the potentiometer position.*/

long map(long x, long in_min, long in_max, long out_min, long out_max)                                                    // Maps a value from one range to another (used for mapping ADC values to relative current)
{                                                                                                                         // Opens the current code block.
  if (in_max == in_min)                                                                                                   // Prevent division by zero if the input range is zero
    return out_min;                                                                                                       // Return the minimum output value if the input range is zero to avoid undefined behavior
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;                                                // Perform the mapping calculation and return the mapped value
}                                                                                                                         // Closes the current code block.

#if R2D_DEBUG_LOGS                                                                                                        // Compiles the following block only when this preprocessor condition is true.
/* Retarget printf() to COM1 (Virtual COM Port of the NUCLEO through ST-LINK).
 * COM1 must be enabled in the .ioc and initialized before the first log.
 * Serial settings: 115200 baud, 8 data bits, no parity, 1 stop bit. */
int _write(int file, char *ptr, int len)                                                                                  // Defines the low-level write function used to redirect standard output.
{                                                                                                                         // Opens the current code block.
  (void)file;                                                                                                             // Marks file as intentionally unused to avoid compiler warnings.

  if ((ptr == NULL) || (len <= 0))                                                                                        // Checks whether (ptr == NULL) || (len <= 0) before executing the conditional block.
  {                                                                                                                       // Opens the current code block.
    return 0;                                                                                                             // Returns 0 to the caller.
  }                                                                                                                       // Closes the current code block.

  if (HAL_UART_Transmit(&hcom_uart[COM1],                                                                                 // Checks whether HAL_UART_Transmit(&hcom_uart[COM1], before executing the conditional block.
                        (uint8_t *)ptr,                                                                                   // Provides (uint8_t *)ptr as the next value or argument in the current multi-line expression.
                        (uint16_t)len,                                                                                    // Provides (uint16_t)len as the next value or argument in the current multi-line expression.
                        HAL_MAX_DELAY) != HAL_OK)                                                                         // Continues the current multi-line expression or function call.
  {                                                                                                                       // Opens the current code block.
    return -1;                                                                                                            // Returns -1 to the caller.
  }                                                                                                                       // Closes the current code block.

  return len;                                                                                                             // Returns len to the caller.
}                                                                                                                         // Closes the current code block.
#endif                                                                                                                    // Ends the current conditional-compilation block.

/* USER CODE END 0 */

#pragma endregion                                                                                                         // Ends the current IDE folding region.

/**
 * @brief  The application entry point.
 * @retval int
 */

#pragma region main                                                                                                       // Starts the IDE folding region for main.

int main(void)                                                                                                            // Main function: Entry point of the program
{                                                                                                                         // Opens the current code block.
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();                                                                                                             // Initialize the Hardware Abstraction Layer (HAL) library
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  SystemClock_Config();                                                                                                   // Configure the system clock
  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();                                                                                                         // Initialize GPIO pins
  MX_FDCAN1_Init();                                                                                                       // Initialize FDCAN1 peripheral
  MX_ADC1_Init();                                                                                                         // Initialize ADC1 peripheral
  MX_ADC2_Init();                                                                                                         // Initialize ADC2 peripheral
  /* USER CODE BEGIN 2 */

  /* Initialize COM1 / Virtual COM Port before any diagnostic log is printed.
   * CubeMX board configuration must have VCP enabled (PA2=LPUART1_TX, PA3=LPUART1_RX). */
  BspCOMInit.BaudRate   = 115200;                                                                                         // Sets the COM1 BaudRate serial parameter to 115200.
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;                                                                              // Sets the COM1 WordLength serial parameter to COM_WORDLENGTH_8B.
  BspCOMInit.StopBits   = COM_STOPBITS_1;                                                                                 // Sets the COM1 StopBits serial parameter to COM_STOPBITS_1.
  BspCOMInit.Parity     = COM_PARITY_NONE;                                                                                // Sets the COM1 Parity serial parameter to COM_PARITY_NONE.
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;                                                                             // Sets the COM1 HwFlowCtl serial parameter to COM_HWCONTROL_NONE.

  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)                                                                  // Checks whether BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE before executing the conditional block.
  {                                                                                                                       // Opens the current code block.
    Error_Handler();                                                                                                      // Transfers execution to the application error handler.
  }                                                                                                                       // Closes the current code block.

  /* Disable stdout buffering so every R2D_LOG() is sent immediately. */
  setvbuf(stdout, NULL, _IONBF, 0);                                                                                       // Configures standard output buffering for immediate serial log delivery.

  R2D_LOG("\r\n[SERIAL] COM1 ready: 115200 baud, 8N1\r\n");                                                               // Writes this diagnostic message to the configured R2D debug output.

  // Start FDCAN
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)                                                                                // If starting FDCAN fails, call the error handler
    Error_Handler();                                                                                                      // Transfers execution to the application error handler.

  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U) != HAL_OK)                              // Enable the interrupt generated when a new message enters RX FIFO0
    Error_Handler();                                                                                                      // Transfers execution to the application error handler.

  // Calibrate both ADC peripherals
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)                                                    // If ADC1 calibration fails, call the error handler
    Error_Handler();                                                                                                      // Transfers execution to the application error handler.

  if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK)                                                    // If ADC2 calibration fails, call the error handler
    Error_Handler();                                                                                                      // Transfers execution to the application error handler.

  // Configure Global CAN Header for Extended ID
  TxHeader.IdType = FDCAN_EXTENDED_ID;                                                                                    // Set the CAN ID type to Extended ID
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;                                                                                // Set the CAN frame type to Data Frame
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;                                                                                // Set the CAN data length to 8 bytes
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;                                                                        // Set the Error State Indicator to Active
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;                                                                                 // Disable Bit Rate Switching (BRS)
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;                                                                                  // Set the CAN format to Classic CAN
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;                                                                       // Disable Tx Event FIFO
  TxHeader.MessageMarker = 0;                                                                                             // Set the Message Marker to 0

  // --- SEND SAFETY LIMITS (ONCE AT STARTUP) ---

  #pragma region safety limits                                                                                            // Starts the IDE folding region for safety limits.

  // 1. Set Max AC Current to 15A (Value: 150)
  memset(TxData, 0, sizeof(TxData));                                                                                      // Clear the TxData buffer
  TxData[0] = (LIMIT_MAX_AC_CURRENT >> 8) & 0xFF;                                                                         // MSB
  TxData[1] = LIMIT_MAX_AC_CURRENT & 0xFF;                                                                                // LSB
  uint32_t limit_ac_id = (PACKET_ID_SET_MAX_AC << 8)| NODE_ID;                                                            // Create the CAN ID for setting max AC current
  Send_CAN_Message(limit_ac_id, TxData, 8U);                                                                              // Send the CAN message to set max AC current
  Delay_10_MS();                                                                                                          // Small delay to ensure processing

  // 2. Set Max Brake Current to 0A (Value: 0)
  memset(TxData, 0, sizeof(TxData));                                                                                      // Clear the TxData buffer
  TxData[0] = (LIMIT_MAX_BRAKE_CURRENT >> 8) & 0xFF;                                                                      // MSB
  TxData[1] = LIMIT_MAX_BRAKE_CURRENT & 0xFF;                                                                             // LSB
  uint32_t limit_brake_id = (PACKET_ID_SET_MAX_BRAKE << 8) | NODE_ID;                                                     // Create the CAN ID for setting max brake current
  Send_CAN_Message(limit_brake_id, TxData, 8U);                                                                           // Send the CAN message to set max brake current
  Delay_10_MS();                                                                                                          // Small delay to ensure processing

  #pragma endregion                                                                                                       // Ends the current IDE folding region.

  /* Executed only first time to obtain the activation signal for R2D
   * and allow PDU to start R2D sequence. */

  #pragma region R2D Activation                                                                                           // Starts the IDE folding region for R2D Activation.

  Wait_And_Send_R2D_Activation();                                                                                         // Wait for ARRANQUE and a valid 4-20 mA brake signal to be held for 2 seconds and send the R2D activation message

  #pragma endregion                                                                                                       // Ends the current IDE folding region.

  #pragma region buzzer Pin                                                                                               // Starts the IDE folding region for buzzer Pin.

  HAL_GPIO_WritePin(BUZZER_GPIO_Port,                                                                                     // The BUZZER Pin must be initialized shut down to prevent noises without the logic to be active
                  BUZZER_Pin,                                                                                             // Provides BUZZER_Pin as the next value or argument in the current multi-line expression.
                  GPIO_PIN_RESET);                                                                                        // Executes this statement as part of the current control flow.

  #pragma endregion                                                                                                       // Ends the current IDE folding region.

  /* USER CODE END 2 */

  /* Infinite loop */

#pragma region main loop                                                                                                  // Starts the IDE folding region for main loop.


  /* USER CODE BEGIN 3 */
  while (1)                                                                                                               // Repeats the enclosed block while 1 remains true.
  {                                                                                                                       // Opens the current code block.
    /* --- 1. Read Potentiometer and check the diference between both
    * potentiometers --- */

    #pragma region APPS lecture                                                                                           // Starts the IDE folding region for APPS lecture.

    bool apps_read_ok = true;                                                                                             // Remains true only if both APPS conversions are acquired correctly

    if (HAL_ADC_Start(&hadc1) != HAL_OK)                                                                                  // Start the two-rank ADC1 sequence
    {                                                                                                                     // Opens the current code block.
      apps_read_ok = false;                                                                                               // A failed start makes the accelerator request invalid
    }                                                                                                                     // Closes the current code block.

    if (apps_read_ok &&                                                                                                   // Checks whether apps_read_ok && before executing the conditional block.
        (HAL_ADC_PollForConversion(&hadc1, 20U) == HAL_OK))                                                               // Wait for APPS1, configured as rank 1
    {                                                                                                                     // Opens the current code block.
      adc_value = (uint16_t)HAL_ADC_GetValue(&hadc1);                                                                     // Read the first ADC1 conversion
      APPS1 = (uint16_t)map(adc_value, 0L, APPS1_ADC_MAX, 0L, REL_CURRENT_MAX);                                           // Scale the 0-3.0 V APPS1 signal to 0-1000
    }                                                                                                                     // Closes the current code block.
    else                                                                                                                  // Executes this branch when the preceding condition is false.
    {                                                                                                                     // Opens the current code block.
      apps_read_ok = false;                                                                                               // Do not use a stale APPS1 value after a conversion failure
    }                                                                                                                     // Closes the current code block.

    if (apps_read_ok &&                                                                                                   // Checks whether apps_read_ok && before executing the conditional block.
        (HAL_ADC_PollForConversion(&hadc1, 20U) == HAL_OK))                                                               // Wait for APPS2, configured as rank 2
    {                                                                                                                     // Opens the current code block.
      adc_value = (uint16_t)HAL_ADC_GetValue(&hadc1);                                                                     // Read the second ADC1 conversion
      APPS2 = (uint16_t)map(adc_value, 0L, APPS2_ADC_MAX, 0L, REL_CURRENT_MAX);                                           // Scale the full-range APPS2 signal to 0-1000
    }                                                                                                                     // Closes the current code block.
    else                                                                                                                  // Executes this branch when the preceding condition is false.
    {                                                                                                                     // Opens the current code block.
      apps_read_ok = false;                                                                                               // Do not use a stale APPS2 value after a conversion failure
    }                                                                                                                     // Closes the current code block.

    if (HAL_ADC_Stop(&hadc1) != HAL_OK)                                                                                   // Stop ADC1 after completing the two-rank sequence
    {                                                                                                                     // Opens the current code block.
      apps_read_ok = false;                                                                                               // Sets apps_read_ok to false.
    }                                                                                                                     // Closes the current code block.

    #pragma endregion                                                                                                     // Ends the current IDE folding region.

    #pragma region APPS plausibility check                                                                                // Starts the IDE folding region for APPS plausibility check.

    /* --- 2. APPS plausibility and relative-current calculation --- */

    if (!apps_read_ok)                                                                                                    // Checks whether !apps_read_ok before executing the conditional block.
    {                                                                                                                     // Opens the current code block.
      APPS_implausibility_error = true;                                                                                   // ADC acquisition failure is handled as an immediate safe-state fault
      APPS_error_timer_started = false;                                                                                   // Sets APPS_error_timer_started to false.
      APPS_error_start_tick = 0U;                                                                                         // Sets APPS_error_start_tick to 0U.
      differential = 0.0f;                                                                                                // Sets differential to 0.0f.
    }                                                                                                                     // Closes the current code block.
    else                                                                                                                  // Executes this branch when the preceding condition is false.
    {                                                                                                                     // Opens the current code block.
      differential = fabsf((float)APPS1 - (float)APPS2);                                                                  // Absolute difference in the common 0-1000 scaled range

      if (differential > APPS_MAX_DIFFERENCE)                                                                             // Checks whether differential > APPS_MAX_DIFFERENCE before executing the conditional block.
      {                                                                                                                   // Opens the current code block.
        if (!APPS_error_timer_started)                                                                                    // Start the timer on the first implausible sample
        {                                                                                                                 // Opens the current code block.
          APPS_error_start_tick = HAL_GetTick();                                                                          // Sets APPS_error_start_tick to HAL_GetTick().
          APPS_error_timer_started = true;                                                                                // Sets APPS_error_timer_started to true.
        }                                                                                                                 // Closes the current code block.

        if ((uint32_t)(HAL_GetTick() - APPS_error_start_tick) >= APPS_ERROR_TIME_MS)                                      // Declare the fault only if it persists for the configured interval
        {                                                                                                                 // Opens the current code block.
          APPS_implausibility_error = true;                                                                               // Sets APPS_implausibility_error to true.
        }                                                                                                                 // Closes the current code block.
      }                                                                                                                   // Closes the current code block.
      else                                                                                                                // Executes this branch when the preceding condition is false.
      {                                                                                                                   // Opens the current code block.
        APPS_error_timer_started = false;                                                                                 // Cancel a transient discrepancy that disappears before the timeout
        APPS_error_start_tick = 0U;                                                                                       // Sets APPS_error_start_tick to 0U.
        APPS_implausibility_error = false;                                                                                // Sets APPS_implausibility_error to false.
      }                                                                                                                   // Closes the current code block.
    }                                                                                                                     // Closes the current code block.

    if (APPS_implausibility_error)                                                                                        // Checks whether APPS_implausibility_error before executing the conditional block.
    {                                                                                                                     // Opens the current code block.
      target_rel_current = 0U;                                                                                            // Request zero torque while the APPS signals are invalid
    }                                                                                                                     // Closes the current code block.
    else                                                                                                                  // Executes this branch when the preceding condition is false.
    {                                                                                                                     // Opens the current code block.
      target_rel_current = (uint16_t)(((uint32_t)APPS1 + (uint32_t)APPS2) / 2U);                                          // Average both valid accelerator channels without 16-bit overflow
    }                                                                                                                     // Closes the current code block.

    #pragma endregion                                                                                                     // Ends the current IDE folding region.

    /* target_rel_current = map(adc_value, 0, 4095, 0, REL_CURRENT_MAX); -->
    * original mapping with only one channel*/

    #pragma region allow DE if permited                                                                                   // Starts the IDE folding region for allow DE if permited.

    /* --- 3. Send Drive Enable (Keep-Alive) ---
     * ID: (0x0C << 8) | 0x1B = 0xC1B */
    
    memset(TxData, 0, sizeof(TxData));                                                                                    // Clear the TxData buffer
    if (APPS_implausibility_error)                                                                                           // APPS plausibility fault disables drive
    {                                                                                                                     // Opens the current code block.
      TxData[0] = 0x00U;                                                                                                  // 0 = Drive Not Allowed
    }                                                                                                                     // Closes the current code block.
    else                                                                                                                  // Executes this branch when the preceding condition is false.
    {                                                                                                                     // Opens the current code block.
      TxData[0] = 0x01U;                                                                                                  // 1 = Drive Allowed
    }                                                                                                                     // Closes the current code block.
    uint32_t drive_enable_id = (PACKET_ID_DRIVE_ENABLE << 8) | NODE_ID;                                                   // Create the CAN ID for drive enable message
    Send_CAN_Message(drive_enable_id, TxData, 8U);                                                                        // Send the CAN message to enable drive

    #pragma endregion                                                                                                     // Ends the current IDE folding region.

    /* --- 4. Send Set Relative Current Command ---
     * ID: (0x05 << 8) | 0x1B = 0x51B
     * Value is 2 bytes, Big Endian (Table 15)*/

    #pragma region send relative current                                                                                  // Starts the IDE folding region for send relative current.

    memset(TxData, 0, sizeof(TxData));                                                                                    // Clear the TxData buffer
    TxData[0] = (target_rel_current >> 8) & 0xFF;                                                                         // MSB
    TxData[1] = target_rel_current & 0xFF;                                                                                // LSB
    uint32_t set_rel_current_id = (PACKET_ID_SET_REL_CURRENT << 8) | NODE_ID;                                             // Create the CAN ID for setting relative current
    Send_CAN_Message(set_rel_current_id, TxData, 8U);                                                                     // Send the CAN message to set relative current

    #pragma endregion                                                                                                     // Ends the current IDE folding region.

    /* --- 5. Wait ---
     * Wait for the configured main-loop period before the next iteration
     */
    HAL_Delay(MAIN_LOOP_PERIOD_MS);                                                                                       // Execute APPS supervision and CAN commands approximately every 10 ms
  }                                                                                                                       // Closes the current code block.
  /* USER CODE END 3 */
}                                                                                                                         // Closes the current code block.

#pragma endregion                                                                                                         // Ends the current IDE folding region.

/**
 * @brief System Clock Configuration
 * @retval None
 */

#pragma region system clock configuration                                                                                 // Starts the IDE folding region for system clock configuration.

void SystemClock_Config(void)                                                                                             // Configures the system clock to use the HSI oscillator and PLL to achieve the desired system frequency
{                                                                                                                         // Opens the current code block.
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};                                                                             // Structure to configure the RCC oscillators (HSI, PLL, etc.)
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};                                                                             // Structure to configure the CPU, AHB, and APB bus clocks

  /** Configure the main internal regulator output voltage
   */

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);                                                    // Set the voltage scaling to Scale 1 Boost mode for maximum performance

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;                                                              // Use the High-Speed Internal (HSI) oscillator
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;                                                                                // Turn on the HSI oscillator
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;                                                     // Use the default calibration value for HSI
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;                                                                            // Turn on the Phase-Locked Loop (PLL)
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;                                                                    // Use HSI as the PLL source
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;                                                                             // Set PLLM to divide by 4 (HSI is 16 MHz, so VCO input = 4 MHz)
  RCC_OscInitStruct.PLL.PLLN = 85;                                                                                        // Set PLLN to multiply by 85 (VCO output = 4 MHz * 85 = 340 MHz)
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;                                                                             // Set PLLP to divide by 2 (PLL output = 340 MHz / 2 = 170 MHz)
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;                                                                             // Set PLLQ to divide by 2 (PLL output = 340 MHz / 2 = 170 MHz)
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;                                                                             // Set PLLR to divide by 2 (PLL output = 340 MHz / 2 = 170 MHz)
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)                                                                    // If oscillator configuration fails, call the error handler
  {                                                                                                                       // Opens the current code block.
    Error_Handler();                                                                                                      // Transfers execution to the application error handler.
  }                                                                                                                       // Closes the current code block.

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;    // Configure HCLK, SYSCLK, PCLK1, and PCLK2 clocks
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;                                                               // Use PLL output as the system clock source
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;                                                                      // Set AHB clock divider to 1 (HCLK = SYSCLK)
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;                                                                       // Set APB1 clock divider to 1 (PCLK1 = HCLK)
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;                                                                       // Set APB2 clock divider to 1 (PCLK2 = HCLK)

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)                                                 // If clock configuration fails, call the error handler
  {                                                                                                                       // Opens the current code block.
    Error_Handler();                                                                                                      // Transfers execution to the application error handler.
  }                                                                                                                       // Closes the current code block.
}                                                                                                                         // Closes the current code block.

#pragma endregion                                                                                                         // Ends the current IDE folding region.

/* USER CODE BEGIN 4 */

/* Reads a 16-bit unsigned Motorola/Big-Endian value from two consecutive bytes. */

#pragma region read big-endian                                                                                            // Starts the IDE folding region for read big-endian.

static uint16_t Read_BE_U16(const uint8_t *data)                                                                          // Reads an unsigned 16-bit value from two bytes in Big-Endian format
{                                                                                                                         // Opens the current code block.
  return ((uint16_t)data[0] << 8U) |                                                                                      // Shift the first byte to the high position and combine with the second byte
          (uint16_t)data[1];                                                                                              // Return the combined 16-bit value
}                                                                                                                         // Closes the current code block.

/* Reads a 32-bit unsigned Motorola/Big-Endian value from four consecutive bytes. */

static uint32_t Read_BE_U32(const uint8_t *data)                                                                          // Reads an unsigned 32-bit value from four bytes in Big-Endian format
{                                                                                                                         // Opens the current code block.
  return ((uint32_t)data[0] << 24U) |                                                                                     // Shift the first byte to the highest position and combine with the next three bytes 
         ((uint32_t)data[1] << 16U) |                                                                                     // Shift the second byte to the next position and combine with the next two bytes
         ((uint32_t)data[2] << 8U)  |                                                                                     // Shift the third byte to the next position and combine with the last byte
          (uint32_t)data[3];                                                                                              // Return the combined 32-bit value
}                                                                                                                         // Closes the current code block.

#pragma endregion                                                                                                         // Ends the current IDE folding region.

/* Called automatically by the STM32 HAL when a new CAN message enters RX FIFO0. */

#pragma region HAL Rx callback                                                                                            // Starts the IDE folding region for HAL Rx callback.

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)                                          // Callback function called when a new message enters RX FIFO0
{                                                                                                                         // Opens the current code block.
  if ((hfdcan == NULL) || (hfdcan->Instance != FDCAN1))                                                                   // Ignore null handles and callbacks from other FDCAN peripherals
  {                                                                                                                       // Opens the current code block.
    return;                                                                                                               // Returns from the current function.
  }                                                                                                                       // Closes the current code block.

  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U)                                                                 // Check that a new RX FIFO0 message caused the callback
  {                                                                                                                       // Opens the current code block.
    return;                                                                                                               // Returns from the current function.
  }                                                                                                                       // Closes the current code block.

  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U)                                                       // Drain every message currently waiting in RX FIFO0
  {                                                                                                                       // Opens the current code block.
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)                                      // Copy the next message header and payload into the global RX buffers
    {                                                                                                                     // Opens the current code block.
      CAN_reception_error = true;                                                                                         // Record the error without blocking inside the interrupt
      return;                                                                                                             // Returns from the current function.
    }                                                                                                                     // Closes the current code block.

    Process_Received_CAN_Message(&RxHeader, RxData);                                                                      // Decode the message according to its identifier and DBC layout
  }                                                                                                                       // Closes the current code block.
}                                                                                                                         // Closes the current code block.

#pragma endregion                                                                                                         // Ends the current IDE folding region.

/* Decodes the received PDU and AMS messages defined in the latest DBC. */

#pragma region decode CAN messages                                                                                        // Starts the IDE folding region for decode CAN messages.

static void Process_Received_CAN_Message(const FDCAN_RxHeaderTypeDef *header, const uint8_t *data)                        // Decodes the received CAN message based on its identifier and DBC layout
{                                                                                                                         // Opens the current code block.
  if ((header == NULL) || (data == NULL))                                                                                 // Reject invalid pointers
  {                                                                                                                       // Opens the current code block.
    return;                                                                                                               // Returns from the current function.
  }                                                                                                                       // Closes the current code block.

  if ((header->IdType != FDCAN_EXTENDED_ID) ||                                                                            // The DBC messages use 29-bit extended identifiers
      (header->RxFrameType != FDCAN_DATA_FRAME) ||                                                                        // Ignore remote frames
      (header->FDFormat != FDCAN_CLASSIC_CAN) ||                                                                          // The network uses Classic CAN 2.0
      (header->DataLength != FDCAN_DLC_BYTES_8))                                                                          // All messages in the current DBC have a DLC of 8 bytes
  {                                                                                                                       // Opens the current code block.
    return;                                                                                                               // Returns from the current function.
  }                                                                                                                       // Closes the current code block.

  switch (header->Identifier)                                                                                             // Selects behavior according to the value of header->Identifier.
  {                                                                                                                       // Opens the current code block.
    case CAN_ID_R2D:                                                                                                      // PDU R2D message: ID 0x301B
    {                                                                                                                     // Opens the current code block.
      R2D_status = Read_BE_U16(&data[0]);                                                                                 // Bytes 0-1: R2D_Status
      R2D_internal_status = Read_BE_U16(&data[2]);                                                                        // Bytes 2-3: R2D_Internal_status
      R2D_air_positive_active = ((data[4] & 0x01U) != 0U);                                                                // Byte 4, bit 0: R2D_AIR_POS
      R2D_air_negative_active = ((data[4] & 0x02U) != 0U);                                                                // Byte 4, bit 1: R2D_AIR_NEG
      R2D_precharge_active = ((data[4] & 0x04U) != 0U);                                                                   // Byte 4, bit 2: R2D_PRECHARGE
      R2D_discharge_active = ((data[4] & 0x08U) != 0U);                                                                   // Byte 4, bit 3: R2D_DISCHARGE
      R2D_activation_sequence_vcu = ((data[4] & 0x10U) != 0U);                                                            // Byte 4, bit 4: R2D_Active_Sequence_VCU

      R2D_error_flags = Read_BE_U16(&data[6]);                                                                            // Bytes 6-7: R2D_Error_Flags; byte 5 is unused
      PDU_last_R2D_tick = HAL_GetTick();                                                                                  // Save the reception time for future timeout supervision
      break;                                                                                                              // Exits the current switch branch or loop.
    }                                                                                                                     // Closes the current code block.

    case CAN_ID_AMS_TEMPERATURES:                                                                                         // AMS temperatures message: ID 0x501B
    {                                                                                                                     // Opens the current code block.
      const uint32_t temperature_max_raw = Read_BE_U32(&data[0]);                                                         // Bytes 0-3: Temperature_max raw value
      const uint32_t temperature_min_raw = Read_BE_U32(&data[4]);                                                         // Bytes 4-7: Temperature_min raw value

      AMS_temperature_max = ((float)temperature_max_raw * 0.1f) - 10.0f;                                                  // DBC conversion: physical = raw * 0.1 - 10
      AMS_temperature_min = ((float)temperature_min_raw * 0.1f) - 10.0f;                                                  // DBC conversion: physical = raw * 0.1 - 10
      AMS_last_temperature_tick = HAL_GetTick();                                                                          // Save the reception time for future timeout supervision
      break;                                                                                                              // Exits the current switch branch or loop.
    }                                                                                                                     // Closes the current code block.

    case CAN_ID_BRAKE_LIGHTS:                                                                                             // Brake lights message: ID 0x601B
    {                                                                                                                     // Opens the current code block.
      const uint32_t brake_lights_raw = Read_BE_U32(&data[0]);                                                            // Bytes 0-3: Brake_Lights raw value
      brake_lights_active = (brake_lights_raw == 1U);                                                                     // DBC table: 0 = Brake_off, 1 = Brake_on
      brake_lights_last_tick = HAL_GetTick();                                                                             // Save the reception time for future timeout supervision
      break;                                                                                                              // Exits the current switch branch or loop.
    }                                                                                                                     // Closes the current code block.

    case CAN_ID_AMS_BATTERY_MEASUREMENTS:                                                                                 // AMS battery measurements message: ID 0x701B
    {                                                                                                                     // Opens the current code block.
      const uint32_t battery_current_raw = Read_BE_U32(&data[0]);                                                         // Bytes 0-3: Battery_Current raw value
      const uint32_t battery_voltage_raw = Read_BE_U32(&data[4]);                                                         // Bytes 4-7: Battery_Voltage raw value

      AMS_battery_current = (float)battery_current_raw * 0.1f;                                                            // DBC conversion: physical = raw * 0.1 A
      AMS_battery_voltage = (float)battery_voltage_raw * 0.01f;                                                           // DBC conversion: physical = raw * 0.01 V
      AMS_last_measurement_tick = HAL_GetTick();                                                                          // Save the reception time for future timeout supervision
      break;                                                                                                              // Exits the current switch branch or loop.
    }                                                                                                                     // Closes the current code block.

    default:                                                                                                              // Handles all switch values not matched by a previous case.
    {                                                                                                                     // Opens the current code block.
      break;                                                                                                              // Ignore CAN messages that are not used by this VCU
    }                                                                                                                     // Closes the current code block.
  }                                                                                                                       // Closes the current code block.
}                                                                                                                         // Closes the current code block.

#pragma endregion                                                                                                         // Ends the current IDE folding region.

#pragma region send R2D                                                                                                   // Starts the IDE folding region for send R2D.

void Send_R2D_Activation(void)                                                                                            // Sends the R2D activation message over CAN to enable the drive
{                                                                                                                         // Opens the current code block.
  memset(TxData, 0, sizeof(TxData));                                                                                      // Clear the TxData buffer to ensure no residual data is sent

  /* R2D_Active_Sequence_VCU: start bit 36, length 1 bit. */
  TxData[R2D_ACTIVATION_BYTE] |= R2D_ACTIVATION_MASK;                                                                     // Set byte 4, bit 4

  R2D_LOG("[CAN TX] R2D activation request\r\n");                                                                         // Writes this diagnostic message to the configured R2D debug output.
  R2D_LOG("[CAN TX] ID   = 0x%08lX\r\n", (unsigned long)CAN_ID_R2D);                                                      // Sets R2D_LOG("[CAN TX] ID to 0x%08lX\r\n", (unsigned long)CAN_ID_R2D).
  R2D_LOG("[CAN TX] DATA = %02X %02X %02X %02X %02X %02X %02X %02X\r\n",                                                  // Provides R2D_LOG("[CAN TX] DATA = %02X %02X %02X %02X %02X %02X %02X %02X\r\n" as the next value or argument in the current multi-line expression.
          TxData[0], TxData[1], TxData[2], TxData[3],                                                                     // Provides TxData[0], TxData[1], TxData[2], TxData[3] as the next value or argument in the current multi-line expression.
          TxData[4], TxData[5], TxData[6], TxData[7]);                                                                    // Executes this statement as part of the current control flow.

  Send_CAN_Message(CAN_ID_R2D, TxData, 8U);                                                                               // Send the R2D activation request
}                                                                                                                         // Closes the current code block.

#pragma endregion                                                                                                         // Ends the current IDE folding region.

#pragma region Check R2D condition                                                                                        // Starts the IDE folding region for Check R2D condition.

static bool Read_Brake_Channel(void)                                                                                      // Reads and validates the single 4-20 mA brake sensor using ADC2
{                                                                                                                         // Opens the current code block.
  bool brake_read_ok = true;                                                                                              // Declares brake_read_ok and initializes it with true.

  if (HAL_ADC_Start(&hadc2) != HAL_OK)                                                                                    // Starts ADC2 conversion
  {                                                                                                                       // Opens the current code block.
    brake_read_ok = false;                                                                                                // Sets brake_read_ok to false.
  }                                                                                                                       // Closes the current code block.

  if (brake_read_ok &&                                                                                                    // Checks whether brake_read_ok && before executing the conditional block.
      (HAL_ADC_PollForConversion(&hadc2, 20U) == HAL_OK))                                                                 // Waits for the brake conversion
  {                                                                                                                       // Opens the current code block.
    brake_adc_value = (uint16_t)HAL_ADC_GetValue(&hadc2);                                                                 // Reads the brake input from PB2 / ADC2_IN12
    map(brake_adc_value, BRAKE_ADC_MIN, BRAKE_ADC_MAX, 0L, REL_CURRENT_MAX);                                              // Calls map with the configured arguments.
    brake_current_mA =                                                                                                    // Continues the current program structure without changing the configured logic.
        (((float)brake_adc_value * BRAKE_ADC_REFERENCE_V) / BRAKE_ADC_FULL_SCALE) /                                       // Continues the current multi-line logical or arithmetic expression.
        BRAKE_SHUNT_RESISTOR_OHM * 1000.0f;                                                                               // Converts ADC counts -> voltage -> loop current in mA

    brake_signal_valid =                                                                                                  // Continues the current program structure without changing the configured logic.
        (brake_adc_value >= BRAKE_ADC_FAULT_LOW) &&                                                                       // Continues the current multi-line logical or arithmetic expression.
        (brake_adc_value <= BRAKE_ADC_FAULT_HIGH);                                                                         // Validates that the signal remains inside the expected 4-20 mA diagnostic window

    if (brake_adc_value <= BRAKE_ADC_MIN)                                                                                 // Checks whether brake_adc_value <= BRAKE_ADC_MIN before executing the conditional block.
    {                                                                                                                     // Opens the current code block.
      BRAKE = 0U;                                                                                                         // 4 mA or less corresponds to the minimum valid brake command
    }                                                                                                                     // Closes the current code block.
    else if (brake_adc_value >= BRAKE_ADC_MAX)                                                                            // Checks the alternative condition brake_adc_value >= BRAKE_ADC_MAX.
    {                                                                                                                     // Opens the current code block.
      BRAKE = REL_CURRENT_MAX;                                                                                            // 20 mA or more corresponds to the maximum brake command
    }                                                                                                                     // Closes the current code block.
    else                                                                                                                  // Executes this branch when the preceding condition is false.
    {                                                                                                                     // Opens the current code block.
      BRAKE = (uint16_t)map(brake_adc_value,                                                                              // Provides BRAKE = (uint16_t)map(brake_adc_value as the next value or argument in the current multi-line expression.
                            BRAKE_ADC_MIN,                                                                                // Provides BRAKE_ADC_MIN as the next value or argument in the current multi-line expression.
                            BRAKE_ADC_MAX,                                                                                // Provides BRAKE_ADC_MAX as the next value or argument in the current multi-line expression.
                            0L,                                                                                           // Provides 0L as the next value or argument in the current multi-line expression.
                            REL_CURRENT_MAX);                                                                              // Maps the nominal 4-20 mA span to 0-1000
    }                                                                                                                     // Closes the current code block.
  }                                                                                                                       // Closes the current code block.
  else                                                                                                                    // Executes this branch when the preceding condition is false.
  {                                                                                                                       // Opens the current code block.
    brake_read_ok = false;                                                                                                // Sets brake_read_ok to false.
    brake_signal_valid = false;                                                                                           // An ADC acquisition failure invalidates the brake signal
  }                                                                                                                       // Closes the current code block.

  if (HAL_ADC_Stop(&hadc2) != HAL_OK)                                                                                     // Stops ADC2 after the conversion
  {                                                                                                                       // Opens the current code block.
    brake_read_ok = false;                                                                                                // Sets brake_read_ok to false.
    brake_signal_valid = false;                                                                                           // Sets brake_signal_valid to false.
  }                                                                                                                       // Closes the current code block.

  return brake_read_ok && brake_signal_valid;                                                                             // Only a correctly acquired and electrically valid signal is accepted
}                                                                                                                         // Closes the current code block.

bool R2D_Inputs_Are_Active(uint16_t brake_adc)                                                                            // Checks ARRANQUE and the single valid 4-20 mA brake channel
{                                                                                                                         // Opens the current code block.
  const bool start_button_pressed =                                                                                       // Continues the current program structure without changing the configured logic.
      (HAL_GPIO_ReadPin(ARRANQUE_GPIO_Port, ARRANQUE_Pin) == ARRANQUE_ACTIVE_STATE);                                      // Continues the current multi-line expression or function call.

  const bool brake_pressed =                                                                                              // Continues the current program structure without changing the configured logic.
      (brake_adc >= BRAKE_ACTIVE_THRESHOLD_ADC);                                                                          // Approx. 12 mA (50% of the 4-20 mA measurement span)

  return start_button_pressed && brake_pressed && brake_signal_valid;                                                      // R2D is valid only with ARRANQUE pressed and a valid brake signal above threshold
}                                                                                                                         // Closes the current code block.

#pragma endregion                                                                                                         // Ends the current IDE folding region.

#pragma region Wait Send R2D                                                                                              // Starts the IDE folding region for Wait Send R2D.

void Wait_And_Send_R2D_Activation(void)                                                                                   // Blocks startup until ARRANQUE and the brake input are valid for two seconds
{                                                                                                                         // Opens the current code block.
  uint32_t hold_start_tick = 0U;                                                                                          // Tick at which brake and ARRANQUE first become simultaneously active
  bool hold_timer_started = false;                                                                                        // Indicates whether the two-second validation timer is running
  uint32_t last_input_log_tick = 0U;                                                                                      // Declares last_input_log_tick and initializes it with 0U.
  uint32_t last_pdu_log_tick = 0U;                                                                                        // Declares last_pdu_log_tick and initializes it with 0U.
  bool previous_button_pressed = false;                                                                                   // Declares previous_button_pressed and initializes it with false.
  bool previous_brake_pressed = false;                                                                                    // Declares previous_brake_pressed and initializes it with false.
  bool previous_brake_valid = false;                                                                                      // Declares previous_brake_valid and initializes it with false.
  bool first_input_sample = true;                                                                                         // Declares first_input_sample and initializes it with true.

  R2D_LOG("\r\n==================================================\r\n");                                                  // Writes this diagnostic message to the configured R2D debug output.
  R2D_LOG("       VCU - R2D STARTUP DIAGNOSTIC\r\n");                                                                     // Writes this diagnostic message to the configured R2D debug output.
  R2D_LOG("==================================================\r\n");                                                      // Writes this diagnostic message to the configured R2D debug output.
  R2D_LOG("[R2D] Waiting for valid brake + ARRANQUE button\r\n");                                                         // Writes this diagnostic message to the configured R2D debug output.
  R2D_LOG("[R2D] Brake threshold ADC = %u\r\n", BRAKE_ACTIVE_THRESHOLD_ADC);                                              // Sets R2D_LOG("[R2D] Brake threshold ADC to %u\r\n", BRAKE_ACTIVE_THRESHOLD_ADC).
  R2D_LOG("[R2D] Required hold time  = %lu ms\r\n", (unsigned long)R2D_HOLD_TIME_MS);                                     // Sets R2D_LOG("[R2D] Required hold time to %lu ms\r\n", (unsigned long)R2D_HOLD_TIME_MS).

  while (1)                                                                                                               // Repeats the enclosed block while 1 remains true.
  {                                                                                                                       // Opens the current code block.
    const bool brake_read_ok = Read_Brake_Channel();                                                                      // Declares brake_read_ok and initializes it with Read_Brake_Channel().
    const bool button_pressed =                                                                                           // Continues the current program structure without changing the configured logic.
        (HAL_GPIO_ReadPin(ARRANQUE_GPIO_Port, ARRANQUE_Pin) == ARRANQUE_ACTIVE_STATE);                                    // Continues the current multi-line expression or function call.
    const bool brake_pressed = (brake_adc_value >= BRAKE_ACTIVE_THRESHOLD_ADC);                                           // Declares brake_pressed and initializes it with (brake_adc_value >= BRAKE_ACTIVE_THRESHOLD_ADC).
    const uint32_t now = HAL_GetTick();                                                                                   // Declares now and initializes it with HAL_GetTick().

    if (first_input_sample ||                                                                                             // Checks whether first_input_sample || before executing the conditional block.
        (button_pressed != previous_button_pressed) ||                                                                    // Continues the current multi-line logical or arithmetic expression.
        (brake_pressed != previous_brake_pressed) ||                                                                      // Continues the current multi-line logical or arithmetic expression.
        (brake_signal_valid != previous_brake_valid) ||                                                                   // Continues the current multi-line logical or arithmetic expression.
        ((uint32_t)(now - last_input_log_tick) >= R2D_LOG_PERIOD_MS))                                                     // Continues the current multi-line expression or function call.
    {                                                                                                                     // Opens the current code block.
      const uint32_t brake_current_centi_mA =                                                                             // Continues the current program structure without changing the configured logic.
          (brake_current_mA > 0.0f) ? (uint32_t)(brake_current_mA * 100.0f + 0.5f) : 0U;                                  // Continues the current multi-line expression or function call.

      R2D_LOG("[INPUT] Button=%u | Brake=%u | Valid=%u | ReadOK=%u | ADC=%u | I=%lu.%02lu mA | BRAKE=%u/1000\r\n",        // Provides R2D_LOG("[INPUT] Button=%u | Brake=%u | Valid=%u | ReadOK=%u | ADC=%u | I=%lu.%02lu mA | BRAKE=%u/1000\r\n" as the next value or argument in the current multi-line expression.
              button_pressed ? 1U : 0U,                                                                                   // Provides button_pressed ? 1U : 0U as the next value or argument in the current multi-line expression.
              brake_pressed ? 1U : 0U,                                                                                    // Provides brake_pressed ? 1U : 0U as the next value or argument in the current multi-line expression.
              brake_signal_valid ? 1U : 0U,                                                                               // Provides brake_signal_valid ? 1U : 0U as the next value or argument in the current multi-line expression.
              brake_read_ok ? 1U : 0U,                                                                                    // Provides brake_read_ok ? 1U : 0U as the next value or argument in the current multi-line expression.
              brake_adc_value,                                                                                            // Provides brake_adc_value as the next value or argument in the current multi-line expression.
              (unsigned long)(brake_current_centi_mA / 100U),                                                             // Provides (unsigned long)(brake_current_centi_mA / 100U) as the next value or argument in the current multi-line expression.
              (unsigned long)(brake_current_centi_mA % 100U),                                                             // Provides (unsigned long)(brake_current_centi_mA % 100U) as the next value or argument in the current multi-line expression.
              BRAKE);                                                                                                     // Executes this statement as part of the current control flow.

      previous_button_pressed = button_pressed;                                                                           // Sets previous_button_pressed to button_pressed.
      previous_brake_pressed = brake_pressed;                                                                             // Sets previous_brake_pressed to brake_pressed.
      previous_brake_valid = brake_signal_valid;                                                                          // Sets previous_brake_valid to brake_signal_valid.
      last_input_log_tick = now;                                                                                          // Sets last_input_log_tick to now.
      first_input_sample = false;                                                                                         // Sets first_input_sample to false.
    }                                                                                                                     // Closes the current code block.

    if (brake_read_ok && R2D_Inputs_Are_Active(brake_adc_value))                                                          // Checks whether brake_read_ok && R2D_Inputs_Are_Active(brake_adc_value) before executing the conditional block.
    {                                                                                                                     // Opens the current code block.
      if (!hold_timer_started)                                                                                            // Checks whether !hold_timer_started before executing the conditional block.
      {                                                                                                                   // Opens the current code block.
        hold_start_tick = now;                                                                                            // Sets hold_start_tick to now.
        hold_timer_started = true;                                                                                        // Sets hold_timer_started to true.
        R2D_LOG("[R2D] Conditions valid -> starting 2 s hold timer\r\n");                                                 // Writes this diagnostic message to the configured R2D debug output.
      }                                                                                                                   // Closes the current code block.
      else if ((uint32_t)(now - hold_start_tick) >= R2D_HOLD_TIME_MS)                                                     // Checks the alternative condition (uint32_t)(now - hold_start_tick) >= R2D_HOLD_TIME_MS.
      {                                                                                                                   // Opens the current code block.
        uint32_t r2d_wait_start_tick;                                                                                     // Declares r2d_wait_start_tick as a uint32_t variable used by this module.
        uint16_t previous_r2d_status = 0xFFFFU;                                                                           // Declares previous_r2d_status and initializes it with 0xFFFFU.
        uint16_t previous_r2d_internal_status = 0xFFFFU;                                                                  // Declares previous_r2d_internal_status and initializes it with 0xFFFFU.
        uint16_t previous_r2d_errors = 0xFFFFU;                                                                           // Declares previous_r2d_errors and initializes it with 0xFFFFU.

        R2D_LOG("[R2D] Hold completed successfully (%lu ms)\r\n",                                                         // Provides R2D_LOG("[R2D] Hold completed successfully (%lu ms)\r\n" as the next value or argument in the current multi-line expression.
                (unsigned long)(now - hold_start_tick));                                                                  // Continues the current multi-line expression or function call.

        R2D_status = 0U;                                                                                                  // Sets R2D_status to 0U.
        R2D_internal_status = 0U;                                                                                         // Sets R2D_internal_status to 0U.
        R2D_error_flags = 0U;                                                                                             // Sets R2D_error_flags to 0U.
        CAN_reception_error = false;                                                                                      // Sets CAN_reception_error to false.
        PDU_last_R2D_tick = 0U;                                                                                           // Sets PDU_last_R2D_tick to 0U.

        R2D_LOG("[R2D] Sending activation request to PDU...\r\n");                                                        // Writes this diagnostic message to the configured R2D debug output.
        Send_R2D_Activation();                                                                                            // Sends the R2D activation request to the PDU over CAN.
        r2d_wait_start_tick = HAL_GetTick();                                                                              // Sets r2d_wait_start_tick to HAL_GetTick().
        last_pdu_log_tick = r2d_wait_start_tick;                                                                          // Sets last_pdu_log_tick to r2d_wait_start_tick.

        R2D_LOG("[R2D] Waiting for fresh PDU status = 0x%04X (ACTIVE)\r\n", R2D_STATUS_ACTIVE);                           // Sets R2D_LOG("[R2D] Waiting for fresh PDU status to 0x%04X (ACTIVE)\r\n", R2D_STATUS_ACTIVE).

        while ((R2D_status != R2D_STATUS_ACTIVE) ||                                                                       // Repeats the enclosed block while (R2D_status != R2D_STATUS_ACTIVE remains true.
               (PDU_last_R2D_tick == 0U))                                                                                 // Continues the current multi-line expression or function call.
        {                                                                                                                 // Opens the current code block.
          const uint32_t pdu_now = HAL_GetTick();                                                                         // Declares pdu_now and initializes it with HAL_GetTick().

          if ((R2D_status != previous_r2d_status) ||                                                                      // Checks whether (R2D_status != previous_r2d_status before executing the conditional block.
              (R2D_internal_status != previous_r2d_internal_status) ||                                                    // Continues the current multi-line logical or arithmetic expression.
              (R2D_error_flags != previous_r2d_errors) ||                                                                 // Continues the current multi-line logical or arithmetic expression.
              ((uint32_t)(pdu_now - last_pdu_log_tick) >= R2D_PDU_LOG_PERIOD_MS))                                         // Continues the current multi-line expression or function call.
          {                                                                                                               // Opens the current code block.
            R2D_LOG("[PDU] Status=0x%04X | Internal=0x%04X | AIR+=%u | AIR-=%u | PRE=%u | DIS=%u | VCUbit=%u | Errors=0x%04X | Fresh=%u\r\n", // Provides R2D_LOG("[PDU] Status=0x%04X | Internal=0x%04X | AIR+=%u | AIR-=%u | PRE=%u | DIS=%u | VCUbit=%u | Errors=0x%04X | Fresh=%u\r\n" as the next value or argument in the current multi-line expression.
                    R2D_status,                                                                                           // Provides R2D_status as the next value or argument in the current multi-line expression.
                    R2D_internal_status,                                                                                  // Provides R2D_internal_status as the next value or argument in the current multi-line expression.
                    R2D_air_positive_active ? 1U : 0U,                                                                    // Provides R2D_air_positive_active ? 1U : 0U as the next value or argument in the current multi-line expression.
                    R2D_air_negative_active ? 1U : 0U,                                                                    // Provides R2D_air_negative_active ? 1U : 0U as the next value or argument in the current multi-line expression.
                    R2D_precharge_active ? 1U : 0U,                                                                       // Provides R2D_precharge_active ? 1U : 0U as the next value or argument in the current multi-line expression.
                    R2D_discharge_active ? 1U : 0U,                                                                       // Provides R2D_discharge_active ? 1U : 0U as the next value or argument in the current multi-line expression.
                    R2D_activation_sequence_vcu ? 1U : 0U,                                                                // Provides R2D_activation_sequence_vcu ? 1U : 0U as the next value or argument in the current multi-line expression.
                    R2D_error_flags,                                                                                      // Provides R2D_error_flags as the next value or argument in the current multi-line expression.
                    (PDU_last_R2D_tick != 0U) ? 1U : 0U);                                                                 // Continues the current multi-line expression or function call.

            previous_r2d_status = R2D_status;                                                                             // Sets previous_r2d_status to R2D_status.
            previous_r2d_internal_status = R2D_internal_status;                                                           // Sets previous_r2d_internal_status to R2D_internal_status.
            previous_r2d_errors = R2D_error_flags;                                                                        // Sets previous_r2d_errors to R2D_error_flags.
            last_pdu_log_tick = pdu_now;                                                                                  // Sets last_pdu_log_tick to pdu_now.
          }                                                                                                               // Closes the current code block.

          if (R2D_error_flags != 0U)                                                                                      // Checks whether R2D_error_flags != 0U before executing the conditional block.
          {                                                                                                               // Opens the current code block.
            R2D_LOG("[ERROR] PDU R2D error flags = 0x%04X\r\n", R2D_error_flags);                                         // Sets R2D_LOG("[ERROR] PDU R2D error flags to 0x%04X\r\n", R2D_error_flags).
            Error_Handler();                                                                                              // Transfers execution to the application error handler.
          }                                                                                                               // Closes the current code block.

          if (CAN_reception_error)                                                                                        // Checks whether CAN_reception_error before executing the conditional block.
          {                                                                                                               // Opens the current code block.
            R2D_LOG("[ERROR] FDCAN reception error while waiting for PDU\r\n");                                           // Writes this diagnostic message to the configured R2D debug output.
            Error_Handler();                                                                                              // Transfers execution to the application error handler.
          }                                                                                                               // Closes the current code block.

          if ((uint32_t)(pdu_now - r2d_wait_start_tick) >= R2D_ACTIVE_TIMEOUT_MS)                                         // Checks whether (uint32_t)(pdu_now - r2d_wait_start_tick) >= R2D_ACTIVE_TIMEOUT_MS before executing the conditional block.
          {                                                                                                               // Opens the current code block.
            R2D_LOG("[ERROR] Timeout: PDU did not report R2D ACTIVE within %lu ms\r\n",                                   // Provides R2D_LOG("[ERROR] Timeout: PDU did not report R2D ACTIVE within %lu ms\r\n" as the next value or argument in the current multi-line expression.
                    (unsigned long)R2D_ACTIVE_TIMEOUT_MS);                                                                // Continues the current multi-line expression or function call.
            Error_Handler();                                                                                              // Transfers execution to the application error handler.
          }                                                                                                               // Closes the current code block.

          HAL_Delay(R2D_STATUS_CHECK_PERIOD_MS);                                                                          // Blocks execution for the requested delay interval.
        }                                                                                                                 // Closes the current code block.

        R2D_LOG("[R2D] PDU confirms ACTIVE -> Status=0x%04X\r\n", R2D_status);                                            // Sets R2D_LOG("[R2D] PDU confirms ACTIVE -> Status to 0x%04X\r\n", R2D_status).
        R2D_LOG("[R2D] AIR+=%u | AIR-=%u | PRE=%u | DIS=%u | Errors=0x%04X\r\n",                                          // Provides R2D_LOG("[R2D] AIR+=%u | AIR-=%u | PRE=%u | DIS=%u | Errors=0x%04X\r\n" as the next value or argument in the current multi-line expression.
                R2D_air_positive_active ? 1U : 0U,                                                                        // Provides R2D_air_positive_active ? 1U : 0U as the next value or argument in the current multi-line expression.
                R2D_air_negative_active ? 1U : 0U,                                                                        // Provides R2D_air_negative_active ? 1U : 0U as the next value or argument in the current multi-line expression.
                R2D_precharge_active ? 1U : 0U,                                                                           // Provides R2D_precharge_active ? 1U : 0U as the next value or argument in the current multi-line expression.
                R2D_discharge_active ? 1U : 0U,                                                                           // Provides R2D_discharge_active ? 1U : 0U as the next value or argument in the current multi-line expression.
                R2D_error_flags);                                                                                         // Executes this statement as part of the current control flow.

        R2D_LOG("[R2D] Activating buzzer for %lu ms...\r\n",                                                              // Provides R2D_LOG("[R2D] Activating buzzer for %lu ms...\r\n" as the next value or argument in the current multi-line expression.
                (unsigned long)R2D_BUZZER_TIME_MS);                                                                       // Continues the current multi-line expression or function call.
        Sound_R2D_Buzzer();                                                                                               // Activates the R2D buzzer sequence.
        R2D_LOG("[R2D] Buzzer finished\r\n");                                                                             // Writes this diagnostic message to the configured R2D debug output.
        R2D_LOG("[R2D] STARTUP SEQUENCE COMPLETE -> entering main control loop\r\n");                                     // Writes this diagnostic message to the configured R2D debug output.
        R2D_LOG("==================================================\r\n\r\n");                                            // Writes this diagnostic message to the configured R2D debug output.
        return;                                                                                                           // Returns from the current function.
      }                                                                                                                   // Closes the current code block.
    }                                                                                                                     // Closes the current code block.
    else                                                                                                                  // Executes this branch when the preceding condition is false.
    {                                                                                                                     // Opens the current code block.
      if (hold_timer_started)                                                                                             // Checks whether hold_timer_started before executing the conditional block.
      {                                                                                                                   // Opens the current code block.
        R2D_LOG("[R2D] Conditions lost before 2 s -> hold timer RESET\r\n");                                              // Writes this diagnostic message to the configured R2D debug output.
      }                                                                                                                   // Closes the current code block.
      hold_timer_started = false;                                                                                         // Sets hold_timer_started to false.
    }                                                                                                                     // Closes the current code block.

    HAL_Delay(R2D_CHECK_PERIOD_MS);                                                                                       // Blocks execution for the requested delay interval.
  }                                                                                                                       // Closes the current code block.
}                                                                                                                         // Closes the current code block.

#pragma endregion                                                                                                         // Ends the current IDE folding region.
#pragma region buzzer                                                                                                     // Starts the IDE folding region for buzzer.

void Sound_R2D_Buzzer(void)                                                                                               // Activate the sound of the buzzer
{                                                                                                                         // Opens the current code block.
    HAL_GPIO_WritePin(BUZZER_GPIO_Port,                                                                                   // Set on the pin of the buzzer
                      BUZZER_Pin,                                                                                         // Provides BUZZER_Pin as the next value or argument in the current multi-line expression.
                      GPIO_PIN_SET);                                                                                      // Executes this statement as part of the current control flow.

    HAL_Delay(R2D_BUZZER_TIME_MS);                                                                                        // Waits the time for the buzzer to sound

    HAL_GPIO_WritePin(BUZZER_GPIO_Port,                                                                                   // Shut down the pin
                      BUZZER_Pin,                                                                                         // Provides BUZZER_Pin as the next value or argument in the current multi-line expression.
                      GPIO_PIN_RESET);                                                                                    // Executes this statement as part of the current control flow.
}                                                                                                                         // Closes the current code block.

#pragma endregion                                                                                                         // Ends the current IDE folding region.
#pragma region send CAN mesagges                                                                                          // Starts the IDE folding region for send CAN mesagges.

void Send_CAN_Message(uint32_t id, uint8_t *data, uint32_t len)                                                           // Sends a CAN message with the specified ID, data, and length
{                                                                                                                         // Opens the current code block.
  TxHeader.Identifier = id;                                                                                               // Set the CAN ID for the message to be sent
  if (len == 0)                                                                                                           // If the length is 0, set the Data Length Code (DLC) to 0 bytes
    TxHeader.DataLength = FDCAN_DLC_BYTES_0;                                                                              // Set the Data Length Code (DLC) with a macro to 0 bytes
  else if (len == 1)                                                                                                      // If the length is 1, set the Data Length Code (DLC) to 1 byte
    TxHeader.DataLength = FDCAN_DLC_BYTES_1;                                                                              // Set the Data Length Code (DLC) with a macro to 1 byte
  else if (len == 2)                                                                                                      // If the length is 2, set the Data Length Code (DLC) to 2 bytes
    TxHeader.DataLength = FDCAN_DLC_BYTES_2;                                                                              // Set the Data Length Code (DLC) with a macro to 2 bytes
  else if (len == 3)                                                                                                      // If the length is 3, set the Data Length Code (DLC) to 3 bytes
    TxHeader.DataLength = FDCAN_DLC_BYTES_3;                                                                              // Set the Data Length Code (DLC) with a macro to 3 bytes
  else if (len == 4)                                                                                                      // If the length is 4, set the Data Length Code (DLC) to 4 bytes
    TxHeader.DataLength = FDCAN_DLC_BYTES_4;                                                                              // Set the Data Length Code (DLC) with a macro to 4 bytes
  else if (len == 5)                                                                                                      // If the length is 5, set the Data Length Code (DLC) to 5 bytes
    TxHeader.DataLength = FDCAN_DLC_BYTES_5;                                                                              // Set the Data Length Code (DLC) with a macro to 5 bytes
  else if (len == 6)                                                                                                      // If the length is 6, set the Data Length Code (DLC) to 6 bytes
    TxHeader.DataLength = FDCAN_DLC_BYTES_6;                                                                              // Set the Data Length Code (DLC) with a macro to 6 bytes
  else if (len == 7)                                                                                                      // If the length is 7, set the Data Length Code (DLC) to 7 bytes
    TxHeader.DataLength = FDCAN_DLC_BYTES_7;                                                                              // Set the Data Length Code (DLC) with a macro to 7 bytes
  else                                                                                                                    // If the length is 8 or more, set the Data Length Code (DLC) to 8 bytes (maximum allowed)
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;                                                                              // Set the Data Length Code (DLC) with a macro to 8 bytes

  const uint32_t tx_wait_start_tick = HAL_GetTick();                                                                      // Start the bounded wait for a free Tx FIFO element

  while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0U)                                                                    // Repeats the enclosed block while HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0U remains true.
  {                                                                                                                       // Opens the current code block.
    if ((uint32_t)(HAL_GetTick() - tx_wait_start_tick) >= CAN_TX_TIMEOUT_MS)                                              // Checks whether (uint32_t)(HAL_GetTick() - tx_wait_start_tick) >= CAN_TX_TIMEOUT_MS before executing the conditional block.
    {                                                                                                                     // Opens the current code block.
      Error_Handler();                                                                                                    // Prevent an infinite lock if the transmit FIFO never becomes available
    }                                                                                                                     // Closes the current code block.
  }                                                                                                                       // Closes the current code block.

  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, data) != HAL_OK)                                                 // If adding the message to the Tx FIFO fails, call the error handler
  {                                                                                                                       // Opens the current code block.
    Error_Handler();                                                                                                      // Call the error handler to indicate a failure in sending the CAN message
  }                                                                                                                       // Closes the current code block.
}                                                                                                                         // Closes the current code block.

#pragma endregion                                                                                                         // Ends the current IDE folding region.

#pragma region Handler error                                                                                              // Starts the IDE folding region for Handler error.

void Error_Handler(void)                                                                                                  // Places the application in a visible non-driving fault state
{                                                                                                                         // Opens the current code block.
  R2D_LOG("[ERROR] Error_Handler entered -> control loop stopped\r\n");                                                   // Writes this diagnostic message to the configured R2D debug output.

  while (1)                                                                                                               // Repeats the enclosed block while 1 remains true.
  {                                                                                                                       // Opens the current code block.
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);                                                                                // Blink the onboard LED while the principal control loop remains stopped
    HAL_Delay(200U);                                                                                                      // SysTick remains enabled so the diagnostic blink continues operating
  }                                                                                                                       // Closes the current code block.
}                                                                                                                         // Closes the current code block.

#pragma endregion                                                                                                         // Ends the current IDE folding region.

#pragma region Check HAL parameters                                                                                       // Starts the IDE folding region for Check HAL parameters.

#ifdef USE_FULL_ASSERT                                                                                                    // If full assert is enabled, this function is called when an assertion fails
void assert_failed(uint8_t *file, uint32_t line)                                                                          // Reports the source location through the debugger and stops normal control
{                                                                                                                         // Opens the current code block.
  (void)file;                                                                                                             // Marks file as intentionally unused to avoid compiler warnings.
  (void)line;                                                                                                             // Marks line as intentionally unused to avoid compiler warnings.
  Error_Handler();                                                                                                        // Transfers execution to the application error handler.
}                                                                                                                         // Closes the current code block.
#endif /* USE_FULL_ASSERT */                                            
       /* USER CODE END 4 */

#pragma endregion                                                                                                         // Ends the current IDE folding region.
