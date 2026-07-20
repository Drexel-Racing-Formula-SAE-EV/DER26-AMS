/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f7xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "main.h"
#include "stm32f7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app.h"
#include "ext_drivers/ams_safety.h"
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
extern CAN_HandleTypeDef hcan1;
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim7;

/* USER CODE BEGIN EV */
extern app_data_t app;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M7 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */
  ams_safety_panic(AMS_PANIC_NMI);
  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
  ams_safety_panic(AMS_PANIC_HARDFAULT);
  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */
  ams_safety_panic(AMS_PANIC_MEMMANAGE);
  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */
  ams_safety_panic(AMS_PANIC_BUSFAULT);
  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */
  ams_safety_panic(AMS_PANIC_USAGEFAULT);
  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32F7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles CAN1 RX0 interrupts.
  */
void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */

  /* USER CODE END CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */

  /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */

  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

  /* USER CODE END TIM7_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    cli_device_t *cli = &app.board.cli;
    uint8_t c;

    if((huart == NULL) || (cli->huart == NULL) || (huart->Instance != cli->huart->Instance))
    {
        return;
    }

    c = cli->c;

    if(cli->msg_pending)
    {
        /* Keep a completed command intact until the CLI task copies it. Bytes
         * received meanwhile are intentionally dropped and counted. */
        cli->index = 0u;
        cli->rx_drop_count++;
        (void)cli_uart_start_rx(cli);
        return;
    }

    if((c == '\r') || (c == '\n'))
    {
        if(cli->index > 0u)
        {
            cli->line[cli->index] = '\0';
            cli->index = 0u;
            cli->msg_count++;
            cli->msg_pending = true;
        }
    }
    else if((c == '\b') || (c == 127u))
    {
        if(cli->index > 0u)
        {
            cli->index--;
        }
    }
    else if(cli->index < (CLI_LINESZ - 1u))
    {
        cli->line[cli->index++] = (char)c;
    }
    else
    {
        cli->rx_drop_count++;
    }

    /* RX ISR only receives bytes and re-arms RX. All CLI output happens from
     * the CLI task through cli_printline(). */
    (void)cli_uart_start_rx(cli);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    cli_device_t *cli = &app.board.cli;
    uint32_t error_code;
    HAL_StatusTypeDef status;

    if((huart == NULL) || (cli->huart == NULL) || (huart->Instance != cli->huart->Instance))
    {
        return;
    }

    error_code = HAL_UART_GetError(huart);
    cli_uart_note_error(cli, error_code);

    /* Preserve a completed pending command, but discard a partial command
     * because a framing/noise/overrun event may have corrupted it. */
    if(cli->msg_pending == false)
    {
        cli->index = 0u;
        cli->line[0] = '\0';
    }

    __HAL_UART_CLEAR_FLAG(huart,
                          UART_CLEAR_OREF | UART_CLEAR_NEF |
                          UART_CLEAR_PEF | UART_CLEAR_FEF);
    __HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);
    huart->ErrorCode = HAL_UART_ERROR_NONE;

    /* ORE/RTO are blocking HAL errors, so RxState is normally READY here and
     * must be re-armed. For non-blocking PE/FE/NE errors reception can still be
     * BUSY_RX; the periodic task-side health service will recover any
     * inconsistent state that remains. */
    if(huart->RxState == HAL_UART_STATE_READY)
    {
        status = cli_uart_start_rx(cli);
        if(status == HAL_OK)
        {
            cli->rx_recovery_count++;
        }
    }
}
/* USER CODE END 1 */
