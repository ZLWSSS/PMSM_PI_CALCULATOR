/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32f4xx_it.c
 * @brief   Interrupt Service Routines.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f4xx_it.h"
#include "main.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "SEGGER_SYSVIEW.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_tim.h"
#include "task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
uint8_t SystickDivider_CountUB = 2;
// uint8_t Systick_CheckCan = 4;
uint8_t SystickDivider_Count = 0;
uint8_t Velocity_LoopCountUB = 10;
uint8_t Velocity_LoopCount = 0;
uint32_t software_timer = 0;
uint32_t report_counter = 0;
uint32_t Vbus_Tick;
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
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern CAN_HandleTypeDef hcan1;
extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart4;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
 * @brief This function handles Non maskable interrupt.
 */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

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

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
 * @brief This function handles System service call via SWI instruction.
 */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
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

/**
 * @brief This function handles Pendable request for system service.
 */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
 * @brief This function handles System tick timer.
 */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
  // 2000hz ok
  // LL_GPIO_SetOutputPin(Test_Pin_1_GPIO_Port, Test_Pin_1_Pin);
  // #ifdef DEBUG
  //  SEGGER_SYSVIEW_RecordEnterISR();
  // #endif
  if (SystickDivider_Count == SystickDivider_CountUB)
  {
    /* USER CODE END SysTick_IRQn 0 */
    HAL_IncTick();
    /* USER CODE BEGIN SysTick_IRQn 1 */
    SystickDivider_Count = 0;
    software_timer++;
    Vbus_Tick++;
    Cmd_Fresh_Check(software_timer);
  }
  SystickDivider_Count++;

  // Position_Loop();
  // #ifdef DEBUG
  //   SEGGER_SYSVIEW_RecordExitISR();
  // #endif
  //  LL_GPIO_ResetOutputPin(Test_Pin_1_GPIO_Port, Test_Pin_1_Pin);
  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
 * @brief This function handles ADC1, ADC2 and ADC3 interrupts.
 */
void ADC_IRQHandler(void)
{
  /* USER CODE BEGIN ADC_IRQn 0 */
  //LL_GPIO_SetOutputPin(Test_Pin_1_GPIO_Port, Test_Pin_1_Pin);
  //   #ifdef DEBUG
  //    SEGGER_SYSVIEW_RecordEnterISR();
  //   #endif
  //  only adc1 enter
  //  wait for J_adc2 finish
  while (!LL_ADC_IsActiveFlag_JEOS(ADC2))
  {
  }
  LL_ADC_ClearFlag_JEOS(ADC1);
  LL_ADC_ClearFlag_JEOS(ADC2);
  Current_Loop();
  LL_ADC_EnableIT_JEOS(ADC1);
  //LL_GPIO_ResetOutputPin(Test_Pin_1_GPIO_Port, Test_Pin_1_Pin);
  /* USER CODE END ADC_IRQn 0 */
  // HAL_ADC_IRQHandler(&hadc1);
  // HAL_ADC_IRQHandler(&hadc2);
  // HAL_ADC_IRQHandler(&hadc3);
  /* USER CODE BEGIN ADC_IRQn 1 */

  /* USER CODE END ADC_IRQn 1 */
}

/**
 * @brief This function handles CAN1 RX0 interrupt.
 */
void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */
#ifdef DEBUG
  SEGGER_SYSVIEW_RecordEnterISR();
#endif
  uint32_t interrupt = READ_REG(hcan1.Instance->IER);
  if ((interrupt & CAN_IT_RX_FIFO0_MSG_PENDING) != 0U)
  {
    // ensure pending
    if ((hcan1.Instance->RF0R & CAN_RF0R_FMP0) != 0U)
    {
      CAN_MessagePendingCBF(&hcan1);
    }
  }
  /* USER CODE END CAN1_RX0_IRQn 0 */
  // HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */
#ifdef DEBUG
  SEGGER_SYSVIEW_RecordExitISR();
#endif
  /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
 * @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
 */
void TIM1_UP_TIM10_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */

  if (LL_TIM_IsActiveFlag_UPDATE(TIM1))
  {
    //LL_GPIO_SetOutputPin(Test_Pin_2_GPIO_Port, Test_Pin_2_Pin);
    LL_TIM_ClearFlag_UPDATE(TIM1);
    Velocity_LoopCount++;
    if (Velocity_LoopCount == Velocity_LoopCountUB)
    {
      // #ifdef DEBUG
      //       SEGGER_SYSVIEW_RecordEnterISR();
      // #endif
      Velocity_Loop();
      Velocity_LoopCount = 0;
      // #ifdef DEBUG
      //       SEGGER_SYSVIEW_RecordExitISR();
      // #endif
    }
    Sample_Encoder();
    // LL_GPIO_ResetOutputPin(Test_Pin_1_GPIO_Port, Test_Pin_1_Pin);
    //  if(LL_TIM_COUNTERDIRECTION_UP == LL_TIM_GetDirection(TIM1))
    //{
    //    LL_GPIO_SetOutputPin(Blue_LED_GPIO_Port, Blue_LED_Pin);
    //  }
    //LL_GPIO_ResetOutputPin(Test_Pin_2_GPIO_Port, Test_Pin_2_Pin);
  }
  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
  // HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

/**
 * @brief This function handles UART4 global interrupt.
 */
void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */

  /* USER CODE END UART4_IRQn 0 */
  HAL_UART_IRQHandler(&huart4);
  /* USER CODE BEGIN UART4_IRQn 1 */

  /* USER CODE END UART4_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
