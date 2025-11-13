/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines (TIM2 XY + TIM3 Z + USB).
  ******************************************************************************
  */
/* USER CODE END Header */

#include "stm32f1xx_hal.h"
#include "stm32f1xx_it.h"
#include "motion_executor.h"    /* TIM2 */
#include "motion_executor_z.h"  /* TIM3 */
#include "usbd_conf.h"

extern PCD_HandleTypeDef hpcd_USB_FS;

/* Cortex-M3 Exception Handlers */
void NMI_Handler(void)            { while (1) { } }
void HardFault_Handler(void)      { while (1) { } }
void MemManage_Handler(void)      { while (1) { } }
void BusFault_Handler(void)       { while (1) { } }
void UsageFault_Handler(void)     { while (1) { } }
void SVC_Handler(void)            { }
void DebugMon_Handler(void)       { }
void PendSV_Handler(void)         { }
void SysTick_Handler(void)        { HAL_IncTick(); }

/* TIM2 IRQ: X-Y gantry motion */
void TIM2_IRQHandler(void)
{
  Motion_TIM2_IRQHandler();
}

/* TIM3 IRQ: Z axis motion */
void TIM3_IRQHandler(void)
{
  Motion_TIM3_IRQHandler();
}

/* USB FS IRQ (CDC) */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
  HAL_PCD_IRQHandler(&hpcd_USB_FS);
}

/* USER CODE BEGIN 1 */
/* Additional peripheral handlers can be added here */
/* USER CODE END 1 */
