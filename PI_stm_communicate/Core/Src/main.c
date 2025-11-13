/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (5-axis: X/Y, Z steppers + Rotation/Gripper servos)
  *
  * Cleaned version:
  *  - Removes all legacy LED blinker code (no PB6 GPIO toggling, no A/B/C commands,
  *    no blink state or executor).
  *  - Only MOVE and SERVO parsers are present.
  *  - MX_GPIO_Init() is "gutted": only enables GPIO clocks; all pin modes are set
  *    in Motion_InitXY(), Motion_InitZ(), and Servo_Init().
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

#include "motion_executor.h"     /* X/Y on TIM2 */
#include "motion_executor_z.h"   /* Z on TIM3 */
#include "servo_executor.h"      /* Rotation/Gripper on TIM4 (PB6/PB7) */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* If not declared elsewhere, provide RX/TX handling defaults */
#ifndef RX_BUFFER_SIZE
#define RX_BUFFER_SIZE 64
#endif

/* If you use a TX queue elsewhere, you can keep CDC_Transmit_Enqueue/CDC_ProcessTxQueue.
   Otherwise, define them to use CDC_Transmit_FS directly here. */
#ifndef CDC_HELPERS_DECLARED
static inline int CDC_Transmit_Enqueue(uint8_t* Buf, uint16_t Len) {
  return (int)CDC_Transmit_FS(Buf, Len);
}
static inline void CDC_ProcessTxQueue(void) {
  /* No-op when not using a ring buffer */
}
#endif

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

/* Simple RX buffer + flag set by CDC_Receive_FS (in usbd_cdc_if.c) */
uint8_t g_usb_rx_buffer[RX_BUFFER_SIZE];
volatile uint8_t g_new_usb_data = 0;

/* -------------------------------------------------------------------------- */
/* Main                                                                       */
/* -------------------------------------------------------------------------- */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();          /* ONLY enables GPIO port clocks */
  MX_USB_DEVICE_Init();

  /* Initialize motion subsystems */
  Motion_InitXY();         /* TIM2 */
  Motion_InitZ();          /* TIM3 */
  Servo_Init();            /* TIM4 PB6/PB7 */

  while (1)
  {
    /* Keep TX flowing if using a queued TX implementation */
    CDC_ProcessTxQueue();

    if (g_new_usb_data == 1)
    {
      char *command = (char*)g_usb_rx_buffer;
      /* Strip CR/LF to make parsing easier */
      command[strcspn(command, "\r\n")] = '\0';

      int valid = 0;
      int custom_replied = 0;

      /* ---------------- MOVE Command ---------------- */
      if (strncmp(command, "MOVE", 4) == 0) {
        int have_x = 0, have_y = 0, have_z = 0;
        int32_t x_mm = 0, y_mm = 0, z_mm = 0;
        uint32_t feed_mm_min = 300;

        char *xptr = strstr(command, "X");
        if (xptr) { x_mm = (int32_t)strtol(xptr + 1, NULL, 10); have_x = 1; }
        char *yptr = strstr(command, "Y");
        if (yptr) { y_mm = (int32_t)strtol(yptr + 1, NULL, 10); have_y = 1; }
        char *zptr = strstr(command, "Z");
        if (zptr) { z_mm = (int32_t)strtol(zptr + 1, NULL, 10); have_z = 1; }
        char *sptr = strstr(command, "S");
        if (sptr) { long s = strtol(sptr + 1, NULL, 10); if (s > 0) feed_mm_min = (uint32_t)s; }

        if (!have_x && !have_y && !have_z) {
          valid = 0;
          goto send_reply;
        }

        /* If any axis is busy, reject with BUSY */
        if (Motion_XY_IsBusy() || Motion_Z_IsBusy()) {
          static const uint8_t busy[] = "BUSY\r\n";
          CDC_Transmit_Enqueue((uint8_t*)busy, sizeof(busy) - 1);
          valid = 0;
          goto send_reply;
        }

        if (have_z && !have_x && !have_y) {
          /* Z-only */
          valid = Move_Z(z_mm, feed_mm_min) ? 1 : 0;
        } else {
          /* XY move (missing component treated as 0 relative) */
          if (!have_x) x_mm = 0;
          if (!have_y) y_mm = 0;
          valid = Move_Gantry_XY(x_mm, y_mm, feed_mm_min) ? 1 : 0;
        }
      }
      /* ---------------- SERVO Command ---------------- */
      else if (strncmp(command, "SERVO", 5) == 0) {
        int have_r = 0, have_g = 0;
        long r = -1, g = -1;

        char *rptr = strstr(command, "R");
        if (rptr) { r = strtol(rptr + 1, NULL, 10); if (r < 0) r = 0; if (r > 180) r = 180; have_r = 1; }

        char *gptr = strstr(command, "G");
        if (gptr) { g = strtol(gptr + 1, NULL, 10); if (g < 0) g = 0; if (g > 180) g = 180; have_g = 1; }

        if (have_r) Servo_Set_Rotation((uint8_t)r);
        if (have_g) Servo_Set_Gripper((uint8_t)g);

        valid = (have_r || have_g) ? 1 : 0;

        if (valid) {
          static const uint8_t sresp[] = "ACK SERVO OK\r\n";
          CDC_Transmit_Enqueue((uint8_t*)sresp, sizeof(sresp) - 1);
          custom_replied = 1;
        }
      }

send_reply:
      if (!custom_replied) {
        uint8_t ack[96];
        int n = snprintf((char*)ack, sizeof(ack),
                         valid ? "ACK %s OK\r\n" : "ERR %s\r\n", command);
        if (n < 0) n = 0;
        if (n > (int)sizeof(ack)) n = (int)sizeof(ack);
        CDC_Transmit_Enqueue(ack, (uint16_t)n);
      }

      g_new_usb_data = 0;
    }

    HAL_Delay(1);
  }
}

/* -------------------------------------------------------------------------- */
/* System Clock Configuration                                                 */
/* -------------------------------------------------------------------------- */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState       = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL     = RCC_PLL_MUL6;  /* 48 MHz SYSCLK for USB */
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                     RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;   /* APB1 24 MHz -> TIM on APB1 doubles to 48 MHz */
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) { Error_Handler(); }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection    = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) { Error_Handler(); }
}

/* -------------------------------------------------------------------------- */
/* GPIO Init (Gutted: clocks only)                                            */
/* -------------------------------------------------------------------------- */
/**
  * @brief GPIO Initialization Function
  * @note  Critical Fix: ONLY enable GPIO clocks; do NOT configure any pins here.
  *        All pin modes are set in Motion_InitXY(), Motion_InitZ(), Servo_Init().
  */
static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
}

/* -------------------------------------------------------------------------- */
void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file; (void)line;
}
#endif
