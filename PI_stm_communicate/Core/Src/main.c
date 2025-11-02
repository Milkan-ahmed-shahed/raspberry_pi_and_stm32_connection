/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

#include <string.h>  /* strcmp, strcspn */
#include <stdio.h>   /* snprintf */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

/* USER CODE BEGIN PV */
volatile uint32_t blink_duration_ms = 0; /* Total blink time (0 = off) */
volatile uint32_t blink_delay_ms    = 0; /* Interval between toggles */
uint32_t blink_start_time           = 0; /* Start time of active blink */

uint8_t g_usb_rx_buffer[RX_BUFFER_SIZE];
volatile uint8_t g_new_usb_data = 0;     /* 0=no new data, 1=new data ready */
/* USER CODE END PV */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();

  /* Infinite loop */
  while (1)
  {
    /* Keep USB TX flowing */
    CDC_ProcessTxQueue();

    /* Parse any new command */
    if (g_new_usb_data == 1)
    {
      char *command = (char*)g_usb_rx_buffer;

      /* Trim CR/LF */
      command[strcspn(command, "\r\n")] = '\0';

      /* Default: invalid until matched */
      int valid = 0;

      if (strcmp(command, "A") == 0) {
        blink_duration_ms = 5000;  /* 5s */
        blink_delay_ms    = 100;   /* fast */
        valid = 1;
      } else if (strcmp(command, "B") == 0) {
        blink_duration_ms = 10000; /* 10s */
        blink_delay_ms    = 500;   /* medium */
        valid = 1;
      } else if (strcmp(command, "C") == 0) {
        blink_duration_ms = 20000; /* 20s */
        blink_delay_ms    = 1000;  /* slow */
        valid = 1;
      }

      /* Send ACK/ERR */
      uint8_t ack[32];
      int n = snprintf((char*)ack, sizeof(ack),
                       valid ? "ACK %s OK\r\n" : "ERR %s\r\n", command);
      if (n < 0) n = 0;
      if (n > (int)sizeof(ack)) n = sizeof(ack);
      CDC_Transmit_Enqueue(ack, (uint16_t)n);

      /* Arm blink if a valid command */
      if (valid && blink_duration_ms > 0) {
        blink_start_time = HAL_GetTick();
      }

      /* Clear flag */
      g_new_usb_data = 0;
    }

    /* Non-blocking blink executor */
    if (blink_duration_ms > 0)
    {
      uint32_t now = HAL_GetTick();

      if ((now - blink_start_time) >= blink_duration_ms)
      {
        blink_duration_ms = 0;
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); /* OFF */
      }
      else
      {
        static uint32_t last_toggle = 0;
        if (blink_delay_ms > 0 && (now - last_toggle) >= blink_delay_ms)
        {
          HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_6);
          last_toggle = now;
        }
      }
    }

    HAL_Delay(1); /* prevent a hot loop */
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
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
  RCC_OscInitStruct.PLL.PLLMUL     = RCC_PLL_MUL6;  /* SYSCLK = 48 MHz */
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                     RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) { Error_Handler(); }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection    = RCC_USBCLKSOURCE_PLL; /* 48 MHz to USB */
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* PB6 as output push-pull (external LED) */
  GPIO_InitStruct.Pin   = GPIO_PIN_6;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); /* LED OFF */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file; (void)line;
}
#endif /* USE_FULL_ASSERT */
