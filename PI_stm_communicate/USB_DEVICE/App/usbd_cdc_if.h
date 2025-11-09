/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_cdc_if.h
  * @version        : v2.0_Cube
  * @brief          : Header for usbd_cdc_if.c file.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBD_CDC_IF_H__
#define __USBD_CDC_IF_H__

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc.h"
#include "usbd_def.h"
#include <stdint.h>

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @{
  */

/** @defgroup USBD_CDC_IF USBD_CDC_IF
  * @{
  */

/** @defgroup USBD_CDC_IF_Exported_Defines USBD_CDC_IF_Exported_Defines
  * @{
  */
/* Define size for the receive and transmit buffer over CDC */
#define APP_RX_DATA_SIZE  1024
#define APP_TX_DATA_SIZE  1024

/* Small RX command buffer shared with main.c */
#ifndef RX_BUFFER_SIZE
#define RX_BUFFER_SIZE    64
#endif
/* Externs implemented in main.c */
extern uint8_t        g_usb_rx_buffer[RX_BUFFER_SIZE];
extern volatile uint8_t g_new_usb_data;

/* Mark that helper prototypes are available to avoid duplicate fallbacks */
#ifndef CDC_HELPERS_DECLARED
#define CDC_HELPERS_DECLARED 1
#endif
/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Types USBD_CDC_IF_Exported_Types
  * @{
  */
/* Add user types here if needed */
/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
  * @{
  */
/** CDC Interface callback. */
extern USBD_CDC_ItfTypeDef USBD_Interface_fops_FS;
/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_FunctionsPrototype USBD_CDC_IF_Exported_FunctionsPrototype
  * @{
  */

/* Standard API used by application to send data immediately (may return BUSY) */
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

/* Non-blocking TX queue helpers (implemented in usbd_cdc_if.c) */
void    CDC_ProcessTxQueue(void);
int     CDC_Transmit_Enqueue(uint8_t* Buf, uint16_t Len);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __USBD_CDC_IF_H__ */
