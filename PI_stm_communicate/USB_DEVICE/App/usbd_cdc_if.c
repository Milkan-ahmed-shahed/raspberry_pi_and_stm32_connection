/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @version        : v2.0_Cube
  * @brief          : USB device CDC interface (Virtual COM Port)
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_if.h"
#include <string.h>

/* TX queue ------------------------------------------------------------------*/
#define TX_QUEUE_DEPTH 4
static uint8_t  tx_q_buf[TX_QUEUE_DEPTH][APP_TX_DATA_SIZE];
static uint16_t tx_q_len[TX_QUEUE_DEPTH];
static volatile uint8_t tx_q_head = 0;
static volatile uint8_t tx_q_tail = 0;
static volatile uint8_t tx_busy_flag = 0;

/* Forward declarations for CDC fops (must be before fops struct) */
static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t* pbuf, uint32_t *Len);

/* IMPORTANT: external linkage callbacks the middleware may call */
int8_t CDC_TransmitCplt_FS(uint8_t* Buf, uint32_t *Len, uint8_t epnum);
void   USBD_CDC_TransmitCplt(USBD_HandleTypeDef *pdev, uint8_t epnum);

/* Queue helpers (exported in header) */
int  CDC_Transmit_Enqueue(uint8_t* Buf, uint16_t Len);
void CDC_ProcessTxQueue(void);

/* USB buffers ---------------------------------------------------------------*/
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

/* Provided by the device core */
extern USBD_HandleTypeDef hUsbDeviceFS;

/* USBD interface callbacks (exactly 4 entries) */
USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS
};

/* Init ----------------------------------------------------------------------*/
static int8_t CDC_Init_FS(void)
{
  /* Set buffers */
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);

  /* Prepare to receive first packet */
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return (USBD_OK);
}

static int8_t CDC_DeInit_FS(void)
{
  return (USBD_OK);
}

static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  (void)pbuf; (void)length;
  switch (cmd)
  {
    case CDC_SET_LINE_CODING: break;
    case CDC_GET_LINE_CODING: break;
    case CDC_SET_CONTROL_LINE_STATE: break;
    default: break;
  }
  return (USBD_OK);
}

/* Receive: copy data into a small buffer and raise a flag for the main loop */
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
#ifndef RX_BUFFER_SIZE
#define RX_BUFFER_SIZE 64
#endif
  extern uint8_t g_usb_rx_buffer[RX_BUFFER_SIZE];
  extern volatile uint8_t g_new_usb_data;

  uint32_t copy_len = (*Len < (RX_BUFFER_SIZE - 1)) ? *Len : (RX_BUFFER_SIZE - 1);
  memcpy(g_usb_rx_buffer, Buf, copy_len);
  g_usb_rx_buffer[copy_len] = '\0';
  g_new_usb_data = 1;

  /* Re-arm reception */
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);

  return (USBD_OK);
}

/* Try immediate send, else enqueue */
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;

  if (hcdc && hcdc->TxState == 0)
  {
    USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
    uint8_t res = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
    if (res == USBD_OK)
    {
      tx_busy_flag = 1;
      return USBD_OK;
    }
    /* else fallthrough to enqueue */
  }

  if (CDC_Transmit_Enqueue(Buf, Len) == 0)
  {
    /* Kick the queue so transmit starts immediately if idle */
    CDC_ProcessTxQueue();
    return USBD_OK;
  }

  return USBD_BUSY;
}

/* Enqueue with tiny critical section */
int CDC_Transmit_Enqueue(uint8_t* Buf, uint16_t Len)
{
  uint8_t next_head = (tx_q_head + 1) % TX_QUEUE_DEPTH;
  if (next_head == tx_q_tail) return -1; /* queue full */
  if (Len > APP_TX_DATA_SIZE) Len = APP_TX_DATA_SIZE;

  memcpy(tx_q_buf[tx_q_head], Buf, Len);

  __disable_irq();
  tx_q_len[tx_q_head] = Len;
  tx_q_head = next_head;
  __enable_irq();

  return 0;
}

/* Dequeue/send next if not busy. */
void CDC_ProcessTxQueue(void)
{
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;

  if (tx_busy_flag && hcdc && hcdc->TxState == 0)
  {
    tx_busy_flag = 0;
  }

  if (tx_busy_flag) return;
  if (tx_q_head == tx_q_tail) return;

  uint8_t idx = tx_q_tail;
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, tx_q_buf[idx], tx_q_len[idx]);
  if (USBD_CDC_TransmitPacket(&hUsbDeviceFS) == USBD_OK)
  {
    __disable_irq();
    tx_q_tail = (tx_q_tail + 1) % TX_QUEUE_DEPTH;
    tx_busy_flag = 1;
    __enable_irq();
  }
}

int8_t CDC_TransmitCplt_FS(uint8_t* Buf, uint32_t *Len, uint8_t epnum)
{
  (void)Buf; (void)Len; (void)epnum;
  tx_busy_flag = 0;
  CDC_ProcessTxQueue();
  return (USBD_OK);
}

void USBD_CDC_TransmitCplt(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  (void)pdev; (void)epnum;
  tx_busy_flag = 0;
  CDC_ProcessTxQueue();
}
