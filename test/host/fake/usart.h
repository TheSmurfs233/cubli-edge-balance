/**
  ******************************************************************************
  * @file    usart.h   (HOST TEST STUB)
  * @brief   Shadows Core/Inc/usart.h during host tests.
  *
  *          huart1 = N100 Mini IMU link (USART1, 921600)
  *          huart2 = debug console    (USART2, 1000000)
  ******************************************************************************
  */

#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern USART_TypeDef      fake_usart1;
extern USART_TypeDef      fake_usart2;

extern DMA_HandleTypeDef  hdma_usart1_rx;
extern DMA_HandleTypeDef  hdma_usart1_tx;
extern DMA_HandleTypeDef  hdma_usart2_rx;
extern DMA_HandleTypeDef  hdma_usart2_tx;

#define USART1 (&fake_usart1)
#define USART2 (&fake_usart2)

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */
