/**
  ******************************************************************************
  * @file    bsp_imu.h
  * @brief   IMU module UART driver (USART1: PB6=TX / PB7=RX, 921600 8N1, RX via DMA)
  *
  * Layering:
  *   Mid/mid_fdilink.c - pure protocol parsing (no HAL, portable as-is);
  *   Bsp/bsp_imu.c     - this file, HAL glue (DMA moves, TX queue, config commands).
  * Porting to another STM32 project only touches this layer: init + 3 ISR forwarders.
  ******************************************************************************
  */

#ifndef BSP_IMU_H
#define BSP_IMU_H

#include "main.h"
#include "mid_fdilink.h"

/* Init: arm circular RX DMA, clear statistics. Call after MX_USART1_UART_Init() */
void     imu_init(void);

/* Main-loop polling: feed received bytes to the parser and advance the TX queue.
   Must be called periodically. */
void     imu_poll(void);

/* ===== Receive status ===== */
uint8_t  imu_stream_ok(void);           /* 1 = a valid frame arrived within IMU_FRAME_TIMEOUT_MS */
uint32_t imu_frame_age_ms(void);        /* ms since the last valid frame */
uint32_t imu_rx_bytes(void);            /* accumulated bytes received on USART1 */
uint32_t imu_rx_drop(void);             /* bytes dropped because the software queue was full */
uint32_t imu_rx_error(void);            /* HAL receive error count (includes a loose USB cable) */
uint32_t imu_tx_stuck(void);            /* TX watchdog recoveries */

/* ===== Data access ===== */
const fdilink_ctx_t *imu_ctx(void);
uint8_t  imu_get_imu(fdilink_imu_t *out);    /* 1 = at least one 0x40 frame received */
uint8_t  imu_get_ahrs(fdilink_ahrs_t *out);  /* 1 = at least one 0x41 frame received */
const uint8_t *imu_last_frame(uint8_t *len); /* raw bytes of the latest frame, header+tail included */

/* ===== Sending / configuration ===== */
uint8_t  imu_is_config_mode(void);      /* 1 = #fconfig sent, module is in ASCII config mode */
void     imu_exit_config_mode(void);    /* leave config mode: clear the flag, drop ASCII leftovers */
uint8_t  imu_send(const char *text);    /* non-blocking: appends CRLF, 1 = queued successfully */
uint32_t imu_ascii_avail(void);
uint32_t imu_ascii_read(uint8_t *dst, uint32_t max);

/* Blocking single config command exchange (main loop only). Returns bytes received. */
uint32_t imu_cmd(const char *text, uint8_t *reply, uint32_t reply_max, uint32_t timeout_ms);

/* Sniff buffer: copy the latest IMU_UART_SNIFF_SIZE raw bytes (at most max, oldest first) and
   return how many were copied. Used to check the frame layout / CRC offline; independent of
   the parse results. */
uint32_t imu_sniff(uint8_t *dst, uint32_t max);

/* ===== ISR entry points dispatched by bsp_uart.c callbacks by Instance (internal use) ===== */
void imu_uart_rx_event_isr(UART_HandleTypeDef *huart, uint16_t Size);
void imu_uart_tx_cplt_isr(UART_HandleTypeDef *huart);
void imu_uart_error_isr(UART_HandleTypeDef *huart);

#endif /* BSP_IMU_H */

/************************ (C) COPYRIGHT 2026 *********************END OF FILE****/
