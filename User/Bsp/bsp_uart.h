/**
  ******************************************************************************
  * @file    bsp_uart.h
  * @brief   Debug UART (USART2, PA2=TX/PA3=RX -> on-board CH340N -> USB-C).
  *
  *   RX: DMA circular + IDLE detect -> ring buffer, zero CPU load
  *   TX: application writes into a 4KB ring buffer, the main loop pumps it
  *       out in 64-byte DMA chunks. Fully non-blocking.
  ******************************************************************************
  */

#ifndef __BSP_UART_H
#define __BSP_UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void     bsp_uart_init(void);

/* Bitmask of CubeMX-vs-driver mismatches found by bsp_uart_init().
 * Non-zero means logs may be garbage or RX may lose buffers; the banner shows
 * the raw value so a bad .ioc regeneration is visible at power-on. */
#define BSP_UART_CFG_BAUD        0x01u   /* Init.BaudRate != DBG_UART_BAUD      */
#define BSP_UART_CFG_NO_DMA      0x02u   /* no RX DMA handle linked             */
#define BSP_UART_CFG_DMA_MODE    0x04u   /* RX DMA is not CIRCULAR              */
uint8_t  bsp_uart_cfg_error(void);

/* Queue data for transmission, returns bytes accepted (non-blocking) */
uint16_t bsp_uart_tx_write(const uint8_t *src, uint16_t len);
/* Call repeatedly from the main loop to keep the DMA moving */
void     bsp_uart_tx_pump(void);
uint16_t bsp_uart_tx_pending(void);
/* Free bytes left in the TX ring (use before writing a whole frame) */
uint16_t bsp_uart_tx_space(void);
uint8_t  bsp_uart_tx_idle(void);
/* How many times the TX watchdog had to force-recover a lost completion
 * event. Non-zero means the link glitched at least once; the driver already
 * recovered, nothing to do, but it is worth watching in "stat". */
uint32_t bsp_uart_tx_stuck_count(void);

/* Read received bytes from the ring buffer */
uint16_t bsp_uart_rx_read(uint8_t *dst, uint16_t max);
uint16_t bsp_uart_rx_avail(void);
uint32_t bsp_uart_rx_drop_count(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_UART_H */
