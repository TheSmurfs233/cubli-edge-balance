/**
  ******************************************************************************
  * @file    bsp_uart.c
  * @brief   USART2 non-blocking driver (DMA + ring buffers).
  *
  * RX path:
  *   USART2(PA3) -> DMA1_CH3 (CIRCULAR) -> rx_dma_buf -> IDLE/half/full IRQ
  *               -> RxEventCallback -> rx_ring
  *
  * TX path:
  *   app -> tx_ring -> bsp_uart_tx_pump() (main loop)
  *       -> tx_dma_buf -> DMA1_CH4 (NORMAL) -> USART2(PA2)
  *
  *   s_tx_busy is cleared only by HAL_UART_TxCpltCallback. A lost completion
  *   event would silence the link forever, so the pump owns a watchdog
  *   (DBG_TX_STUCK_MS): once a chunk has clearly overrun its duration it
  *   aborts the transfer and releases the flag.
  *   bsp_uart_tx_stuck_count() reports how often that happened.
  *
  * DMA mode note:
  *   CubeMX generates DMA_NORMAL for RX. bsp_uart_init() switches it to
  *   CIRCULAR at runtime without touching the .ioc. If CubeMX regenerates the
  *   project, HAL_UART_MspInit writes NORMAL again, but app_init() calls
  *   bsp_uart_init() afterwards and restores CIRCULAR automatically.
  ******************************************************************************
  */

#include "bsp_uart.h"
#include "bsp_imu.h"
#include "board_cfg.h"
#include "usart.h"
#include "mid_ringbuf.h"
#include <string.h>

/* ------------------------------------------------------------------ */
/* Internal buffers                                                    */
/* ------------------------------------------------------------------ */
static uint8_t   s_rx_dma_buf[DBG_RX_BUF_SIZE];
static uint16_t  s_rx_last_pos;                 /* last DMA position reported */
static uint8_t   s_rx_ring_mem[DBG_RX_RING_SIZE];
static ringbuf_t s_rx_ring;
static uint32_t  s_rx_drop_cnt;

static uint8_t   s_tx_ring_mem[DBG_TX_RING_SIZE];
static ringbuf_t s_tx_ring;
static uint8_t   s_tx_dma_buf[DBG_TX_DMA_CHUNK];
static volatile uint8_t s_tx_busy;              /* 1 = DMA transmit in flight */
static uint32_t  s_tx_start_ms;                 /* when s_tx_busy was set */
static uint32_t  s_tx_stuck_cnt;                /* watchdog recoveries */
static uint32_t  s_tx_retry_cnt;                /* consecutive HAL_BUSY retries */
static uint32_t  s_tx_busy_cnt;                 /* HAL_BUSY recoveries (forced abort) */
static uint8_t   s_cfg_err;                     /* BSP_UART_CFG_* bitmask */

/* ------------------------------------------------------------------ */
/* Init                                                                */
/* ------------------------------------------------------------------ */
void bsp_uart_init(void)
{
    ring_init(&s_rx_ring, s_rx_ring_mem, (uint16_t)sizeof(s_rx_ring_mem));
    ring_init(&s_tx_ring, s_tx_ring_mem, (uint16_t)sizeof(s_tx_ring_mem));
    s_rx_last_pos = 0u;
    s_rx_drop_cnt = 0u;
    s_tx_busy     = 0u;
    s_tx_start_ms = 0u;
    s_tx_stuck_cnt = 0u;
    s_tx_retry_cnt = 0u;
    s_tx_busy_cnt  = 0u;

    /* Cross-check the two things this driver *assumes* about the CubeMX side.
     * Both have been wrong before and both fail silently:
     *   - a baud mismatch prints readable-looking garbage, not an error;
     *   - a NORMAL-mode RX DMA makes the position-wrap logic drop whole buffers
     *     each time the circular buffer would have wrapped.
     * Neither shows up in "0 errors / 0 warnings". */
    s_cfg_err = 0u;
    if (huart2.Init.BaudRate != (uint32_t)DBG_UART_BAUD) {
        s_cfg_err |= (uint8_t)BSP_UART_CFG_BAUD;
    }
    if ((huart2.hdmarx == 0) || (huart2.hdmarx->Instance == 0)) {
        s_cfg_err |= (uint8_t)BSP_UART_CFG_NO_DMA;
    } else if (huart2.hdmarx->Init.Mode != DMA_CIRCULAR) {
        s_cfg_err |= (uint8_t)BSP_UART_CFG_DMA_MODE;
    }

    /* RX DMA mode comes from CubeMX (.ioc: USART2_RX = CIRCULAR). The firmware
     * used to force-circular here; that override was removed on 2026-10-25 once
     * the .ioc was fixed, so this module no longer rewrites a CubeMX setting. */

    /* Start reception: callback fires on buffer-full or line-idle */
    (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart2, s_rx_dma_buf, DBG_RX_BUF_SIZE);
}

uint8_t bsp_uart_cfg_error(void)
{
    return s_cfg_err;
}

/* ------------------------------------------------------------------ */
/* Transmit                                                            */
/* ------------------------------------------------------------------ */
uint16_t bsp_uart_tx_write(const uint8_t *src, uint16_t len)
{
    return ring_push(&s_tx_ring, src, len);
}

void bsp_uart_tx_pump(void)
{
    uint16_t n;

    if (s_tx_busy) {
        /* The completion callback is the only thing that clears s_tx_busy.
         * If it never arrives (UART error during TX, DMA channel glitch) the
         * whole debug link would stay silent forever. Force recovery once the
         * transfer has clearly overrun its duration. */
        if ((uint32_t)(HAL_GetTick() - s_tx_start_ms) < (uint32_t)DBG_TX_STUCK_MS) {
            return;
        }
        (void)HAL_UART_AbortTransmit(&huart2);
        s_tx_busy = 0u;
        s_tx_stuck_cnt++;
    }

    n = ring_peek(&s_tx_ring, s_tx_dma_buf, DBG_TX_DMA_CHUNK);
    if (n == 0u) { return; }

    s_tx_busy     = 1u;
    s_tx_start_ms = HAL_GetTick();
    if (HAL_UART_Transmit_DMA(&huart2, s_tx_dma_buf, n) != HAL_OK) {
        /* Keep the data queued and retry next round.  If HAL keeps saying
         * busy the handle is wedged (gState never returns to READY, which is
         * how tx_pending froze at 412 B on 2026-11-06): force one abort so
         * the state machine is reset instead of staying mute forever. */
        s_tx_busy = 0u;
        s_tx_retry_cnt++;
        if (s_tx_retry_cnt >= DBG_TX_BUSY_ABORT) {
            (void)HAL_UART_AbortTransmit(&huart2);
            s_tx_retry_cnt = 0u;
            s_tx_busy_cnt++;
        }
        return;
    }
    s_tx_retry_cnt = 0u;
    /* The DMA holds its own copy of the data, safe to drop from the ring */
    ring_drop(&s_tx_ring, n);
}

uint16_t bsp_uart_tx_pending(void)
{
    return ring_avail(&s_tx_ring);
}

uint16_t bsp_uart_tx_space(void)
{
    return ring_space(&s_tx_ring);
}

uint8_t bsp_uart_tx_idle(void)
{
    return ((s_tx_busy == 0u) && (ring_avail(&s_tx_ring) == 0u)) ? 1u : 0u;
}

uint32_t bsp_uart_tx_stuck_count(void)
{
    return s_tx_stuck_cnt;
}

uint32_t bsp_uart_tx_busy_count(void)
{
    return s_tx_busy_cnt;
}

uint32_t bsp_uart_handle_state(void)
{
    return (uint32_t)huart2.gState;
}

uint32_t bsp_uart_handle_error(void)
{
    return huart2.ErrorCode;
}

/* ------------------------------------------------------------------ */
/* Receive                                                             */
/* ------------------------------------------------------------------ */
uint16_t bsp_uart_rx_read(uint8_t *dst, uint16_t max)
{
    return ring_pop(&s_rx_ring, dst, max);
}

uint16_t bsp_uart_rx_avail(void)
{
    return ring_avail(&s_rx_ring);
}

uint32_t bsp_uart_rx_drop_count(void)
{
    return s_rx_drop_cnt;
}

/* ------------------------------------------------------------------ */
/* HAL callbacks (must be defined exactly once in the whole project)   */
/* ------------------------------------------------------------------ */
/**
  * @brief  DMA receive event: idle / half-full / full all land here.
  * @param  Size  DMA write position (0 .. DBG_RX_BUF_SIZE)
  *
  * Uses huart->RxEventType to distinguish a buffer wrap from a plain position
  * update. This matters: when the circular buffer fills up, HAL reports
  * Size = RxXferSize (the full size) and the DMA position restarts at 0. If we
  * only compared positions, two consecutive "full" events would look identical
  * and a whole buffer of data would be silently dropped.
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    uint16_t last;
    uint16_t seg;

    /* USART1 belongs to the IMU module. HAL weak callbacks can only be defined
     * once per project, so this file dispatches by Instance; see bsp_imu.c. */
    if (huart->Instance == USART1) {
        imu_uart_rx_event_isr(huart, Size);
        return;
    }

    if (huart->Instance != USART2) { return; }
    if (Size > DBG_RX_BUF_SIZE)    { Size = DBG_RX_BUF_SIZE; }

    last = s_rx_last_pos;

    /* ---- Case 1: the circular buffer just wrapped (full lap) ---- */
    if ((huart->RxEventType == HAL_UART_RXEVENT_TC) || (Size >= DBG_RX_BUF_SIZE)) {
        if (last < DBG_RX_BUF_SIZE) {
            seg = (uint16_t)(DBG_RX_BUF_SIZE - last);
            if (ring_push(&s_rx_ring, &s_rx_dma_buf[last], seg) != seg) {
                s_rx_drop_cnt++;
            }
        }
        s_rx_last_pos = 0u;
        return;
    }

    /* ---- Case 2: idle / half-complete, Size is the current position ---- */
    if (Size == last) { return; }

    if (Size > last) {
        seg = (uint16_t)(Size - last);
        if (ring_push(&s_rx_ring, &s_rx_dma_buf[last], seg) != seg) {
            s_rx_drop_cnt++;
        }
    } else {
        /* Position went backwards without a full-transfer event: treat as wrap */
        seg = (uint16_t)(DBG_RX_BUF_SIZE - last);
        if ((seg > 0u) &&
            (ring_push(&s_rx_ring, &s_rx_dma_buf[last], seg) != seg)) {
            s_rx_drop_cnt++;
        }
        if ((Size > 0u) &&
            (ring_push(&s_rx_ring, &s_rx_dma_buf[0], Size) != Size)) {
            s_rx_drop_cnt++;
        }
    }
    s_rx_last_pos = Size;
}

/**
  * @brief  DMA transmit complete: just clear the flag.
  *
  * The next chunk is started by bsp_uart_tx_pump() from the main loop instead
  * of from this interrupt, to avoid racing HAL_DMA_Start_IT against the DMA
  * state machine. 64 bytes at 1Mbaud take ~640us to shift out, while the main
  * loop runs at tens of kHz, so there is no visible gap.
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        imu_uart_tx_cplt_isr(huart);
        return;
    }

    if (huart->Instance != USART2) { return; }
    s_tx_busy = 0u;
}

/**
  * @brief  UART error (overrun/noise/framing): clear flags and restart RX so
  *         reception can never get stuck permanently.
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        imu_uart_error_isr(huart);
        return;
    }

    if (huart->Instance != USART2) { return; }

    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);

    s_rx_last_pos = 0u;
    (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart2, s_rx_dma_buf, DBG_RX_BUF_SIZE);
}
