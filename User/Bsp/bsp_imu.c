/**
  ******************************************************************************
  * @file    bsp_imu.c
  * @brief   IMU module UART driver (USART1 + DMA1_Channel1 circular RX)
  *
  * Data flow:
  *   module TX1 -> PB7 -> USART1 -> DMA1_Channel1 (CIRCULAR) -> s_rx_dma_buf
  *       -> HAL_UARTEx_RxEventCallback -> imu_uart_rx_event_isr()
  *       -> s_rx_stream (software queue) -> imu_poll() -> mid_fdilink parser
  *   config commands -> s_tx_ring -> imu_poll() -> s_tx_dma_buf (32 B)
  *       -> HAL_UART_Transmit_IT -> PB6 -> module RX1
  *
  * Interrupt context only moves bytes and bumps counters; all parsing happens in the
  * main loop (AGENTS.md 3.3: interrupt handlers stay short).
  ******************************************************************************
  */

#include "main.h"
#include "usart.h"
#include "board_cfg.h"
#include "mid_ringbuf.h"
#include "mid_fdilink.h"
#include "bsp_imu.h"

#include <string.h>

/* hdma_usart1_rx is defined in usart.c; CubeMX does not declare it in usart.h */
extern DMA_HandleTypeDef hdma_usart1_rx;

/* ===== Static storage ===== */
static uint8_t s_rx_dma_buf[IMU_RX_BUF_SIZE];
static uint8_t s_rx_stream_buf[IMU_RX_STREAM_RING];
static uint8_t s_ascii_buf[IMU_ASCII_RING_SIZE];
static uint8_t s_tx_ring_buf[IMU_TX_RING_SIZE];
static uint8_t s_tx_dma_buf[IMU_TX_DMA_CHUNK];

/* Sniff ring: imu_poll() records every byte before feeding the parser. Used only for
   offline frame-structure checks. The size must be a power of two (the index wraps by
   AND). Touched only from the main loop, so there are no concurrency issues. */
static uint8_t  s_sniff_buf[IMU_UART_SNIFF_SIZE];
static uint32_t s_sniff_pos;   /* bytes written so far (natural wrap; low bits are the index) */

static ringbuf_t s_rx_stream;
static ringbuf_t s_ascii_ring;
static ringbuf_t s_tx_ring;

static fdilink_ctx_t s_ctx;

/* ===== Runtime state (anything shared with an ISR is volatile) ===== */
static volatile uint32_t s_rx_last_pos;    /* DMA position already drained */
static volatile uint32_t s_rx_bytes;       /* total bytes received */
static volatile uint32_t s_rx_drop_cnt;    /* bytes dropped because the software queue was full */
static volatile uint32_t s_rx_err_cnt;     /* HAL receive error count */
static volatile uint32_t s_ascii_drop_cnt; /* bytes dropped from the ASCII reply queue */
static volatile uint8_t  s_tx_busy;        /* 1 = HAL_UART_Transmit_IT in flight */
static volatile uint32_t s_tx_stuck_cnt;   /* TX watchdog recoveries */
static uint32_t          s_tx_start_tick;
static uint8_t           s_rx_circular;
static uint8_t           s_ascii_mode;     /* 1 = module is in ASCII configuration mode */
static uint32_t          s_last_frame_tick;

/* ===== Internal helpers ===== */
static void     rx_restart(void);
static void     imu_tx_pump(void);
static uint8_t  dma_is_circular(const DMA_HandleTypeDef *hdma);
static void     stream_push(const uint8_t *src, uint32_t len);

static uint8_t dma_is_circular(const DMA_HandleTypeDef *hdma)
{
  if (hdma == 0)
  {
    return 0u;
  }
  return ((hdma->Instance->CCR & DMA_CCR_CIRC) != 0u) ? 1u : 0u;
}

static void stream_push(const uint8_t *src, uint32_t len)
{
  uint32_t n = ring_push(&s_rx_stream, src, len);

  if (n < len)
  {
    s_rx_drop_cnt += (len - n);
  }
}

/* Re-arm the "receive until idle" DMA transfer and reset the position pointer */
static void rx_restart(void)
{
  s_rx_last_pos = 0u;

  /* Same trap as the debug USART2: the line is idle right after reset, so the IDLE
     flag is already set. Leaving it set makes HAL fire one callback immediately with
     Size == RxXferSize, which is then mistaken for a completely full buffer. */
  __HAL_UART_CLEAR_IDLEFLAG(&huart1);

  (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart1, s_rx_dma_buf, (uint16_t)IMU_RX_BUF_SIZE);
}

/* ===== Initialisation ===== */

void imu_init(void)
{
  (void)ring_init(&s_rx_stream, s_rx_stream_buf, IMU_RX_STREAM_RING);
  (void)ring_init(&s_ascii_ring, s_ascii_buf, IMU_ASCII_RING_SIZE);
  (void)ring_init(&s_tx_ring, s_tx_ring_buf, IMU_TX_RING_SIZE);

  fdilink_init(&s_ctx);

  s_rx_last_pos   = 0u;
  s_rx_bytes      = 0u;
  s_rx_drop_cnt   = 0u;
  s_rx_err_cnt    = 0u;
  s_ascii_drop_cnt = 0u;
  s_tx_busy       = 0u;
  s_tx_stuck_cnt  = 0u;
  s_tx_start_tick = 0u;
  s_ascii_mode    = 0u;
  s_sniff_pos     = 0u;
  s_last_frame_tick = HAL_GetTick();

  /* RX DMA circular/normal mode is decided by CubeMX (.ioc); we only record it here */
  s_rx_circular = dma_is_circular(huart1.hdmarx);
  rx_restart();
}

/* ===== Main-loop polling ===== */

void imu_poll(void)
{
  uint8_t  buf[64];
  uint32_t n;
  uint32_t frames0;
  uint8_t  guard;

  imu_tx_pump();

  for (guard = 0u; guard < 32u; guard++)
  {
    n = ring_pop(&s_rx_stream, buf, (uint32_t)sizeof(buf));
    if (n == 0u)
    {
      break;
    }
    s_rx_bytes += n;

    if (s_ascii_mode == 0u)
    {
      /* Sniffing: record the protocol stream only (ASCII replies have no frame structure) */
      uint32_t k;

      for (k = 0u; k < n; k++)
      {
        s_sniff_buf[s_sniff_pos & (IMU_UART_SNIFF_SIZE - 1u)] = buf[k];
        s_sniff_pos++;
      }
    }

    if (s_ascii_mode != 0u)
    {
      /* Config mode: the module only sends ASCII replies; pass them to the app layer */
      uint32_t w = ring_push(&s_ascii_ring, buf, n);

      if (w < n)
      {
        s_ascii_drop_cnt += (n - w);
      }
    }
    else
    {
      frames0 = s_ctx.stat.frames;
      (void)fdilink_feed(&s_ctx, buf, n);
      if (s_ctx.stat.frames != frames0)
      {
        s_last_frame_tick = HAL_GetTick();
      }
    }
  }
}

/* ===== TX (interrupt driven + ring queue, non-blocking) ===== */

static void imu_tx_pump(void)
{
  uint16_t len;

  if (s_tx_busy != 0u)
  {
    if ((uint32_t)(HAL_GetTick() - s_tx_start_tick) >= IMU_TX_STUCK_MS)
    {
      (void)HAL_UART_AbortTransmit(&huart1);
      s_tx_busy = 0u;
      s_tx_stuck_cnt++;
    }
    else
    {
      return;
    }
  }

  /* mid_ringbuf (cubli baseline) copies into dst and returns the byte count. */
  len = ring_peek(&s_tx_ring, s_tx_dma_buf, (uint16_t)IMU_TX_DMA_CHUNK);
  if (len == 0u)
  {
    return;
  }

  s_tx_start_tick = HAL_GetTick();
  s_tx_busy = 1u;
  if (HAL_UART_Transmit_IT(&huart1, s_tx_dma_buf, len) != HAL_OK)
  {
    s_tx_busy = 0u;
    s_tx_stuck_cnt++;
    return;
  }

  /* Handed to the UART; safe to drop from the ring now */
  ring_drop(&s_tx_ring, len);
}

uint8_t imu_send(const char *text)
{
  uint32_t len;
  uint32_t w;
  uint8_t  enter;
  uint8_t  leave;

  if (text == 0)
  {
    return 0u;
  }
  len = (uint32_t)strlen(text);

  /* Mode switching is detected only by command prefix, and the flag MUST be flipped
     after the command has really been queued: flipping it first while the TX ring is
     full leaves "flag says config mode, command never sent" (cfg on but still
     streaming).
     - #fconfig   -> enter config mode (replies then go to the ASCII ring)
     - #fdeconfig -> leave config mode  (module resumes the data stream)
     #freboot / #freset are NOT handled here: they still need y/n and a *#OK reply,
     and clearing the flag early would let the FDILink parser eat those replies. */
  enter = (strncmp(text, "#fconfig", 8u) == 0) ? 1u : 0u;
  leave = (strncmp(text, "#fdeconfig", 10u) == 0) ? 1u : 0u;

  if (ring_space(&s_tx_ring) < (len + 2u))
  {
    return 0u;
  }
  w  = ring_push(&s_tx_ring, (const uint8_t *)text, len);
  w += ring_push(&s_tx_ring, (const uint8_t *)"\r\n", 2u);
  if (w != (len + 2u))
  {
    return 0u;
  }

  if (enter != 0u)
  {
    s_ascii_mode = 1u;
  }
  else if (leave != 0u)
  {
    s_ascii_mode = 0u;
    ring_reset(&s_ascii_ring);
  }

  return 1u;
}

/* ===== Status queries ===== */

uint8_t imu_stream_ok(void)
{
  return (imu_frame_age_ms() < IMU_FRAME_TIMEOUT_MS) ? 1u : 0u;
}

uint32_t imu_frame_age_ms(void)
{
  return (uint32_t)(HAL_GetTick() - s_last_frame_tick);
}

uint32_t imu_rx_bytes(void)
{
  return s_rx_bytes;
}

uint32_t imu_rx_drop(void)
{
  return s_rx_drop_cnt;
}

uint32_t imu_rx_error(void)
{
  return s_rx_err_cnt;
}

uint32_t imu_tx_stuck(void)
{
  return s_tx_stuck_cnt;
}

const fdilink_ctx_t *imu_ctx(void)
{
  return &s_ctx;
}

uint8_t imu_get_imu(fdilink_imu_t *out)
{
  if ((out == 0) || (s_ctx.imu_valid == 0u))
  {
    return 0u;
  }
  (void)memcpy(out, &s_ctx.imu, sizeof(*out));
  return 1u;
}

uint8_t imu_get_ahrs(fdilink_ahrs_t *out)
{
  if ((out == 0) || (s_ctx.ahrs_valid == 0u))
  {
    return 0u;
  }
  (void)memcpy(out, &s_ctx.ahrs, sizeof(*out));
  return 1u;
}

const uint8_t *imu_last_frame(uint8_t *len)
{
  if (len != 0)
  {
    *len = s_ctx.last_frame_len;
  }
  return s_ctx.last_frame;
}

uint8_t imu_is_config_mode(void)
{
  return s_ascii_mode;
}

void imu_exit_config_mode(void)
{
  s_ascii_mode = 0u;
  ring_reset(&s_ascii_ring);
}

uint32_t imu_ascii_avail(void)
{
  return ring_avail(&s_ascii_ring);
}

uint32_t imu_ascii_read(uint8_t *dst, uint32_t max)
{
  return ring_pop(&s_ascii_ring, dst, max);
}

/* A reply containing "#OK" counts as success (the module answers #OK or *#OK) */
static uint8_t reply_has_ok(const uint8_t *p, uint32_t len)
{
  uint32_t i;

  if ((p == 0) || (len < 3u))
  {
    return 0u;
  }
  for (i = 0u; (i + 3u) <= len; i++)
  {
    if ((p[i] == '#') && (p[i + 1u] == 'O') && (p[i + 2u] == 'K'))
    {
      return 1u;
    }
  }
  return 0u;
}

uint32_t imu_cmd(const char *text, uint8_t *reply, uint32_t reply_max, uint32_t timeout_ms)
{
  uint32_t got = 0u;
  uint32_t t0;
  uint32_t cap;

  /* reply must hold at least 1 byte + '\0', else reply_max - 1u underflows to 0xFFFFFFFF */
  if ((reply == 0) || (reply_max < 2u))
  {
    reply = 0;
    cap   = 0u;
  }
  else
  {
    cap = reply_max - 1u;
  }

  ring_reset(&s_ascii_ring);

  if (imu_send(text) == 0u)
  {
    return 0u;
  }

  t0 = HAL_GetTick();
  while ((uint32_t)(HAL_GetTick() - t0) < timeout_ms)
  {
    imu_poll();

    if ((reply != 0) && (got < cap))
    {
      got += ring_pop(&s_ascii_ring, reply + got, cap - got);
    }

    if (reply_has_ok(reply, got) != 0u)
    {
      break;
    }
    HAL_Delay(1u);
  }

  if (reply != 0)
  {
    if (got < cap)
    {
      got += ring_pop(&s_ascii_ring, reply + got, cap - got);
    }
    reply[got] = '\0';
  }
  return got;
}

/* ===== Debug sniffing ===== */

uint32_t imu_sniff(uint8_t *dst, uint32_t max)
{
  uint32_t avail;
  uint32_t n;
  uint32_t i;
  uint32_t start;

  if (dst == 0)
  {
    return 0u;
  }

  avail = (s_sniff_pos < (uint32_t)IMU_UART_SNIFF_SIZE) ? s_sniff_pos
                                                        : (uint32_t)IMU_UART_SNIFF_SIZE;
  n = (avail < max) ? avail : max;

  /* Copy the newest n bytes from oldest to newest (ring index = low bits of the counter) */
  start = (s_sniff_pos - n) & (IMU_UART_SNIFF_SIZE - 1u);
  for (i = 0u; i < n; i++)
  {
    dst[i] = s_sniff_buf[(start + i) & (IMU_UART_SNIFF_SIZE - 1u)];
  }
  return n;
}

/* ===== ISR entry points dispatched from the HAL callbacks ===== */

void imu_uart_rx_event_isr(UART_HandleTypeDef *huart, uint16_t Size)
{
  uint32_t pos;

  /* Case 1: the transfer really completed (TC) - drain the tail and reset the position */
  if (huart->RxEventType == HAL_UART_RXEVENT_TC)
  {
    pos = IMU_RX_BUF_SIZE;
    if (s_rx_last_pos < pos)
    {
      stream_push(&s_rx_dma_buf[s_rx_last_pos], pos - s_rx_last_pos);
    }
    s_rx_last_pos = 0u;

    if (s_rx_circular == 0u)
    {
      rx_restart();
    }
    return;
  }

  /* Case 2: Size == RxXferSize means "IDLE happened exactly as DMA wrapped to 0" */
  if ((uint32_t)Size >= IMU_RX_BUF_SIZE)
  {
    if (s_rx_last_pos > 0u)
    {
      stream_push(&s_rx_dma_buf[s_rx_last_pos], IMU_RX_BUF_SIZE - s_rx_last_pos);
    }
    s_rx_last_pos = 0u;
    return;
  }

  /* Case 3: position moved forward - take the delta only */
  if ((uint32_t)Size > s_rx_last_pos)
  {
    stream_push(&s_rx_dma_buf[s_rx_last_pos], (uint32_t)Size - s_rx_last_pos);
  }
  /* Case 4: position went backwards - wrapped without a TC; take two segments */
  else if ((uint32_t)Size < s_rx_last_pos)
  {
    stream_push(&s_rx_dma_buf[s_rx_last_pos], IMU_RX_BUF_SIZE - s_rx_last_pos);
    if (Size > 0u)
    {
      stream_push(&s_rx_dma_buf[0], Size);
    }
  }
  else
  {
    /* Position unchanged: no new data */
  }

  s_rx_last_pos = (uint32_t)Size;
}

void imu_uart_tx_cplt_isr(UART_HandleTypeDef *huart)
{
  (void)huart;
  s_tx_busy = 0u;   /* only clear the flag; the main loop pumps the next chunk */
}

void imu_uart_error_isr(UART_HandleTypeDef *huart)
{
  s_rx_err_cnt++;

  __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);
  huart->ErrorCode = HAL_UART_ERROR_NONE;

  rx_restart();
}

/************************ (C) COPYRIGHT 2026 *********************END OF FILE****/
