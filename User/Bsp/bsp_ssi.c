/**
  ******************************************************************************
  * @file    bsp_ssi.c
  * @brief   MT6701 SSI frame reader, DMA based (see bsp_ssi.h).
  *
  * Layer: Bsp.
  *
  * Why DMA: the blocking version (HAL_SPI_Receive, later a hand-written polling
  * loop) cost 6.2us per frame, of which only 2.26us was pure shifting and about
  * 1.3us was the CPU spinning on RXNE. In S6 the angle is needed BEFORE the
  * Park transform, so those microseconds land inside the control chain. Moving
  * the RX side to DMA leaves just the three dummy writes (~0.3us) on the CPU.
  *
  * SPI1_RX is on DMA1_Channel5 in this project (CubeMX: Dma.SPI1_RX.5, DMAMUX
  * request DMA_REQUEST_SPI1_RX). dsh_foc had the same transfer on channel 3, so
  * every channel-dependent constant below is a macro and bsp_ssi_init() checks
  * it against the handle CubeMX built.
  ******************************************************************************
  */

#include "bsp_ssi.h"
#include "main.h"
#include "stm32g4xx_hal.h"

extern SPI_HandleTypeDef hspi1;

/* SPI1_RX = DMA1_Channel5. IFCR/ISR flag names are channel specific, so the
 * number appears in four places; keep them together. */
#define SSI_DMA            DMA1_Channel5
#define SSI_IFCR_CGIF      DMA_IFCR_CGIF5
#define SSI_ISR_TCIF       DMA_ISR_TCIF5
/* TXE comes back within one bit time (94 ns ~ 16 cycles); a generous 128
 * iterations is still under 1 us. The old 20000 could have blocked the
 * 20 kHz ISR for over a millisecond if the SPI ever stalled. */
#define SSI_SPIN_LIMIT    128u
#define SSI_BUSY_LIMIT    4u      /* frames in flight before we force a reset */
/* bsp_ssi_resync() parks the bus this many NOP iterations (~4 cycles each at
 * 170 MHz, so 11000 is about 260us) before it restarts the shift engine. */
#define SSI_RESYNC_QUIET_LOOPS 11000u

static volatile uint8_t s_rx[3];
static volatile uint8_t s_busy;
static volatile uint32_t s_ovr;   /* SPI RX overruns (a byte was lost) */
static volatile uint32_t s_resyncs; /* bsp_ssi_resync() calls */
static uint8_t  s_stuck;  /* consecutive ISR periods still busy */
static uint8_t  s_cfg_err; /* DMA channel in code != the one CubeMX configured */


/* Drop whatever is left in the SPI receive path.
 *
 * A frame is exactly three bytes and the DMA takes exactly three, so any byte
 * sitting in RXFIFO/DR when a frame starts means an earlier transfer was
 * aborted while it was still shifting. Those stray bytes are picked up by the
 * next frame's DMA and every frame after it is then read two bytes late: the
 * three bytes are real but rotated (measured 2026-11-05: 0x64 0x0D 0x7C is
 * 0x7C 0x64 0x0D rotated by two bytes, and the rotate-by-16 angle matched the
 * last good one). The CRC then rejects ~90 % of frames, and because nothing
 * realigned the reader it stayed that way until a power cycle re-initialised
 * the SPI. Flushing here makes every frame start at a byte boundary. */
static void ssi_flush_rx(void)
{
    uint32_t i;

    for (i = 0u; i < 8u; i++) {
        if ((SPI1->SR & (SPI_SR_RXNE | SPI_SR_OVR)) == 0u) {
            break;
        }
        (void)SPI1->DR;                 /* reading DR clears RXNE */
        (void)SPI1->SR;                 /* reading DR then SR clears OVR */
    }
}

void bsp_ssi_init(void)
{
    /* CSN idles high. SPI1 and hdma_spi1_rx are both initialised by CubeMX:
     * MX_DMA_Init() calls HAL_DMA_Init(&hdma_spi1_rx) with the same settings
     * this driver needs (SPI1_RX, byte, memory increment, normal mode), so
     * only the pieces HAL_DMA_Start() would normally set are done here. */
    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);

    /* Cross-check that the channel compiled in matches what CubeMX wired up.
     * A .ioc change can move SPI1_RX to another channel; without this check the
     * driver would silently poll the wrong TC flag and the encoder would read
     * stale bytes with every CRC still passing. */
    if ((hspi1.hdmarx == 0) || (hspi1.hdmarx->Instance != SSI_DMA)) {
        s_cfg_err = 1u;
        return;
    }
    s_cfg_err = 0u;

    /* CubeMX also enables DMA1_Channel5 in the NVIC, which is its default when a
     * DMA request is added. That is harmless here: this driver polls the TC flag
     * and never sets TCIE, so no interrupt request can ever be generated. */
    SSI_DMA->CPAR  = (uint32_t)&SPI1->DR;
    SSI_DMA->CMAR  = (uint32_t)s_rx;
    /* Channel priority (VERY_HIGH) and normal mode come from CubeMX. */

    s_busy = 0u;

    /* The HAL enables SPE per transfer; this driver pokes the registers, so
     * enable it once here. */
    SPI1->CR1 |= SPI_CR1_SPE;
}

void bsp_ssi_start(void)
{
    uint32_t spin;
    uint8_t  i;

    if (s_cfg_err != 0u) {
        return;                 /* wrong DMA channel: never touch the bus */
    }

    if (s_busy != 0u) {
        /* A previous transfer is still in flight. Normally the main loop
         * collects it long before this, so seeing several ISR periods in a row
         * means the DMA or the frame is wedged: drop it and start over. */
        if (++s_stuck >= SSI_BUSY_LIMIT) {
            bsp_ssi_abort();
        }
        return;
    }
    s_stuck = 0u;

    SSI_DMA->CCR   &= ~DMA_CCR_EN;
    ssi_flush_rx();                 /* start every frame on a byte boundary */
    SSI_DMA->CNDTR  = 3u;
    DMA1->IFCR      = SSI_IFCR_CGIF;
    SSI_DMA->CCR   |= DMA_CCR_EN;
    SPI1->CR2      |= SPI_CR2_RXDMAEN;

    /* CSN low (BSRR reset) */
    CSN_GPIO_Port->BSRR = (uint32_t)CSN_Pin << 16;

    /* Three dummy bytes generate the 24 clocks: this is the only CPU time the
     * transfer costs, the DMA collects the incoming bytes meanwhile. */
    for (i = 0u; i < 3u; i++) {
        spin = 0u;
        while (((SPI1->SR & SPI_SR_TXE) == 0u) && (spin < SSI_SPIN_LIMIT)) {
            spin++;
        }
        *(volatile uint8_t *)&SPI1->DR = 0xFFu;
    }

    s_busy = 1u;
}

uint8_t bsp_ssi_poll(uint8_t frame[3])
{
    if (s_busy == 0u) {
        return 0u;
    }
    if ((DMA1->ISR & SSI_ISR_TCIF) == 0u) {
        return 0u;                    /* still shifting */
    }

    SPI1->CR2   &= ~SPI_CR2_RXDMAEN;
    SSI_DMA->CCR &= ~DMA_CCR_EN;

    /* CSN high (BSRR set) */
    CSN_GPIO_Port->BSRR = CSN_Pin;

    s_busy = 0u;

    if (frame != 0) {
        frame[0] = s_rx[0];
        frame[1] = s_rx[1];
        frame[2] = s_rx[2];
    }

    /* OVR means a received byte was overwritten before the DMA took it: the
     * SPI RX buffer is one byte deep and the DMA can lose the arbitration to
     * the UART DMA or the CPU. The frame is then corrupted even if the CRC
     * happens to pass, so report it separately. Clear by reading DR then SR. */
    if ((SPI1->SR & SPI_SR_OVR) != 0u) {
        (void)SPI1->DR;
        (void)SPI1->SR;
        s_ovr++;
        return 2u;
    }

    return 1u;
}

uint32_t bsp_ssi_ovr_count(void)
{
    return s_ovr;
}

uint8_t bsp_ssi_busy(void)
{
    return s_busy;
}

uint8_t bsp_ssi_cfg_error(void)
{
    return s_cfg_err;
}

void bsp_ssi_abort(void)
{
    SPI1->CR2   &= ~SPI_CR2_RXDMAEN;
    SSI_DMA->CCR &= ~DMA_CCR_EN;
    CSN_GPIO_Port->BSRR = CSN_Pin;          /* CSN high */
    ssi_flush_rx();                         /* leave no half-frame behind */
    s_busy  = 0u;
    s_stuck = 0u;
}

void bsp_ssi_resync(void)
{
    uint32_t i;

    if (s_cfg_err != 0u) {
        return;
    }

    /* Release the slave first: the MT6701 only realigns its frame counter when
     * it sees the clock line idle, not merely a gap between two bursts. */
    CSN_GPIO_Port->BSRR = CSN_Pin;
    SPI1->CR2   &= ~SPI_CR2_RXDMAEN;
    SSI_DMA->CCR &= ~DMA_CCR_EN;

    for (i = 0u; i < SSI_RESYNC_QUIET_LOOPS; i++) {
        __NOP();
    }

    /* A half-shifted frame also leaves the SPI shift register mid-byte. Toggling
     * SPE throws that state away; the second flush clears the RXFIFO/OVR that
     * disabling the peripheral can leave behind. */
    SPI1->CR1 &= ~SPI_CR1_SPE;
    ssi_flush_rx();
    SPI1->CR1 |= SPI_CR1_SPE;
    ssi_flush_rx();

    s_busy   = 0u;
    s_stuck  = 0u;
    s_resyncs++;
}

uint32_t bsp_ssi_resync_count(void)
{
    return s_resyncs;
}

uint8_t bsp_ssi_read(uint8_t frame[3])
{
    uint32_t spin = 0u;

    bsp_ssi_abort();                        /* drop anything left over */
    bsp_ssi_start();
    while ((bsp_ssi_poll(frame) == 0u) && (spin < 500000u)) {
        spin++;
    }
    if (spin >= 500000u) {
        bsp_ssi_abort();
        return 1u;
    }
    return 0u;
}

void bsp_ssi_diag(uint32_t *cr1, uint32_t *ccipr2)
{
    if (cr1 != 0)    { *cr1    = SPI1->CR1; }
    if (ccipr2 != 0) { *ccipr2  = RCC->CCIPR2; }
}
