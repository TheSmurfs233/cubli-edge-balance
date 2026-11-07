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
#include "board_cfg.h"
#include "bsp_prof.h"

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
#define SSI_BUSY_LIMIT    20u     /* starts skipped before the frame counts as wedged */
/* bsp_ssi_resync() holds the clock line idle this long before it restarts the
 * shift engine. Measured on the target: a NOP loop of the same nominal length
 * was optimised into nothing by the compiler, so the pause never happened and
 * the reader stayed misaligned (resync ran 2586 times in 30 s without one good
 * frame afterwards). Count DWT cycles instead. */
/* 2500us, not 300us: measured on the target 2026-11-07, the MT6701 keeps
 * counting clocks across a short CSN-high gap, so a 300us pause restarted
 * the shift engine inside the still-open frame and the reader stayed at
 * bit 8 forever (four-byte frame with the idle 0xFF in the middle). Only a
 * millisecond-scale silence drives the internal bit counter back to 0. */
#define SSI_RESYNC_QUIET_US   2500u

static volatile uint8_t s_rx[BSP_SSI_FRAME_LEN];
static volatile uint8_t s_busy;
static volatile uint32_t s_ovr;   /* SPI RX overruns (a byte was lost) */
static volatile uint32_t s_resyncs; /* bus parks (force resync) */
static volatile uint32_t s_resync_busy; /* parks forced by a wedged frame */
static volatile uint32_t s_quiet_until; /* DWT tick before which the clock must stay idle */
static volatile uint8_t  s_quiet_pend;  /* 1 = SPE is off, waiting for the silence to expire */
static volatile uint8_t  s_resync_pend; /* 1 = park the bus as soon as the frame lands */

static void ssi_park(void);   /* forward: called from poll() and resync() */
static uint8_t  s_stuck;  /* consecutive ISR periods still busy */
static uint8_t  s_cfg_err; /* DMA channel in code != the one CubeMX configured */


/* Drop whatever is left in the SPI receive path.
 *
 * A frame is exactly four bytes and the DMA takes exactly four, so any byte
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

    /* 5.31 MHz instead of the 10.625 MHz the .ioc prescaler would give. The
     * MT6701 allows 10 MHz and the board had been running at 10.625 MHz, but
     * the SSI ribbon runs next to the phase cables: once the wheel turned the
     * reader corrupted 30 % of the frames (docs/00 #059). Halving the clock
     * doubles the setup/hold margin; a 4-byte frame then shifts in 6.0 us,
     * still far inside the 50 us control period, and the value is re-applied
     * here so a CubeMX regeneration cannot silently undo it. */
    SPI1->CR1 = (SPI1->CR1 & ~(uint32_t)SPI_CR1_BR) |
                (uint32_t)SPI_CR1_BR_1 | (uint32_t)SPI_CR1_BR_0;

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

    if (s_quiet_pend != 0u) {
        if ((int32_t)(s_quiet_until - bsp_prof_now()) > 0) {
            return;             /* still inside the forced silence (see resync) */
        }
        s_quiet_pend = 0u;
        SPI1->CR1 |= SPI_CR1_SPE;
        ssi_flush_rx();
    }

    if (s_busy != 0u) {
        /* A previous transfer is still in flight. Normally the main loop
         * collects it long before this, so seeing several ISR periods in a row
         * means the DMA or the frame is wedged: drop it and start over. */
        if (++s_stuck >= SSI_BUSY_LIMIT) {
            /* Do not abort here: raising CSN mid-frame leaves the MT6701
             * counting inside the frame, and that bit-level slip is exactly
             * what the rotation decoder cannot undo (docs/00 #055, #057).
             * Park the bus and give it a clean silence instead. This is the
             * last resort, so it must not defer: a transfer that never sets
             * TCIF would defer forever. */
            bsp_ssi_force_resync();
        }
        return;
    }
    s_stuck = 0u;

    SSI_DMA->CCR   &= ~DMA_CCR_EN;
    ssi_flush_rx();                 /* start every frame on a byte boundary */
    SSI_DMA->CNDTR  = BSP_SSI_FRAME_LEN;
    DMA1->IFCR      = SSI_IFCR_CGIF;
    SSI_DMA->CCR   |= DMA_CCR_EN;
    SPI1->CR2      |= SPI_CR2_RXDMAEN;

    /* CSN low (BSRR reset) */
    CSN_GPIO_Port->BSRR = (uint32_t)CSN_Pin << 16;

    /* Four dummy bytes generate the 32 clocks: 0.75us more shifting than a
     * 24-bit read, and it buys the one-byte tolerance the decoder needs. The
     * DMA collects the incoming bytes meanwhile. */
    for (i = 0u; i < BSP_SSI_FRAME_LEN; i++) {
        spin = 0u;
        while (((SPI1->SR & SPI_SR_TXE) == 0u) && (spin < SSI_SPIN_LIMIT)) {
            spin++;
        }
        *(volatile uint8_t *)&SPI1->DR = 0xFFu;
    }

    s_busy = 1u;
}

uint8_t bsp_ssi_poll(uint8_t frame[BSP_SSI_FRAME_LEN])
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

    if (s_resync_pend != 0u) {
        ssi_park();                 /* frame is safely in: now the bus may rest */
    }

    if (frame != 0) {
        frame[0] = s_rx[0];
        frame[1] = s_rx[1];
        frame[2] = s_rx[2];
        frame[3] = s_rx[3];
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

/* Busy-wait for a fixed time. The guard keeps a stalled DWT (CYCCNT not
 * running) from hanging the control loop. */
void bsp_ssi_resync(void)
{
    if (s_cfg_err != 0u) {
        return;
    }

    /* Cutting a frame in flight is what creates bit-level phase slips, and a
     * bit-level slip cannot be undone by the rotation decoder (only whole-byte
     * ones can). So when the line is busy, remember the request and park the
     * bus right after the frame lands instead of raising CSN now.
     * Measured 2026-11-07: resync called on every 5 bad frames turned into a
     * loop (283 resyncs in 12 s) because each one cut another frame. */
    if (s_busy != 0u) {
        s_resync_pend = 1u;
        return;
    }

    ssi_park();
}

static void ssi_park(void)
{
    /* Release the slave: the MT6701 only realigns its frame counter when it
     * sees the clock line idle, not merely a gap between two bursts.
     *
     * The silence is NOT spun out here. This function is reached from the
     * 20 kHz control interrupt (via mid_enc_update), and blocking it for
     * 2.5 ms starves the current loop (measured: enc slot 109 us average,
     * 2514 us worst case). Instead the bus is parked and bsp_ssi_start()
     * refuses to arm frames until the deadline has passed, so the interrupt
     * only sees one extra compare per call. */
    CSN_GPIO_Port->BSRR = CSN_Pin;
    SPI1->CR2   &= ~SPI_CR2_RXDMAEN;
    SSI_DMA->CCR &= ~DMA_CCR_EN;

    s_quiet_until = bsp_prof_now() + (SSI_RESYNC_QUIET_US * PHASE_CYC_PER_US);
    s_quiet_pend  = 1u;

    /* A half-shifted frame also leaves the SPI shift register mid-byte. Toggling
     * SPE throws that state away; bsp_ssi_start() flushes the RXFIFO/OVR again
     * when it turns the peripheral back on. */
    SPI1->CR1 &= ~SPI_CR1_SPE;
    ssi_flush_rx();

    s_busy        = 0u;
    s_stuck       = 0u;
    s_resync_pend = 0u;
    s_resyncs++;
}

uint32_t bsp_ssi_resync_count(void)
{
    return s_resyncs;
}

void bsp_ssi_force_resync(void)
{
    if (s_cfg_err != 0u) {
        return;
    }
    s_resync_busy++;
    ssi_park();                 /* parks, silences and counts the resync */
}

uint32_t bsp_ssi_resync_busy_count(void)
{
    return s_resync_busy;
}

uint8_t bsp_ssi_quiet(void)
{
    return s_quiet_pend;
}

uint8_t bsp_ssi_read(uint8_t frame[BSP_SSI_FRAME_LEN])
{
    uint32_t spin = 0u;

    /* Never abort a frame that is still shifting: CSN rising mid-frame
     * leaves the MT6701 counting inside the frame, and command-time reads are
     * exactly what produced the 8/16/24-bit phase slips seen on hardware. */
    while ((bsp_ssi_busy() != 0u) && (spin < 200000u)) {
        spin++;
    }
    spin = 0u;

    bsp_ssi_abort();                        /* drop anything left over */
    bsp_ssi_start();
    while ((bsp_ssi_busy() == 0u) && (spin < 4000000u)) {
        spin++;                             /* waits out a resync silence */
        bsp_ssi_start();
    }
    spin = 0u;
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
