/**
  ******************************************************************************
  * @file    bsp_ssi.h
  * @brief   MT6701 SSI (SPI mode) frame reader, DMA based.
  *
  * Layer: Bsp.
  *
  * SPI1 is configured by CubeMX as master, 8-bit, CPOL=0 / CPHA=1 (mode 1),
  * MSB first, 10.625 MHz, NSS software. CSN is a plain GPIO (PA15).
  * One frame is exactly 24 clocks: D[13:0], Mg[3:0], CRC[5:0].
  *
  * The RX side runs on DMA1 channel 3 (DMAMUX request 10). The CPU only writes
  * the three dummy bytes that generate the clocks (~0.3us); the 2.26us of
  * shifting happens in the background, so the control loop can do useful work
  * while the frame is clocked in.
  ******************************************************************************
  */

#ifndef __BSP_SSI_H
#define __BSP_SSI_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Four bytes, not three: the MT6701 is a plain shift register with no frame
 * sync, so after any aborted transfer the reader can sit one byte off and
 * every frame fails CRC (18#L-16, docs/00 #052). Reading 32 bits lets the
 * decoder search all nine 24-bit windows and lock on to the real frame. */
#define BSP_SSI_FRAME_LEN   4u

void bsp_ssi_init(void);

/* Arm one 32-bit transfer: CSN goes low, the RX DMA is armed and four dummy
 * bytes are queued. Returns immediately after the last byte is accepted
 * (~2.3us of TXE waiting, 3.0us of shifting in the background). A second call
 * while a transfer is still in flight is ignored. */
void bsp_ssi_start(void);

/* Non-blocking: returns 1 and copies the frame once the transfer finished,
 * 0 while it is still in flight. */
/* 0 = not ready, 1 = frame ready, 2 = ready but the SPI lost a byte (OVR) */
uint8_t bsp_ssi_poll(uint8_t frame[BSP_SSI_FRAME_LEN]);

/* Cumulative SPI RX overruns (one byte overwritten before the DMA read it). */
uint32_t bsp_ssi_ovr_count(void);

/* 1 while a transfer is armed but not yet collected. */
uint8_t bsp_ssi_busy(void);

/* 1 if the DMA channel compiled into this driver does not match the one CubeMX
 * configured for SPI1_RX (bsp_ssi_init() then refuses to touch the bus). */
uint8_t bsp_ssi_cfg_error(void);

/* Drop any transfer in flight and release CSN (recovery path). */
void bsp_ssi_abort(void);

/* Heavy recovery: release CSN, hold the clock quiet long enough for the MT6701
 * to lose sync on the frame, then restart the shift engine. Needed when a
 * frame slipped by a byte: abort() alone lets the next frame start in the
 * middle of the previous one (18#L-16, docs/00 #050). Blocking, ~260us. */
void bsp_ssi_resync(void);

/* Park the bus right now, even if a frame is in flight. Last resort for a
 * transfer that never completes: waiting cannot fix that, and the bit-level
 * slip it causes is recoverable once the slave sees a real clock silence. */
void bsp_ssi_force_resync(void);

/* 1 while the bus is inside its forced silence after a resync. Callers that
 * watch for "no new frame" must not treat that window as a stall. */
uint8_t bsp_ssi_quiet(void);

/* Park the bus right now, even if a frame is in flight. Last resort for a
 * transfer that never completes: waiting cannot fix that, and the bit-level
 * slip it causes is recoverable once the slave sees a real clock silence. */
void bsp_ssi_force_resync(void);

/* 1 while the bus is inside its forced silence after a resync. Callers that
 * watch for "no new frame" must not treat that window as a stall. */
uint8_t bsp_ssi_quiet(void);

/* How many times bsp_ssi_resync() ran since power on. */
uint32_t bsp_ssi_resync_count(void);

/* Blocking convenience wrapper (start + spin until done).
 * Returns 0 on success, non-zero if the transfer failed. */
uint8_t bsp_ssi_read(uint8_t frame[3]);

/* Diagnostic: SPI1 CR1 and RCC_CCIPR2, used by the "enc bench" command. */
void bsp_ssi_diag(uint32_t *cr1, uint32_t *ccipr2);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_SSI_H */
