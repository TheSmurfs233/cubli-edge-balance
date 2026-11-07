/**
  ******************************************************************************
  * @file    bsp_img.h
  * @brief   CRC32 over the application flash region, as a corruption probe.
  *
  * Layer: Bsp.
  *
  * Why: the bench keeps hitting faults whose instruction and data values do
  * not match what was flashed (00#067 .. #071). Two very different causes
  * produce that, and they need opposite fixes:
  *   a) the flash array content really changed, or
  *   b) the content is intact and the fetch/data path is being disturbed.
  * Recomputing a CRC over the flashed words separates the two.
  ******************************************************************************/

#include <stdint.h>

#ifndef __BSP_IMG_H
#define __BSP_IMG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Start and length of the region covered (application area, cal page excluded). */
#define BSP_IMG_START ((uint32_t)0x08000000u)
#define BSP_IMG_SIZE  ((uint32_t)(0x20000u - 0x800u))

/* CRC32 (the STM32 CRC unit, poly 0x04C11DB7, init 0xFFFFFFFF, no final xor)
 * over BSP_IMG_START .. BSP_IMG_START + BSP_IMG_SIZE. About 130 us at 170 MHz. */
uint32_t bsp_img_crc(void);

/* Same computation over an arbitrary range, for the tests and for diagnostics. */
uint32_t bsp_img_crc_range(uint32_t start, uint32_t size);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_IMG_H */
