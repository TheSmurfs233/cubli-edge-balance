/**
  ******************************************************************************
  * @file    bsp_img.c
  * @brief   CRC32 over the application flash region (see bsp_img.h).
  *
  * Layer: Bsp.
  *
  * Uses the STM32 CRC peripheral: 32-bit data words, polynomial 0x04C11DB7,
  * init 0xFFFFFFFF, no input/output reflection, no final xor. That is not the
  * zlib CRC32, and it does not need to be - the only requirement is that the
  * same bytes always give the same number.
  *
  * Cost: the CRC unit eats one word per cycle, so 126976 bytes = 31744 words
  * is about 190 us including the loop overhead. Called only from the command
  * handler, never from the control loop.
  ******************************************************************************/

#include "bsp_img.h"
#include "stm32g4xx_hal.h"

uint32_t bsp_img_crc_range(uint32_t start, uint32_t size)
{
    const volatile uint32_t *w = (const volatile uint32_t *)start;
    uint32_t n = size / 4u;
    uint32_t crc;

    __HAL_RCC_CRC_CLK_ENABLE();
    CRC->CR = CRC_CR_RESET;

    for (uint32_t i = 0u; i < n; ++i)
    {
        CRC->DR = w[i];
    }

    crc = CRC->DR;
    return crc;
}

uint32_t bsp_img_crc(void)
{
    return bsp_img_crc_range(BSP_IMG_START, BSP_IMG_SIZE);
}
