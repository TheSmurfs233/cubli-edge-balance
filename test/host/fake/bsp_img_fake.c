#include "bsp_img.h"

/* Host stand-in for User/Bsp/bsp_img.c: there is no flash region and no CRC
 * peripheral on a PC. A fixed value keeps the command output deterministic. */

uint32_t bsp_img_crc_range(uint32_t start, uint32_t size)
{
    (void)start;
    (void)size;
    return 0xC0DEC0DEu;
}

uint32_t bsp_img_crc(void)
{
    return 0xC0DEC0DEu;
}
