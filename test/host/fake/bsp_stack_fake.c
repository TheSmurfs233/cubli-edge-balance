/* Host stand-in for bsp_stack.c: the bench build has no linker-placed stack
 * region, so it reports a fixed, obviously fake depth. */
#include "bsp_stack.h"

static uint32_t s_painted;

void bsp_stack_paint(void)
{
    s_painted = 2048u;
}

uint32_t bsp_stack_peak_bytes(void)
{
    return (s_painted == 0u) ? 0u : 512u;
}

uint32_t bsp_stack_size_bytes(void)
{
    return 2048u;
}

static uint32_t fake_sp_bad;

uint32_t bsp_stack_sp(void)
{
    return 0x20003A00u;
}

uint8_t bsp_stack_sp_check(void)
{
    return (fake_sp_bad == 0u) ? 1u : 0u;
}

uint32_t bsp_stack_sp_bads(void)
{
    return fake_sp_bad;
}

uint32_t bsp_stack_sp_lo(void)
{
    return 0x20003600u;
}

uint32_t bsp_stack_sp_hi(void)
{
    return 0x20003A00u;
}
