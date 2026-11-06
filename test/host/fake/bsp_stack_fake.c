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
