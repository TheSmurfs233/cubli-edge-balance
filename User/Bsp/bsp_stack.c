/**
 * @file    bsp_stack.c
 * @brief   Stack high-water mark for the 2 KB MSP stack (diagnostic only).
 *
 * The scatter file places the stack at the top of RAM:
 *
 *   STACK  0x20003258 .. 0x20003A58   (2048 B, __initial_sp = 0x20003A58)
 *
 * Below it sit the command buffers (s_rep 0x20003174, s_line 0x20003134,
 * s_run 0x20003114, s_bal 0x200030e4). If the stack ever grows past its
 * bottom, those statics are the first casualties, so knowing the peak depth
 * tells us whether an overrun is plausible at all.
 *
 * The unused part of the stack is painted with BSP_STACK_PATTERN right after
 * startup; the low end that is no longer the pattern is the deepest the stack
 * has ever been.
 */
#include "bsp_stack.h"

/** Set by the linker: one past the last stack byte. */
extern uint32_t __initial_sp;

/** Stack size from startup_stm32g431xx.s (Stack_Size EQU 0x800). */
#define BSP_STACK_SIZE 2048u

/** Pattern written into the never-used part of the stack. */
#define BSP_STACK_PATTERN 0xC0DEC0DEu

/** Smallest gap kept between the paint cursor and the live stack pointer. */
#define BSP_STACK_GUARD 64u

static uint32_t s_painted;

static uint32_t stack_top(void)
{
    return (uint32_t)(uintptr_t)&__initial_sp;
}

void bsp_stack_paint(void)
{
    uint32_t top = stack_top();
    uint32_t sp = (uint32_t)__current_sp();
    uint32_t lo = top - BSP_STACK_SIZE;
    uint32_t i;

    s_painted = 0u;
    if (sp < lo + BSP_STACK_GUARD)
    {
        return; /* already too deep to paint safely */
    }
    for (i = lo; (i + BSP_STACK_GUARD) < sp; i += 4u)
    {
        *(volatile uint32_t *)i = BSP_STACK_PATTERN;
        s_painted += 4u;
    }
}

static uint32_t s_sp_lo_seen = 0xFFFFFFFFu;
static uint32_t s_sp_hi_seen;
static uint32_t s_sp_bad;

uint32_t bsp_stack_sp(void)
{
    return (uint32_t)__current_sp();
}

uint8_t bsp_stack_sp_check(void)
{
    uint32_t sp  = (uint32_t)__current_sp();
    uint32_t top = stack_top();
    uint32_t lo  = top - BSP_STACK_SIZE;

    if (sp < s_sp_lo_seen) { s_sp_lo_seen = sp; }
    if (sp > s_sp_hi_seen) { s_sp_hi_seen = sp; }

    if ((sp < (lo + BSP_STACK_GUARD)) || (sp > top))
    {
        ++s_sp_bad;
        return 0u;
    }
    return 1u;
}

uint32_t bsp_stack_sp_bads(void)
{
    return s_sp_bad;
}

uint32_t bsp_stack_sp_lo(void)
{
    return s_sp_lo_seen;
}

uint32_t bsp_stack_sp_hi(void)
{
    return s_sp_hi_seen;
}

uint32_t bsp_stack_peak_bytes(void)
{
    uint32_t top = stack_top();
    uint32_t lo = top - BSP_STACK_SIZE;
    uint32_t i;

    if (s_painted == 0u)
    {
        return 0u;
    }
    for (i = lo; i < top; i += 4u)
    {
        if (*(volatile uint32_t *)i != BSP_STACK_PATTERN)
        {
            break;
        }
    }
    return top - i;
}

uint32_t bsp_stack_size_bytes(void)
{
    return BSP_STACK_SIZE;
}
