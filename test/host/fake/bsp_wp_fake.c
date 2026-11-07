#include "bsp_wp.h"
#include "mid_log.h"

#include <stdint.h>

/* Host stand-in for the DWT watchpoint. There is no DWT on the PC, so the fake
 * just remembers what was armed and lets the tests drive the recorder. */

uint32_t fake_wp_arm_addr;
uint32_t fake_wp_arm_good;
uint32_t fake_wp_arm_calls;
uint32_t fake_wp_disarm_calls;
uint32_t fake_wp_clear_calls;

static uint8_t s_armed;
static uint32_t s_addr;
static uint32_t s_good;
static uint32_t s_hits;
static uint32_t s_bad;

void bsp_wp_arm(uint32_t addr, uint32_t good)
{
    fake_wp_arm_addr  = addr;
    fake_wp_arm_good  = good;
    ++fake_wp_arm_calls;
    s_armed = 1u;
    s_addr  = addr;
    s_good  = good;
}

void bsp_wp_disarm(void)
{
    ++fake_wp_disarm_calls;
    s_armed = 0u;
}

uint8_t bsp_wp_armed(void)
{
    return s_armed;
}

uint32_t bsp_wp_addr(void)
{
    return (s_armed != 0u) ? s_addr : 0u;
}

uint32_t bsp_wp_hits(void)
{
    return s_hits;
}

uint32_t bsp_wp_bad(void)
{
    return s_bad;
}

void bsp_wp_clear(void)
{
    ++fake_wp_clear_calls;
    s_hits = 0u;
    s_bad  = 0u;
}

void bsp_wp_on_monitor(const volatile uint32_t *frame)
{
    (void)frame;
    ++s_hits;
}

uint8_t bsp_wp_print(void)
{
    log_printf("wp   = hits %u, bad %u, armed %u, addr %x", (unsigned)s_hits,
               (unsigned)s_bad, (unsigned)s_armed, (unsigned)s_addr);
    return (uint8_t)(s_bad != 0u);
}

uint32_t fake_wp_good_of(uint32_t addr)
{
    return (addr == s_addr) ? s_good : 0u;
}
