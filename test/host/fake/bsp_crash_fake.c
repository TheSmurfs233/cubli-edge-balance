#include "bsp_crash.h"

/* Host stand-in for User/Bsp/bsp_crash.c: the backup domain and the fault
 * status registers do not exist on a PC, but the banner code still calls in.
 * The tests only need the calls to be harmless and observable. */

static uint8_t s_reported;

void bsp_crash_init(void)
{
}

void bsp_crash_capture(uint32_t kind)
{
    (void)kind;
}

void bsp_crash_hold(void)
{
}

uint8_t bsp_crash_valid(void)
{
    return 0u;
}

uint8_t bsp_crash_pending(void)
{
    return 0u;
}

void bsp_crash_report(void)
{
    s_reported = 1u;
}

void bsp_crash_clear(void)
{
}

uint8_t bsp_crash_print_saved(void)
{
    return s_reported;
}

uint8_t fake_crash_reported(void)
{
    return s_reported;
}
