/* Data-write watchpoint on the aliased pointer word.
 *
 * The two crashes of 2026-11-07 (16:14 and 17:00) both faulted in
 * `VSTR s0,[r0,#0xc]` inside app_bal_update, with r0 loaded from s_bal_p
 * (0x20000154) as 0x2002B970 / 0x2000B970 instead of 0x20003170. The very same
 * word had been read three instructions earlier and was still correct, and the
 * code in between writes nothing but s_bal fields. So the store comes from
 * somewhere asynchronous - an ISR or a DMA - and guessing which one has not
 * paid off.
 *
 * DWT comparator 0 can match a write to one 32 bit address and raise
 * DebugMonitor instead of halting, which works with no debugger attached. The
 * handler records PC/LR/R0-R3 and the value that landed there, then disarms
 * itself (a wrong FUNCTION encoding must never turn into an event storm).
 * Read the record with the `wp` console command - the boards keeps running. */

#include "main.h"
#include "bsp_wp.h"
#include "mid_log.h"

/* ARMv7-M DWT_FUNCTION: 0x3 = watchpoint on data address write.
 * The comparator holds the address because DATAVADDR0 is zero (no chaining). */
#define WP_FN_WRITE 0x3u

static volatile uint32_t s_armed;
static volatile uint32_t s_addr;
static volatile uint32_t s_good;
static volatile uint32_t s_hits;
static volatile uint32_t s_bad;
static volatile uint32_t s_pc;
static volatile uint32_t s_lr;
static volatile uint32_t s_val;
static volatile uint32_t s_sp;
static volatile uint32_t s_r[4];

void bsp_wp_disarm(void)
{
    DWT->FUNCTION0 = 0u;
    s_armed = 0u;
}

void bsp_wp_arm(uint32_t addr, uint32_t good)
{
    bsp_wp_disarm();

    s_addr = addr;
    s_good = good;

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_MON_EN_Msk;

    DWT->COMP0     = addr;
    DWT->MASK0     = 0u;
    DWT->FUNCTION0 = WP_FN_WRITE;

    s_armed = 1u;
}

uint8_t bsp_wp_armed(void)
{
    return (uint8_t)(s_armed != 0u);
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
    s_hits = 0u;
    s_bad  = 0u;
    s_pc   = 0u;
    s_lr   = 0u;
    s_sp   = 0u;
    s_val  = 0u;
    s_r[0] = 0u;
    s_r[1] = 0u;
    s_r[2] = 0u;
    s_r[3] = 0u;
}

void bsp_wp_on_monitor(const volatile uint32_t *frame)
{
    uint32_t pc;
    uint32_t lr;
    uint32_t val;

    /* Disarm first: whatever happens next must not raise another debug event. */
    bsp_wp_disarm();

    pc = frame[6]; /* stacked PC */
    lr = frame[5]; /* stacked LR */
    val = *(volatile uint32_t *)s_addr;

    ++s_hits;

    if (val == s_good)
    {
        /* Somebody rewrote the alias with the right value (the guard does this
         * every time it repairs a corruption). Not the culprit. */
        return;
    }

    ++s_bad;
    s_pc   = pc;
    s_lr   = lr;
    s_val  = val;
    s_sp   = (uint32_t)frame;
    s_r[0] = frame[0];
    s_r[1] = frame[1];
    s_r[2] = frame[2];
    s_r[3] = frame[3];
}

uint8_t bsp_wp_print(void)
{
    if (s_bad == 0u)
    {
        log_printf("wp   = hits %u, bad 0, armed %u, addr %x",
                   (unsigned)s_hits, (unsigned)s_armed, (unsigned)s_addr);
        return 0u;
    }

    log_printf("wp   = write %x -> %x (expected %x)", (unsigned)s_addr,
               (unsigned)s_val, (unsigned)s_good);
    log_printf(" wp  = pc %x, lr %x, sp %x", (unsigned)s_pc, (unsigned)s_lr,
               (unsigned)s_sp);
    log_printf(" wp  = r0 %x, r1 %x, r2 %x, r3 %x", (unsigned)s_r[0],
               (unsigned)s_r[1], (unsigned)s_r[2], (unsigned)s_r[3]);
    return 1u;
}
