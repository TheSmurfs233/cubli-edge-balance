/* Data-write watchpoint on the aliased pointer word.
 *
 * The crashes of 2026-11-07 (16:14, 17:00) both faulted in
 * `VSTR s0,[r0,#0xc]` inside app_bal_update, with r0 loaded from s_bal_p
 * (0x20000154) as 0x2002B970 / 0x2000B970 instead of 0x20003170. The very same
 * word had been read three instructions earlier and was still correct, and the
 * code in between writes nothing but s_bal fields. So the store comes from
 * somewhere asynchronous - an ISR or a DMA - and guessing which one has not
 * paid off.
 *
 * DWT comparator 0 can match a write to one 32 bit address and raise
 * DebugMonitor instead of halting, which works with no debugger attached. The
 * handler records PC/LR/R0/R1 and the value that landed there, then disarms
 * itself (a wrong FUNCTION encoding must never turn into an event storm).
 * Read it with the `wp` console command - the board keeps running. The same
 * record is mirrored into the TAMP backup registers, because a later crash
 * plus the reset that follows would otherwise wipe it with .bss. */

#include "main.h"
#include "bsp_wp.h"
#include "mid_log.h"

/* ARMv7-M DWT_FUNCTION: 0x3 = watchpoint on data address write.
 * The comparator holds the address because DATAVADDR0 is zero (no chaining). */
#define WP_FN_WRITE 0x3u

/* Backup-register slots 23..28 are shared with the crash module's stack
 * window: a real fault overwrites them, which is the right priority - we would
 * rather have the crash frame than a stale watchpoint hit. */
#define WP_SLOT_MAGIC 23u
#define WP_SLOT_PC    24u
#define WP_SLOT_LR    25u
#define WP_SLOT_VAL   26u
#define WP_SLOT_R0    27u
#define WP_SLOT_ADDR  28u
#define WP_MAGIC      0x57504F4Bu /* 'WPOK' */

static volatile uint32_t s_armed;
static volatile uint32_t s_addr;
static volatile uint32_t s_good;
static volatile uint32_t s_hits;
static volatile uint32_t s_bad;
static volatile uint32_t s_pc;
static volatile uint32_t s_lr;
static volatile uint32_t s_val;
static volatile uint32_t s_sp;
static volatile uint32_t s_r[2];

static volatile uint32_t *wp_slot(uint32_t idx)
{
    return (&TAMP->BKP0R) + idx;
}

/* The backup domain sits behind the PWR clock and the DBP write protect. */
static void wp_publish(void)
{
    RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
    (void)RCC->APB1ENR1;
    PWR->CR1 |= PWR_CR1_DBP;
    (void)PWR->CR1;
    for (volatile uint32_t i = 0u; i < 32u; ++i)
    {
        __NOP();
    }
}

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

    wp_publish();
    *wp_slot(WP_SLOT_MAGIC) = 0u;
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

    wp_publish();
    *wp_slot(WP_SLOT_PC)    = pc;
    *wp_slot(WP_SLOT_LR)    = lr;
    *wp_slot(WP_SLOT_VAL)   = val;
    *wp_slot(WP_SLOT_R0)    = frame[0];
    *wp_slot(WP_SLOT_ADDR)  = s_addr;
    *wp_slot(WP_SLOT_MAGIC) = WP_MAGIC;
}

uint8_t bsp_wp_print(void)
{
    wp_publish();

    if (s_bad == 0u)
    {
        if (*wp_slot(WP_SLOT_MAGIC) == WP_MAGIC)
        {
            /* A reset wiped .bss, but the backup copy survived: this is the
             * record from before the reset. */
            log_printf("wp   = before reset: addr %x",
                       (unsigned)*wp_slot(WP_SLOT_ADDR));
            log_printf(" wp  = value %x, pc %x, lr %x",
                       (unsigned)*wp_slot(WP_SLOT_VAL),
                       (unsigned)*wp_slot(WP_SLOT_PC),
                       (unsigned)*wp_slot(WP_SLOT_LR));
            log_printf(" wp  = r0 %x", (unsigned)*wp_slot(WP_SLOT_R0));
            return 1u;
        }

        log_printf("wp   = hits %u, bad 0, armed %u, addr %x",
                   (unsigned)s_hits, (unsigned)s_armed, (unsigned)s_addr);
        return 0u;
    }

    log_printf("wp   = write %x -> %x (expected %x)", (unsigned)s_addr,
               (unsigned)s_val, (unsigned)s_good);
    log_printf(" wp  = pc %x, lr %x, sp %x", (unsigned)s_pc, (unsigned)s_lr,
               (unsigned)s_sp);
    log_printf(" wp  = r0 %x, r1 %x", (unsigned)s_r[0], (unsigned)s_r[1]);
    return 1u;
}
