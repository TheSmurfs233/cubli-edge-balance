/* Sticky fault record in the TAMP backup domain.
 *
 * Why: the bench fault shows up as "console silent + LED blinking + the key
 * does nothing". That is a CPU exception, but the cause is invisible after
 * the fact. Six 32 bit backup registers plus the Cortex-M status registers
 * are enough to say which exception fired and where it came from, and they
 * survive the reset button.
 *
 * Cost: 8 words of backup RAM, a handful of instructions in the handler,
 * no HAL calls, no allocation, safe to run with a corrupted stack. */

#include "main.h"
#include "bsp_crash.h"
#include "mid_log.h"
#include "bsp_stack.h"

#define CRASH_MAGIC      0x434C4F47u  /* 'CLOG': fresh, not yet shown */
#define CRASH_MAGIC_SHOWN 0x434C4F52u /* 'CLOR': shown once, still readable */
#define CRASH_SLOT_MAGIC 0u
#define CRASH_SLOT_KIND  1u
#define CRASH_SLOT_PC    2u
#define CRASH_SLOT_LR    3u
#define CRASH_SLOT_PSR   4u
#define CRASH_SLOT_CFSR  5u
#define CRASH_SLOT_HFSR  6u
#define CRASH_SLOT_MMFAR 7u
#define CRASH_SLOT_BFAR  8u
#define CRASH_SLOT_R0    9u
#define CRASH_SLOT_R1   10u
#define CRASH_SLOT_R2   11u
#define CRASH_SLOT_R3   12u
#define CRASH_SLOT_R12  13u
#define CRASH_SLOT_SP   14u
#define CRASH_SLOT_R4   15u   /* .. 22 = R11 */
#define CRASH_SLOT_STK0 23u   /* .. 28 = six words above the frame */
#define CRASH_SLOT_CPACR 29u
#define CRASH_SLOT_FPCCR 30u
#define CRASH_SLOT_FPCAR 31u

/* Filled by the assembler stub below, consumed by bsp_crash_capture(). It is a
 * plain RAM array so the stub needs no stack and no branch to C. */
volatile uint32_t crash_live_regs[8];

__asm void bsp_crash_save_regs(void)
{
    PRESERVE8
    IMPORT  crash_live_regs
    LDR     r0, =crash_live_regs
    STM     r0, {r4-r11}
    BX      lr
}

static volatile uint32_t *crash_slot(uint32_t idx)
{
    volatile uint32_t *base = &TAMP->BKP0R;
    return (base + idx);
}

static void crash_publish(void)
{
    /* The backup domain sits behind the PWR clock and the DBP write protect. */
    RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
    (void)RCC->APB1ENR1;
    PWR->CR1 |= PWR_CR1_DBP;
    (void)PWR->CR1;
    for (volatile uint32_t i = 0u; i < 32u; ++i)
    {
        __NOP();
    }
}

void bsp_crash_init(void)
{
    crash_publish();

    /* Without these the configurable faults all escalate to HardFault and the
     * fault address is lost (HFSR.FORCED, CFSR=IMPRECISERR, no BFARVALID).
     * Enabling the three handlers is what turns "somewhere in there" into an
     * exact address. */
    SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk
               |  SCB_SHCSR_BUSFAULTENA_Msk
               |  SCB_SHCSR_USGFAULTENA_Msk;
    __DSB();
    __ISB();
}

static void crash_store(uint32_t idx, uint32_t value)
{
    *crash_slot(idx) = value;
}

/* The handler and this function push a few words before we get here, so MSP no
 * longer points at the exception frame. The frame is easy to recognise: xPSR
 * has bit 24 set with a non-zero top byte, and PC points into flash. Scanning
 * up a few words is cheaper -- and far more robust -- than assuming a fixed
 * prologue size. */
static const uint32_t *crash_find_frame(const uint32_t *base)
{
    for (uint32_t i = 0u; i < 16u; ++i)
    {
        const uint32_t *f = base + i;

        if ((f[7] & 0x01000000u) == 0u) { continue; }
        if ((f[7] >> 24) == 0u)         { continue; }
        if (f[6] < 0x08000000u)         { continue; }
        if (f[6] >= 0x08020000u)        { continue; }
        return f;
    }
    return base;
}

void bsp_crash_capture(uint32_t kind)
{
    const uint32_t *frame;

    /* No RTOS on this board, so the exception frame is always on MSP:
     * R0 R1 R2 R3 R12 LR PC xPSR. */
    frame = crash_find_frame((const uint32_t *)__get_MSP());

    crash_store(CRASH_SLOT_KIND,  kind);
    crash_store(CRASH_SLOT_PC,    frame[6]);
    crash_store(CRASH_SLOT_LR,    frame[5]);
    crash_store(CRASH_SLOT_PSR,   frame[7]);
    crash_store(CRASH_SLOT_CFSR,  SCB->CFSR);
    crash_store(CRASH_SLOT_HFSR,  SCB->HFSR);
    crash_store(CRASH_SLOT_MMFAR, SCB->MMFAR);
    crash_store(CRASH_SLOT_BFAR,  SCB->BFAR);
    /* The arguments matter as much as the address: an out-of-range index and a
     * corrupted base pointer look identical in BFAR alone. */
    crash_store(CRASH_SLOT_R0,  frame[0]);
    crash_store(CRASH_SLOT_R1,  frame[1]);
    crash_store(CRASH_SLOT_R2,  frame[2]);
    crash_store(CRASH_SLOT_R3,  frame[3]);
    crash_store(CRASH_SLOT_R12, frame[4]);
    crash_store(CRASH_SLOT_SP,  (uint32_t)frame);

    /* R4-R11 were latched by the stub before this function ran. */
    for (uint32_t i = 0u; i < 8u; ++i)
    {
        crash_store(CRASH_SLOT_R4 + i, crash_live_regs[i]);
    }

    /* Six words just above the exception frame are the interrupted frame's own
     * locals: if a stray float write landed in a saved register slot, this
     * window is where it shows up. */
    for (uint32_t i = 0u; i < 6u; ++i)
    {
        crash_store(CRASH_SLOT_STK0 + i, frame[8 + i]);
    }

    /* A VFP instruction raising UsageFault/UNDEFINSTR almost always means the
     * coprocessor access got switched off under us, or that the lazy FPU
     * stacking state is inconsistent. Both are visible in these three. */
    crash_store(CRASH_SLOT_CPACR, SCB->CPACR);
    crash_store(CRASH_SLOT_FPCCR, FPU->FPCCR);
    crash_store(CRASH_SLOT_FPCAR, FPU->FPCAR);

    crash_store(CRASH_SLOT_MAGIC, CRASH_MAGIC);
}

void bsp_crash_hold(void)
{
    /* The blink count names the exception that fired, so a bench test does not
     * need a console: 5 = HardFault, 4 = MemManage, 3 = BusFault, 2 = Usage. */
    uint32_t kind = *crash_slot(CRASH_SLOT_KIND);
    /* Every kind is >= 2, so the old form always blinked twice and told the
     * operator nothing. 5 -> 1 blink, 4 -> 2, 3 -> 3, 2 -> 4. */
    uint32_t blinks = ((kind >= 2u) && (kind <= 5u)) ? (6u - kind) : 2u;

    /* Slow enough to read at a glance. The first version used 400k and 4M
     * NOPs, which at 170 MHz is 2.4 ms on / 2.4 ms off / 24 ms gap: the eye
     * integrates that into "LED looks permanently on", which is also what the
     * armed state looks like. Now: 150 ms on, 150 ms off, 1 s between groups. */
    for (;;)
    {
        for (uint32_t i = 0u; i < blinks; ++i)
        {
            LED_GPIO_Port->BSRR = (uint32_t)LED_Pin << 16u;
            for (volatile uint32_t d = 0u; d < 5000000u; ++d)
            {
                __NOP();
            }
            LED_GPIO_Port->BSRR = (uint32_t)LED_Pin;
            for (volatile uint32_t d = 0u; d < 5000000u; ++d)
            {
                __NOP();
            }
        }
        for (volatile uint32_t d = 0u; d < 33000000u; ++d)
        {
            __NOP();
        }
    }
}

static uint8_t crash_magic_is(uint32_t want)
{
    return (*crash_slot(CRASH_SLOT_MAGIC) == want) ? 1u : 0u;
}

uint8_t bsp_crash_valid(void)
{
    return (uint8_t)(crash_magic_is(CRASH_MAGIC) | crash_magic_is(CRASH_MAGIC_SHOWN));
}

/* 1 = captured but not printed yet (the boot banner owns that one shot). */
uint8_t bsp_crash_pending(void)
{
    return crash_magic_is(CRASH_MAGIC);
}

/* The banner prints the block once and then only marks it as shown: the record
 * itself survives, so 'crash' can replay it long after the boot message is
 * gone. Clearing is an explicit operator action ('crash clear'). */
static void crash_mark_shown(void)
{
    *crash_slot(CRASH_SLOT_MAGIC) = CRASH_MAGIC_SHOWN;
}

void bsp_crash_clear(void)
{
    /* Slot 0 first, so a half-cleared record can never look valid. */
    *crash_slot(CRASH_SLOT_MAGIC) = 0u;
    for (uint32_t i = 1u; i < 8u; ++i)
    {
        *crash_slot(i) = 0u;
    }
}

/* Print the stored block. Does not touch the backup registers, so it can be
 * called any number of times (banner at boot and the 'crash' command). */
static void crash_print_block(void)
{
    if (bsp_crash_valid() != 0u)
    {
        log_printf("crash = kind %u pc %x lr %x psr %x",
                   (unsigned)(*crash_slot(CRASH_SLOT_KIND)),
                   (unsigned)(*crash_slot(CRASH_SLOT_PC)),
                   (unsigned)(*crash_slot(CRASH_SLOT_LR)),
                   (unsigned)(*crash_slot(CRASH_SLOT_PSR)));
        log_printf(" crash cfsr %x hfsr %x mmfar %x bfar %x",
                   (unsigned)(*crash_slot(CRASH_SLOT_CFSR)),
                   (unsigned)(*crash_slot(CRASH_SLOT_HFSR)),
                   (unsigned)(*crash_slot(CRASH_SLOT_MMFAR)),
                   (unsigned)(*crash_slot(CRASH_SLOT_BFAR)));
        log_printf(" crash r0 %x r1 %x r2 %x r3 %x",
                   (unsigned)(*crash_slot(CRASH_SLOT_R0)),
                   (unsigned)(*crash_slot(CRASH_SLOT_R1)),
                   (unsigned)(*crash_slot(CRASH_SLOT_R2)),
                   (unsigned)(*crash_slot(CRASH_SLOT_R3)));
        log_printf(" crash r12 %x sp %x",
                   (unsigned)(*crash_slot(CRASH_SLOT_R12)),
                   (unsigned)(*crash_slot(CRASH_SLOT_SP)));
        log_printf(" crash r4 %x r5 %x r6 %x r7 %x",
                   (unsigned)(*crash_slot(CRASH_SLOT_R4)),
                   (unsigned)(*crash_slot(CRASH_SLOT_R4 + 1u)),
                   (unsigned)(*crash_slot(CRASH_SLOT_R4 + 2u)),
                   (unsigned)(*crash_slot(CRASH_SLOT_R4 + 3u)));
        log_printf(" crash r8 %x r9 %x r10 %x r11 %x",
                   (unsigned)(*crash_slot(CRASH_SLOT_R4 + 4u)),
                   (unsigned)(*crash_slot(CRASH_SLOT_R4 + 5u)),
                   (unsigned)(*crash_slot(CRASH_SLOT_R4 + 6u)),
                   (unsigned)(*crash_slot(CRASH_SLOT_R4 + 7u)));
        log_printf(" crash stk %x %x %x %x",
                   (unsigned)(*crash_slot(CRASH_SLOT_STK0)),
                   (unsigned)(*crash_slot(CRASH_SLOT_STK0 + 1u)),
                   (unsigned)(*crash_slot(CRASH_SLOT_STK0 + 2u)),
                   (unsigned)(*crash_slot(CRASH_SLOT_STK0 + 3u)));
        log_printf(" crash stk %x %x",
                   (unsigned)(*crash_slot(CRASH_SLOT_STK0 + 4u)),
                   (unsigned)(*crash_slot(CRASH_SLOT_STK0 + 5u)));
        log_printf(" crash fpu cpacr %x fpccr %x fpcar %x",
                   (unsigned)(*crash_slot(CRASH_SLOT_CPACR)),
                   (unsigned)(*crash_slot(CRASH_SLOT_FPCCR)),
                   (unsigned)(*crash_slot(CRASH_SLOT_FPCAR)));
        log_printf(" crash stack peak %u B of %u B",
                   (unsigned)bsp_stack_peak_bytes(),
                   (unsigned)bsp_stack_size_bytes());
    }
}

uint8_t bsp_crash_print_saved(void)
{
    if (bsp_crash_valid() == 0u)
    {
        log_printf("crash = none recorded (cleared, or no fault since power on)");
        return 0u;
    }
    crash_print_block();
    return 1u;
}

void bsp_crash_report(void)
{
    /* Reset cause has to be read and cleared once, or it stays latched in
     * the flags forever and every later boot looks the same. */
    uint32_t csr = RCC->CSR;

    if (bsp_crash_pending() != 0u)
    {
        crash_print_block();
        crash_mark_shown();
    }

    log_printf("reset = pin %u por %u soft %u iwdg %u wwdg %u lowpwr %u ob %u",
               (unsigned)((csr & RCC_CSR_PINRSTF) ? 1u : 0u),
               (unsigned)((csr & RCC_CSR_BORRSTF) ? 1u : 0u),
               (unsigned)((csr & RCC_CSR_SFTRSTF) ? 1u : 0u),
               (unsigned)((csr & RCC_CSR_IWDGRSTF) ? 1u : 0u),
               (unsigned)((csr & RCC_CSR_WWDGRSTF) ? 1u : 0u),
               (unsigned)((csr & RCC_CSR_LPWRRSTF) ? 1u : 0u),
               (unsigned)((csr & RCC_CSR_OBLRSTF) ? 1u : 0u));

    RCC->CSR |= RCC_CSR_RMVF;
    RCC->CSR &= ~RCC_CSR_RMVF;
}
