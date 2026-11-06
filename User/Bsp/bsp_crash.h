#ifndef BSP_CRASH_H
#define BSP_CRASH_H

#include <stdint.h>

/* Sticky fault record kept in the TAMP backup registers: it survives the
 * reset the user performs, so the next banner can print where the CPU died.
 * Values are deliberately tiny - the whole point is to survive a crash. */
#define BSP_CRASH_KIND_NONE   0u
#define BSP_CRASH_KIND_HARD   1u
#define BSP_CRASH_KIND_MM     2u
#define BSP_CRASH_KIND_BUS    3u
#define BSP_CRASH_KIND_USAGE  4u

/* Publish the backup domain and drop a "no fault" marker at boot. */
void bsp_crash_init(void);

/* Called first thing in a fault handler.
 *
 * The hardware frame only carries R0-R3/R12/LR/PC/xPSR. R4-R11 are NOT stacked
 * by the CPU, yet they are exactly where a corrupted pointer tends to sit, so
 * they are latched into a plain RAM array before any C code can touch them. */
void bsp_crash_save_regs(void);

/* Called from a fault handler: latch kind, stacked PC/LR/xPSR and the
 * Cortex-M fault status registers. Must not depend on HAL or on the heap. */
void bsp_crash_capture(uint32_t kind);

/* Called from a fault handler after capture: blink the LED in a unique
 * pattern (5 fast + 1 pause) and never return. */
void bsp_crash_hold(void);

/* 1 when the backup registers hold a record from a previous run. */
uint8_t bsp_crash_valid(void);

/* Print the record and the reset cause, then wipe it. Call once per boot. */
void bsp_crash_report(void);

/* Wipe the record (also used by bsp_crash_report). */
void bsp_crash_clear(void);

#endif
