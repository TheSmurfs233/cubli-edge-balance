#ifndef BSP_WP_H
#define BSP_WP_H

#include <stdint.h>

/* Data-write watchpoint on a single RAM word.
 *
 * Why: two crashes ended in exactly the same store, `VSTR s0,[r0,#0xc]`, with
 * r0 read from `s_bal_p` (0x20000154) holding 0x2002B970 / 0x2000B970 instead
 * of 0x20003170. The same word was read twice, three instructions earlier, and
 * was still correct. Nothing in that straight-line code writes it, so an
 * asynchronous writer (ISR or DMA) is inserting a store between two loads.
 *
 * Instead of guessing, let the core trap the write: DWT comparator 0 matches a
 * write to that address and raises DebugMonitor. The handler records PC/LR and
 * the written value, then disarms itself so it cannot loop. Read it back with
 * the `wp` console command - the board keeps running. */

/* Arm the watchpoint on a 4 byte aligned RAM address.
 * `good` is the value the word is supposed to hold: a write that leaves the
 * good value in place is the guard repairing it, not the culprit. Idempotent. */
void bsp_wp_arm(uint32_t addr, uint32_t good);

/* Disarm (also done automatically after the first bad write is captured). */
void bsp_wp_disarm(void);

/* 1 while armed. */
uint8_t bsp_wp_armed(void);

/* Address currently watched (0 when disarmed). */
uint32_t bsp_wp_addr(void);

/* Every matching write, and the subset whose new value was not the good one. */
uint32_t bsp_wp_hits(void);
uint32_t bsp_wp_bad(void);

/* Called from DebugMon_Handler with the stacked exception frame.
 * Must stay tiny and allocation free. */
void bsp_wp_on_monitor(const volatile uint32_t *frame);

/* Print the captured record. Returns 1 when something was captured. */
uint8_t bsp_wp_print(void);

/* Deliberate write to the watched word: 1 when the comparator fired. */
uint8_t bsp_wp_self_test(void);

/* Forget the captured record (keeps the arming state). */
void bsp_wp_clear(void);

#endif
