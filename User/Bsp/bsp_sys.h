/**
  ******************************************************************************
  * @file    bsp_sys.h
  * @brief   Core-level system setup that CubeMX does not generate.
  *
  * Layer: Bsp.
  ******************************************************************************
  */

#include <stdint.h>

#ifndef __BSP_SYS_H
#define __BSP_SYS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Enable the ART accelerator (flash prefetch + instruction cache).
 * Must be called once, after the clock tree has been configured. */
void bsp_sys_init(void);

/* FPU lazy stacking observability. FPCCR.LSPACT means "an FP context save is
 * still owed"; FPCAR is where it will be written. If LSPACT ever sticks while
 * normal code runs, the next FP instruction dumps s0-s15 over whatever lives at
 * FPCAR - which is exactly the shape of our bench crash (float bit patterns
 * appearing in saved registers). Read only, no side effects. */
uint32_t bsp_sys_fpccr(void);
uint32_t bsp_sys_fpcar(void);

/* Lazy versus eager FP context saving.
 * lazy  : ASPEN|LSPEN set. The hardware only reserves 0x68 bytes and writes
 *         them later, guided by FPCAR/LSPACT. Cheaper, but it makes the stack
 *         layout depend on bookkeeping that a stray write can desynchronise.
 * eager : both bits clear. Every exception entry from FP code stores the 17 FP
 *         words immediately, so FPCAR/LSPACT never carry state across an
 *         exception. Costs about 30 cycles and 0x68 bytes per entry.
 * This is a runtime switch so the same binary can A/B the two modes. */
void bsp_sys_fpu_eager(uint8_t on);
uint8_t bsp_sys_fpu_eager_get(void);

/* Context marker: which path owned the CPU when the fault hit. */
extern volatile uint32_t bsp_sys_ctx;
#define BSP_CTX_LOOP   1u
#define BSP_CTX_CMD    2u
#define BSP_CTX_FOC    3u
#define BSP_CTX_BAL    4u

/* 1 when the pointer points into the 32 KB SRAM window. */
uint8_t bsp_sys_ram_ptr_ok(const void *p);

/* Reads the physical r4 register.
 * r4 is callee-saved under AAPCS, so any call that changes it breaks the ABI.
 * The bench crash of 2026-11-07 always faults on the first read after
 * alg_bal_step() that uses r4, so sampling this register either side of a call
 * turns a bus fault into a countable event. Embedded assembler: MOV r0,r4 only,
 * no prologue, no flags, no clobbers. */
uint32_t bsp_sys_get_r4(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_SYS_H */
