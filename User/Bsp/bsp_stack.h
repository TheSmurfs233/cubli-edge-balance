/**
 * @file    bsp_stack.h
 * @brief   Stack high-water mark for the 2 KB MSP stack (diagnostic only).
 */
#ifndef BSP_STACK_H
#define BSP_STACK_H

#include <stdint.h>
#include "main.h"

/** Paint the unused part of the stack. Call once, early, before deep calls. */
void    bsp_stack_paint(void);

/** Bytes of stack ever used since bsp_stack_paint() (0 if never painted). */
uint32_t bsp_stack_peak_bytes(void);

/** Total stack size in bytes. */
uint32_t bsp_stack_size_bytes(void);

/** Current MSP (diagnostic, cheap). */
uint32_t bsp_stack_sp(void);

/** 1 if MSP is inside the intended stack window, 0 (and a counter tick) if not. */
uint8_t  bsp_stack_sp_check(void);

/** Number of out-of-window MSP observations. */
uint32_t bsp_stack_sp_bads(void);

/** Lowest / highest MSP ever observed by bsp_stack_sp_check(). */
uint32_t bsp_stack_sp_lo(void);
uint32_t bsp_stack_sp_hi(void);

#endif /* BSP_STACK_H */
