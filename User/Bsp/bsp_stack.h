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

#endif /* BSP_STACK_H */
