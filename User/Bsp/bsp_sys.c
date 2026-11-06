/**
  ******************************************************************************
  * @file    bsp_sys.c
  * @brief   ART accelerator enable (see bsp_sys.h).
  *
  * Layer: Bsp.
  *
  * At 170MHz the embedded flash needs 4 wait states, and FLASH_ACR resets with
  * prefetch and both caches off. CubeMX never emits those bits, so without this
  * the core stalls on almost every instruction fetch: measured 2.7 cycles per
  * instruction in the 20kHz control chain (see the "phase" command).
  *
  * Only the instruction side is enabled here. The data cache (DCEN) is left off
  * on purpose: bsp_flash.c erases and programs the flash at runtime (calibration
  * record) and the data cache would have to be cleaned around those operations.
  ******************************************************************************
  */

#include "bsp_sys.h"
#include "stm32g4xx_hal.h"

void bsp_sys_init(void)
{
    FLASH->ACR |= (FLASH_ACR_PRFTEN | FLASH_ACR_ICEN);
}

uint32_t bsp_sys_fpccr(void)
{
#ifdef FPU
    return FPU->FPCCR;
#else
    return 0u;   /* host test build: the fake HAL has no core peripherals */
#endif
}

void bsp_sys_fpu_eager(uint8_t on)
{
#ifdef FPU
    if (on != 0u)
    {
        /* Clear LSPEN first, then ASPEN: with LSPACT already clear (checked by
         * the guard) neither bit can leave a save outstanding. */
        FPU->FPCCR &= ~(FPU_FPCCR_LSPEN_Msk | FPU_FPCCR_ASPEN_Msk);
    }
    else
    {
        FPU->FPCCR |= (FPU_FPCCR_ASPEN_Msk | FPU_FPCCR_LSPEN_Msk);
    }
    __DSB();
    __ISB();
#else
    (void)on;
#endif
}

uint8_t bsp_sys_fpu_eager_get(void)
{
#ifdef FPU
    return ((FPU->FPCCR & (FPU_FPCCR_LSPEN_Msk | FPU_FPCCR_ASPEN_Msk)) == 0u) ? 1u : 0u;
#else
    return 1u;
#endif
}

uint32_t bsp_sys_fpcar(void)
{
#ifdef FPU
    return FPU->FPCAR;
#else
    return 0u;
#endif
}

#if defined(__CC_ARM)
__asm uint32_t bsp_sys_get_r4(void)
{
    PRESERVE8
    MOV     r0, r4
    BX      lr
}
#else
uint32_t bsp_sys_get_r4(void)
{
    return 0u;   /* host build: no register to sample */
}
#endif
