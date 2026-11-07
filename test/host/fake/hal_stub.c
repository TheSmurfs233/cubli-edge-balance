/**
  ******************************************************************************
  * @file    hal_stub.c
  * @brief   Host-side implementation of the fake HAL + test hooks.
  ******************************************************************************
  */

#include "fake_api.h"
#include "stm32g4xx_hal.h"
#include "board_cfg.h"
#include "main.h"
#include "tim.h"
#include "usart.h"

#include <string.h>
#include <stdio.h>

/* ------------------------------------------------------------------ */
/* Objects the production code expects to exist                        */
/* ------------------------------------------------------------------ */
GPIO_TypeDef        fake_gpio_a = { 1 };
GPIO_TypeDef        fake_gpio_f = { 2 };
USART_TypeDef       fake_usart1 = { 1 };
USART_TypeDef       fake_usart2 = { 2 };

/* DMA channels 1..5 as assigned by cubli.ioc:
 *   CH1 = USART1_RX (IMU, circular)     CH2 = USART1_TX (IMU, normal)
 *   CH3 = USART2_RX (debug, circular)   CH4 = USART2_TX (debug, normal)
 *   CH5 = SPI1_RX   (MT6701 encoder, normal)                            */
DMA_Channel_TypeDef fake_dma_ch1 = { DMA_CIRCULAR, 0u };
DMA_Channel_TypeDef fake_dma_ch2 = { 0u, 0u };
DMA_Channel_TypeDef fake_dma_ch3 = { DMA_CIRCULAR, 0u };
DMA_Channel_TypeDef fake_dma_ch4 = { 0u, 0u };
DMA_Channel_TypeDef fake_dma_ch5 = { 0u, 0u };

DMA_HandleTypeDef   hdma_usart1_rx = { &fake_dma_ch1, { DMA_CIRCULAR, 0u }, 0 };
DMA_HandleTypeDef   hdma_usart1_tx = { &fake_dma_ch2, { 0u, 0u }, 0 };
DMA_HandleTypeDef   hdma_usart2_rx = { &fake_dma_ch3, { DMA_CIRCULAR, 0u }, 0 };
DMA_HandleTypeDef   hdma_usart2_tx = { &fake_dma_ch4, { 0u, 0u }, 0 };
DMA_HandleTypeDef   hdma_spi1_rx   = { &fake_dma_ch5, { 0u, 0u }, 0 };

/* huart1 = IMU (USART1 921600), huart2 = debug console (USART2 1000000) */
UART_HandleTypeDef  huart1 = { &fake_usart1, { 921600u, 0u, 0u },
                               &hdma_usart1_rx, &hdma_usart1_tx, 0u, 0u };
UART_HandleTypeDef  huart2 = { &fake_usart2, { 1000000u, 0u, 0u },
                               &hdma_usart2_rx, &hdma_usart2_tx, 0u, 0u };

/* TIM1 register model. BDTR defaults to DTG=4 (as programmed in tim.c) so the
 * boot-time dead-time check in app_init() stays quiet. */
TIM_TypeDef         fake_tim1_regs = { .CR1 = 0u, .SR = 0u, .CNT = 0u, .CCER = 0u,
                                          .CCMR1 = 0u, .CCMR2 = 0u,
                                          .CCR1 = 0u, .CCR2 = 0u, .CCR3 = 0u, .CCR4 = 4u,
                                          .BDTR = 0u, .EGR = 0u };

/* ADC2 injected-group model + DWT cycle counter model */
ADC_TypeDef         fake_adc2_regs = { 2048u, 2048u, 2048u, 0u, 0u, 0u, 0u, 0u };
ADC_HandleTypeDef   hadc2          = { &fake_adc2_regs };

/* ADC1 (S7.5 bus voltage). The fake starts with EOC already set and a DR
 * that corresponds to 12.0 V so the driver sees a plausible reading. */
ADC_TypeDef         fake_adc1_regs = { 0u, 0u, 0u, ADC_ISR_EOC, 0u, 0u, 0u, 2127u };
ADC_HandleTypeDef   hadc1          = { &fake_adc1_regs };
DWT_TypeDef         fake_dwt       = { 0u, 0u };
CoreDebug_TypeDef   fake_coredebug = { 0u };

HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef *hadc)
{
    (void)hadc;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef *hadc, uint32_t SingleDiff)
{
    (void)hadc; (void)SingleDiff;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_ADCEx_InjectedStart_IT(ADC_HandleTypeDef *hadc)
{
    (void)hadc;
    return HAL_OK;
}

/* Callbacks implemented by the firmware (bsp_uart.c) */
extern void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
extern void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);

/* ------------------------------------------------------------------ */
/* Time                                                                */
/* ------------------------------------------------------------------ */
static uint32_t s_tick;

uint32_t HAL_GetTick(void)          { return s_tick; }
void     HAL_Delay(uint32_t ms)     { s_tick += ms; }   /* never blocks on the host */

/* ---------------- FLASH: fake ACR + one 2KB page at CAL_FLASH_ADDR -------- */
FLASH_TypeDef fake_flash_regs;
uint8_t fake_flash_page[0x800u];
uint8_t fake_flash_page_enc[0x800u];
static int s_flash_ready;

static void fake_flash_init(void)
{
    if (!s_flash_ready) {
        memset(fake_flash_page, 0xFF, sizeof(fake_flash_page));
        s_flash_ready = 1;
    }
}

void fake_flash_reset(void)
{
    memset(fake_flash_page, 0xFF, sizeof(fake_flash_page));
    s_flash_ready = 1;
}

void fake_flash_read(uint32_t addr, void *dst, uint32_t len)
{
    uint32_t off;

    fake_flash_init();

    if ((addr >= ENC_FLASH_ADDR) && (addr < (ENC_FLASH_ADDR + 0x800u))) {
        off = addr - ENC_FLASH_ADDR;
        if ((off + len) > sizeof(fake_flash_page_enc)) { memset(dst, 0xFF, len); return; }
        memcpy(dst, &fake_flash_page_enc[off], len);
        return;
    }

    off = addr - CAL_FLASH_ADDR;
    if ((off + len) > sizeof(fake_flash_page)) { memset(dst, 0xFF, len); return; }
    memcpy(dst, &fake_flash_page[off], len);
}
void     fake_tick_reset(void)      { s_tick = 0u; }
void     fake_tick_set(uint32_t ms) { s_tick = ms; }
void     fake_tick_add(uint32_t ms)
{
    /* Real time advances both clocks: the millisecond tick and the DWT cycle
     * counter the encoder uses to budget how far the rotor can have moved. */
    s_tick += ms;
    fake_dwt.CYCCNT += ms * (SYS_CORE_CLK_HZ / 1000u);
}
uint32_t fake_tick_get(void)        { return s_tick; }

/* ------------------------------------------------------------------ */
/* GPIO                                                                */
/* ------------------------------------------------------------------ */
static int s_key_pressed;
static int s_led_level;      /* 0 = PF1 low (LED on), 1 = PF1 high */

void fake_key_set(int pressed) { s_key_pressed = pressed; }
int  fake_led_level(void)      { return s_led_level; }

void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init)
{
    (void)port; (void)init;
}

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint32_t pin, GPIO_PinState state)
{
    if ((port == &fake_gpio_f) && (pin == LED_Pin)) {
        s_led_level = (state == GPIO_PIN_SET) ? 1 : 0;
    }
}

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint32_t pin)
{
    if ((port == &fake_gpio_a) && (pin == KEY_Pin)) {
        /* active low: pressed -> RESET */
        return s_key_pressed ? GPIO_PIN_RESET : GPIO_PIN_SET;
    }
    return GPIO_PIN_SET;
}

/* ------------------------------------------------------------------ */
/* DMA                                                                 */
/* ------------------------------------------------------------------ */
HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef *hdma)
{
    if (hdma == NULL) { return HAL_ERROR; }
    hdma->Instance->CCR = hdma->Init.Mode;
    return HAL_OK;
}

/* ------------------------------------------------------------------ */
/* UART TX: capture per link and complete synchronously                */
/*   huart2 = debug console (bsp_uart, DMA path, helpers fake_tx_*)    */
/*   huart1 = IMU module    (bsp_imu,  IT  path, helpers fake_imu_tx_*)*/
/* ------------------------------------------------------------------ */
#define FAKE_TX_MAX (256u * 1024u)

static uint8_t  s_tx_buf[FAKE_TX_MAX];        /* debug console bytes */
static uint32_t s_tx_len;
static int      s_tx_force_busy;
static int      s_tx_call_count;
static int      s_tx_no_complete;
static int      s_tx_abort_count;

static uint8_t  s_imu_tx_buf[FAKE_TX_MAX];    /* IMU command bytes */
static uint32_t s_imu_tx_len;
static int      s_imu_tx_call_count;
static int      s_imu_tx_no_complete;

void fake_tx_reset(void)
{
    s_tx_len = 0u;
    s_tx_force_busy = 0;
    s_tx_call_count = 0;
    s_tx_no_complete = 0;
    s_tx_abort_count = 0;
}
const uint8_t *fake_tx_data(void) { return s_tx_buf; }
uint32_t fake_tx_len(void)        { return s_tx_len; }
int      fake_tx_call_count(void) { return s_tx_call_count; }
void     fake_tx_force_busy(int on) { s_tx_force_busy = on; }
void     fake_tx_no_complete(int on) { s_tx_no_complete = on; }
int      fake_tx_abort_count(void) { return s_tx_abort_count; }

void fake_imu_tx_reset(void)
{
    s_imu_tx_len = 0u;
    s_imu_tx_call_count = 0;
    s_imu_tx_no_complete = 0;
}
const uint8_t *fake_imu_tx_data(void) { return s_imu_tx_buf; }
uint32_t fake_imu_tx_len(void)        { return s_imu_tx_len; }
int      fake_imu_tx_call_count(void) { return s_imu_tx_call_count; }
void     fake_imu_tx_no_complete(int on) { s_imu_tx_no_complete = on; }

static HAL_StatusTypeDef fake_tx_append(UART_HandleTypeDef *huart,
                                        uint8_t *pData, uint16_t size)
{
    if (huart == &huart1) {
        if ((uint32_t)size > (FAKE_TX_MAX - s_imu_tx_len)) { return HAL_ERROR; }
        memcpy(&s_imu_tx_buf[s_imu_tx_len], pData, size);
        s_imu_tx_len += size;
        if (!s_imu_tx_no_complete) { HAL_UART_TxCpltCallback(&huart1); }
        return HAL_OK;
    }

    if ((uint32_t)size > (FAKE_TX_MAX - s_tx_len)) { return HAL_ERROR; }
    memcpy(&s_tx_buf[s_tx_len], pData, size);
    s_tx_len += size;

    /* Simulate the DMA transfer completing immediately, unless the test wants
     * to model a completion that never arrives */
    if (!s_tx_no_complete) {
        HAL_UART_TxCpltCallback(&huart2);
    }
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart,
                                        uint8_t *pData, uint16_t size)
{
    s_tx_call_count++;
    s_imu_tx_call_count += 0;

    if (s_tx_force_busy) {
        s_tx_force_busy = 0;     /* only the next call fails */
        return HAL_BUSY;
    }
    return fake_tx_append(huart, pData, size);
}

HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *huart,
                                       uint8_t *pData, uint16_t size)
{
    if (huart == &huart1) {
        s_imu_tx_call_count++;
    } else {
        s_tx_call_count++;
    }
    return fake_tx_append(huart, pData, size);
}

HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *huart)
{
    (void)huart;
    s_tx_abort_count++;
    return HAL_OK;
}

/* ------------------------------------------------------------------ */
/* UART RX: the test injects bytes into the reception buffer           */
/* ------------------------------------------------------------------ */
typedef struct {
    uint8_t *buf;
    uint16_t size;
    uint16_t pos;
    int      started;
} fake_rx_t;

static fake_rx_t s_rx_dbg;   /* huart2, debug console */
static fake_rx_t s_rx_imu;   /* huart1, IMU link      */

static fake_rx_t *fake_rx_for(UART_HandleTypeDef *huart)
{
    return (huart == &huart1) ? &s_rx_imu : &s_rx_dbg;
}

HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *huart,
                                               uint8_t *pData, uint16_t size)
{
    fake_rx_t *rx = fake_rx_for(huart);

    rx->buf     = pData;
    rx->size    = size;
    rx->pos     = 0u;
    rx->started = 1;
    return HAL_OK;
}

uint16_t fake_rx_pos(void)     { return s_rx_dbg.pos; }
int      fake_rx_started(void) { return s_rx_dbg.started; }
uint16_t fake_imu_rx_pos(void)     { return s_rx_imu.pos; }
int      fake_imu_rx_started(void) { return s_rx_imu.started; }

static void fake_rx_put(fake_rx_t *rx, UART_HandleTypeDef *hu,
                        const uint8_t *data, uint16_t len)
{
    uint16_t i;

    if (!rx->started || (rx->buf == NULL) || (rx->size == 0u)) { return; }

    for (i = 0u; i < len; i++) {
        rx->buf[rx->pos] = data[i];
        rx->pos++;
        if (rx->pos >= rx->size) {
            rx->pos = 0u;
            /* Circular DMA raises a Transfer-Complete event (Size = full
             * buffer) every time it wraps, exactly like real hardware. */
            hu->RxEventType = HAL_UART_RXEVENT_TC;
            HAL_UARTEx_RxEventCallback(hu, rx->size);
        }
    }
    /* Line idle: report the current write position */
    hu->RxEventType = HAL_UART_RXEVENT_IDLE;
    HAL_UARTEx_RxEventCallback(hu, rx->pos);
}

void fake_rx_feed(const uint8_t *data, uint16_t len)
{
    fake_rx_put(&s_rx_dbg, &huart2, data, len);
}

void fake_imu_rx_feed(const uint8_t *data, uint16_t len)
{
    fake_rx_put(&s_rx_imu, &huart1, data, len);
}


void fake_rx_feed_str(const char *s)
{
    fake_rx_feed((const uint8_t *)s, (uint32_t)0u + (uint16_t)strlen(s));
}
