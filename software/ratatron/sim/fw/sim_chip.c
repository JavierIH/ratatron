// The simulated chip: the firmware's main() runs in a coroutine, SysTick is a call every simulated millisecond.
// Python (ratsim/firmware.py) drives it through the sim_* functions at the end: inputs, one ms, outputs.
//
// Time: main() yields back to the simulator every SIM_YIELD_CALLS calls of HAL_GetTick() (every loop that waits
// calls it) and in HAL_Delay(); the simulator then steps the physics 1 ms, runs the SysTick handler and resumes
// main(). The firmware's computations take no simulated time.
#define _GNU_SOURCE
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ucontext.h>
#include "stm32f1xx_hal.h"
#include "sim_api.h"

#define SIM_YIELD_CALLS 4
#define SIM_STACK_BYTES (512 * 1024)

GPIO_TypeDef sim_gpioa, sim_gpiob, sim_gpioc;
TIM_TypeDef sim_tim4;
ADC_TypeDef sim_adc1;
RCC_TypeDef sim_rcc;
FLASH_TypeDef sim_flash;
SCB_TypeDef sim_scb = {.CPUID = 0x411FC231u};
DBGMCU_TypeDef sim_dbgmcu = {.IDCODE = 0x20036410u};
DWT_TypeDef sim_dwt;
uint32_t SystemCoreClock = 72000000u;
uint16_t sim_flash_size_kb = 64;

extern int firmware_main(void);
extern void app_systick(void);

static ucontext_t sim_ctx, fw_ctx;
static char *fw_stack;
static volatile uint32_t tick;
static uint8_t in_systick, state = SIM_RUNNING;
static unsigned calls;

static void yield(void){
    calls = 0;
    swapcontext(&fw_ctx, &sim_ctx);
}

// Parks the firmware for good (a fault, Error_Handler, a reset): the simulator sees `state`.
void sim_halt(uint8_t why){
    state = why;
    for(;;) yield();
}

static void fw_entry(void){
    firmware_main();
    sim_halt(SIM_HALTED);
}

// ---- HAL ---------------------------------------------------------------------------------------------
void HAL_Init(void){}
void HAL_IncTick(void){ tick++; }
uint32_t HAL_GetTick(void){
    if(!in_systick && ++calls >= SIM_YIELD_CALLS) yield();
    return tick;
}
void HAL_Delay(uint32_t ms){
    const uint32_t start = tick;
    while(tick - start < ms + 1u) yield();
}
void NVIC_SystemReset(void){
    sim_halt(SIM_RESET);
    __builtin_unreachable();
}
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init){ (void)port; (void)init; }
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin){
    return (port->IDR & pin) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState s){
    if(s == GPIO_PIN_SET) port->ODR |= pin;
    else port->ODR &= ~(uint32_t)pin;
}
static void gpio_apply(GPIO_TypeDef *p){    // what BSRR/BRR writes do on the chip
    const uint32_t set = p->BSRR & 0xFFFFu, reset = (p->BSRR >> 16) | p->BRR;
    p->ODR = (p->ODR & ~reset) | set;
    p->BSRR = p->BRR = 0;
}

// ADC: infrared.c configures the scan and hands its DMA buffer over; the simulator fills it.
static uint8_t rank_of_channel[16];
static uint16_t *adc_buffer;
static uint32_t adc_length;
HAL_StatusTypeDef HAL_ADC_Init(ADC_HandleTypeDef *h){ (void)h; return HAL_OK; }
HAL_StatusTypeDef HAL_ADC_ConfigChannel(ADC_HandleTypeDef *h, ADC_ChannelConfTypeDef *c){
    (void)h;
    if(c->Channel < 16) rank_of_channel[c->Channel] = (uint8_t)c->Rank;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef *h){ (void)h; return HAL_OK; }
HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef *h, uint32_t *buffer, uint32_t length){
    (void)h;
    adc_buffer = (uint16_t *)buffer;
    adc_length = length;
    return HAL_OK;
}

// ---- Simulator API -------------------------------------------------------------------------------------
int sim_boot(void){
    fw_stack = malloc(SIM_STACK_BYTES);
    if(!fw_stack) return 0;
    getcontext(&fw_ctx);
    fw_ctx.uc_stack.ss_sp = fw_stack;
    fw_ctx.uc_stack.ss_size = SIM_STACK_BYTES;
    fw_ctx.uc_link = NULL;
    makecontext(&fw_ctx, fw_entry, 0);
    swapcontext(&sim_ctx, &fw_ctx);     // the boot runs until main() first waits
    return 1;
}

uint8_t sim_step(void){
    if(state != SIM_RUNNING) return state;
    HAL_IncTick();
    sim_dwt.CYCCNT = tick * 72000u;
    in_systick = 1;
    app_systick();
    in_systick = 0;
    swapcontext(&sim_ctx, &fw_ctx);
    gpio_apply(&sim_gpioa);
    gpio_apply(&sim_gpiob);
    gpio_apply(&sim_gpioc);
    return state;
}

uint32_t sim_time_ms(void){ return tick; }

void sim_set_buttons(uint8_t start, uint8_t select){   // pressed = high (external resistors)
    if(start) sim_gpioc.IDR |= GPIO_PIN_13; else sim_gpioc.IDR &= ~(uint32_t)GPIO_PIN_13;
    if(select) sim_gpiob.IDR |= GPIO_PIN_5; else sim_gpiob.IDR &= ~(uint32_t)GPIO_PIN_5;
}

uint8_t sim_leds(void){     // bit 5 = LED 1 ... bit 0 = LED 6
    const uint32_t b = sim_gpiob.ODR, a = sim_gpioa.ODR;
    return (uint8_t)(((b >> 13) & 1u) << 5 | ((b >> 14) & 1u) << 4 | ((b >> 15) & 1u) << 3 |
                     ((a >> 3) & 1u) << 2 | ((a >> 4) & 1u) << 1 | ((a >> 5) & 1u));
}

// Raw ADC counts per channel (the sensors' wiring: FL CH9, FR CH6, SL CH8, SR CH7), `frac` dithers the 16
// oversampled conversions so the average carries the fraction.
void sim_set_adc(uint8_t channel, float counts){
    if(!adc_buffer || channel >= 16 || !rank_of_channel[channel]) return;
    if(counts < 0.0f) counts = 0.0f;
    if(counts > 4095.0f) counts = 4095.0f;
    const unsigned per = 4u, pos = rank_of_channel[channel] - 1u, n = adc_length / per;
    const uint16_t base = (uint16_t)counts;
    const unsigned up = (unsigned)((counts - (float)base) * (float)n + 0.5f);
    for(unsigned k = 0; k < n; k++) adc_buffer[k * per + pos] = (uint16_t)(base + (k < up && base < 4095u));
}
