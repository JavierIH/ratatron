// The simulator's stand-in for the STM32 HAL: just what the firmware's application files and the reused drivers
// (gpio.c, infrared.c) use. Registers are plain structs the simulator reads and writes (sim_chip.c).
#ifndef SIM_STM32F1XX_HAL_H
#define SIM_STM32F1XX_HAL_H

#include <stdint.h>
#include <stddef.h>

#define __weak __attribute__((weak))
#define __IO volatile

typedef enum { HAL_OK = 0, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef enum { DISABLE = 0, ENABLE = 1 } FunctionalState;

// ---- Core ------------------------------------------------------------------------------------------
void HAL_Init(void);
uint32_t HAL_GetTick(void);         // main context: also where the simulated CPU yields to the next millisecond
void HAL_Delay(uint32_t ms);
void HAL_IncTick(void);
void NVIC_SystemReset(void) __attribute__((noreturn));
static inline void __disable_irq(void){}        // the main context never runs during the SysTick handler
static inline void __enable_irq(void){}
static inline uint32_t __get_PRIMASK(void){ return 0; }
static inline void __set_PRIMASK(uint32_t m){ (void)m; }
static inline void __DSB(void){}
static inline void __ISB(void){}
static inline void __NOP(void){}
static inline void __WFI(void){}

// ---- GPIO --------------------------------------------------------------------------------------------
typedef struct { volatile uint32_t CRL, CRH, IDR, ODR, BSRR, BRR, LCKR; } GPIO_TypeDef;
typedef struct { uint32_t Pin, Mode, Pull, Speed; } GPIO_InitTypeDef;
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET } GPIO_PinState;
extern GPIO_TypeDef sim_gpioa, sim_gpiob, sim_gpioc;
#define GPIOA (&sim_gpioa)
#define GPIOB (&sim_gpiob)
#define GPIOC (&sim_gpioc)
#define GPIO_PIN_0  0x0001u
#define GPIO_PIN_1  0x0002u
#define GPIO_PIN_2  0x0004u
#define GPIO_PIN_3  0x0008u
#define GPIO_PIN_4  0x0010u
#define GPIO_PIN_5  0x0020u
#define GPIO_PIN_6  0x0040u
#define GPIO_PIN_7  0x0080u
#define GPIO_PIN_8  0x0100u
#define GPIO_PIN_9  0x0200u
#define GPIO_PIN_10 0x0400u
#define GPIO_PIN_11 0x0800u
#define GPIO_PIN_12 0x1000u
#define GPIO_PIN_13 0x2000u
#define GPIO_PIN_14 0x4000u
#define GPIO_PIN_15 0x8000u
#define GPIO_MODE_INPUT         0u
#define GPIO_MODE_OUTPUT_PP     1u
#define GPIO_MODE_AF_PP         2u
#define GPIO_MODE_ANALOG        3u
#define GPIO_NOPULL             0u
#define GPIO_SPEED_FREQ_LOW     0u
#define GPIO_SPEED_FREQ_MEDIUM  1u
#define GPIO_SPEED_FREQ_HIGH    2u
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin);
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state);
#define __HAL_RCC_GPIOA_CLK_ENABLE() do{}while(0)
#define __HAL_RCC_GPIOB_CLK_ENABLE() do{}while(0)
#define __HAL_RCC_GPIOC_CLK_ENABLE() do{}while(0)
#define __HAL_RCC_DMA1_CLK_ENABLE()  do{}while(0)
#define __HAL_RCC_ADC1_CLK_ENABLE()  do{}while(0)

// ---- Timers (motor_emergency_stop() writes TIM4 directly) ----------------------------------------------
typedef struct { volatile uint32_t CR1, CNT, ARR, CCR1, CCR2, CCR3, CCR4; } TIM_TypeDef;
extern TIM_TypeDef sim_tim4;
#define TIM4 (&sim_tim4)

// ---- ADC + DMA (infrared.c) ----------------------------------------------------------------------------
typedef struct { int dummy; } ADC_TypeDef;
extern ADC_TypeDef sim_adc1;
#define ADC1 (&sim_adc1)
typedef struct {
    uint32_t ScanConvMode, ContinuousConvMode, DiscontinuousConvMode, ExternalTrigConv, DataAlign,
             NbrOfConversion, NbrOfDiscConversion;
} ADC_InitTypeDef;
typedef struct { ADC_TypeDef *Instance; ADC_InitTypeDef Init; } ADC_HandleTypeDef;
typedef struct { uint32_t Channel, Rank, SamplingTime; } ADC_ChannelConfTypeDef;
typedef struct { int dummy; } DMA_HandleTypeDef;
#define ADC_SCAN_ENABLE             1u
#define ADC_SOFTWARE_START          0u
#define ADC_DATAALIGN_RIGHT         0u
#define ADC_SAMPLETIME_71CYCLES_5   6u
#define ADC_CHANNEL_0 0u
#define ADC_CHANNEL_1 1u
#define ADC_CHANNEL_2 2u
#define ADC_CHANNEL_3 3u
#define ADC_CHANNEL_4 4u
#define ADC_CHANNEL_5 5u
#define ADC_CHANNEL_6 6u
#define ADC_CHANNEL_7 7u
#define ADC_CHANNEL_8 8u
#define ADC_CHANNEL_9 9u
HAL_StatusTypeDef HAL_ADC_Init(ADC_HandleTypeDef *h);
HAL_StatusTypeDef HAL_ADC_ConfigChannel(ADC_HandleTypeDef *h, ADC_ChannelConfTypeDef *c);
HAL_StatusTypeDef HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef *h);
HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef *h, uint32_t *buffer, uint32_t length);

// ---- Registers some diagnostics print (STATUS in DEV_TOOLS builds) ----------------------------------------
typedef struct { volatile uint32_t CR, CFGR, CIR, APB2RSTR, APB1RSTR, AHBENR, APB2ENR, APB1ENR, BDCR, CSR; } RCC_TypeDef;
typedef struct { volatile uint32_t ACR, KEYR, OPTKEYR, SR, CR, AR, RESERVED, OBR, WRPR; } FLASH_TypeDef;
typedef struct { volatile uint32_t CPUID; } SCB_TypeDef;
typedef struct { volatile uint32_t IDCODE, CR; } DBGMCU_TypeDef;
extern RCC_TypeDef sim_rcc;
extern FLASH_TypeDef sim_flash;
extern SCB_TypeDef sim_scb;
extern DBGMCU_TypeDef sim_dbgmcu;
#define RCC (&sim_rcc)
#define FLASH (&sim_flash)
#define SCB (&sim_scb)
#define DBGMCU (&sim_dbgmcu)
typedef struct { volatile uint32_t CTRL, CYCCNT; } DWT_TypeDef;   // CYCCNT: the simulated time, 72 per us
extern DWT_TypeDef sim_dwt;
#define DWT (&sim_dwt)
extern uint32_t SystemCoreClock;
#define FLASHSIZE_BASE ((uintptr_t)&sim_flash_size_kb)
extern uint16_t sim_flash_size_kb;

#endif
