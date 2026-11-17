// Stand-ins for the drivers that touch timers, UART, flash and the clock; the others (gpio.c, infrared.c) are the
// firmware's own, on the fake registers of sim_chip.c.
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "stm32f1xx_hal.h"
#include "sim_api.h"
#include "encoder.h"
#include "error.h"
#include "flash_store.h"
#include "health.h"
#include "motor.h"
#include "pwm.h"
#include "sysclock.h"
#include "uart.h"

// ---- Motors, encoders ---------------------------------------------------------------------------------
static int16_t requested[2];
static int32_t enc_total[2];
static uint32_t idle_since;

void PWM_Init(void){}
void pwm_set(uint32_t channel, uint16_t duty){ (void)channel; (void)duty; }
void MOTOR_Init(void){}
void motor_set(motor_t motor, int16_t pwm){
    if(pwm > PWM_MAX) pwm = PWM_MAX;
    if(pwm < -PWM_MAX) pwm = -PWM_MAX;
    requested[motor] = pwm;
    TIM4->CCR3 = TIM4->CCR4 = 1u;   // driven: motor_emergency_stop() zeroes them
}
int16_t motor_get(motor_t motor){ return requested[motor]; }
void sim_get_pwm(int16_t *left, int16_t *right){
    const uint8_t stopped = TIM4->CCR3 == 0u && TIM4->CCR4 == 0u;
    *left = stopped ? 0 : requested[MOTOR_L];
    *right = stopped ? 0 : requested[MOTOR_R];
}

void ENCODER_Init(void){}
void encoder_tick(void){}
void sim_set_encoders(int32_t left, int32_t right){
    if(left != enc_total[ENCODER_L] || right != enc_total[ENCODER_R]) idle_since = HAL_GetTick();
    enc_total[ENCODER_L] = left;
    enc_total[ENCODER_R] = right;
}
int32_t encoder_total(encoder_t e){ return enc_total[e]; }
uint16_t encoder_raw(encoder_t e){ return (uint16_t)enc_total[e]; }
uint32_t encoder_idle_ms(void){ return HAL_GetTick() - idle_since; }

// ---- UART: lines in, text out, never full (the real one drops messages beyond its queue at 9600 baud) --------
#define RX_LINES 16
#define RX_LEN 64
#define TX_BYTES (256 * 1024)
static char rx[RX_LINES][RX_LEN];
static unsigned rx_head, rx_count;
static char tx[TX_BYTES];
static uint32_t tx_len;

void UART_Init(void){}
void uart_start_receive(void){}
void uart_retime(void){}
void uart_waiting(void){}
void uart_send(const char *text){
    const size_t n = strlen(text);
    if(tx_len + n <= TX_BYTES){
        memcpy(tx + tx_len, text, n);
        tx_len += (uint32_t)n;
    }
}
void print(const char *format, ...){
    char text[512];
    va_list args;
    va_start(args, format);
    vsnprintf(text, sizeof(text), format, args);
    va_end(args);
    uart_send(text);
}
uint8_t uart_tx_idle(void){ return 1; }
uint8_t uart_tx_full(void){ return 0; }
void uart_flush(uint32_t timeout_ms){ (void)timeout_ms; }
void uart_wait_space(uint32_t timeout_ms){ (void)timeout_ms; }
uint8_t uart_read_line(char *buffer, uint8_t size){
    if(!rx_count) return 0;
    strncpy(buffer, rx[rx_head], (size_t)size - 1u);
    buffer[size - 1u] = '\0';
    rx_head = (rx_head + 1u) % RX_LINES;
    rx_count--;
    return 1;
}
void sim_uart_rx(const char *line){
    if(rx_count == RX_LINES) return;
    char *slot = rx[(rx_head + rx_count) % RX_LINES];
    strncpy(slot, line, RX_LEN - 1u);
    slot[RX_LEN - 1u] = '\0';
    rx_count++;
}
uint32_t sim_uart_tx(char *buffer, uint32_t size){
    const uint32_t n = tx_len < size ? tx_len : size;
    memcpy(buffer, tx, n);
    memmove(tx, tx + n, tx_len - n);
    tx_len -= n;
    return n;
}

// ---- Flash: the last two pages, kept by the simulator in a file between runs ------------------------------
static uint8_t flash[FLASH_STORE_SIZE];
static uint32_t flash_writes;
static flash_timing_t timing;
static uint8_t flash_loaded;

void sim_flash_load(const uint8_t *data, uint32_t size){
    memset(flash, 0xFF, sizeof(flash));
    memcpy(flash, data, size < sizeof(flash) ? size : sizeof(flash));
    flash_loaded = 1;
}
const uint8_t *sim_flash_image(void){ return flash; }
uint32_t sim_flash_writes(void){ return flash_writes; }
const void *flash_store_data(void){
    if(!flash_loaded) sim_flash_load(NULL, 0);
    return flash;
}
uint8_t flash_store_blocked(void){ return 0; }
const flash_timing_t *flash_store_timing(void){ return &timing; }
uint8_t flash_store_erase(uint8_t page){
    if(page >= FLASH_STORE_PAGES) return 0;
    memset(flash + page * FLASH_STORE_PAGE_SIZE, 0xFF, FLASH_STORE_PAGE_SIZE);
    flash_writes++;
    return 1;
}
uint8_t flash_store_program(uint16_t offset, const void *data, uint16_t len){
    if((offset & 1u) || (len & 1u) || offset + len > FLASH_STORE_SIZE) return 0;
    for(uint16_t i = 0; i < len; i++){
        if(flash[offset + i] != 0xFFu) return 0;    // only erased flash can be programmed
    }
    memcpy(flash + offset, data, len);
    flash_writes++;
    return 1;
}

// ---- Clock, health, errors -------------------------------------------------------------------------------
static const char *reset_cause = "power-on";
void sim_set_reset_cause(const char *cause){ reset_cause = cause; }
void SystemClock_Config(void){}
clock_source_t sysclock_source(void){ return CLOCK_HSE; }
uint8_t sysclock_recover(void){ return 0; }
void health_init(void){}
void health_alive(void){}
const char *health_reset_cause(void){ return reset_cause; }
uint32_t health_stack_free(void){ return 0; }
uint32_t health_tick_us_max(void){ return 0; }
uint8_t health_take_stall(uint32_t *ms, uint32_t *pc, uint32_t *lr){ (void)ms; (void)pc; (void)lr; return 0; }
void Error_Handler(void){
    motor_emergency_stop();
    print("!! Error_Handler: the firmware stopped\n");
    sim_halt(SIM_FAULT);
}
