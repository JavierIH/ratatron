// What the simulator (ratsim/firmware.py, through ctypes) calls in the firmware library.
#ifndef SIM_API_H
#define SIM_API_H

#include <stdint.h>

enum { SIM_RUNNING = 0, SIM_RESET = 1, SIM_HALTED = 2, SIM_FAULT = 3 };

int sim_boot(void);                         // runs the boot until main() first waits
uint8_t sim_step(void);                     // one millisecond: SysTick, then main() until it waits; the state
uint32_t sim_time_ms(void);
void sim_halt(uint8_t why) __attribute__((noreturn));

void sim_set_buttons(uint8_t start, uint8_t select);
uint8_t sim_leds(void);
void sim_set_adc(uint8_t channel, float counts);
void sim_set_encoders(int32_t left, int32_t right);    // ticks since boot, forward positive
void sim_get_pwm(int16_t *left, int16_t *right);        // as motor_set() asked, 0 after an emergency stop

void sim_uart_rx(const char *line);         // one console line (without the newline)
uint32_t sim_uart_tx(char *buffer, uint32_t size);     // takes what the firmware sent

void sim_flash_load(const uint8_t *data, uint32_t size);
const uint8_t *sim_flash_image(void);
uint32_t sim_flash_writes(void);            // changes since boot: the simulator saves the image when it moves
void sim_set_reset_cause(const char *cause);

#endif
