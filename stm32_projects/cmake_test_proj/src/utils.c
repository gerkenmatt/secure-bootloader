#include "utils.h"
#include "stm32f7xx.h"

// Simple delay function using core cycles
void delay_ms(uint32_t ms)
{
    // At 16MHz, each cycle is 62.5ns
    // So for 1ms we need 16000 cycles
    volatile uint32_t cycles = 16000 * ms;
    while (cycles--) {
        __NOP(); // No operation - prevents optimization
    }
} 