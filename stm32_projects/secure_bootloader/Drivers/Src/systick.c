#include "systick.h"
#include "stm32f767xx.h"

static volatile uint32_t systick_ms_counter = 0;

void systick_init(void) 
{
    // --- Clock Configuration ---
    uint32_t hclk_freq = 16000000; // Assume 16MHz HSI

    // Configure the SysTick to fire an interrupt every 1 millisecond.
    if (SysTick_Config(hclk_freq / 1000))
    {
        while(1); // Trap in an infinite loop on error.
    }
}

/**
 * @brief The SysTick interrupt handler, called automatically every 1ms.
 */
void SysTick_Handler(void) 
{
    systick_ms_counter++;
}

/**
 * @brief Provides the current millisecond count since startup.
 * @return The current value of the SysTick millisecond counter.
 */
uint32_t get_systick(void) 
{
    return systick_ms_counter;
}