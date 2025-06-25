#include "stm32f767xx.h"

// --- SysTick Globals and Functions ---

// This global variable will be incremented by the SysTick ISR.
static volatile uint32_t systick_ms_counter = 0;

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

/**
 * @brief A precise delay function based on the SysTick timer.
 * @param ms The number of milliseconds to delay.
 */
void delay_ms(uint32_t ms) 
{
    uint32_t start_tick = get_systick();
    while ((get_systick() - start_tick) < ms);
}

/**
 * @brief Initializes the system clock and peripherals.
 */
static void system_init(void)
{
    // --- Clock Configuration ---
    uint32_t hclk_freq = 16000000; // Assume 16MHz HSI

    // --- SysTick Configuration ---
    // Configure the SysTick to fire an interrupt every 1 millisecond.
    if (SysTick_Config(hclk_freq / 1000))
    {
        while(1); // Trap in an infinite loop on error.
    }

    // --- GPIO Clock Enable ---
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
}

// --- Main Entry Point ---
int main(void) 
{
    int n = 0;
    
    // Initialize system clocks, SysTick, and GPIO.
    system_init();

    // Configure PB0 (Green), PB7 (Blue), and PB14 (Red) as outputs.
    GPIOB->MODER &= ~((3UL << (0 * 2)) | (3UL << (7 * 2)) | (3UL << (14 * 2)));
    GPIOB->MODER |= ((1UL << (0 * 2)) | (1UL << (7 * 2)) | (1UL << (14 * 2)));

    while (1) 
	{
        if (!n) 
		{
            GPIOB->ODR ^= (1UL << 0); // Toggle Green
            n++;
        } 
		else if(n == 1) 
		{
            GPIOB->ODR ^= (1UL << 7); // Toggle Blue
            n++;
        } 
		else if (n == 2) 
		{
            GPIOB->ODR ^= (1UL << 14); // Toggle Red
            n = 0;
        }
        
        delay_ms(50); // Delay for 50 milliseconds.
    }
    return 0;
}