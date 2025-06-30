#include "stm32f767xx.h"


// magic number and the backup register to use for signaling boot success
#define BOOT_SUCCESS_MAGIC      0xDEADBEEF
#define BOOT_SUCCESS_BKP_REG    RTC_BKP_DR1 // Use Backup Register 1


// This global variable will be incremented by the SysTick ISR.
static volatile uint32_t systick_ms_counter = 0;

/**
 * @brief Signals to the bootloader that the application has booted successfully.
 */
void signal_boot_success(void)
{
    // 1. Enable the Power Interface Controller (PWR) clock.
    // The DBP (Disable Backup Protection) bit is in the PWR domain.
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;

    // 2. Disable backup domain protection.
    // This allows writing to the RTC backup registers.
    PWR->CR1 |= PWR_CR1_DBP;

    // It's good practice to wait until the bit is actually set.
    while ((PWR->CR1 & PWR_CR1_DBP) == 0);

    // 3. Write the magic number directly to Backup Register 1 (BKP1R).
    // The RTC peripheral does not need to be clocked or running for this.
    RTC->BKP1R = BOOT_SUCCESS_MAGIC;

    // 4. (Optional) Re-enable backup domain protection.
    PWR->CR1 &= ~PWR_CR1_DBP;
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

    signal_boot_success();

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