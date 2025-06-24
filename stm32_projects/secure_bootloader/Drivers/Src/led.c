/**
 * @file led.c
 * @brief Implementation of the LED driver.
 */

#include "led.h"
#include "stm32f767xx.h"

// -----------------------------------------------------------------------------
// Module-Private Constants
// -----------------------------------------------------------------------------

#define LED_GPIO_PORT       GPIOB
#define LED_RCC_AHB1ENR_BIT RCC_AHB1ENR_GPIOBEN

#define LED_GREEN_PIN       0
#define LED_BLUE_PIN        7
#define LED_RED_PIN         14

#define LED_GREEN_MASK      (1UL << LED_GREEN_PIN)
#define LED_BLUE_MASK       (1UL << LED_BLUE_PIN)
#define LED_RED_MASK        (1UL << LED_RED_PIN)
#define LED_ALL_MASK        (LED_GREEN_MASK | LED_BLUE_MASK | LED_RED_MASK)

// -----------------------------------------------------------------------------
// Public Function Implementations
// -----------------------------------------------------------------------------

void led_init(void)
{
    // 1. Enable the clock for GPIOB in the Reset and Clock Control (RCC) peripheral.
    RCC->AHB1ENR |= LED_RCC_AHB1ENR_BIT;

    // 2. Configure the LED pins as General-Purpose Outputs.
    // First, clear the mode bits for all three pins.
    LED_GPIO_PORT->MODER &= ~((3UL << (LED_GREEN_PIN * 2)) |
                              (3UL << (LED_BLUE_PIN * 2))  |
                              (3UL << (LED_RED_PIN * 2)));

    // Second, set the mode to '01' (General purpose output mode) for all three pins.
    LED_GPIO_PORT->MODER |= ((1UL << (LED_GREEN_PIN * 2)) |
                             (1UL << (LED_BLUE_PIN * 2))  |
                             (1UL << (LED_RED_PIN * 2)));

    // 3. Ensure all LEDs are off by default.
    led_off(LED_ALL);
}

void led_on(led_color_t color)
{
    // Use the atomic Bit Set/Reset Register (BSRR) to set the pin high.
    // Writing to the lower 16 bits of BSRR sets the corresponding pin.
    switch (color)
    {
        case LED_GREEN:
            LED_GPIO_PORT->BSRR = LED_GREEN_MASK;
            break;
        case LED_BLUE:
            LED_GPIO_PORT->BSRR = LED_BLUE_MASK;
            break;
        case LED_RED:
            LED_GPIO_PORT->BSRR = LED_RED_MASK;
            break;
        case LED_ALL:
            LED_GPIO_PORT->BSRR = LED_ALL_MASK;
            break;
    }
}

void led_off(led_color_t color)
{
    // Use the atomic Bit Set/Reset Register (BSRR) to set the pin low.
    // Writing to the upper 16 bits of BSRR resets the corresponding pin.
    switch (color)
    {
        case LED_GREEN:
            LED_GPIO_PORT->BSRR = (LED_GREEN_MASK << 16);
            break;
        case LED_BLUE:
            LED_GPIO_PORT->BSRR = (LED_BLUE_MASK << 16);
            break;
        case LED_RED:
            LED_GPIO_PORT->BSRR = (LED_RED_MASK << 16);
            break;
        case LED_ALL:
            LED_GPIO_PORT->BSRR = (LED_ALL_MASK << 16);
            break;
    }
}

void led_toggle(led_color_t color)
{
    uint32_t toggle_mask = 0;
    switch (color)
    {
        case LED_GREEN:
            toggle_mask = LED_GREEN_MASK;
            break;
        case LED_BLUE:
            toggle_mask = LED_BLUE_MASK;
            break;
        case LED_RED:
            toggle_mask = LED_RED_MASK;
            break;
        case LED_ALL:
            toggle_mask = LED_ALL_MASK;
            break;
    }
    LED_GPIO_PORT->ODR ^= toggle_mask;
}