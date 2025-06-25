#include "led_control.h"
#include "stm32f7xx.h"

int gpio_init(void) {
    // Enable clock for GPIOB
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    
    // Configure PB0, PB7, and PB14 as outputs
    GPIOB->MODER &= ~((3UL << (LED_GREEN_PIN * 2)) | 
                      (3UL << (LED_BLUE_PIN * 2)) | 
                      (3UL << (LED_RED_PIN * 2)));
    GPIOB->MODER |= ((1UL << (LED_GREEN_PIN * 2)) | 
                     (1UL << (LED_BLUE_PIN * 2)) | 
                     (1UL << (LED_RED_PIN * 2)));
    
    // Initialize all LEDs to off state
    GPIOB->ODR &= ~((1UL << LED_GREEN_PIN) | 
                    (1UL << LED_BLUE_PIN) | 
                    (1UL << LED_RED_PIN));
    
    return 0;
}

int toggle_led(led_state_t led) {
    uint32_t pin;
    
    switch(led) {
        case LED_GREEN:
            pin = LED_GREEN_PIN;
            break;
        case LED_BLUE:
            pin = LED_BLUE_PIN;
            break;
        case LED_RED:
            pin = LED_RED_PIN;
            break;
        default:
            return -1;
    }
    
    GPIOB->ODR ^= (1UL << pin);
    return 0;
}

int set_led(led_state_t led, uint8_t state) {
    uint32_t pin;
    
    switch(led) {
        case LED_GREEN:
            pin = LED_GREEN_PIN;
            break;
        case LED_BLUE:
            pin = LED_BLUE_PIN;
            break;
        case LED_RED:
            pin = LED_RED_PIN;
            break;
        default:
            return -1;
    }
    
    if (state) {
        GPIOB->ODR |= (1UL << pin);
    } else {
        GPIOB->ODR &= ~(1UL << pin);
    }
    
    return 0;
}

int get_led_state(led_state_t led) {
    uint32_t pin;
    
    switch(led) {
        case LED_GREEN:
            pin = LED_GREEN_PIN;
            break;
        case LED_BLUE:
            pin = LED_BLUE_PIN;
            break;
        case LED_RED:
            pin = LED_RED_PIN;
            break;
        default:
            return -1;
    }
    
    return (GPIOB->ODR & (1UL << pin)) ? 1 : 0;
} 