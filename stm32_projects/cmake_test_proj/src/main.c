#include "stm32f7xx.h"  // Only need core CMSIS header
#include "led_control.h"
#include "utils.h"

int main(void)
{
    // Initialize GPIO
    gpio_init();

    // Initialize LED state
    led_state_t current_led = LED_GREEN;

    // Main loop
    while (1)
    {
        if (toggle_led(current_led) != 0) {
            // Handle LED toggle error
            continue;
        }
        
        current_led = (current_led + 1) % LED_COUNT;
        delay_ms(50);  // delay between toggles
    }
}

