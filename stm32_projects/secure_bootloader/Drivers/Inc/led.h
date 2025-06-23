/**
 * @file led.h
 * @brief LED control interface for STM32 bootloader.
 */
#ifndef LED_H
#define LED_H

// -----------------------------------------------------------------------------
// Function Prototypes
// -----------------------------------------------------------------------------

/**
 * @brief Initializes the LED GPIO(s).
 */
void led_init(void);

/**
 * @brief Turns the LED on.
 */
void led_on(void);

/**
 * @brief Turns the LED off.
 */
void led_off(void);

/**
 * @brief Toggles the LED state.
 */
void led_toggle(void);

#endif // LED_H
