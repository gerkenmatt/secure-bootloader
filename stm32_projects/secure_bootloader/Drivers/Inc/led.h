/**
 * @file led.h
 * @brief LED driver for controlling on-board LEDs.
 * @details This module provides a simple interface to initialize and control
 * the state (on, off, toggle) of the Green, Blue, and Red LEDs
 * typically found on STM32 development boards.
 */

#ifndef LED_H
#define LED_H

#include <stdint.h>

// -----------------------------------------------------------------------------
// Public Data Structures
// -----------------------------------------------------------------------------

/**
 * @brief Enumeration for the available LED colors.
 */
typedef enum {
    LED_GREEN, ///< The Green LED (typically on PB0)
    LED_BLUE,  ///< The Blue LED (typically on PB7)
    LED_RED,   ///< The Red LED (typically on PB14)
    LED_ALL    ///< A utility to control all LEDs at once
} led_color_t;

// -----------------------------------------------------------------------------
// Public Function Prototypes
// -----------------------------------------------------------------------------

/**
 * @brief Initializes the GPIO pins connected to the LEDs.
 * @details This function configures the necessary GPIO pins as outputs.
 * It must be called once before any other function in this module.
 */
void led_init(void);

/**
 * @brief Turns the specified LED on.
 * @param color The LED to turn on.
 */
void led_on(led_color_t color);

/**
 * @brief Turns the specified LED off.
 * @param color The LED to turn off.
 */
void led_off(led_color_t color);

/**
 * @brief Toggles the state of the specified LED.
 * @param color The LED to toggle.
 */
void led_toggle(led_color_t color);

#endif // LED_H