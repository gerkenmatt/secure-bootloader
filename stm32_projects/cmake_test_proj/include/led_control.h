#ifndef LED_CONTROL_H
#define LED_CONTROL_H

#include <stdint.h>

/**
 * @brief LED pin definitions for GPIOB
 */
#define LED_GREEN_PIN    0   // PB0
#define LED_BLUE_PIN     7   // PB7
#define LED_RED_PIN      14  // PB14

/**
 * @brief Enumeration of available LED states
 */
typedef enum {
    LED_GREEN = 0,  /**< Green LED on PB0 */
    LED_BLUE,       /**< Blue LED on PB7 */
    LED_RED,        /**< Red LED on PB14 */
    LED_COUNT
} led_state_t;

/**
 * @brief Initialize GPIO pins for LED control
 * @return 0 on success, -1 on failure
 */
int gpio_init(void);

/**
 * @brief Toggle the specified LED
 * @param led The LED to toggle (LED_GREEN, LED_BLUE, or LED_RED)
 * @return 0 on success, -1 on invalid LED
 */
int toggle_led(led_state_t led);

/**
 * @brief Set the state of a specific LED
 * @param led The LED to control
 * @param state 1 to turn on, 0 to turn off
 * @return 0 on success, -1 on failure
 */
int set_led(led_state_t led, uint8_t state);

/**
 * @brief Get the current state of an LED
 * @param led The LED to check
 * @return 1 if LED is on, 0 if LED is off, -1 on error
 */
int get_led_state(led_state_t led);

#endif /* LED_CONTROL_H */ 