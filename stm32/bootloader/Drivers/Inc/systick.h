#ifndef SYSTICK_H
#define SYSTICK_H

#include <stdint.h>

// -----------------------------------------------------------------------------
// Public Function Prototypes
// -----------------------------------------------------------------------------

/**
 * @brief Initializes the SysTick timer for millisecond timing.
 * This function sets up the SysTick timer to generate an interrupt every 1 millisecond.
 */
void systick_init(void);

/**
 * @brief Provides the current millisecond count since startup.
 * @return The current value of the SysTick millisecond counter.
 */
uint32_t get_systick(void);


#endif // SYSTICK_H