#include "stm32f7xx.h"  // Ensure correct include path for STM32F7 headers

void delay(volatile uint32_t count) {
    while (count--);
}

int main(void) {
    int n = 0;
    /* Enable clock for GPIOB */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    /* Configure PB0, PB7, and PB14 as outputs */
    GPIOB->MODER &= ~((3UL << (0 * 2)) | (3UL << (7 * 2)) | (3UL << (14 * 2)));
    GPIOB->MODER |= ((1UL << (0 * 2)) | (1UL << (7 * 2)) | (1UL << (14 * 2)));

    while (1) {
        /* Toggle PB0, PB7, and PB14 */
        //GPIOB->ODR ^= ((1UL << 0) | (1UL << 7) | (1UL << 14));
	if (!n)
	{	
		//TOGGLE GREEN
		GPIOB->ODR ^= (1UL << 0);
		n++;
	}
	else if(n == 1)
	{
		//TOGGLE BLUE
		GPIOB->ODR ^= (1UL << 7);
		n++;
		
	}
	else if (n == 2)
	{
		//TOGGLE RED
		GPIOB->ODR ^= (1UL << 14);
		n = 0;
	}
        delay(300000);
    }
    return 0;
}

