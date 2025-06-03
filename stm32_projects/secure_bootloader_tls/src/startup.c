#include "stm32f7xx.h"

extern int main(void);

/* Symbols defined in the linker script */
extern unsigned long _estack, _etext, _sdata, _edata, _sbss, _ebss;

void Reset_Handler(void) {
    unsigned long *src = &_etext;
    unsigned long *dest = &_sdata;
    while (dest < &_edata) {
        *dest++ = *src++;
    }
    dest = &_sbss;
    while (dest < &_ebss) {
        *dest++ = 0;
    }
    main();
    while (1);
}

void Default_Handler(void) {
    while (1);
}

// void HardFault_Handler(void)
// {
//     while (1)
//     {
//         // led blinks
//         GPIOB->ODR ^= (1UL << 7); //toggle blue
//         GPIOB->ODR ^= (1UL << 14); //toggle red
//         for (volatile int i = 0; i < 1000000; i++);
//     }
// }

void HardFault_Handler(void)
{
    uint32_t hfsr = SCB->HFSR;
    uint32_t cfsr = SCB->CFSR;
    uint8_t mmfsr = (uint8_t)(cfsr & 0xFF);
    uint8_t bfsr  = (uint8_t)((cfsr >> 8) & 0xFF);
    uint16_t ufsr = (uint16_t)((cfsr >> 16) & 0xFFFF);

    // Optional: capture fault addresses if valid
    volatile uint32_t mmfar = SCB->MMFAR;
    volatile uint32_t bfar  = SCB->BFAR;

    while (1)
    {
        if (hfsr & (1 << 30)) { // Forced HardFault
            GPIOB->ODR ^= (1UL << 7);  // blue
        }

        if (bfsr) {
            for (int i = 0; i < 1; i++) {
                GPIOB->ODR ^= (1UL << 14); // red
                for (volatile int d = 0; d < 100000; d++);
                GPIOB->ODR ^= (1UL << 14);
                for (volatile int d = 0; d < 100000; d++);
            }
        } else if (mmfsr) {
            for (int i = 0; i < 2; i++) {
                GPIOB->ODR ^= (1UL << 14);
                for (volatile int d = 0; d < 100000; d++);
                GPIOB->ODR ^= (1UL << 14);
                for (volatile int d = 0; d < 100000; d++);
            }
        } else if (ufsr) {
            for (int i = 0; i < 3; i++) {
                GPIOB->ODR ^= (1UL << 14);
                for (volatile int d = 0; d < 100000; d++);
                GPIOB->ODR ^= (1UL << 14);
                for (volatile int d = 0; d < 100000; d++);
            }
        }

        for (volatile int i = 0; i < 1000000; i++);
    }
}

/* Minimal vector table placed in the .isr_vector section */
__attribute__ ((section(".isr_vector")))
void (* const vector_table[])(void) = {
    (void (*)(void)) &_estack,  // Initial Stack Pointer
    Reset_Handler,             // Reset Handler
    Default_Handler,           // NMI
    HardFault_Handler,         // Hard Fault
    HardFault_Handler,         // MemManage
    HardFault_Handler,         // BusFault
    HardFault_Handler,         // UsageFault
    0, 0, 0, 0,                // Reserved
    Default_Handler,           // SVCall
    Default_Handler,           // Debug Monitor
    0,                         // Reserved
    Default_Handler,           // PendSV
    Default_Handler            // SysTick
    /* Add additional interrupt handlers as needed */
};

