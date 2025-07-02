/**
 * @file startup_stm32f767xx.c
 * @brief CMSIS-compliant startup file for the STM32F767xx microcontroller.
 *
 * This file contains the vector table, the Reset_Handler, and default
 * handlers for system exceptions and interrupts. It initializes the .data
 * and .bss sections and then calls the main application entry point.
 */

#include <stdint.h>
#include "stm32f767xx.h"

// -----------------------------------------------------------------------------
// Forward Declarations 
// -----------------------------------------------------------------------------

extern int main(void);
extern void USART6_IRQHandler(void);
extern void SysTick_Handler(void);

/* Symbols defined in the linker script */
// These symbols are provided by the linker script (.ld file) and represent
// the start and end addresses of different memory sections.
extern unsigned long _estack, _etext, _sdata, _edata, _sbss, _ebss;


// -----------------------------------------------------------------------------
// Handler Definitions
// -----------------------------------------------------------------------------

void Reset_Handler(void)
{
    // Copy initialized data from Flash to RAM (.data section).
    // The linker script places initialized data in Flash starting at `_etext` (end of text section)
    // and its RAM destination starts at `_sdata`.
    unsigned long *src = &_etext;
    unsigned long *dest = &_sdata;
    while (dest < &_edata)
    {
        *dest++ = *src++;
    }

    // Zero-fill the uninitialized data section (.bss section).
    // This ensures that global and static variables not explicitly initialized
    // by the programmer start with a known value of zero.
    dest = &_sbss;
    while (dest < &_ebss)
    {
        *dest++ = 0;
    }

    // Call the main application entry point.
    // This transfers control from the startup code to the application's C main function.
    main();

    // This infinite loop acts as a fallback if `main` ever exits, preventing unpredictable behavior.
    while (1);
}

void Default_Handler(void) 
{
    while (1);
}

void HardFault_Handler(void)
{
    // uint32_t hfsr = SCB->HFSR;
    // uint32_t cfsr = SCB->CFSR;
    // uint8_t mmfsr = (uint8_t)(cfsr & 0xFF);
    // uint8_t bfsr  = (uint8_t)((cfsr >> 8) & 0xFF);
    // uint16_t ufsr = (uint16_t)((cfsr >> 16) & 0xFFFF);

    while (1)
    {
        for (volatile int i = 0; i < 1000000; i++);
    }
}

// -----------------------------------------------------------------------------
// Main Vector Table
// -----------------------------------------------------------------------------

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
    SysTick_Handler,           // SysTick

    /* External Interrupts - Add handlers as needed */
    Default_Handler,           // IRQ 0: WWDG
    Default_Handler,           // IRQ 1: PVD
    Default_Handler,           // IRQ 2: TAMP_STAMP
    Default_Handler,           // IRQ 3: RTC_WKUP
    Default_Handler,           // IRQ 4: FLASH
    Default_Handler,           // IRQ 5: RCC
    Default_Handler,           // IRQ 6: EXTI0
    Default_Handler,           // IRQ 7: EXTI1
    Default_Handler,           // IRQ 8: EXTI2
    Default_Handler,           // IRQ 9: EXTI3
    Default_Handler,           // IRQ 10: EXTI4
    Default_Handler,           // IRQ 11: DMA1_Stream0
    Default_Handler,           // IRQ 12: DMA1_Stream1
    Default_Handler,           // IRQ 13: DMA1_Stream2
    Default_Handler,           // IRQ 14: DMA1_Stream3
    Default_Handler,           // IRQ 15: DMA1_Stream4
    Default_Handler,           // IRQ 16: DMA1_Stream5
    Default_Handler,           // IRQ 17: DMA1_Stream6
    Default_Handler,           // IRQ 18: ADC
    Default_Handler,           // IRQ 19: CAN1_TX
    Default_Handler,           // IRQ 20: CAN1_RX0
    Default_Handler,           // IRQ 21: CAN1_RX1
    Default_Handler,           // IRQ 22: CAN1_SCE
    Default_Handler,           // IRQ 23: EXTI9_5
    Default_Handler,           // IRQ 24: TIM1_BRK_TIM9
    Default_Handler,           // IRQ 25: TIM1_UP_TIM10
    Default_Handler,           // IRQ 26: TIM1_TRG_COM_TIM11
    Default_Handler,           // IRQ 27: TIM1_CC
    Default_Handler,           // IRQ 28: TIM2
    Default_Handler,           // IRQ 29: TIM3
    Default_Handler,           // IRQ 30: TIM4
    Default_Handler,           // IRQ 31: I2C1_EV
    Default_Handler,           // IRQ 32: I2C1_ER
    Default_Handler,           // IRQ 33: I2C2_EV
    Default_Handler,           // IRQ 34: I2C2_ER
    Default_Handler,           // IRQ 35: SPI1
    Default_Handler,           // IRQ 36: SPI2
    Default_Handler,           // IRQ 37: USART1
    Default_Handler,           // IRQ 38: USART2
    Default_Handler,           // IRQ 39: USART3
    Default_Handler,           // IRQ 40: EXTI15_10
    Default_Handler,           // IRQ 41: RTC_Alarm
    Default_Handler,           // IRQ 42: OTG_FS_WKUP
    Default_Handler,           // IRQ 43: TIM8_BRK_TIM12
    Default_Handler,           // IRQ 44: TIM8_UP_TIM13
    Default_Handler,           // IRQ 45: TIM8_TRG_COM_TIM14
    Default_Handler,           // IRQ 46: TIM8_CC
    Default_Handler,           // IRQ 47: DMA1_Stream7
    Default_Handler,           // IRQ 48: FMC
    Default_Handler,           // IRQ 49: SDMMC1
    Default_Handler,           // IRQ 50: TIM5
    Default_Handler,           // IRQ 51: SPI3
    Default_Handler,           // IRQ 52: UART4
    Default_Handler,           // IRQ 53: UART5
    Default_Handler,           // IRQ 54: TIM6_DAC
    Default_Handler,           // IRQ 55: TIM7
    Default_Handler,           // IRQ 56: DMA2_Stream0
    Default_Handler,           // IRQ 57: DMA2_Stream1
    Default_Handler,           // IRQ 58: DMA2_Stream2
    Default_Handler,           // IRQ 59: DMA2_Stream3
    Default_Handler,           // IRQ 60: DMA2_Stream4
    Default_Handler,           // IRQ 61: ETH
    Default_Handler,           // IRQ 62: ETH_WKUP
    Default_Handler,           // IRQ 63: CAN2_TX
    Default_Handler,           // IRQ 64: CAN2_RX0
    Default_Handler,           // IRQ 65: CAN2_RX1
    Default_Handler,           // IRQ 66: CAN2_SCE
    Default_Handler,           // IRQ 67: OTG_FS
    Default_Handler,           // IRQ 68: DMA2_Stream5
    Default_Handler,           // IRQ 69: DMA2_Stream6
    Default_Handler,           // IRQ 70: DMA2_Stream7
    USART6_IRQHandler          // IRQ 71: USART6 
};
