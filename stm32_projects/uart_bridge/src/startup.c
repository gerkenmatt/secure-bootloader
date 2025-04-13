extern int main(void);

/* Symbols defined in the linker script */
extern unsigned long _estack, _etext, _sdata, _edata, _sbss, _ebss;

void USART6_IRQHandler(void);

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
void Dummy_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));

/* Minimal vector table placed in the .isr_vector section */
__attribute__ ((section(".isr_vector")))
void (* const vector_table[])(void) = {
    (void (*)(void)) &_estack,  //  0: Initial Stack Pointer
    Reset_Handler,              //  1: Reset Handler
    Default_Handler,            //  2: NMI
    Default_Handler,            //  3: Hard Fault
    Default_Handler,            //  4: MemManage
    Default_Handler,            //  5: BusFault
    Default_Handler,            //  6: UsageFault
    0, 0, 0, 0,                 //  7–10: Reserved
    Default_Handler,            // 11: SVCall
    Default_Handler,            // 12: Debug Monitor
    0,                          // 13: Reserved
    Default_Handler,            // 14: PendSV
    Default_Handler,            // 15: SysTick

    // External Interrupts (start from position 16)
    [87] = USART6_IRQHandler,     // Position 98 = IRQ number 82 for USART6
    [99] = Dummy_IRQHandler  // <-- forces vector table size
};
