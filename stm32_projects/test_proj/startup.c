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

/* Minimal vector table placed in the .isr_vector section */
__attribute__ ((section(".isr_vector")))
void (* const vector_table[])(void) = {
    (void (*)(void)) &_estack,  // Initial Stack Pointer
    Reset_Handler,             // Reset Handler
    Default_Handler,           // NMI
    Default_Handler,           // Hard Fault
    Default_Handler,           // MemManage
    Default_Handler,           // BusFault
    Default_Handler,           // UsageFault
    0, 0, 0, 0,                // Reserved
    Default_Handler,           // SVCall
    Default_Handler,           // Debug Monitor
    0,                         // Reserved
    Default_Handler,           // PendSV
    Default_Handler            // SysTick
    /* Add additional interrupt handlers as needed */
};

