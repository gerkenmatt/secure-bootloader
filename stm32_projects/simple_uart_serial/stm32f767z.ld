/* Linker script for STM32F767ZIT Application */

/* Default to 0x08000000 if FLASH_ADDR is not defined */
__FLASH_ORIGIN = DEFINED(__FLASH_ORIGIN) ? __FLASH_ORIGIN : 0x08000000;

MEMORY
{
    FLASH (rx)      : ORIGIN =  __FLASH_ORIGIN, LENGTH = 512K   /* Full flash */
    RAM (xrw)       : ORIGIN = 0x20000000, LENGTH = 512K    /* Full RAM */
}

/* Highest address of the user mode stack */
_estack = ORIGIN(RAM) + LENGTH(RAM);

/* Entry Point */
ENTRY(Reset_Handler)
SECTIONS
{
    .isr_vector : {
        . = ALIGN(4);
        KEEP(*(.isr_vector))
        . = ALIGN(4);
    } >FLASH

    .text : {
        . = ALIGN(4);
        *(.text)
        *(.text*)
        *(.init)
        *(.fini)
        . = ALIGN(4);
        _etext = .;
    } >FLASH

    .rodata : {
        . = ALIGN(4);
        *(.rodata)
        *(.rodata*)
        . = ALIGN(4);
    } >FLASH

    .data : AT (_etext) {
        . = ALIGN(4);
        _sdata = .;
        *(.data)
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    } >RAM

    .bss : {
        . = ALIGN(4);
        _sbss = .;
        *(.bss)
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
    } >RAM

    /* Set stack top to end of RAM */
    __StackTop = ORIGIN(RAM) + LENGTH(RAM);
    __StackLimit = __StackTop - 0x1000;
    PROVIDE(_estack = __StackTop);
}