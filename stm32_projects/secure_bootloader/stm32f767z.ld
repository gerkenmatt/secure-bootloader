/* Linker script for STM32F767ZIT */

/* Memory regions for STM32F767ZI A/B Partition Swap
 * These values should be kept in sync with bootloader.h
 */
MEMORY
{
    /* Bootloader Region (128KB). Contains bootloader code . */
    /* Sectors 0-3 @ 32KB each = 128KB */
    BOOTLOADER_FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 128K

    /* Application Slot A (768KB). One of two swappable application slots. */
    /* Sectors 5, 6, 7 @ 256KB each */
    SLOTA (rx)            : ORIGIN = 0x08040000, LENGTH = 768K

    /* Application Slot B (768KB). The second swappable application slot. */
    /* Sectors 8, 9, 10 @ 256KB each */
    SLOTB (rx)            : ORIGIN = 0x08100000, LENGTH = 768K

    BOOT_CONFIG (rx)      : ORIGIN = 0x081C0000, LENGTH = 256K 

    /* RAM Regions (remain unchanged) */
    DTCM_RAM (xrw)        : ORIGIN = 0x20000000, LENGTH = 128K
    RAM (xrw)             : ORIGIN = 0x20020000, LENGTH = 384K
    ITCM_RAM (xrw)        : ORIGIN = 0x00000000, LENGTH = 16K
}

/* Highest address of the user mode stack */
_estack = ORIGIN(RAM) + LENGTH(RAM);

/* Entry Point */
ENTRY(Reset_Handler)

SECTIONS
{
    /* Bootloader code and data */
    .text :
    {
        . = ALIGN(4);
        _stext = .;
        KEEP(*(.isr_vector))
        *(.text)
        *(.text*)
        *(.rodata)
        *(.rodata*)
        . = ALIGN(4);
        _etext = .;
        _sidata = .;    /* Start of initialized data in flash */
    } >BOOTLOADER_FLASH

    /* Bootloader configuration section (last page of bootloader) */
    .bootloader_config :
    {
        . = ALIGN(4);
        _sboot_config = .;
        KEEP(*(.bootloader_config))
        . = ALIGN(4);
        _eboot_config = .;
    } >BOOTLOADER_FLASH

    /* Data section */
    .data :
    {
        . = ALIGN(4);
        _sdata = .;        /* Start of data section in RAM */
        *(.data)
        *(.data*)
        . = ALIGN(4);
        _edata = .;        /* End of data section in RAM */
    } >RAM AT>BOOTLOADER_FLASH

    /* BSS section */
    .bss :
    {
        . = ALIGN(4);
        _sbss = .;         /* Start of BSS section */
        *(.bss)
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;         /* End of BSS section */
        _end  = .;         /* C library expects this symbol */
    } >RAM

    /* Heap section (grows upward) */
    _heap_start = _end;
    _stack_size = 0x4000; /* Define stack size */
    _heap_end = ORIGIN(RAM) + LENGTH(RAM) - _stack_size; 

    /* Define stack symbols based on top of RAM */
    __StackLimit = _heap_end; /* Lowest stack address */
    __StackTop = ORIGIN(RAM) + LENGTH(RAM); /* Initial SP */

    _sstack = __StackLimit;
    _estack = __StackTop; /* This should be used for initial SP */

    . = _heap_end; /* Move current location counter to start of stack area */

    ._user_stack (NOLOAD) : /* NOLOAD if it's just for SP and bounds, not initialized data */
    {
        . = ALIGN(8);
        PROVIDE(_sstack = .);
        . = . + _stack_size;
        . = ALIGN(8);
        PROVIDE(_estack = .); /* This _estack would be 0x20080000 */
    } >RAM

    /* Remove unused sections */
    /*/DISCARD/ :
    {
        libc.a ( * )
        libm.a ( * )
        libgcc.a ( * )
    }*/
}

