/* Linker script for STM32F767ZIT */

/* Memory regions for STM32F767ZI
 * These values should be kept in sync with bootloader.h
 */
MEMORY
{
    /* Bootloader Reserved Region (256KB total) */
    /* Physical Flash is aliased at 0x00200000 for faster instruction fetch via ITCM */
    BOOTLOADER_FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 256K
    
    /* Active Application Region (256KB) */
    APP_ACTIVE (rx)       : ORIGIN = 0x08040000, LENGTH = 256K

    /* Firmware Storage Slots (512KB each) */
    SLOT0 (rx)            : ORIGIN = 0x08080000, LENGTH = 512K  /* Sector 6 & 7 */
    SLOT1 (rx)            : ORIGIN = 0x080C0000, LENGTH = 512K  /* Sector 8 & 9 */
    
    /* RAM Regions */
    DTCM_RAM (xrw)        : ORIGIN = 0x20000000, LENGTH = 128K  /* Data TCM RAM */
    RAM (xrw)             : ORIGIN = 0x20020000, LENGTH = 384K  /* SRAM1 + SRAM2 */
    ITCM_RAM (xrw)        : ORIGIN = 0x00000000, LENGTH = 16K   /* Instruction TCM RAM (SRAM, not the flash alias) */
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

