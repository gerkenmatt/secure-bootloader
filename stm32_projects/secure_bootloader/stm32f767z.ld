/* Linker script for STM32F767ZIT */

/* Memory regions for STM32F767ZI */
MEMORY
{
    /* Bootloader region (32KB) */
    BOOTLOADER_FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 128K
    
    /* Application region (512KB) */
    APP_FLASH (rx)       : ORIGIN = 0x08040000, LENGTH = 512K
    
    /* RAM regions */
    DTCM_RAM (xrw)      : ORIGIN = 0x20000000, LENGTH = 128K    /* Data TCM RAM */
    RAM (xrw)           : ORIGIN = 0x20020000, LENGTH = 384K    /* SRAM1 + SRAM2 */
    ITCM_RAM (xrw)      : ORIGIN = 0x00000000, LENGTH = 16K     /* Instruction TCM RAM */
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

    /* Stack section */
    ._user_stack :
    {
        . = ALIGN(8);
        _sstack = .;
        . = . + 0x1000;    /* 4KB of stack */
        . = ALIGN(8);
        _estack = .;
    } >RAM

    /* Remove unused sections */
    /*/DISCARD/ :
    {
        libc.a ( * )
        libm.a ( * )
        libgcc.a ( * )
    }*/
}

