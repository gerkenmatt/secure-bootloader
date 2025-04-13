/* Linker script for STM32F767ZIT */

/* Default to 0x08000000 if FLASH_ADDR is not defined */
__FLASH_ORIGIN = DEFINED(__FLASH_ORIGIN) ? __FLASH_ORIGIN : 0x08000000;

MEMORY
{
    FLASH (rx)      : ORIGIN =  __FLASH_ORIGIN, LENGTH = 512K   /* just 512KB for now */
    RAM (xrw)       : ORIGIN = 0x20000000, LENGTH = 512K    /* Full RAM */
}
/* Entry Point */
ENTRY(Reset_Handler)
SECTIONS
{
  .isr_vector :
  {
    KEEP(*(.isr_vector))
  } > FLASH

  .text :
  {
    *(.text*)
    *(.rodata*)
    _etext = .;
  } > FLASH

  .data : AT (_etext)
  {
    _sdata = .;
    *(.data*)
    _edata = .;
  } > RAM

  .bss :
  {
    _sbss = .;
    *(.bss*)
    *(COMMON)
    _ebss = .;
  } > RAM

  _estack = ORIGIN(RAM) + LENGTH(RAM);
}

