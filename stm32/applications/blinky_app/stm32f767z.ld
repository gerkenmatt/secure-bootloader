/* Linker script for STM32F767ZIT */

/*
 * Symbols defined in Makefile:
 * __FLASH_ORIGIN: Start address of FLASH
 * __FLASH_LENGTH: Total length of FLASH
 *
 * This script uses these symbols.
 */

/* Default to 0x08000000 if __FLASH_ORIGIN is not defined by Makefile */
__FLASH_ORIGIN = DEFINED(__FLASH_ORIGIN) ? __FLASH_ORIGIN : 0x08000000;
/* Default to 1024K if __FLASH_LENGTH is not defined by Makefile */
__FLASH_LENGTH = DEFINED(__FLASH_LENGTH) ? __FLASH_LENGTH : 1024K;

MEMORY
{
  /* Use __FLASH_LENGTH passed from Makefile */
  FLASH (rx)      : ORIGIN =  __FLASH_ORIGIN, LENGTH = __FLASH_LENGTH
  RAM (xrw)       : ORIGIN = 0x20000000, LENGTH = 512K  /* Full RAM, adjust if needed */
}

/* Entry Point */
ENTRY(Reset_Handler)

SECTIONS
{
  .isr_vector :
  {
    KEEP(*(.isr_vector)) /* Keep the interrupt vector table */
  } > FLASH

  .text :
  {
    *(.text*)           /* .text sections (code) */
    *(.rodata*)         /* .rodata sections (constants, strings, etc.) */
    /* _etext will be defined after our custom padding section */
  } > FLASH

  /* _etext marks the end of the data that is initialized in FLASH */
  /* It now includes the .text, .rodata, and our .custom_padding section */
  _etext = .; /* End of .text, .rodata, and .custom_padding sections in FLASH */

  /* .data section (initialized data) */
  /* LMA (Load Memory Address) is in FLASH, immediately after _etext */
  /* VMA (Virtual Memory Address) is in RAM */
  .data : AT (_etext)
  {
    _sdata = .;         /* Start of .data in RAM */
    *(.data*)           /* .data sections */
    _edata = .;         /* End of .data in RAM */
  } > RAM

  /* .bss section (uninitialized data) */
  /* This section occupies space in RAM but not in the FLASH image (.bin file) */
  .bss :
  {
    _sbss = .;          /* Start of .bss in RAM */
    *(.bss*)
    *(COMMON)
    _ebss = .;          /* End of .bss in RAM */
  } > RAM

  /* Define stack pointer to be at the top of RAM */
  /* Ensure it doesn't collide with other RAM sections like .data or .bss */
  _estack = ORIGIN(RAM) + LENGTH(RAM); /* Top of stack */

}