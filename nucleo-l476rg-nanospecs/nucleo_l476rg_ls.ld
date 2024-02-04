ENTRY(Reset_Handler)

MEMORY
{
  FLASH1(rx):ORIGIN =0x08000000,LENGTH =512K
  FLASH2(rx):ORIGIN =0x08080000,LENGTH =512K
  SRAM1(rwx):ORIGIN =0x20000000,LENGTH =96K
  SRAM2(rwx):ORIGIN =0x10000000,LENGTH =32K
}

_Min_Heap_Size = 0x200; /* required amount of heap */
_Min_Stack_Size = 0x400; /* required amount of stack */
_estack = ORIGIN(SRAM1) + LENGTH(SRAM1);

SECTIONS
{
  .text :
  {
    KEEP(*(.isr_vector))
    *(.text)
    *(.text.*)
    *(.init)
    *(.fini)
    *(.rodata)
    *(.rodata.*)
    . = ALIGN(4);
    _etext = .;
  }> FLASH1

  _la_data = LOADADDR(.data);

  .data :
  {
    _sdata = .;
    *(.data)
    *(.data.*)
    . = ALIGN(4);
    _edata = .;
  }> SRAM1 AT> FLASH1

  .bss :
  {
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss.*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
    . = ALIGN(4);
    end = .;
    __end__ = .;
    _end = .;
  }> SRAM1
}