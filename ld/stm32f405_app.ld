MEMORY {
  FLASH  (rx)  : ORIGIN = 0x08000000, LENGTH = 1024K
  RAM    (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
  CCMRAM (rwx) : ORIGIN = 0x10000000, LENGTH = 64K
}
ENTRY(Reset_Handler)
_estack = ORIGIN(RAM) + LENGTH(RAM);

SECTIONS {
  .isr_vector : { KEEP(*(.isr_vector)) } > FLASH
  .text : {
    *(.text.Reset_Handler)
    *(.text*) *(.rodata*) KEEP(*(.init)) KEEP(*(.fini))
  } > FLASH
  .ARM.extab : { *(.ARM.extab*) } > FLASH
  .ARM.exidx : { *(.ARM.exidx*) } > FLASH

  .preinit_array : { __preinit_array_start = .; KEEP(*(.preinit_array*)) __preinit_array_end = .; } > FLASH
  .init_array    : { __init_array_start = .; KEEP(*(SORT(.init_array.*))) KEEP(*(.init_array*)) __init_array_end = .; } > FLASH
  .fini_array    : { __fini_array_start = .; KEEP(*(SORT(.fini_array.*))) KEEP(*(.fini_array*)) __fini_array_end = .; } > FLASH
  _etext = .;

  .data : AT(_etext) { . = ALIGN(4); _sdata = .; *(.data*) *(.ramfunc*) . = ALIGN(4); _edata = .; } > RAM
  _sidata = LOADADDR(.data);

  .bss (NOLOAD) : { . = ALIGN(4); _sbss = .; *(.bss*) *(COMMON) . = ALIGN(4); _ebss = .; } > RAM
  .noinit (NOLOAD) : { *(.noinit*) } > RAM

  PROVIDE(_estack = ORIGIN(RAM) + LENGTH(RAM));
}
