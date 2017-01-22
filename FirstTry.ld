/*
Linker script for FirstTry
*/

ENTRY(_start)

MEMORY
{
	FLASH (rx): ORIGIN = 0x08000000, LENGTH = 64k
	/* SYSMEM (rx): ORIGIN = 0x1FFFF000, LENGTH = 2064 */
	SRAM (rwx) : ORIGIN = 0x20000000, LENGTH = 20k
}

SECTIONS
{	
    /* Default vectors */
    /*
    .vectors :
    {
    	KEEP(*start.o(.vectors))
    } >FLASH
    */
    .text :
    {
     	. = ALIGN(.,0x4);
     	KEEP(*start.o(.vectors))
        *(.init)
        *start.o(.text)
        *FirstTry.o(.text)
        *(.text)
        *(.text*)
        *(.rodata)
        *(.rodata*)
    } >FLASH
    
    .data :
    {
    	. = ALIGN(0x8);
        *(.data)
        *(.data*)
    } >SRAM

    .bss :
    {
    	. = ALIGN(0x8);
        *(.bss)
        *(.bss*)
        stacks = .;
		. = . + 2048;	/* usr stack size */
		__usr_stack = .;
		. = . + 4096;	/* sys stack size */
		__sys_stack = .;
    } >SRAM
    
}
