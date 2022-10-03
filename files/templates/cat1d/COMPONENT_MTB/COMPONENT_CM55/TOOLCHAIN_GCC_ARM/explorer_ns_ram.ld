OUTPUT_FORMAT ("elf32-littlearm", "elf32-bigarm", "elf32-littlearm")
GROUP(-lgcc -lc -lnosys )
SEARCH_DIR(.)
GROUP(libgcc.a libc.a libm.a libnosys.a)

ENTRY(Reset_Handler)

ITCM_NS_START               = 0x00000000;
ITCM_S_START                = 0x10000000;
ITCM_SIZE                   = 0x00040000;

DTCM_NS_START               = 0x20000000;
DTCM_S_START                = 0x30000000;
DTCM_SIZE                   = 0x00040000;

RRAM_NS_START               = 0x22000000;
RRAM_S_START                = 0x32000000;
RRAM_SIZE                   = 0x02000000;

RRAM_NS_SAHB_START          = RRAM_NS_START;
RRAM_S_SAHB_START           = RRAM_S_START;

RRAM_NS_CBUS_START          = 0x02000000;
RRAM_S_CBUS_START           = 0x12000000;

SRAM0_NS_SAHB_START         = 0x24000000;
SRAM0_S_SAHB_START          = 0x34000000;
SRAM0_NS_CBUS_START         = 0x04000000;
SRAM0_S_CBUS_START          = 0x14000000;
SRAM0_SIZE                  = 0x00080000;

SRAM1_NS_SAHB_START         = 0x24080000;
SRAM1_S_SAHB_START          = 0x34080000;
SRAM1_NS_CBUS_START         = 0x04080000;
SRAM1_S_CBUS_START          = 0x14080000;
SRAM1_SIZE                  = 0x00080000;

SOCMEM_NS_RAM_SAHB_START    = 0x26000000;
SOCMEM_S_RAM_SAHB_START     = 0x36000000;
SOCMEM_NS_RAM_CBUS_START    = 0x06000000;
SOCMEM_S_RAM_CBUS_START     = 0x16000000;
SOCMEM_RAM_SIZE             = 0x00500000;

FLASH_NS_SAHB_START         = 0x60000000;
FLASH_S_SAHB_START          = 0x70000000;
FLASH_NS_CBUS_START         = 0x08000000;
FLASH_S_CBUS_START          = 0x18000000;
FLASH_SIZE                  = 0x04000000;

MMIO_NS_M33SYS_START        = 0x42000000;
MMIO_S_M33SYS_START         = 0x52000000;
MMIO_M33SYS_SIZE            = 0x02000000;

MMIO_NS_M55APP_START        = 0x44000000;
MMIO_S_M55APP_START         = 0x54000000;
MMIO_M55APP_SIZE            = 0x08000000;

/*----------------------------------------*/

RAMVECTORS_ALIGNMENT        = 512;

MSP_NS_STACK_SIZE           = 0x0000800;

CODE_VMA                    = ITCM_NS_START;
CODE_LMA                    = ITCM_NS_START;
CODE_SIZE                   = ITCM_SIZE;

DATA_VMA                    = DTCM_NS_START;
DATA_LMA                    = DTCM_NS_START;
DATA_SIZE                   = DTCM_SIZE;

SRAM_VMA                    = SRAM0_NS_SAHB_START;
SRAM_LMA                    = SRAM0_NS_SAHB_START;
SRAM_SIZE                   = SRAM0_SIZE + SRAM1_SIZE;

SOCMEM_RAM_VMA             = SOCMEM_NS_RAM_CBUS_START;
SOCMEM_RAM_LMA             = SOCMEM_NS_RAM_SAHB_START;

RRAM_VMA                    = RRAM_NS_START;
RRAM_LMA                    = RRAM_NS_START;

XIP_VMA                     = FLASH_NS_CBUS_START;
XIP_LMA                     = FLASH_NS_CBUS_START;
XIP_SIZE                    = FLASH_SIZE;


MEMORY
{
    code       (rx)     : ORIGIN = ITCM_NS_START,    LENGTH = CODE_SIZE
    data       (rwx)    : ORIGIN = DATA_VMA,         LENGTH = DATA_SIZE
    sram       (rwx)    : ORIGIN = SRAM_VMA,         LENGTH = SRAM_SIZE
    rram       (rwx)    : ORIGIN = RRAM_VMA,         LENGTH = RRAM_SIZE
    socmemRam  (rwx)    : ORIGIN = SOCMEM_RAM_VMA,  LENGTH = SOCMEM_RAM_SIZE
    xip        (rwx)    : ORIGIN = XIP_VMA,          LENGTH = XIP_SIZE
}

SECTIONS
{
    .appText ORIGIN(code) : AT(CODE_LMA)
    {
        KEEP(*(.vectors))
        *(.text*)
        KEEP(*(.text))      /* Program code */
        KEEP(*(.init))
        KEEP(*(.fini))
        /* .ctors */
        *crtbegin.o(.ctors)
        *crtbegin?.o(.ctors)
        *(EXCLUDE_FILE(*crtend?.o *crtend.o) .ctors)
        *(SORT(.ctors.*))
        *(.ctors)
        /* .dtors */
        *crtbegin.o(.dtors)
        *crtbegin?.o(.dtors)
        *(EXCLUDE_FILE(*crtend?.o *crtend.o) .dtors)
        *(SORT(.dtors.*))
        *(.dtors)
        *(.rodata*)    /* Read only data */
    } > code

    .copy.table :
    {
        . = ALIGN(4);
        __copy_table_start__ = .;

        __copy_table_end__ = .;
    } > code

    /* To clear multiple BSS sections,
     * uncomment .zero.table section and,
     * define __STARTUP_CLEAR_BSS_MULTIPLE in CAT1B devices */
    .zero.table :
    {
        . = ALIGN(4);
        __zero_table_start__ = .;
        LONG (__bss_start__)
        LONG ((__bss_end__ - __bss_start__)/4)
        __zero_table_end__ = .;
    } > code

    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > code

    __exidx_start = .;

    .ARM.exidx :
    {
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    } > code
    __exidx_end = .;

    .ramVectors ORIGIN(data) (NOLOAD):
    {
        . = ALIGN(RAMVECTORS_ALIGNMENT);
        __ram_vectors_start__ = .;
        KEEP(*(.ram_vectors))
        __ram_vectors_end__   = .;
    } > data

    .data :
    {
        *(.data*)      /* Data memory */
    } > data

    .noinit (NOLOAD) : ALIGN(8)
    {
        KEEP(*(.noinit))
    } > data

    .bss (NOLOAD):
    {
        . = ALIGN(4);
        __bss_start__ = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        __bss_end__ = .;
    } > data

    .heap (NOLOAD):
    {
        . = ALIGN(8);
        __HeapBase = .;
        __end__ = .;
        end = __end__;
        KEEP(*(.heap*))
        . = ALIGN(8);
        . = ORIGIN(data) + LENGTH(data) - MSP_NS_STACK_SIZE;
        __HeapLimit = .;
    } > data

    . = ALIGN(32);
    __StackTop = ORIGIN(data) + LENGTH(data);
    __StackLimit = __StackTop - MSP_NS_STACK_SIZE;
    PROVIDE(__stack = __StackTop);

    /* Check if data + heap + stack exceeds RAM limit */
    ASSERT(__StackLimit >= __HeapLimit, "region RAM overflowed with stack")

    cy_xip ORIGIN(xip) : AT(XIP_LMA)
    {
        __cy_xip_start = .;
        KEEP(*(.cy_xip))
        __cy_xip_end = .;
    } > xip

    cy_socmemRam ORIGIN(socmemRam) : AT(SOCMEM_RAM_LMA)
    {
        __cy_socmem_ram_start = .;
        KEEP(*(.cy_socmemRam))
        __cy_socmem_ram_end = .;
    } > socmemRam

    cy_rram ORIGIN(rram) : AT(RRAM_LMA)
    {
        __cy_rram_start = .;
        KEEP(*(.cy_rram))
        __cy_rram_end = .;
    } > rram

    cy_sram ORIGIN(sram) : AT(SRAM_LMA)
    {
        __cy_sram_start = .;
        KEEP(*(.cy_sram))
        __cy_sram_end = .;
    } > sram
}
