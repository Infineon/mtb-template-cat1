OUTPUT_FORMAT ("elf32-littlearm", "elf32-bigarm", "elf32-littlearm")
SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)
ENTRY(Reset_Handler)

/* Common definitions for the device  */
RRAM_NS_START                    = 0x22000000;
RRAM_S_START                     = 0x32000000;
RRAM_SIZE                        = 0x00080000;

RRAM_NVM_MAIN_NS_START           = 0x22000000;
RRAM_NVM_MAIN_S_START            = 0x32000000;
RRAM_NVM_MAIN_NS_CBUS_START      = 0x02000000;
RRAM_NVM_MAIN_S_CBUS_START       = 0x12000000;
RRAM_NVM_MAIN_SIZE               = 0x00020000;

RRAM_NVM_WORK_NS_START           = 0x23000000;
RRAM_NVM_WORK_S_START            = 0x33000000;
RRAM_NVM_WORK_NS_CBUS_START      = 0x03000000;
RRAM_NVM_WORK_S_CBUS_START       = 0x13000000;
RRAM_NVM_WORK_SIZE               = 0x00020000;

RRAM_NVM_SFLASH_NS_START         = 0x23600000;
RRAM_NVM_SFLASH_S_START          = 0x33600000;
RRAM_NVM_SFLASH_NS_CBUS_START    = 0x03600000;
RRAM_NVM_SFLASH_S_CBUS_START     = 0x13600000;
RRAM_NVM_SFLASH_SIZE             = 0x00020000;

RRAM_NVM_SECURE_NS_START         = 0x23C00000;
RRAM_NVM_SECURE_S_START          = 0x33C00000;
RRAM_NVM_SECURE_NS_CBUS_START    = 0x03C00000;
RRAM_NVM_SECURE_S_CBUS_START     = 0x13C00000;
RRAM_NVM_SECURE_SIZE             = 0x00020000;

SRAM0_NS_START                   = 0x24000000;
SRAM0_S_START                    = 0x34000000;
SRAM0_NS_CBUS_START              = 0x04000000;
SRAM0_S_CBUS_START               = 0x14000000;
SRAM0_SIZE                       = 0x00040000;

SRAM1_NS_START                   = 0x24040000;
SRAM1_S_START                    = 0x34040000;
SRAM1_NS_CBUS_START              = 0x04040000;
SRAM1_S_CBUS_START               = 0x14040000;
SRAM1_SIZE                       = 0x00040000;

SOCMEM_SRAM_NS_START             = 0x26000000;
SOCMEM_SRAM_S_START              = 0x36000000;
SOCMEM_SRAM_NS_CBUS_START        = 0x06000000;
SOCMEM_SRAM_S_CBUS_START         = 0x16000000;
SOCMEM_SRAM_SIZE                 = 0x00180000;

XIP_PORT0_NS_START               = 0x60000000;
XIP_PORT0_S_START                = 0x70000000;
XIP_PORT0_NS_CBUS_START          = 0x08000000;
XIP_PORT0_S_CBUS_START           = 0x18000000;
XIP_PORT0_SIZE                   = 0x04000000;

XIP_PORT1_NS_START               = 0x64000000;
XIP_PORT1_S_START                = 0x74000000;
XIP_PORT1_NS_CBUS_START          = 0x0C000000;
XIP_PORT1_S_CBUS_START           = 0x1C000000;
XIP_PORT1_SIZE                   = 0x04000000;


/* Defines specific to CM33 non-secure image */

RAMVECTORS_ALIGNMENT             = 512;
MSP_STACK_SIZE                   = 0x00000800;
HEAP_SIZE                        = 0x00001000;

CODE_SRAM_OFFSET                 = 0x0001EC00; /* 58K + 1K + 64K */
CODE_SRAM_SIZE                   = 0x0000EC00; /* 59K */
CODE_SRAM_VMA                    = SRAM0_NS_CBUS_START + CODE_SRAM_OFFSET;
CODE_SRAM_LMA                    = SRAM0_NS_START + CODE_SRAM_OFFSET;

DATA_SRAM_OFFSET                 = 0x0002D800;
DATA_SRAM_SIZE                   = 0x00010000;
DATA_SRAM_VMA                    = SRAM0_NS_START + DATA_SRAM_OFFSET;
DATA_SRAM_LMA                    = SRAM0_NS_START + DATA_SRAM_OFFSET;

DATA_SOCMEMSRAM_OFFSET           = 0x00000000;
DATA_SOCMEMSRAM_SIZE             = 0x00000000; /* Currently entire socmem ram is given to CM55 */
DATA_SOCMEMSRAM_VMA              = SOCMEM_SRAM_NS_START + DATA_SOCMEMSRAM_OFFSET;
DATA_SOCMEMSRAM_LMA              = SOCMEM_SRAM_NS_START + DATA_SOCMEMSRAM_OFFSET;

CODE_XIP0_OFFSET                 = 0x00040000; /* 256K */
CODE_XIP0_SIZE                   = 0x00040000; /* 256K */
CODE_XIP0_VMA                    = XIP_PORT0_NS_CBUS_START + CODE_XIP0_OFFSET;
CODE_XIP0_LMA                    = XIP_PORT0_NS_START + CODE_XIP0_OFFSET;

CODE_RRAM_NVM_MAIN_OFFSET        = 0x0000CC00; /* 50K + 1K */
CODE_RRAM_NVM_MAIN_SIZE          = 0x00006C00; /* 27K */
CODE_RRAM_NVM_MAIN_VMA           = RRAM_NVM_MAIN_NS_CBUS_START + CODE_RRAM_NVM_MAIN_OFFSET;
CODE_RRAM_NVM_MAIN_LMA           = RRAM_NVM_MAIN_NS_START + CODE_RRAM_NVM_MAIN_OFFSET;

DATA_RRAM_NVM_WORK_OFFSET        = 0x00007000; /* 28K */
DATA_RRAM_NVM_WORK_SIZE          = 0x0000C800; /* 50K */
DATA_RRAM_NVM_WORK_VMA           = RRAM_NVM_WORK_NS_START + DATA_RRAM_NVM_WORK_OFFSET;

MEMORY
{
    code_sram          (rwx)   : ORIGIN = CODE_SRAM_VMA,            LENGTH = CODE_SRAM_SIZE
    data_sram          (rw)    : ORIGIN = DATA_SRAM_VMA,            LENGTH = DATA_SRAM_SIZE
    code_rram          (rx)    : ORIGIN = CODE_RRAM_NVM_MAIN_VMA,   LENGTH = CODE_RRAM_NVM_MAIN_SIZE
    data_rram          (rw)    : ORIGIN = DATA_RRAM_NVM_WORK_VMA,   LENGTH = DATA_RRAM_NVM_WORK_SIZE
    data_socmemRam     (rw)    : ORIGIN = DATA_SOCMEMSRAM_VMA,      LENGTH = DATA_SOCMEMSRAM_SIZE
    code_xip0          (rwx)   : ORIGIN = CODE_XIP0_VMA,            LENGTH = CODE_XIP0_SIZE
}

SECTIONS
{
    .appText ORIGIN(code_rram) : AT(CODE_RRAM_NVM_MAIN_LMA)
    {
        . = ALIGN(4);
        __Vectors = . ;
        KEEP(*(.vectors))
        . = ALIGN(4);
        __Vectors_End = .;

        . = ALIGN(4);
        *(.text*)
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
    } > code_rram

    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > code_rram

    __exidx_start = .;

    .ARM.exidx :
    {
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    } > code_rram

    __exidx_end = .;

    .copy.table :
    {
        . = ALIGN(4);
        __copy_table_start__ = .;

        /* Copy vector table to RAM */
        LONG (__Vectors)                                        /* From */
        LONG (__ram_vectors_start__)                            /* To   */
        LONG ((__Vectors_End - __Vectors)/4)                    /* Size */

        /* Copy .data section to RAM */
        LONG (__cy_sram_data_lma__)                               /* From */
        LONG (__data_start__)                                   /* To   */
        LONG ((__data_end__ - __data_start__)/4)                /* Size */

        /* Copy appTextRam section to RAM */
        LONG (__cy_sram_code_lma__)                             /* From */
        LONG (__cy_sram_code_start__)                           /* To   */
        LONG ((__cy_sram_code_end__ - __cy_sram_code_start__)/4)  /* Size */

        /* Copy socmem data section to SOCMEMRAM */
        LONG (__cy_socmemsram_data_lma__)                             /* From */
        LONG (__cy_socmem_sram_start__)                           /* To   */
        LONG ((__cy_socmem_sram_end__ - __cy_socmem_sram_start__)/4)  /* Size */

        __copy_table_end__ = .;
    } > code_rram

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
    } > code_rram

    __cy_sram_code_lma__ = .;

    cy_sram_code : AT(__cy_sram_code_lma__)
    {
        . = ALIGN(4);
        __cy_sram_code_start__ = .;
        KEEP(*(.cy_ramfunc*))
        . = ALIGN(4);
        __cy_sram_code_end__ = .;
    } > code_sram

    .ramVectors (NOLOAD):
    {
        . = ALIGN(RAMVECTORS_ALIGNMENT);
        __ram_vectors_start__ = .;
        KEEP(*(.ram_vectors))
        __ram_vectors_end__   = .;
    } > data_sram

    .noinit (NOLOAD) : ALIGN(8)
    {
        KEEP(*(.noinit))
    } > data_sram

    __cy_sram_data_lma__ = __cy_sram_code_lma__ + (__cy_sram_code_end__ - __cy_sram_code_start__);

    .data : AT(__cy_sram_data_lma__)
    {
        . = ALIGN(4);
        __data_start__ = .;

        *(vtable)
        *(.data*)

        . = ALIGN(4);
        /* preinit data */
        PROVIDE_HIDDEN (__preinit_array_start = .);
        KEEP(*(.preinit_array))
        PROVIDE_HIDDEN (__preinit_array_end = .);

        . = ALIGN(4);
        /* init data */
        PROVIDE_HIDDEN (__init_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array))
        PROVIDE_HIDDEN (__init_array_end = .);

        . = ALIGN(4);
        /* finit data */
        PROVIDE_HIDDEN (__fini_array_start = .);
        KEEP(*(SORT(.fini_array.*)))
        KEEP(*(.fini_array))
        PROVIDE_HIDDEN (__fini_array_end = .);

        KEEP(*(.jcr*))
        . = ALIGN(4);

        __data_end__ = .;
    } > data_sram

    .bss (NOLOAD):
    {
        . = ALIGN(4);
        __bss_start__ = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        __bss_end__ = .;
    } > data_sram

    .heap (NOLOAD):
    {
        . = ALIGN(8);
        __HeapBase = .;
        __end__ = .;
        end = __end__;
        KEEP(*(.heap*))
        . = ALIGN(8);
        . = ORIGIN(data_sram) + LENGTH(data_sram) - MSP_STACK_SIZE;
        __HeapLimit = .;
    } > data_sram

    . = ALIGN(32);
    __StackTop = ORIGIN(data_sram) + LENGTH(data_sram);
    __StackLimit = __StackTop - MSP_STACK_SIZE;
    PROVIDE(__stack = __StackTop);

    /* Check if data + heap + stack exceeds RAM limit */
    ASSERT(__StackLimit >= __HeapLimit, "region RAM overflowed with stack")

    cy_xip ORIGIN(code_xip0) : AT(CODE_XIP0_LMA)
    {
        __cy_xip_start = .;
        KEEP(*(.cy_xip))
        __cy_xip_end = .;
    } > code_xip0

    __cy_socmemsram_data_lma__ = __cy_sram_data_lma__ + (__data_end__ - __data_start__);

    cy_socmemRam : AT(__cy_socmemsram_data_lma__)
    {
        __cy_socmem_sram_start__ = .;
        KEEP(*(.cy_socmemRam))
        __cy_socmem_sram_end__ = .;
    } > data_socmemRam

    cy_rram_data :
    {
        __cy_rram_data_start__ = .;
        KEEP(*(.cy_rram_data))
        __cy_rram_data_end__ = .;
    } > data_rram AT>data_rram
}