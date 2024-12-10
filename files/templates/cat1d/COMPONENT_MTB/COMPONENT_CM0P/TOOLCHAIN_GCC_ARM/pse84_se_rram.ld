/***************************************************************************//**
* \file pse84_se_rram.ld
* \version 1.0.0
*
* Linker file for the GNU C compiler.
*
* The main purpose of the linker script is to describe how the sections in the
* input files should be mapped into the output file, and to control the memory
* layout of the output file.
*
* \note The entry point location starts at 0x13C00000. The valid
* application image should be placed there.
*
* \note The linker files included with the PDL template projects must be generic
* and handle all common use cases. Your project may not use every section
* defined in the linker files. In that case you may see warnings during the
* build process. In your project, you can simply comment out or remove the
* relevant code in the linker file.
*
********************************************************************************
* \copyright
* Copyright (c) (2020-2022), Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.
* SPDX-License-Identifier: Apache-2.0
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

OUTPUT_FORMAT ("elf32-littlearm", "elf32-bigarm", "elf32-littlearm")
GROUP(-lgcc -lc -lnosys )
SEARCH_DIR(.)
GROUP(libgcc.a libc.a libm.a libnosys.a)
ENTRY(Reset_Handler)

/* The size of the stack section at the end of CM4 SRAM */
STACK_SIZE = 0x1000;

SERAM_START_ADDR         = 0x30000000; /* sec addr */
SERAM_SIZE               = 0x00010000; /* 64K */
SERAM_CODE_START         = SERAM_START_ADDR;
SERAM_CODE_SIZE          = 0x00000000;
SERAM_DATA_START         = SERAM_CODE_START + SERAM_CODE_SIZE;
SERAM_DATA_SIZE          = SERAM_SIZE - SERAM_CODE_SIZE;

SRAM0_S_SAHB_START       = 0x34000000; /* This value has to be sync with cm33 linker script */
SRAM0_S_CM33_SIZE        = 0x00020000; /* This value has to be sync with cm33 linker script */

RRAM_NVM_SECURE_S_START  = 0x13C00000;
RRAM_NVM_SECURE_SIZE     = 0x00040000;

RRAM_NVM_SECURE_PROTIF_S_START  = 0x13800000;
RRAM_NVM_SECURE_PROTIF_SIZE     = 0x00030000;

RRAM_NVM_MAIN_S_START    = 0x12000000;
RRAM_NVM_MAIN_SIZE       = 0x00040000;


RAMVECTORS_ALIGNMENT                = 128;

/* vma for code region */
CODE_VMA                    = SERAM_CODE_START; /* 0x20000000 */
/* lma for code region */
CODE_LMA                    = SERAM_CODE_START; /* 0x20000000 */
/* vma for data region */
DATA_VMA                    = SERAM_DATA_START; /* 0x20008000 */
/* lma for data region */
DATA_LMA                    = SERAM_DATA_START; /* 0x20008000 */

CODE_RRAM_VMA      = RRAM_NVM_SECURE_PROTIF_S_START;
CODE_RRAM_LMA      = RRAM_NVM_SECURE_PROTIF_S_START;
RRAM_CODE_SIZE     = RRAM_NVM_SECURE_SIZE;

/* shared memory between cm0 and cm33-sec */
SRAM0_SHARED_SIZE           = 0x00002000; /* This value has to be sync with cm33 linker script */
SRAM0_SHARED_CM0_SIZE       = 0x00001000; /* This value has to be sync with cm33 linker script */
SRAM0_SHARED_LMA            = SRAM0_S_SAHB_START + SRAM0_S_CM33_SIZE - SRAM0_SHARED_SIZE;

/* The MEMORY section below describes the location and size of blocks of memory in the target.
* Use this section to specify the memory regions available for allocation.
*/
MEMORY
{
    /* The ram and flash regions control RAM and flash memory allocation for the CM33 core.
     */
    code                    (rx)    : ORIGIN = CODE_VMA,        LENGTH = SERAM_CODE_SIZE
    code_rram               (rx)    : ORIGIN = CODE_RRAM_VMA,   LENGTH = RRAM_CODE_SIZE
    data                    (rwx)   : ORIGIN = DATA_VMA,        LENGTH = SERAM_DATA_SIZE
    shared_ram0             (rwx)   : ORIGIN = SRAM0_SHARED_LMA LENGTH = SRAM0_SHARED_CM0_SIZE
}

/* Library configurations */
GROUP(libgcc.a libc.a libm.a libnosys.a)

SECTIONS
{
    /* Cortex-M0 application ram area */
    .text ORIGIN(code_rram) : AT(CODE_RRAM_LMA)
    {
        /* Cortex-M4 flash vector table */
        . = ALIGN(4);
        __Vectors = . ;
        KEEP(*(.vectors))
        . = ALIGN(4);
        __Vectors_End = .;
        __Vectors_Size = __Vectors_End - __Vectors;
        __end__ = .;

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

        /* Read-only code (constants). */
        *(.rodata .rodata.* .constdata .constdata.* .conststring .conststring.*)

        KEEP(*(.eh_frame*))
    } > code_rram

    /* To copy multiple ROM to RAM sections,
     * uncomment .copy.table section and,
     * define __STARTUP_COPY_MULTIPLE in startup_psoc6_02_cm4.S */
    .copy.table :
    {
        . = ALIGN(4);
        __copy_table_start__ = .;
        LONG (__start_of_data_la)
        LONG (__start_of_data_ea)
        LONG ((__end_of_data_ea - __start_of_data_ea)/4)
        __copy_table_end__ = .;
    } > code_rram

    /* To clear multiple BSS sections,
     * uncomment .zero.table section and,
     * define __STARTUP_CLEAR_BSS_MULTIPLE in startup_psoc6_02_cm4.S */
    .zero.table :
    {
        . = ALIGN(4);
        __zero_table_start__ = .;
        LONG (__bss_start__)
        LONG (__bss_end__ - __bss_start__)
        __zero_table_end__ = .;
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

    .ramVectors ORIGIN(data) (NOLOAD) :
    {
        . = ALIGN(RAMVECTORS_ALIGNMENT);
        __ram_vectors_start__ = .;
        KEEP(*(.ram_vectors))
        __ram_vectors_end__   = .;
    } > data

    __start_of_data_la = __exidx_end;

    .data : AT (__start_of_data_la)
    {
        . = ALIGN(4);
        __start_of_data_ea = .;

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

        KEEP(*(.cy_ramfunc*))
        . = ALIGN(4);

        __end_of_data_ea = .;
    } > data


    /* Place variables in the section that should not be initialized during the
    *  device startup.
    */
    .noinit (NOLOAD) : ALIGN(8)
    {
      KEEP(*(.noinit))
    } > data


    /* The uninitialized global or static variables are placed in this section.
    *
    * The NOLOAD attribute tells linker that .bss section does not consume
    * any space in the image. The NOLOAD attribute changes the .bss type to
    * NOBITS, and that  makes linker to A) not allocate section in memory, and
    * A) put information to clear the section with all zeros during application
    * loading.
    *
    * Without the NOLOAD attribute, the .bss section might get PROGBITS type.
    * This  makes linker to A) allocate zeroed section in memory, and B) copy
    * this section to RAM during application loading.
    */
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
        __HeapBase = .;
        __end__ = .;
        end = __end__;
        KEEP(*(.heap*))
        . = ORIGIN(data) + LENGTH(data) - STACK_SIZE;
        __HeapLimit = .;
    } > data

    .cy_sharedmem (NOLOAD):
    {
        . = ALIGN(4);
        __public_ram_start__ = .;
        KEEP(*(.cy_sharedmem_sec))
        KEEP(*(.cy_sharedmem))
        . = ALIGN(4);
        __public_ram_end__ = .;
    } > shared_ram0

    /* .stack_dummy section doesn't contains any symbols. It is only
     * used for linker to calculate size of stack sections, and assign
     * values to stack symbols later */
    .stack_dummy (NOLOAD):
    {
        KEEP(*(.stack*))
    } > data


    /* Set stack top to end of RAM, and stack limit move down by
     * size of stack_dummy section */
    __StackTop = ORIGIN(data) + LENGTH(data);
    __StackLimit = __StackTop - SIZEOF(.stack_dummy);
    PROVIDE(__stack = __StackTop);

    /* Check if data + heap + stack exceeds RAM limit */
    ASSERT(__StackLimit >= __HeapLimit, "region RAM overflowed with stack")
}

/* EOF */
