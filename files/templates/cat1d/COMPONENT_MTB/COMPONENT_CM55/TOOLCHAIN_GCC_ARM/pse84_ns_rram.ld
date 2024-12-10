/***************************************************************************//**
* \file pse84_ns_rram.ld
* \version 1.0.0
*
* Linker file for the GNU C compiler.
*
* The main purpose of the linker script is to describe how the sections in the
* input files should be mapped into the output file, and to control the memory
* layout of the output file.
*
* \note The entry point location starts at 0x0202E000. The valid
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

/* Common definitions for the device  */
ITCM_NS_START                    = 0x00000000;
ITCM_S_START                     = 0x10000000;
ITCM_SIZE                        = 0x00040000;

DTCM_NS_START                    = 0x20000000;
DTCM_S_START                     = 0x30000000;
DTCM_SIZE                        = 0x00040000;

RRAM_NS_START                    = 0x22000000;
RRAM_S_START                     = 0x32000000;
RRAM_SIZE                        = 0x00080000;

RRAM_NVM_MAIN_NS_START           = 0x22000000;
RRAM_NVM_MAIN_S_START            = 0x32000000;
RRAM_NVM_MAIN_NS_CBUS_START      = 0x02000000;
RRAM_NVM_MAIN_S_CBUS_START       = 0x12000000;
RRAM_NVM_MAIN_SIZE               = 0x00040000;

RRAM_NVM_SECURE_NS_START         = 0x23C00000;
RRAM_NVM_SECURE_S_START          = 0x33C00000;
RRAM_NVM_SECURE_NS_CBUS_START    = 0x03C00000;
RRAM_NVM_SECURE_S_CBUS_START     = 0x13C00000;
RRAM_NVM_SECURE_SIZE             = 0x00040000;

SRAM0_NS_START                   = 0x24000000;
SRAM0_S_START                    = 0x34000000;
SRAM0_NS_CBUS_START              = 0x04000000;
SRAM0_S_CBUS_START               = 0x14000000;
SRAM0_SIZE                       = 0x00080000;

SRAM1_NS_START                   = 0x24080000;
SRAM1_S_START                    = 0x34080000;
SRAM1_NS_CBUS_START              = 0x04080000;
SRAM1_S_CBUS_START               = 0x14080000;
SRAM1_SIZE                       = 0x00080000;
SOCMEM_SRAM_NS_START             = 0x26000000;
SOCMEM_SRAM_S_START              = 0x36000000;
SOCMEM_SRAM_NS_CBUS_START        = 0x06000000;
SOCMEM_SRAM_S_CBUS_START         = 0x16000000;
SOCMEM_SRAM_SIZE                 = 0x00500000;

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

RAMVECTORS_ALIGNMENT             = 1024;

/* To set stack size in application add makefile ldflags: --defsym,APP_MSP_STACK_SIZE=0x00001000 */
__StackSize                   = DEFINED(APP_MSP_STACK_SIZE) ? APP_MSP_STACK_SIZE : 0x00000800;
HEAP_SIZE                        = 0x00001000;
SRAM1_SHARED_SIZE                = 0x00002000; /* 8K memory reserved for cm33-s, cm33-ns and cm55 IPC data */
SRAM1_SHARED_CM33S_SIZE          = 0x00000800; /* Out of 8K shared memory, 2K is reserved for CM33-S */
SRAM1_SHARED_CM33NS_SIZE         = 0x00000800; /* Out of 8K shared memory, 2K is reserved for CM33-NS */
SRAM1_SHARED_LMA                 = (SRAM1_NS_START + SRAM1_SIZE - SRAM1_SHARED_SIZE) + SRAM1_SHARED_CM33S_SIZE + SRAM1_SHARED_CM33NS_SIZE;

RESERVED_SOCMEM_MEM_SIZE         = 0x00100000; /* 1M memory reserved for SHARED SOCMEM  */
RESERVED_SOCMEM_LMA              = (SOCMEM_SRAM_NS_START + SOCMEM_SRAM_SIZE) - RESERVED_SOCMEM_MEM_SIZE;

RRAM_NVM_MAIN_NS_M55CODE_OFFSET  = (208*1024); /* This value should be same as in value in partition_cat1d */
RRAM_NVM_MAIN_NS_M55CODE_SIZE    = (40*1024);  /* This value should be same as in value in partition_cat1d */

CM55_APP_HEADER_SIZE             = 0x00000400;

CODE_SRAM_OFFSET                 = 0x0;
CODE_SRAM_SIZE                   = 0x40000;
CODE_SRAM_VMA                    = SRAM1_NS_CBUS_START + CODE_SRAM_OFFSET;
CODE_SRAM_LMA                    = SRAM1_NS_START + CODE_SRAM_OFFSET;

DATA_SRAM_OFFSET                 = CODE_SRAM_OFFSET + CODE_SRAM_SIZE;
DATA_SRAM_SIZE                   = SRAM1_SIZE - SRAM1_SHARED_SIZE - CODE_SRAM_SIZE;
DATA_SRAM_VMA                    = SRAM1_NS_START + DATA_SRAM_OFFSET;
DATA_SRAM_LMA                    = SRAM1_NS_START + DATA_SRAM_OFFSET;

CODE_RRAM_OFFSET                 = RRAM_NVM_MAIN_NS_M55CODE_OFFSET;
CODE_RRAM_SIZE                   = RRAM_NVM_MAIN_NS_M55CODE_SIZE - CM55_APP_HEADER_SIZE;
CODE_RRAM_VMA                    = RRAM_NVM_MAIN_NS_CBUS_START + CODE_RRAM_OFFSET + CM55_APP_HEADER_SIZE;
CODE_RRAM_LMA                    = RRAM_NVM_MAIN_NS_CBUS_START + CODE_RRAM_OFFSET + CM55_APP_HEADER_SIZE;

CODE_SOCMEM_OFFSET                 = 0x00000000;
CODE_SOCMEM_SIZE                   = 0x00100000;
CODE_SOCMEM_VMA                    = SOCMEM_SRAM_NS_CBUS_START + CODE_SOCMEM_OFFSET;
CODE_SOCMEM_LMA                    = SOCMEM_SRAM_NS_START + CODE_SOCMEM_OFFSET;

DATA_SOCMEM_OFFSET                 = CODE_SOCMEM_OFFSET + CODE_SOCMEM_SIZE;
DATA_SOCMEM_SIZE                   = 0x00100000;
DATA_SOCMEM_VMA                    = SOCMEM_SRAM_NS_START + DATA_SOCMEM_OFFSET;
DATA_SOCMEM_LMA                    = SOCMEM_SRAM_NS_START + DATA_SOCMEM_OFFSET;

CM55_XIP0_OFFSET                  = 0x00400000;
CODE_XIP0_SIZE                    = 0x00400000;
CODE_XIP0_LMA                     = XIP_PORT0_NS_START + CM55_XIP0_OFFSET;
CODE_XIP0_VMA                     = (XIP_PORT0_NS_START + CM55_XIP0_OFFSET);

IMAGE_HEADER_SIZE                 = 0;

MEMORY
{
    xip0             (rwx)  : ORIGIN = CODE_XIP0_VMA,       LENGTH = CODE_XIP0_SIZE
    rram_code        (rwx)  : ORIGIN = CODE_RRAM_VMA,       LENGTH = CODE_RRAM_SIZE
    itcm             (rwx)  : ORIGIN = ITCM_NS_START,       LENGTH = ITCM_SIZE
    dtcm             (rw)   : ORIGIN = DTCM_NS_START,       LENGTH = DTCM_SIZE
    sram_code        (rwx)  : ORIGIN = CODE_SRAM_VMA,       LENGTH = CODE_SRAM_SIZE
    sram_data        (rw)   : ORIGIN = DATA_SRAM_VMA,       LENGTH = DATA_SRAM_SIZE
    socmem_code      (rwx)  : ORIGIN = CODE_SOCMEM_LMA,     LENGTH = CODE_SOCMEM_SIZE
    socmem_data      (rw)   : ORIGIN = DATA_SOCMEM_VMA,     LENGTH = DATA_SOCMEM_SIZE
    shared_ram1      (rw)   : ORIGIN = SRAM1_SHARED_LMA,    LENGTH = SRAM1_SHARED_CM33NS_SIZE
    reserved_socmem  (rw)   : ORIGIN = RESERVED_SOCMEM_LMA, LENGTH = RESERVED_SOCMEM_MEM_SIZE

}

SECTIONS
{
    /* Image Header */
    .imgHeader ORIGIN(rram_code) (NOLOAD):
    {
      . = . + IMAGE_HEADER_SIZE; /* 32 bytes for header */
    } > rram_code
    /* Code Flash XIP */

    __app_text_vma__ = ORIGIN(rram_code) + IMAGE_HEADER_SIZE;
    __app_text_lma__ = CODE_RRAM_LMA + IMAGE_HEADER_SIZE;
    .appText __app_text_vma__ : AT (__app_text_lma__)
    {
        . = ALIGN(4);
         __text_begin = .;
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
        /* Read only data */
        *(.rodata .rodata.* .constdata .constdata.* .conststring .conststring.*)
        . = ALIGN(4);
        __text_end = .;
    } > rram_code

    /* Flash */
    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > rram_code

    __exidx_start = .;
     /* Flash */
    .ARM.exidx :
    {
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    } > rram_code

    __exidx_end = .;
          . = ALIGN(4);
      __copy_table_start__ = .;
     /* Flash */
     .copy.table : AT (__copy_table_start__ + (__app_text_lma__ - __app_text_vma__))
    {
        /* Copy vector table to RAM */
        /* From */
        /* To   */
        /* Size */
        LONG((__data_lma__ - CODE_RRAM_LMA) + CODE_RRAM_VMA)                /* From */
        LONG(__data_vma__)                                                  /* To   */
        LONG((__data_end__ - __data_start__)/4)                             /* Size */

        LONG((__cy_sram_code_lma_start - CODE_RRAM_LMA) + CODE_RRAM_VMA)    /* From */
        LONG(CODE_SRAM_LMA)                                                 /* To   */
        LONG((__cy_sram_code_vma_end__ - __cy_sram_code_vma_start__)/4)     /* Size */

        LONG((__cy_socmem_data_lma_start - CODE_RRAM_LMA) + CODE_RRAM_VMA)  /* From */
        LONG(__cy_socmem_data_vma_start__)                                  /* To   */
        LONG((__cy_socmem_data_vma_end__ - __cy_socmem_data_vma_start__)/4) /* Size */


        LONG((__cy_socmem_code_lma_start - CODE_RRAM_LMA) + CODE_RRAM_VMA)  /* From */
        LONG(CODE_SOCMEM_LMA)                                               /* To   */
        LONG((__cy_socmem_code_vma_end__ - __cy_socmem_code_vma_start__)/4) /* Size */

        LONG(__cy_itcm_lma_start)                                           /* From */
        LONG(__cy_itcm_vma_start__)                                         /* To   */
        LONG((__cy_itcm_vma_end__ - __cy_itcm_vma_start__)/4)                 /* Size */


      __copy_table_end__ = .;

    } > rram_code

    . = ALIGN(4);
    __zero_table_start__ = . ;
    /* To clear multiple BSS sections */
    .zero.table : AT (__zero_table_start__ + (__app_text_lma__ - __app_text_vma__))
    {

        LONG (__bss_start__)
        LONG ((__bss_end__ - __bss_start__)/4)
        __zero_table_end__ = .;

    } > rram_code

    . = ALIGN(4);
    __cy_socmem_code_lma_start = .  + (__app_text_lma__ - __app_text_vma__);

     /* LMA SocMEM VMA Flash */
    .cy_socmem_code ORIGIN(socmem_code) : AT(__cy_socmem_code_lma_start )
    {
        . = ALIGN(4);
         __cy_socmem_code_vma_start__ = .;
        KEEP(*(.cy_socmem_code))
        . = ALIGN(4);
        __cy_socmem_code_vma_end__ = .;

    }  > socmem_code
     __cy_socmem_code_lma_end = __cy_socmem_code_lma_start + __cy_socmem_code_vma_end__ -__cy_socmem_code_vma_start__ ;

    __cy_socmem_data_lma_start = __cy_socmem_code_lma_end;

     /* LMA SocMEM VMA Flash */
    .cy_socmem_data ORIGIN(socmem_data) : AT(__cy_socmem_data_lma_start)
    {
        . = ALIGN(4);
         __cy_socmem_data_vma_start__ = .;
        KEEP(*(.cy_socmem_data))
        . = ALIGN(4);
        __cy_socmem_data_vma_end__ = .;

    }  > socmem_data
     __cy_socmem_data_lma_end = __cy_socmem_data_lma_start + __cy_socmem_data_vma_end__ -__cy_socmem_data_vma_start__ ;

    __cy_itcm_lma_start = __cy_socmem_data_lma_end;
     /* LMA ITCM VMA Flash */
    .cy_itcm ORIGIN(itcm) : AT(__cy_itcm_lma_start)
    {
        . = ALIGN(4);
         __cy_itcm_vma_start__ = .;
        KEEP(*(.cy_itcm))
        . = ALIGN(4);
        __cy_itcm_vma_end__ = .;

    }  > itcm
     __cy_itcm_lma_end = __cy_itcm_lma_start + __cy_itcm_vma_end__ -__cy_itcm_vma_start__ ;
    . = __cy_itcm_lma_end;

   . = ALIGN(4);
    __flash_end = .;
    .ram_vectors ORIGIN(dtcm)(NOLOAD):
    {
        . = ALIGN(RAMVECTORS_ALIGNMENT);
        __ram_vectors_start__ = .;
        KEEP(*(.ram_vectors))
        __ram_vectors_end__   = .;
    } > dtcm

     __data_vma__ = __ram_vectors_end__;
     __data_lma__ =  __flash_end;

    .data __data_vma__ : AT(__data_lma__)
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
    } > dtcm

    .noinit (NOLOAD) : ALIGN(8)
    {
        KEEP(*(.noinit))
         . = ALIGN(4);
     __noinit_end__   = .;
    } > dtcm
    .bss (NOLOAD):
    {
        . = ALIGN(4);
        __bss_start__ = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        __bss_end__ = .;
    } > dtcm

    .heap (NOLOAD):
    {
        . = ALIGN(8);
        __HeapBase = .;
        __end__ = .;
        end = __end__;
        KEEP(*(.heap*))
        . = ALIGN(8);
        . = ORIGIN(dtcm) + LENGTH(dtcm) - __StackSize;
        __HeapLimit = .;
    } > dtcm

    . = ALIGN(4);
    __cy_sram_code_lma_start = __data_lma__ + (__data_end__ - __data_start__);

     /* LMA SRAM VMA Flash */
    .cy_sram_code ORIGIN(sram_code) : AT(__cy_sram_code_lma_start)
    {
        . = ALIGN(4);
         __cy_sram_code_vma_start__ = .;
        KEEP(*(.cy_sram_code))
        . = ALIGN(4);
        __cy_sram_code_vma_end__ = .;

    }  > sram_code
     __cy_sram_code_lma_end = __cy_sram_code_lma_start + __cy_sram_code_vma_end__ -__cy_sram_code_vma_start__ ;
    . = __cy_sram_code_lma_end;

    .reserved_socmem (NOLOAD):
    {
      . = . + RESERVED_SOCMEM_MEM_SIZE; /* reserved socmem memory */
    }  > reserved_socmem

    .cy_sharedmem (NOLOAD):
    {
        . = ALIGN(4);
        __public_ram_start__ = .;
        KEEP(*(.cy_sharedmem))
        . = ALIGN(4);
        __public_ram_end__ = .;
    } > shared_ram1


    . = ALIGN(32);
    __StackTop = ORIGIN(dtcm) + LENGTH(dtcm);
    __StackLimit = __StackTop - __StackSize;
    PROVIDE(__stack = __StackTop);

    /* Check if data + heap + stack exceeds RAM limit */
    ASSERT(__StackLimit >= __HeapLimit, "region RAM overflowed with stack")

}
