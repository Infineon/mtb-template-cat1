/***************************************************************************//**
* \file startup_cat1d.h
* \version 1.0
*
* \brief Common startup header file for CAT1D devices. 
*
********************************************************************************
* \copyright
* Copyright 2016-2021 Cypress Semiconductor Corporation
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


#ifndef STARTUP_CAT1D_H_
#define STARTUP_CAT1D_H_

#if defined(EXPLORER_PSVP)
    #include "explorer_config.h"
#endif

#if defined (CY_DOXYGEN) || defined (EXPLORER_PSVP) /* Declarations for EXPLORER */

#define CM33_FIXED_EXP_NR       (15u)
#define VECTORTABLE_SIZE        (MXCM33_SYSTEM_INT_NR + CM33_FIXED_EXP_NR + 1u) /* +1 is for Stack pointer */
#define VECTORTABLE_ALIGN       (512) /* alignment for 85 entries (85x4=340) is 512 bytes */

#if defined(__ARMCC_VERSION) || defined (CY_DOXYGEN)
    typedef void(* ExecFuncPtrRw)(void) interrupt_type;
    typedef  void(* const ExecFuncPtr)(void) interrupt_type;     /* typedef for the function pointers in the vector table */
    extern ExecFuncPtrRw __ns_vector_table_rw[VECTORTABLE_SIZE] __attribute__( ( section(".bss.noinit.RESET_RAM"))) __attribute__((aligned(VECTORTABLE_ALIGN)));  /**< Non-secure vector table in flash/ROM */
#elif defined (__GNUC__) || defined (CY_DOXYGEN)
    #define interrupt_type __attribute__((interrupt))
    typedef void(* interrupt_type ExecFuncPtrRw)(void);
    typedef void(* interrupt_type ExecFuncPtr)(void) ;           /* typedef for the function pointers in the vector table */
    extern ExecFuncPtrRw __ns_vector_table_rw[VECTORTABLE_SIZE]   __attribute__( ( section(".ram_vectors"))) __attribute__((aligned(VECTORTABLE_ALIGN)));  /**< Non-secure vector table in flash/ROM */
#elif defined (__ICCARM__) || defined (CY_DOXYGEN)
    typedef interrupt_type void(* ExecFuncPtrRw)(void) ;
    typedef interrupt_type void(* const ExecFuncPtr)(void) ;     /* typedef for the function pointers in the vector table */
    extern ExecFuncPtrRw __ns_vector_table_rw[VECTORTABLE_SIZE]   __attribute__( ( section(".intvec_ram"))) __attribute__((aligned(VECTORTABLE_ALIGN)));  /**< Non-secure vector table in flash/ROM */
#else
    #error "An unsupported toolchain"
#endif  /* (__ARMCC_VERSION) */
extern ExecFuncPtr __ns_vector_table[]; /**< Non-secure vector table in non-secure SRAM */
#endif

#if defined (CY_DOXYGEN) || defined (EXPLORER_PSVP) /* Declarations for EXPLORER */
#if defined (CY_PDL_TZ_ENABLED) || defined (CY_DOXYGEN) /* Declarations for secure world */
#if defined(__ARMCC_VERSION)
    typedef void(* const ExecFuncPtr)(void) interrupt_type; /* typedef for the function pointers in the vector table */
    typedef void(* ExecFuncPtrRw)(void) interrupt_type;
    extern ExecFuncPtrRw __s_vector_table_rw[VECTORTABLE_SIZE]    __attribute__( ( section( ".bss.noinit"))) __attribute__((aligned(VECTORTABLE_ALIGN))); /**< Secure vector table in flash/ROM */
#elif defined (__GNUC__)
    typedef void(* interrupt_type ExecFuncPtr)(void) ;           /* typedef for the function pointers in the vector table */
    typedef void(* interrupt_type ExecFuncPtrRw)(void);
    extern ExecFuncPtrRw __s_vector_table_rw[VECTORTABLE_SIZE]   __attribute__( ( section(".noinit"))) __attribute__((aligned(VECTORTABLE_ALIGN))); /**< Secure vector table in flash/ROM */
#elif defined (__ICCARM__)
    typedef interrupt_type void(* const ExecFuncPtr)(void) ; /* typedef for the function pointers in the vector table */
    typedef interrupt_type void(* ExecFuncPtrRw)(void) ;
    extern ExecFuncPtrRw __s_vector_table_rw[VECTORTABLE_SIZE]   __attribute__( ( section(".noinit"))) __attribute__((aligned(VECTORTABLE_ALIGN))); /**< Secure vector table in flash/ROM */
#endif
extern ExecFuncPtr __s_vector_table[] ; /**< secure vector table in secure SRAM */
#endif
#if (!defined (CY_PDL_TZ_ENABLED)) || defined (CY_DOXYGEN) /* Declarations for non-secure world */
#if defined(__ARMCC_VERSION)
    typedef  void(* const ExecFuncPtr)(void) interrupt_type;     /* typedef for the function pointers in the vector table */
    typedef void(* ExecFuncPtrRw)(void) interrupt_type;
    extern ExecFuncPtrRw __ns_vector_table_rw[VECTORTABLE_SIZE]    __attribute__( ( section( ".bss.noinit"))) __attribute__((aligned(VECTORTABLE_ALIGN)));
#elif defined (__GNUC__)
    typedef void(* interrupt_type ExecFuncPtr)(void) ;           /* typedef for the function pointers in the vector table */
    typedef void(* interrupt_type ExecFuncPtrRw)(void);
    extern ExecFuncPtrRw __ns_vector_table_rw[VECTORTABLE_SIZE]   __attribute__( ( section(".noinit"))) __attribute__((aligned(VECTORTABLE_ALIGN)));
#elif defined (__ICCARM__)
    typedef interrupt_type void(* const ExecFuncPtr)(void) ;     /* typedef for the function pointers in the vector table */
    typedef interrupt_type void(* ExecFuncPtrRw)(void) ;
    extern ExecFuncPtrRw __ns_vector_table_rw[VECTORTABLE_SIZE]   __attribute__( ( section(".noinit"))) __attribute__((aligned(VECTORTABLE_ALIGN)));
#endif
extern ExecFuncPtr __ns_vector_table[]; /**< Non-secure vector table in non-secure SRAM */
#endif
#endif

#endif /* STARTUP_CAT1D_H_ */

