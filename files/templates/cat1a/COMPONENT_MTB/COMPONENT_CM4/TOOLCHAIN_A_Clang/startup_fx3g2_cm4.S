/**************************************************************************//**
 * @file     startup_fx3g2_cm4.S
 * @brief    CMSIS Core Device Startup File for
 *           ARMCM4 Device Series
 * @version  V5.00
 * @date     02. March 2016
 ******************************************************************************/
/*
 * Copyright (c) 2009-2016 ARM Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

    /* Address of the NMI handler */
    #define CY_NMI_HANLDER_ADDR         0x0000000D

    /* The CPU VTOR register */
    #define CY_CPU_VTOR_ADDR            0xE000ED08

    .syntax    unified
    .section  __STACK , __stack
    .align 3

#ifdef __STACK_SIZE
    .equ    Stack_Size, __STACK_SIZE
#else
    .equ    Stack_Size, 0x00001000
#endif
    .globl    __StackTop
    .globl    __StackLimit

__StackLimit:
    .space    Stack_Size
    .equ    __StackTop, . - Stack_Size

    .section __HEAP, __heap
    .align    3
#ifdef __HEAP_SIZE
    .equ    Heap_Size, __HEAP_SIZE
#else
    .equ    Heap_Size, 0x00000400
#endif
    .globl    __HeapBase
__HeapBase:
    .if    Heap_Size
    .space    Heap_Size
    .endif

    .section __VECT, ___Vectors
    .align 2
    .globl    ___Vectors
___Vectors:
    .long    __StackTop            /* Top of Stack */
    .long    Reset_Handler         /* Reset Handler */
    .long    CY_NMI_HANLDER_ADDR   /* NMI Handler */
    .long    HardFault_Handler     /* Hard Fault Handler */
    .long    MemManage_Handler     /* MPU Fault Handler */
    .long    BusFault_Handler      /* Bus Fault Handler */
    .long    UsageFault_Handler    /* Usage Fault Handler */
    .long    0                     /* Reserved */
    .long    0                     /* Reserved */
    .long    0                     /* Reserved */
    .long    0                     /* Reserved */
    .long    SVC_Handler           /* SVCall Handler */
    .long    DebugMon_Handler      /* Debug Monitor Handler */
    .long    0                     /* Reserved */
    .long    PendSV_Handler        /* PendSV Handler */
    .long    SysTick_Handler       /* SysTick Handler */

     /* External interrupts                             Description */
    .long    ioss_interrupts_gpio_0_IRQHandler       /* GPIO Port Interrupt #0 */
    .long    ioss_interrupts_gpio_1_IRQHandler       /* GPIO Port Interrupt #1 */
    .long    ioss_interrupts_gpio_4_IRQHandler       /* GPIO Port Interrupt #4 */
    .long    ioss_interrupts_gpio_5_IRQHandler       /* GPIO Port Interrupt #5 */
    .long    ioss_interrupts_gpio_6_IRQHandler       /* GPIO Port Interrupt #6 */
    .long    ioss_interrupts_gpio_7_IRQHandler       /* GPIO Port Interrupt #7 */
    .long    ioss_interrupts_gpio_8_IRQHandler       /* GPIO Port Interrupt #8 */
    .long    ioss_interrupts_gpio_9_IRQHandler       /* GPIO Port Interrupt #9 */
    .long    ioss_interrupts_gpio_10_IRQHandler      /* GPIO Port Interrupt #10 */
    .long    ioss_interrupts_gpio_11_IRQHandler      /* GPIO Port Interrupt #11 */
    .long    ioss_interrupts_gpio_12_IRQHandler      /* GPIO Port Interrupt #12 */
    .long    ioss_interrupts_gpio_13_IRQHandler      /* GPIO Port Interrupt #13 */
    .long    ioss_interrupt_gpio_IRQHandler          /* GPIO All Ports */
    .long    ioss_interrupt_vdd_IRQHandler           /* GPIO Supply Detect Interrupt */
    .long    scb_0_interrupt_IRQHandler              /* Serial Communication Block #0 (DeepSleep capable) */
    .long    srss_interrupt_mcwdt_0_IRQHandler       /* Multi Counter Watchdog Timer interrupt */
    .long    srss_interrupt_mcwdt_1_IRQHandler       /* Multi Counter Watchdog Timer interrupt */
    .long    usbhs_interrupts_dpslp_IRQHandler       /* USBHS device interrupt (DeepSleep capable) */
    .long    srss_interrupt_IRQHandler               /* Other combined Interrupts for SRSS (LVD, WDT, CLKCAL) */
    .long    cpuss_interrupts_ipc_0_IRQHandler       /* CPUSS Inter Process Communication Interrupt #0 */
    .long    cpuss_interrupts_ipc_1_IRQHandler       /* CPUSS Inter Process Communication Interrupt #1 */
    .long    cpuss_interrupts_ipc_2_IRQHandler       /* CPUSS Inter Process Communication Interrupt #2 */
    .long    cpuss_interrupts_ipc_3_IRQHandler       /* CPUSS Inter Process Communication Interrupt #3 */
    .long    cpuss_interrupts_ipc_4_IRQHandler       /* CPUSS Inter Process Communication Interrupt #4 */
    .long    cpuss_interrupts_ipc_5_IRQHandler       /* CPUSS Inter Process Communication Interrupt #5 */
    .long    cpuss_interrupts_ipc_6_IRQHandler       /* CPUSS Inter Process Communication Interrupt #6 */
    .long    cpuss_interrupts_ipc_7_IRQHandler       /* CPUSS Inter Process Communication Interrupt #7 */
    .long    cpuss_interrupts_ipc_8_IRQHandler       /* CPUSS Inter Process Communication Interrupt #8 */
    .long    cpuss_interrupts_ipc_9_IRQHandler       /* CPUSS Inter Process Communication Interrupt #9 */
    .long    cpuss_interrupts_ipc_10_IRQHandler      /* CPUSS Inter Process Communication Interrupt #10 */
    .long    cpuss_interrupts_ipc_11_IRQHandler      /* CPUSS Inter Process Communication Interrupt #11 */
    .long    cpuss_interrupts_ipc_12_IRQHandler      /* CPUSS Inter Process Communication Interrupt #12 */
    .long    cpuss_interrupts_ipc_13_IRQHandler      /* CPUSS Inter Process Communication Interrupt #13 */
    .long    cpuss_interrupts_ipc_14_IRQHandler      /* CPUSS Inter Process Communication Interrupt #14 */
    .long    cpuss_interrupts_ipc_15_IRQHandler      /* CPUSS Inter Process Communication Interrupt #15 */
    .long    0                                       /* Reserved */
    .long    0                                       /* Reserved */
    .long    0                                       /* Reserved */
    .long    0                                       /* Reserved */
    .long    0                                       /* Reserved */
    .long    0                                       /* Reserved */
    .long    0                                       /* Reserved */
    .long    0                                       /* Reserved */
    .long    0                                       /* Reserved */
    .long    0                                       /* Reserved */
    .long    0                                       /* Reserved */
    .long    scb_1_interrupt_IRQHandler              /* Serial Communication Block #1 */
    .long    scb_2_interrupt_IRQHandler              /* Serial Communication Block #2 */
    .long    scb_3_interrupt_IRQHandler              /* Serial Communication Block #3 */
    .long    scb_4_interrupt_IRQHandler              /* Serial Communication Block #4 */
    .long    scb_5_interrupt_IRQHandler              /* Serial Communication Block #5 */
    .long    scb_6_interrupt_IRQHandler              /* Serial Communication Block #6 */
    .long    cpuss_interrupts_dmac_0_IRQHandler      /* CPUSS DMAC, Channel #0 */
    .long    cpuss_interrupts_dmac_1_IRQHandler      /* CPUSS DMAC, Channel #1 */
    .long    cpuss_interrupts_dmac_2_IRQHandler      /* CPUSS DMAC, Channel #2 */
    .long    cpuss_interrupts_dmac_3_IRQHandler      /* CPUSS DMAC, Channel #3 */
    .long    cpuss_interrupts_dmac_4_IRQHandler      /* CPUSS DMAC, Channel #4 */
    .long    cpuss_interrupts_dmac_5_IRQHandler      /* CPUSS DMAC, Channel #5 */
    .long    cpuss_interrupts_dw0_0_IRQHandler       /* CPUSS DataWire #0, Channel #0 */
    .long    cpuss_interrupts_dw0_1_IRQHandler       /* CPUSS DataWire #0, Channel #1 */
    .long    cpuss_interrupts_dw0_2_IRQHandler       /* CPUSS DataWire #0, Channel #2 */
    .long    cpuss_interrupts_dw0_3_IRQHandler       /* CPUSS DataWire #0, Channel #3 */
    .long    cpuss_interrupts_dw0_4_IRQHandler       /* CPUSS DataWire #0, Channel #4 */
    .long    cpuss_interrupts_dw0_5_IRQHandler       /* CPUSS DataWire #0, Channel #5 */
    .long    cpuss_interrupts_dw0_6_IRQHandler       /* CPUSS DataWire #0, Channel #6 */
    .long    cpuss_interrupts_dw0_7_IRQHandler       /* CPUSS DataWire #0, Channel #7 */
    .long    cpuss_interrupts_dw0_8_IRQHandler       /* CPUSS DataWire #0, Channel #8 */
    .long    cpuss_interrupts_dw0_9_IRQHandler       /* CPUSS DataWire #0, Channel #9 */
    .long    cpuss_interrupts_dw0_10_IRQHandler      /* CPUSS DataWire #0, Channel #10 */
    .long    cpuss_interrupts_dw0_11_IRQHandler      /* CPUSS DataWire #0, Channel #11 */
    .long    cpuss_interrupts_dw0_12_IRQHandler      /* CPUSS DataWire #0, Channel #12 */
    .long    cpuss_interrupts_dw0_13_IRQHandler      /* CPUSS DataWire #0, Channel #13 */
    .long    cpuss_interrupts_dw0_14_IRQHandler      /* CPUSS DataWire #0, Channel #14 */
    .long    cpuss_interrupts_dw0_15_IRQHandler      /* CPUSS DataWire #0, Channel #15 */
    .long    cpuss_interrupts_dw0_16_IRQHandler      /* CPUSS DataWire #0, Channel #16 */
    .long    cpuss_interrupts_dw0_17_IRQHandler      /* CPUSS DataWire #0, Channel #17 */
    .long    cpuss_interrupts_dw0_18_IRQHandler      /* CPUSS DataWire #0, Channel #18 */
    .long    cpuss_interrupts_dw0_19_IRQHandler      /* CPUSS DataWire #0, Channel #19 */
    .long    cpuss_interrupts_dw0_20_IRQHandler      /* CPUSS DataWire #0, Channel #20 */
    .long    cpuss_interrupts_dw0_21_IRQHandler      /* CPUSS DataWire #0, Channel #21 */
    .long    cpuss_interrupts_dw0_22_IRQHandler      /* CPUSS DataWire #0, Channel #22 */
    .long    cpuss_interrupts_dw0_23_IRQHandler      /* CPUSS DataWire #0, Channel #23 */
    .long    cpuss_interrupts_dw1_0_IRQHandler       /* CPUSS DataWire #1, Channel #0 */
    .long    cpuss_interrupts_dw1_1_IRQHandler       /* CPUSS DataWire #1, Channel #1 */
    .long    cpuss_interrupts_dw1_2_IRQHandler       /* CPUSS DataWire #1, Channel #2 */
    .long    cpuss_interrupts_dw1_3_IRQHandler       /* CPUSS DataWire #1, Channel #3 */
    .long    cpuss_interrupts_dw1_4_IRQHandler       /* CPUSS DataWire #1, Channel #4 */
    .long    cpuss_interrupts_dw1_5_IRQHandler       /* CPUSS DataWire #1, Channel #5 */
    .long    cpuss_interrupts_dw1_6_IRQHandler       /* CPUSS DataWire #1, Channel #6 */
    .long    cpuss_interrupts_dw1_7_IRQHandler       /* CPUSS DataWire #1, Channel #7 */
    .long    cpuss_interrupts_dw1_8_IRQHandler       /* CPUSS DataWire #1, Channel #8 */
    .long    cpuss_interrupts_dw1_9_IRQHandler       /* CPUSS DataWire #1, Channel #9 */
    .long    cpuss_interrupts_dw1_10_IRQHandler      /* CPUSS DataWire #1, Channel #10 */
    .long    cpuss_interrupts_dw1_11_IRQHandler      /* CPUSS DataWire #1, Channel #11 */
    .long    cpuss_interrupts_dw1_12_IRQHandler      /* CPUSS DataWire #1, Channel #12 */
    .long    cpuss_interrupts_dw1_13_IRQHandler      /* CPUSS DataWire #1, Channel #13 */
    .long    cpuss_interrupts_dw1_14_IRQHandler      /* CPUSS DataWire #1, Channel #14 */
    .long    cpuss_interrupts_dw1_15_IRQHandler      /* CPUSS DataWire #1, Channel #15 */
    .long    cpuss_interrupts_dw1_16_IRQHandler      /* CPUSS DataWire #1, Channel #16 */
    .long    cpuss_interrupts_dw1_17_IRQHandler      /* CPUSS DataWire #1, Channel #17 */
    .long    cpuss_interrupts_dw1_18_IRQHandler      /* CPUSS DataWire #1, Channel #18 */
    .long    cpuss_interrupts_dw1_19_IRQHandler      /* CPUSS DataWire #1, Channel #19 */
    .long    cpuss_interrupts_dw1_20_IRQHandler      /* CPUSS DataWire #1, Channel #20 */
    .long    cpuss_interrupts_dw1_21_IRQHandler      /* CPUSS DataWire #1, Channel #21 */
    .long    cpuss_interrupts_dw1_22_IRQHandler      /* CPUSS DataWire #1, Channel #22 */
    .long    cpuss_interrupts_dw1_23_IRQHandler      /* CPUSS DataWire #1, Channel #23 */
    .long    cpuss_interrupts_fault_0_IRQHandler     /* CPUSS Fault Structure Interrupt #0 */
    .long    cpuss_interrupts_fault_1_IRQHandler     /* CPUSS Fault Structure Interrupt #1 */
    .long    cpuss_interrupt_crypto_IRQHandler       /* CRYPTO Accelerator Interrupt */
    .long    cpuss_interrupt_fm_IRQHandler           /* FLASH Macro Interrupt */
    .long    cpuss_interrupts_cm4_fp_IRQHandler      /* Floating Point operation fault */
    .long    cpuss_interrupts_cm0_cti_0_IRQHandler   /* CM0+ CTI #0 */
    .long    cpuss_interrupts_cm0_cti_1_IRQHandler   /* CM0+ CTI #1 */
    .long    cpuss_interrupts_cm4_cti_0_IRQHandler   /* CM4 CTI #0 */
    .long    cpuss_interrupts_cm4_cti_1_IRQHandler   /* CM4 CTI #1 */
    .long    tcpwm_0_interrupts_0_IRQHandler         /* TCPWM #0, Counter #0 */
    .long    tcpwm_0_interrupts_1_IRQHandler         /* TCPWM #0, Counter #1 */
    .long    tcpwm_0_interrupts_2_IRQHandler         /* TCPWM #0, Counter #2 */
    .long    tcpwm_0_interrupts_3_IRQHandler         /* TCPWM #0, Counter #3 */
    .long    tcpwm_0_interrupts_4_IRQHandler         /* TCPWM #0, Counter #4 */
    .long    tcpwm_0_interrupts_5_IRQHandler         /* TCPWM #0, Counter #5 */
    .long    tcpwm_0_interrupts_6_IRQHandler         /* TCPWM #0, Counter #6 */
    .long    tcpwm_0_interrupts_7_IRQHandler         /* TCPWM #0, Counter #7 */
    .long    tdm_0_interrupts_rx_IRQHandler          /* TDM #0 Rx Interrupt. */
    .long    tdm_0_interrupts_tx_IRQHandler          /* TDM #0 Tx Interrupt. */
    .long    smif_interrupt_IRQHandler               /* Serial Memory Interface interrupt */
    .long    usb_interrupt_hi_IRQHandler             /* USB Interrupt */
    .long    usb_interrupt_med_IRQHandler            /* USB Interrupt */
    .long    usb_interrupt_lo_IRQHandler             /* USB Interrupt */
    .long    usbhs_interrupts_actv_IRQHandler        /* USBHS Device Active interrupt */
    .long    canfd_0_interrupt0_IRQHandler           /* Can #0, Consolidated interrupt #0 */
    .long    canfd_0_interrupts0_0_IRQHandler        /* CAN #0, Interrupt #0, Channel #0 */
    .long    canfd_0_interrupts1_0_IRQHandler        /* CAN #0, Interrupt #1, Channel #0 */
    .long    pdm_0_interrupts_0_IRQHandler           /* PDM #0 Interrupt #0 */
    .long    pdm_0_interrupts_1_IRQHandler           /* PDM #0 Interrupt #1 */
    .long    lvds_core_interrupt_IRQHandler          /* LVDS Core Interrupt */
    .long    lvds_dma_interrupt_IRQHandler           /* LVDS DMA Interrupt */
    .long    lvds_wkup_interrupt_IRQHandler          /* LVDS Wake-Up Interrupt */
    .long    usb32_egrs_dma_interrupt_IRQHandler     /* USB32DEV Egress DMA Interrupt */
    .long    usb32_ingrs_dma_interrupt_IRQHandler    /* USB32DEV Ingress DMA Interrupt */
    .long    usb32_actv_interrupt_IRQHandler         /* USB32DEV Core Interrupt */
    .long    usb32_wkup_interrupt_IRQHandler         /* USB32DEV Wake-Up Interrupt */

    .equ    __VectorsSize, . - ___Vectors

    .section __RAMVECTORS, ___ramVectors
    .align 2
    .globl ___ramVectors

___ramVectors:
    .space  __VectorsSize


    .text
    .thumb_func
    .align 2
    /* Reset handler */
    .globl Reset_Handler

Reset_Handler:
    bl Cy_OnResetUser
    cpsid i

/*  Single section scheme.
 *
 *  The ranges of copy from/to are specified by following symbols
 *    __etext: LMA of start of the section to copy from. Usually end of text
 *    __data_start__: VMA of start of the section to copy to
 *    __data_end__: VMA of end of the section to copy to
 *
 *  All addresses must be aligned to 4 bytes boundary.
 */
    ldr    r0, =___ramVectors
    ldr    r1, =___Vectors
    ldr    r2, =__VectorsSize
    bl     _memcpy

    ldr    r0, =segment$start$__DATA
    ldr    r1, =segment$end$__TEXT
    ldr    r2, =section$start$__DATA$__zerofill
    sub    r2, r0
    bl     _memcpy

    ldr    r0, =section$start$__DATA$__zerofill
    eor    r1, r1
    ldr    r2, =section$end$__DATA$__zerofill
    sub    r2, r0
    bl     _memset

    /* Update Vector Table Offset Register. */
    ldr r0, =___ramVectors
    ldr r1, =CY_CPU_VTOR_ADDR
    str r0, [r1]
    dsb 0xF

    /* Enable the FPU if used */
    bl _Cy_SystemInitFpuEnable

    bl _HeapInit
#ifndef __NO_SYSTEM_INIT
    bl  _SystemInit
#endif

    bl  _main

    /* Should never get here */
    b   .

    .pool

    .text
    .thumb
    .thumb_func
    .align 2

    /* Device startup customization */
    .weak_definition   Cy_OnResetUser
    .global Cy_OnResetUser, Cy_OnResetUser
Cy_OnResetUser:
    bx lr

    .text
    .align    1
    .thumb_func
    .weak_reference    Default_Handler

Default_Handler:
    b    .

    .text
    .thumb_func
    .align  2
    .weak_definition    Cy_SysLib_FaultHandler

Cy_SysLib_FaultHandler:
    b    .

    .text
    .thumb_func
    .align  2

Fault_Handler:
    /* Storing LR content for Creator call stack trace */
    push {LR}
    movs r0, #4
    mov r1, LR
    tst r0, r1
    beq .L_MSP
    mrs r0, PSP
    b .L_API_call
.L_MSP:
    mrs r0, MSP
    /* Compensation of stack pointer address due to pushing 4 bytes of LR */
    adds r0, r0, #4
    nop
.L_API_call:
    bl Cy_SysLib_FaultHandler
    b   .

.macro    def_fault_Handler    fault_handler_name
    .weak_definition    \fault_handler_name
    .set    \fault_handler_name, Fault_Handler
    .endm

/*    Macro to define default handlers. Default handler
 *    will be weak symbol and just dead loops. They can be
 *    overwritten by other handlers */
    .macro    def_irq_handler    handler_name
    .weak_definition    \handler_name
    .set    \handler_name, Default_Handler
    .endm

    def_irq_handler    NMI_Handler

    def_fault_Handler HardFault_Handler
    def_fault_Handler MemManage_Handler
    def_fault_Handler BusFault_Handler
    def_fault_Handler UsageFault_Handler

    def_irq_handler    SVC_Handler
    def_irq_handler    DebugMon_Handler
    def_irq_handler    PendSV_Handler
    def_irq_handler    SysTick_Handler

    def_irq_handler    ioss_interrupts_gpio_0_IRQHandler       /* GPIO Port Interrupt #0 */
    def_irq_handler    ioss_interrupts_gpio_1_IRQHandler       /* GPIO Port Interrupt #1 */
    def_irq_handler    ioss_interrupts_gpio_4_IRQHandler       /* GPIO Port Interrupt #4 */
    def_irq_handler    ioss_interrupts_gpio_5_IRQHandler       /* GPIO Port Interrupt #5 */
    def_irq_handler    ioss_interrupts_gpio_6_IRQHandler       /* GPIO Port Interrupt #6 */
    def_irq_handler    ioss_interrupts_gpio_7_IRQHandler       /* GPIO Port Interrupt #7 */
    def_irq_handler    ioss_interrupts_gpio_8_IRQHandler       /* GPIO Port Interrupt #8 */
    def_irq_handler    ioss_interrupts_gpio_9_IRQHandler       /* GPIO Port Interrupt #9 */
    def_irq_handler    ioss_interrupts_gpio_10_IRQHandler      /* GPIO Port Interrupt #10 */
    def_irq_handler    ioss_interrupts_gpio_11_IRQHandler      /* GPIO Port Interrupt #11 */
    def_irq_handler    ioss_interrupts_gpio_12_IRQHandler      /* GPIO Port Interrupt #12 */
    def_irq_handler    ioss_interrupts_gpio_13_IRQHandler      /* GPIO Port Interrupt #13 */
    def_irq_handler    ioss_interrupt_gpio_IRQHandler          /* GPIO All Ports */
    def_irq_handler    ioss_interrupt_vdd_IRQHandler           /* GPIO Supply Detect Interrupt */
    def_irq_handler    scb_0_interrupt_IRQHandler              /* Serial Communication Block #0 (DeepSleep capable) */
    def_irq_handler    srss_interrupt_mcwdt_0_IRQHandler       /* Multi Counter Watchdog Timer interrupt */
    def_irq_handler    srss_interrupt_mcwdt_1_IRQHandler       /* Multi Counter Watchdog Timer interrupt */
    def_irq_handler    usbhs_interrupts_dpslp_IRQHandler       /* USBHS device interrupt (DeepSleep capable) */
    def_irq_handler    srss_interrupt_IRQHandler               /* Other combined Interrupts for SRSS (LVD, WDT, CLKCAL) */
    def_irq_handler    cpuss_interrupts_ipc_0_IRQHandler       /* CPUSS Inter Process Communication Interrupt #0 */
    def_irq_handler    cpuss_interrupts_ipc_1_IRQHandler       /* CPUSS Inter Process Communication Interrupt #1 */
    def_irq_handler    cpuss_interrupts_ipc_2_IRQHandler       /* CPUSS Inter Process Communication Interrupt #2 */
    def_irq_handler    cpuss_interrupts_ipc_3_IRQHandler       /* CPUSS Inter Process Communication Interrupt #3 */
    def_irq_handler    cpuss_interrupts_ipc_4_IRQHandler       /* CPUSS Inter Process Communication Interrupt #4 */
    def_irq_handler    cpuss_interrupts_ipc_5_IRQHandler       /* CPUSS Inter Process Communication Interrupt #5 */
    def_irq_handler    cpuss_interrupts_ipc_6_IRQHandler       /* CPUSS Inter Process Communication Interrupt #6 */
    def_irq_handler    cpuss_interrupts_ipc_7_IRQHandler       /* CPUSS Inter Process Communication Interrupt #7 */
    def_irq_handler    cpuss_interrupts_ipc_8_IRQHandler       /* CPUSS Inter Process Communication Interrupt #8 */
    def_irq_handler    cpuss_interrupts_ipc_9_IRQHandler       /* CPUSS Inter Process Communication Interrupt #9 */
    def_irq_handler    cpuss_interrupts_ipc_10_IRQHandler      /* CPUSS Inter Process Communication Interrupt #10 */
    def_irq_handler    cpuss_interrupts_ipc_11_IRQHandler      /* CPUSS Inter Process Communication Interrupt #11 */
    def_irq_handler    cpuss_interrupts_ipc_12_IRQHandler      /* CPUSS Inter Process Communication Interrupt #12 */
    def_irq_handler    cpuss_interrupts_ipc_13_IRQHandler      /* CPUSS Inter Process Communication Interrupt #13 */
    def_irq_handler    cpuss_interrupts_ipc_14_IRQHandler      /* CPUSS Inter Process Communication Interrupt #14 */
    def_irq_handler    cpuss_interrupts_ipc_15_IRQHandler      /* CPUSS Inter Process Communication Interrupt #15 */
    def_irq_handler    scb_1_interrupt_IRQHandler              /* Serial Communication Block #1 */
    def_irq_handler    scb_2_interrupt_IRQHandler              /* Serial Communication Block #2 */
    def_irq_handler    scb_3_interrupt_IRQHandler              /* Serial Communication Block #3 */
    def_irq_handler    scb_4_interrupt_IRQHandler              /* Serial Communication Block #4 */
    def_irq_handler    scb_5_interrupt_IRQHandler              /* Serial Communication Block #5 */
    def_irq_handler    scb_6_interrupt_IRQHandler              /* Serial Communication Block #6 */
    def_irq_handler    cpuss_interrupts_dmac_0_IRQHandler      /* CPUSS DMAC, Channel #0 */
    def_irq_handler    cpuss_interrupts_dmac_1_IRQHandler      /* CPUSS DMAC, Channel #1 */
    def_irq_handler    cpuss_interrupts_dmac_2_IRQHandler      /* CPUSS DMAC, Channel #2 */
    def_irq_handler    cpuss_interrupts_dmac_3_IRQHandler      /* CPUSS DMAC, Channel #3 */
    def_irq_handler    cpuss_interrupts_dmac_4_IRQHandler      /* CPUSS DMAC, Channel #4 */
    def_irq_handler    cpuss_interrupts_dmac_5_IRQHandler      /* CPUSS DMAC, Channel #5 */
    def_irq_handler    cpuss_interrupts_dw0_0_IRQHandler       /* CPUSS DataWire #0, Channel #0 */
    def_irq_handler    cpuss_interrupts_dw0_1_IRQHandler       /* CPUSS DataWire #0, Channel #1 */
    def_irq_handler    cpuss_interrupts_dw0_2_IRQHandler       /* CPUSS DataWire #0, Channel #2 */
    def_irq_handler    cpuss_interrupts_dw0_3_IRQHandler       /* CPUSS DataWire #0, Channel #3 */
    def_irq_handler    cpuss_interrupts_dw0_4_IRQHandler       /* CPUSS DataWire #0, Channel #4 */
    def_irq_handler    cpuss_interrupts_dw0_5_IRQHandler       /* CPUSS DataWire #0, Channel #5 */
    def_irq_handler    cpuss_interrupts_dw0_6_IRQHandler       /* CPUSS DataWire #0, Channel #6 */
    def_irq_handler    cpuss_interrupts_dw0_7_IRQHandler       /* CPUSS DataWire #0, Channel #7 */
    def_irq_handler    cpuss_interrupts_dw0_8_IRQHandler       /* CPUSS DataWire #0, Channel #8 */
    def_irq_handler    cpuss_interrupts_dw0_9_IRQHandler       /* CPUSS DataWire #0, Channel #9 */
    def_irq_handler    cpuss_interrupts_dw0_10_IRQHandler      /* CPUSS DataWire #0, Channel #10 */
    def_irq_handler    cpuss_interrupts_dw0_11_IRQHandler      /* CPUSS DataWire #0, Channel #11 */
    def_irq_handler    cpuss_interrupts_dw0_12_IRQHandler      /* CPUSS DataWire #0, Channel #12 */
    def_irq_handler    cpuss_interrupts_dw0_13_IRQHandler      /* CPUSS DataWire #0, Channel #13 */
    def_irq_handler    cpuss_interrupts_dw0_14_IRQHandler      /* CPUSS DataWire #0, Channel #14 */
    def_irq_handler    cpuss_interrupts_dw0_15_IRQHandler      /* CPUSS DataWire #0, Channel #15 */
    def_irq_handler    cpuss_interrupts_dw0_16_IRQHandler      /* CPUSS DataWire #0, Channel #16 */
    def_irq_handler    cpuss_interrupts_dw0_17_IRQHandler      /* CPUSS DataWire #0, Channel #17 */
    def_irq_handler    cpuss_interrupts_dw0_18_IRQHandler      /* CPUSS DataWire #0, Channel #18 */
    def_irq_handler    cpuss_interrupts_dw0_19_IRQHandler      /* CPUSS DataWire #0, Channel #19 */
    def_irq_handler    cpuss_interrupts_dw0_20_IRQHandler      /* CPUSS DataWire #0, Channel #20 */
    def_irq_handler    cpuss_interrupts_dw0_21_IRQHandler      /* CPUSS DataWire #0, Channel #21 */
    def_irq_handler    cpuss_interrupts_dw0_22_IRQHandler      /* CPUSS DataWire #0, Channel #22 */
    def_irq_handler    cpuss_interrupts_dw0_23_IRQHandler      /* CPUSS DataWire #0, Channel #23 */
    def_irq_handler    cpuss_interrupts_dw1_0_IRQHandler       /* CPUSS DataWire #1, Channel #0 */
    def_irq_handler    cpuss_interrupts_dw1_1_IRQHandler       /* CPUSS DataWire #1, Channel #1 */
    def_irq_handler    cpuss_interrupts_dw1_2_IRQHandler       /* CPUSS DataWire #1, Channel #2 */
    def_irq_handler    cpuss_interrupts_dw1_3_IRQHandler       /* CPUSS DataWire #1, Channel #3 */
    def_irq_handler    cpuss_interrupts_dw1_4_IRQHandler       /* CPUSS DataWire #1, Channel #4 */
    def_irq_handler    cpuss_interrupts_dw1_5_IRQHandler       /* CPUSS DataWire #1, Channel #5 */
    def_irq_handler    cpuss_interrupts_dw1_6_IRQHandler       /* CPUSS DataWire #1, Channel #6 */
    def_irq_handler    cpuss_interrupts_dw1_7_IRQHandler       /* CPUSS DataWire #1, Channel #7 */
    def_irq_handler    cpuss_interrupts_dw1_8_IRQHandler       /* CPUSS DataWire #1, Channel #8 */
    def_irq_handler    cpuss_interrupts_dw1_9_IRQHandler       /* CPUSS DataWire #1, Channel #9 */
    def_irq_handler    cpuss_interrupts_dw1_10_IRQHandler      /* CPUSS DataWire #1, Channel #10 */
    def_irq_handler    cpuss_interrupts_dw1_11_IRQHandler      /* CPUSS DataWire #1, Channel #11 */
    def_irq_handler    cpuss_interrupts_dw1_12_IRQHandler      /* CPUSS DataWire #1, Channel #12 */
    def_irq_handler    cpuss_interrupts_dw1_13_IRQHandler      /* CPUSS DataWire #1, Channel #13 */
    def_irq_handler    cpuss_interrupts_dw1_14_IRQHandler      /* CPUSS DataWire #1, Channel #14 */
    def_irq_handler    cpuss_interrupts_dw1_15_IRQHandler      /* CPUSS DataWire #1, Channel #15 */
    def_irq_handler    cpuss_interrupts_dw1_16_IRQHandler      /* CPUSS DataWire #1, Channel #16 */
    def_irq_handler    cpuss_interrupts_dw1_17_IRQHandler      /* CPUSS DataWire #1, Channel #17 */
    def_irq_handler    cpuss_interrupts_dw1_18_IRQHandler      /* CPUSS DataWire #1, Channel #18 */
    def_irq_handler    cpuss_interrupts_dw1_19_IRQHandler      /* CPUSS DataWire #1, Channel #19 */
    def_irq_handler    cpuss_interrupts_dw1_20_IRQHandler      /* CPUSS DataWire #1, Channel #20 */
    def_irq_handler    cpuss_interrupts_dw1_21_IRQHandler      /* CPUSS DataWire #1, Channel #21 */
    def_irq_handler    cpuss_interrupts_dw1_22_IRQHandler      /* CPUSS DataWire #1, Channel #22 */
    def_irq_handler    cpuss_interrupts_dw1_23_IRQHandler      /* CPUSS DataWire #1, Channel #23 */
    def_irq_handler    cpuss_interrupts_fault_0_IRQHandler     /* CPUSS Fault Structure Interrupt #0 */
    def_irq_handler    cpuss_interrupts_fault_1_IRQHandler     /* CPUSS Fault Structure Interrupt #1 */
    def_irq_handler    cpuss_interrupt_crypto_IRQHandler       /* CRYPTO Accelerator Interrupt */
    def_irq_handler    cpuss_interrupt_fm_IRQHandler           /* FLASH Macro Interrupt */
    def_irq_handler    cpuss_interrupts_cm4_fp_IRQHandler      /* Floating Point operation fault */
    def_irq_handler    cpuss_interrupts_cm0_cti_0_IRQHandler   /* CM0+ CTI #0 */
    def_irq_handler    cpuss_interrupts_cm0_cti_1_IRQHandler   /* CM0+ CTI #1 */
    def_irq_handler    cpuss_interrupts_cm4_cti_0_IRQHandler   /* CM4 CTI #0 */
    def_irq_handler    cpuss_interrupts_cm4_cti_1_IRQHandler   /* CM4 CTI #1 */
    def_irq_handler    tcpwm_0_interrupts_0_IRQHandler         /* TCPWM #0, Counter #0 */
    def_irq_handler    tcpwm_0_interrupts_1_IRQHandler         /* TCPWM #0, Counter #1 */
    def_irq_handler    tcpwm_0_interrupts_2_IRQHandler         /* TCPWM #0, Counter #2 */
    def_irq_handler    tcpwm_0_interrupts_3_IRQHandler         /* TCPWM #0, Counter #3 */
    def_irq_handler    tcpwm_0_interrupts_4_IRQHandler         /* TCPWM #0, Counter #4 */
    def_irq_handler    tcpwm_0_interrupts_5_IRQHandler         /* TCPWM #0, Counter #5 */
    def_irq_handler    tcpwm_0_interrupts_6_IRQHandler         /* TCPWM #0, Counter #6 */
    def_irq_handler    tcpwm_0_interrupts_7_IRQHandler         /* TCPWM #0, Counter #7 */
    def_irq_handler    tdm_0_interrupts_rx_IRQHandler          /* TDM #0 Rx Interrupt. */
    def_irq_handler    tdm_0_interrupts_tx_IRQHandler          /* TDM #0 Tx Interrupt. */
    def_irq_handler    smif_interrupt_IRQHandler               /* Serial Memory Interface interrupt */
    def_irq_handler    usb_interrupt_hi_IRQHandler             /* USB Interrupt */
    def_irq_handler    usb_interrupt_med_IRQHandler            /* USB Interrupt */
    def_irq_handler    usb_interrupt_lo_IRQHandler             /* USB Interrupt */
    def_irq_handler    usbhs_interrupts_actv_IRQHandler        /* USBHS Device Active interrupt */
    def_irq_handler    canfd_0_interrupt0_IRQHandler           /* Can #0, Consolidated interrupt #0 */
    def_irq_handler    canfd_0_interrupts0_0_IRQHandler        /* CAN #0, Interrupt #0, Channel #0 */
    def_irq_handler    canfd_0_interrupts1_0_IRQHandler        /* CAN #0, Interrupt #1, Channel #0 */
    def_irq_handler    pdm_0_interrupts_0_IRQHandler           /* PDM #0 Interrupt #0 */
    def_irq_handler    pdm_0_interrupts_1_IRQHandler           /* PDM #0 Interrupt #1 */
    def_irq_handler    lvds_core_interrupt_IRQHandler          /* LVDS Core Interrupt */
    def_irq_handler    lvds_dma_interrupt_IRQHandler           /* LVDS DMA Interrupt */
    def_irq_handler    lvds_wkup_interrupt_IRQHandler          /* LVDS Wake-Up Interrupt */
    def_irq_handler    usb32_egrs_dma_interrupt_IRQHandler     /* USB32DEV Egress DMA Interrupt */
    def_irq_handler    usb32_ingrs_dma_interrupt_IRQHandler    /* USB32DEV Ingress DMA Interrupt */
    def_irq_handler    usb32_actv_interrupt_IRQHandler         /* USB32DEV Core Interrupt */
    def_irq_handler    usb32_wkup_interrupt_IRQHandler         /* USB32DEV Wake-Up Interrupt */

    .end


/* [] END OF FILE */
