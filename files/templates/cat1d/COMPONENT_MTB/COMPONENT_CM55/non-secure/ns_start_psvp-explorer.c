#if defined (EXPLORER_PSVP)

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include "explorer_psvp.h"
#include "startup_cat1d.h"
#include "cy_sysint.h"
#include "cy_syspm.h"
#include "cy_syslib.h"
#include "cmsis_compiler.h"


CY_MISRA_FP_BLOCK_START('MISRA C-2012 Rule 8.6', 3, \
'Checked manually. The definition is a part of linker script or application.');
CY_MISRA_DEVIATE_BLOCK_START('ARRAY_VS_SINGLETON', 1, \
'Checked manually. Using pointer as an array will not corrupt or misinterpret adjacent memory locations.');
CY_MISRA_DEVIATE_BLOCK_START('MISRA C-2012 Rule 18.1', 3, \
'Checked manually. Dereferencing a pointer to one beyond the end of an array will not result in undefined behaviour.');
CY_MISRA_DEVIATE_BLOCK_START('MISRA C-2012 Rule 18.3', 1, \
'Checked manually. Attempting to make comparisons between pointers will not result in undefined behaviour.');

/*
** Copyright (c) 2018 Arm Limited. All rights reserved.
*/

// Initial Setup
//----------------------------------------------------------------

#if defined (__ARMCC_VERSION)
extern uint32_t Region$$Table$$Base;
extern uint32_t Region$$Table$$Limit;
typedef  void(*pGenericFunction)(uint8_t *pSrc, uint8_t* pDst, uint32_t len);     /* typedef for the generic function pointers */
#endif

#define CY_SYSINT_EWIC_CTL            (0xE0047000U)	  /**< EWIC control register */
#define CY_SYSINT_EWIC_ENABLE_MSK     (0x1U) 		  /**< EWIC enable mask */

__WEAK interrupt_type void Reset_Handler(void);
interrupt_type void MemManage_Handler(void);
interrupt_type void BusFault_Handler(void);
interrupt_type void UsageFault_Handler(void);
__WEAK interrupt_type void SVC_Handler(void);
interrupt_type void DebugMon_Handler(void);
__WEAK interrupt_type void PendSV_Handler(void);
__WEAK interrupt_type void SysTick_Handler(void);
interrupt_type void InterruptHandler(void);
interrupt_type void NMIException_Handler(void);
__WEAK interrupt_type void HardFault_Handler(void);
void delay_infinite(void);
void SysLib_FaultHandler(uint32_t const *faultStackAddr);
extern int main(void);

#if defined (__ARMCC_VERSION)
void __attribute__((optnone)) Cy_RuntimeInit(void);
#else
void Cy_RuntimeInit(void);
#endif

#if defined(__ARMCC_VERSION)
extern unsigned int Image$$ARM_LIB_STACK$$ZI$$Limit;            /* for (default) One Region model */
interrupt_type extern void __main(void);
typedef void(* ExecFuncPtrRw)(void) interrupt_type;
ExecFuncPtrRw __ns_vector_table_rw[VECTORTABLE_SIZE] __attribute__( ( section(".bss.noinit.RESET_RAM"))) __attribute__((aligned(VECTORTABLE_ALIGN)));
#elif defined (__GNUC__)
extern unsigned int __StackTop;
extern uint32_t __StackLimit;
typedef void(* interrupt_type ExecFuncPtrRw)(void);
ExecFuncPtrRw __ns_vector_table_rw[VECTORTABLE_SIZE]   __attribute__( ( section(".ram_vectors"))) __attribute__((aligned(VECTORTABLE_ALIGN)));
#elif defined (__ICCARM__)
extern unsigned int CSTACK$$Limit;                      /* for (default) One Region model */
interrupt_type extern void  __cmain();
ExecFuncPtrRw __ns_vector_table_rw[VECTORTABLE_SIZE]   __attribute__( ( section(".intvec_ram"))) __attribute__((aligned(VECTORTABLE_ALIGN)));
#else
    #error "An unsupported toolchain"
#endif  /* (__ARMCC_VERSION) */

#if defined (__ARMCC_VERSION)
int main(void);
void __attribute__((optnone)) Cy_RuntimeInit(void)
{
    uint32_t j;

    uint32_t *region_table_start_ptr;
    uint32_t *region_table_end_ptr;
    uint32_t count = 0;
    uint32_t s_addr;
    uint32_t d_addr;
    uint32_t len;
    pGenericFunction pFunc;

    region_table_start_ptr = (uint32_t *)&Region$$Table$$Base;
    region_table_end_ptr = (uint32_t *)&Region$$Table$$Limit;

    /* Get the number of entries in the region table */
    count  = (region_table_end_ptr - region_table_start_ptr)/4;

    for (j = 0; j < count; j++)
    {
        s_addr = *region_table_start_ptr++;
        d_addr = *region_table_start_ptr++;
        len = *region_table_start_ptr++;
        pFunc = (pGenericFunction)((uint32_t)(*region_table_start_ptr++) | 0x1);

        /* Convert CBUS address for SRAM0 to SAHB address */
        if(((d_addr & (uint32_t)SRAM0_CBUS_START_MASK)) == (uint32_t)SRAM0_NS_CBUS)
            d_addr = (uint32_t)(d_addr - (uint32_t)SRAM0_NS_CBUS) + (uint32_t)SRAM0_NS_SAHB;

        /* Convert CBUS address for XIP to SAHB address */
        else if(((d_addr & (uint32_t)XIP_CBUS_START_MASK)) == (uint32_t)XIP_NS_CBUS)
            d_addr = (uint32_t)(d_addr - (uint32_t)XIP_NS_CBUS) + (uint32_t)XIP_NS_SAHB;

        pFunc((uint8_t*)s_addr, (uint8_t*)d_addr, len);
    }
    main();
}
#else
void Cy_RuntimeInit(void)
{
    __PROGRAM_START();
}
#endif

void SysLib_FaultHandler(uint32_t const *faultStackAddr)
{
    Cy_SysLib_FaultHandler(faultStackAddr);
}

// Exception Vector Table & Handlers
//----------------------------------------------------------------
interrupt_type void NMIException_Handler(void)
{
    __asm volatile(
        "bkpt #10\n"
        "B .\n"
    );
}

__WEAK interrupt_type void HardFault_Handler(void)
{
    __asm (
        "MRS R0, CONTROL\n"
        "TST R0, #2\n"
        "ITE EQ\n"
        "MRSEQ R0, MSP\n"
        "MRSNE R0, PSP\n"
        "B SysLib_FaultHandler\n"
    );
}

interrupt_type void MemManage_Handler(void)        {while(true){}}
interrupt_type void BusFault_Handler(void)         {while(true){}}
interrupt_type void UsageFault_Handler(void)       {while(true){}}
__WEAK interrupt_type void SVC_Handler(void)       {while(true){}}
interrupt_type void DebugMon_Handler(void)         {while(true){}}
__WEAK interrupt_type void PendSV_Handler(void)    {while(true){}}
__WEAK interrupt_type void SysTick_Handler(void)   {while(true){}}

interrupt_type void InterruptHandler(void)
{
    __asm volatile(
        "bkpt #1\n"
        "B .\n"
    );
}

ExecFuncPtr __ns_vector_table[] __VECTOR_TABLE_ATTRIBUTE = {
    (ExecFuncPtr)&__INITIAL_SP,           // initial SP
    (ExecFuncPtr)Reset_Handler,           // initial PC/Reset
    (ExecFuncPtr)NMIException_Handler,
    (ExecFuncPtr)HardFault_Handler,
    (ExecFuncPtr)MemManage_Handler,       // Memory Manage Fault
    (ExecFuncPtr)BusFault_Handler,        // Bus Fault
    (ExecFuncPtr)UsageFault_Handler,      // Usage Fault
    0,                                    // Secire Fault
    0,                                    // RESERVED
    0,                                    // RESERVED
    0,                                    // RESERVED
    (ExecFuncPtr)SVC_Handler,             // SVC
    0,                                    // debug
    0,                                    // RESERVED
    (ExecFuncPtr)PendSV_Handler,          // Pend SV
    (ExecFuncPtr)SysTick_Handler,         // Secure systick
    /* External interrupts */
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler,
    (ExecFuncPtr)InterruptHandler
};

// Reset Handler
__WEAK interrupt_type void Reset_Handler(void)
{

#if 0
    /* invalidate the cache */
    ICACHE0->CMD = ICACHE0->CMD | ICACHE_CMD_INV_Msk;
    /*wait for invalidation complete */
    while(ICACHE0->CMD & ICACHE_CMD_INV_Msk);
#endif

    /* Enable EWIC block */
    uint32_t *ptr = (uint32_t *)CY_SYSINT_EWIC_CTL;
    *ptr |= (uint32_t)CY_SYSINT_EWIC_ENABLE_MSK;

    /* Disable I cache */
    ICACHE0->CTL = ICACHE0->CTL & (~ICACHE_CTL_CA_EN_Msk);

    /* Enable ECC */
    //ICACHE0->CTL = ICACHE0->CTL | ICACHE_CTL_ECC_EN_Msk;

    /* Enable I cache */
    ICACHE0->CTL = ICACHE0->CTL | ICACHE_CTL_CA_EN_Msk;

    __disable_irq();

    for (uint32_t count = 0; count < VECTORTABLE_SIZE; count++)
    {
        __ns_vector_table_rw[count] =__ns_vector_table[count];
    }

    SCB->VTOR = (uint32_t)__ns_vector_table_rw;
    __DMB();

    __set_MSPLIM((uint32_t)(&__STACK_LIMIT));

    SystemInit();

    Cy_RuntimeInit();
}

CY_MISRA_BLOCK_END('MISRA C-2012 Rule 18.3');
CY_MISRA_BLOCK_END('MISRA C-2012 Rule 18.1');
CY_MISRA_BLOCK_END('ARRAY_VS_SINGLETON');
CY_MISRA_BLOCK_END('MISRA C-2012 Rule 8.6');

#endif /* defined (EXPLORER_PSVP) */
