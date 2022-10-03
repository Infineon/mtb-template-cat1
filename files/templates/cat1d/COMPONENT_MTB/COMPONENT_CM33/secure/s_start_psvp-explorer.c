/*
** Copyright (c) 2018 Arm Limited. All rights reserved.
*/
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include "explorer_psvp.h"
#include "cy_syslib.h"

CY_MISRA_FP_BLOCK_START('MISRA C-2012 Rule 8.6', 3, \
'Checked manually. The definition is a part of linker script or application.');
CY_MISRA_DEVIATE_BLOCK_START('ARRAY_VS_SINGLETON', 1, \
'Checked manually. Using pointer as an array will not corrupt or misinterpret adjacent memory locations.');
CY_MISRA_DEVIATE_BLOCK_START('MISRA C-2012 Rule 18.1', 3, \
'Checked manually. Dereferencing a pointer to one beyond the end of an array will not result in undefined behaviour.');
CY_MISRA_DEVIATE_BLOCK_START('MISRA C-2012 Rule 18.3', 1, \
'Checked manually. Attempting to make comparisons between pointers will not result in undefined behaviour.');

// Initial Setup
//----------------------------------------------------------------

extern unsigned int __INITIAL_SP;
extern unsigned int __STACK_LIMIT;

#if defined(__ARMCC_VERSION)
    interrupt_type extern void __main(void);
    typedef void(* ExecFuncPtrRw)(void) interrupt_type;
    ExecFuncPtrRw __s_vector_table_rw[VECTORTABLE_SIZE] __attribute__( ( section(".bss.noinit.RESET_RAM"))) __attribute__((aligned(VECTORTABLE_ALIGN)));
#elif defined (__GNUC__)
    typedef void(* interrupt_type ExecFuncPtrRw)(void);
    ExecFuncPtrRw __s_vector_table_rw[VECTORTABLE_SIZE]   __attribute__( ( section(".ram_vectors"))) __attribute__((aligned(VECTORTABLE_ALIGN)));
#elif defined (__ICCARM__)
    interrupt_type extern void  __cmain();
    ExecFuncPtrRw __s_vector_table_rw[VECTORTABLE_SIZE]   __attribute__( ( section(".intvec_ram"))) __attribute__((aligned(VECTORTABLE_ALIGN)));
#else
    #error "An unsupported toolchain"
#endif  /* (__ARMCC_VERSION) */

#if defined (__ARMCC_VERSION)
void __attribute__((optnone)) Cy_RuntimeInit(void);
#else
void Cy_RuntimeInit(void);
#endif

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
interrupt_type void S_Reset_Handler(void);
interrupt_type void S_NMIException_Handler(void) {while(1);}
interrupt_type void S_HardFault_Handler(void)
{
    __asm (
        "TST LR, #0x40\n"
        "BEQ from_nonsecure\n"
        "from_secure:\n"
        "TST LR, #0x04\n"
        "ITE EQ\n"
        "MRSEQ R0, MSP\n"
        "MRSNE R0, PSP\n"
        "B SysLib_FaultHandler \n"
        "from_nonsecure:\n"
        "MRS R0, CONTROL_NS\n"
        "TST R0, #2\n"
        "ITE EQ\n"
        "MRSEQ R0, MSP_NS\n"
        "MRSNE R0, PSP_NS\n"
        "B SysLib_FaultHandler\n"
    );
}

interrupt_type void S_MemManage_Handler(void)         {while(1);}
interrupt_type void S_BusFault_Handler(void)          {while(1);}
interrupt_type void S_UsageFault_Handler(void)        {while(1);}
interrupt_type void S_SecureFault_Handler(void)       {while(1);}
interrupt_type void S_SVC_Handler(void)               {while(1);}
interrupt_type void S_DebugMon_Handler(void)          {while(1);}
interrupt_type void S_PendSV_Handler(void)            {while(1);}
interrupt_type void S_SysTick_Handler(void)           {while(1);}
interrupt_type void S_InterruptHandler(void)          {while(1);}

ExecFuncPtr __s_vector_table[] __VECTOR_TABLE_ATTRIBUTE = {
    (ExecFuncPtr)&__INITIAL_SP,            // initial SP
    (ExecFuncPtr)S_Reset_Handler,          // initial PC/Reset
    (ExecFuncPtr)S_NMIException_Handler,
    (ExecFuncPtr)S_HardFault_Handler,
    (ExecFuncPtr)S_MemManage_Handler,      // Memory Manage Fault
    (ExecFuncPtr)S_BusFault_Handler,       // Bus Fault
    (ExecFuncPtr)S_UsageFault_Handler,     // Usage Fault
    (ExecFuncPtr)S_SecureFault_Handler,    // RESERVED
    (ExecFuncPtr)0,                        // RESERVED
    (ExecFuncPtr)0,                        // RESERVED
    (ExecFuncPtr)0,                        // RESERVED
    (ExecFuncPtr)S_SVC_Handler,
    (ExecFuncPtr)S_DebugMon_Handler,       // RESERVED for debug
    (ExecFuncPtr)0,                        // RESERVED
    (ExecFuncPtr)S_PendSV_Handler,
    (ExecFuncPtr)S_SysTick_Handler,

    /* External interrupts */
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler,
    (ExecFuncPtr)S_InterruptHandler
};

// Reset Handler
interrupt_type void S_Reset_Handler(void)
{

#if 0
    /* invalidate the cache */
    ICACHE0->CMD = ICACHE0->CMD | ICACHE_CMD_INV_Msk;
    /*wait for invalidation complete */
    while(ICACHE0->CMD & ICACHE_CMD_INV_Msk);
#endif
    /* Disable I cache */
    ICACHE0->CTL = ICACHE0->CTL & (~ICACHE_CTL_CA_EN_Msk);

    /* Enable ECC */
    //ICACHE0->CTL = ICACHE0->CTL | ICACHE_CTL_ECC_EN_Msk;

    /* Enable I cache */
    ICACHE0->CTL = ICACHE0->CTL | ICACHE_CTL_CA_EN_Msk;

    __disable_irq();

    for (uint32_t count = 0; count < VECTORTABLE_SIZE; count++)
    {
        __s_vector_table_rw[count] =__s_vector_table[count];
    }

    SCB->VTOR = (uint32_t)__s_vector_table_rw;
    __DMB();

    __set_MSPLIM((uint32_t)(&__STACK_LIMIT));

    SystemInit();

    Cy_RuntimeInit();
}

CY_MISRA_BLOCK_END('MISRA C-2012 Rule 18.3');
CY_MISRA_BLOCK_END('MISRA C-2012 Rule 18.1');
CY_MISRA_BLOCK_END('ARRAY_VS_SINGLETON');
CY_MISRA_BLOCK_END('MISRA C-2012 Rule 8.6');
