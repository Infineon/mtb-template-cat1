#include <stdio.h>
#include <arm_cmse.h>
#include "cy_pdl.h"
#include "partition_ARMCM33.h"

#include "cy_mpc.h"
#include "cy_ppc.h"

#define SRAM1_SIZE 0x00040000;
#define CM33_NS_IMAGE_LOAD_ADDR 0x0200cc00

cy_stc_mpc_cfg_t ramConfig;
cy_stc_mpc_rot_cfg_t rotRamConfig;

cy_stc_ppc_init_t ppcInit;
cy_stc_ppc_attribute_t ppcAttribute;
cy_stc_ppc_pc_mask_t pcMaskConfig;
uint32_t pcLockMask;

//extern const uint32_t __ns_load_addr__;

/* Entry point for second secure image */
typedef void (*funcptr_void) (void) __attribute__((cmse_nonsecure_call));

void SystemInitStage2(void);
void config_mpc(void);
void config_ppc0(void);
void config_ppc1(void);

__WEAK void SecureWeakFunc(void)
{
     /* Empty weak function.
     */
}

void test_stub_handler (void) {
    static int a;
    a++;
}

int main(void)
{
    funcptr_void NonSecure_ResetHandler;
    uint32_t *ns_start_ptr;
    uint32_t *ns_load_addr;
    uint32_t ns_stack;
//    uint32_t *portptr;

    __enable_irq();

#if 0
    /* Set MX Crypto CC312 for non-secure access */
    MXCC0_MXCRYPTOCELL_1_0_AO->AO_APB_FILTERING = 0x0;

    /* restore the initial configuration */
    portptr = (uint32_t*)HSIOM_SECURE_PRT6;
    *portptr = (uint32_t)0xFF;
    portptr = (uint32_t*)HSIOM_SECURE_PRT7;
    *portptr = (uint32_t)0xFF;

    //printf("Secure2: Stage 2 secure vector table set at 0x%x\n",(uint32_t)SCB->VTOR);
#endif

// Enable below piece of code to test APIs to be called from ROT

   /* configure MPC */
    config_mpc();

    /* Configure SAU */
    TZ_SAU_Setup();
    //printf("Secure2: TZ SAU Setup completed\n");

    /* Configure PPC0 for CM33 access */
    config_ppc0();

    /* Configure PPC1 for CM55 access */
    config_ppc1();

    //printf("Secure: Secure image2 loaded\n");

    SecureWeakFunc();
    /* Get secure2 stack pointer */
    //ns_start_ptr = (uint32_t *) (&__ns_load_addr__);
    ns_start_ptr = (uint32_t *) CM33_NS_IMAGE_LOAD_ADDR;
    ns_load_addr = (uint32_t *)(*ns_start_ptr);
    ns_stack = (uint32_t)*ns_load_addr;
    __TZ_set_MSP_NS(ns_stack);

    ns_load_addr++;

    NonSecure_ResetHandler = (funcptr_void)(*(ns_load_addr));

    //printf("\nSecure: Entering Secure2  from 0x%p\n\n", NonSecure_ResetHandler);

    /* Start Secure2 software application */
    NonSecure_ResetHandler();

    /* Non-secure software does not return, this code is not executed */
    //printf("You are not supposed to enter this !\n");
    while(1);

}

void config_mpc(void)
{
    cy_en_mpc_status_t mpcStatus;

    rotRamConfig.regionSize = CY_MPC_SIZE_4KB;
    rotRamConfig.secure = CY_MPC_NON_SECURE;
    rotRamConfig.access = CY_MPC_ACCESS_RW;
    rotRamConfig.pc = CY_MPC_PC_0;

    // SRAM1 SAHB is configured as non-secure 0x24040000
    rotRamConfig.addrOffset = 0;
    rotRamConfig.size = SRAM1_SIZE;
    mpcStatus = Cy_Mpc_ConfigRotMpcStruct(RAMC1_MPC0, &rotRamConfig);
    if (mpcStatus != CY_MPC_SUCCESS) {
        while(1);
    }
}

void config_ppc0(void)
{
    PPC_Type* ppcPtr;
    cy_en_ppc_status_t ppcStatus;

    /* Configure PPC0 for CM33 access */
    // Converting the PPC pointer to secure offset as it points to non-secure offset in player_psvp.h
    ppcPtr = (PPC_Type*)((uint32_t)(PPC0) + 0x10000000);

    // Initialize PPC
    ppcInit.respConfig = CY_PPC_RZWI;
    ppcStatus = Cy_Ppc_InitPpc(ppcPtr, &ppcInit);
    if (ppcStatus != CY_PPC_SUCCESS) {
        while(1);
    }

    ppcAttribute.secAttribute = CY_PPC_NON_SECURE;
    ppcAttribute.secPrivAttribute = CY_PPC_SEC_PRIV;
    ppcAttribute.nsPrivAttribute = CY_PPC_NON_SEC_PRIV;

    // Need to enable folliowing  regions
    //10,
    //45 -148
    //171 - 271
    //0,7

    ppcAttribute.startRegion = PROT_PERI_PCLK0_MAIN;
    ppcAttribute.endRegion = PROT_PERI_PCLK0_MAIN;
    ppcStatus = Cy_Ppc_ConfigAttrib(ppcPtr, &ppcAttribute);
    if (ppcStatus != CY_PPC_SUCCESS) {
        while(1);
    }

    ppcAttribute.startRegion = PROT_MXCM33_BOOT;
    ppcAttribute.endRegion = PROT_HSIOM_PRT21_PRT;
    ppcStatus = Cy_Ppc_ConfigAttrib(ppcPtr, &ppcAttribute);
    if (ppcStatus != CY_PPC_SUCCESS) {
        while(1);
    }

    ppcAttribute.startRegion = PROT_HSIOM_AMUX;
    ppcAttribute.endRegion = PROT_ETH0;
    ppcStatus = Cy_Ppc_ConfigAttrib(ppcPtr, &ppcAttribute);
    if (ppcStatus != CY_PPC_SUCCESS) {
        while(1);
    }

    ppcAttribute.startRegion = PROT_PERI0_MAIN;
    ppcAttribute.endRegion = PROT_PERI0_MAIN;
    ppcStatus = Cy_Ppc_ConfigAttrib(ppcPtr, &ppcAttribute);
    if (ppcStatus != CY_PPC_SUCCESS) {
        while(1);
    }

    ppcAttribute.startRegion = PROT_PERI0_TR;
    ppcAttribute.endRegion = PROT_PERI0_TR;
    ppcStatus = Cy_Ppc_ConfigAttrib(ppcPtr, &ppcAttribute);
    if (ppcStatus != CY_PPC_SUCCESS) {
        while(1);
    }

    // Set PC Mask
    pcMaskConfig.startRegion = 0;
    pcMaskConfig.endRegion = 1023;
    pcMaskConfig.pcMask = 0xFFFFFFFF;
    ppcStatus = Cy_Ppc_SetPcMask(ppcPtr, &pcMaskConfig);
    if (ppcStatus != CY_PPC_SUCCESS) {
        while(1);
    }

#if 0
    // Set PC Lock mask
    pcLockMask = 0x00000000;
    ppcStatus = Cy_Ppc_SetLockMask(ppcPtr, pcLockMask);
    if (ppcStatus != CY_PPC_SUCCESS) {
        while(1);
    }

    // Read PC Lock mask
    if (pcLockMask != Cy_Ppc_GetLockMask(ppcPtr))
    {
       while(1);
    }
#endif
}

void config_ppc1(void)
{
    PPC_Type* ppcPtr;
    cy_en_ppc_status_t ppcStatus;

    /* Configure PPC1 for CM55 access */
    // Converting the PPC pointer to secure offset as it points to non-secure offset in player_psvp.h
    ppcPtr = (PPC_Type*)((uint32_t)(PPC1) + 0x10000000);

    // Initialize PPC
    ppcInit.respConfig = CY_PPC_RZWI;
    ppcStatus = Cy_Ppc_InitPpc(ppcPtr, &ppcInit);
    if (ppcStatus != CY_PPC_SUCCESS) {
        while(1);
    }

    ppcAttribute.secAttribute = CY_PPC_NON_SECURE;
    ppcAttribute.secPrivAttribute = CY_PPC_SEC_PRIV;
    ppcAttribute.nsPrivAttribute = CY_PPC_NON_SEC_PRIV;

    // Need to enable folliowing  regions
    //10,
    //15 -148
    //193 - 306
    //0,7
    // For all PERI1 region IDs, bit 28 to bit 31 should be set to zero
    ppcAttribute.startRegion = (0x0FFFFFFFu) & PROT_PERI_PCLK1_MAIN;
    ppcAttribute.endRegion = (0x0FFFFFFFu) & PROT_PERI_PCLK1_MAIN;
    ppcStatus = Cy_Ppc_ConfigAttrib(ppcPtr, &ppcAttribute);
    if (ppcStatus != CY_PPC_SUCCESS) {
        while(1);
    }

    ppcAttribute.startRegion = (0x0FFFFFFFu) & PROT_MXCM55_BOOT;
    //ppcAttribute.endRegion = (0x0FFFFFFFu) & PROT_SMIF0_CORE1_HSIOM_PRT21_PRT;
    ppcStatus = Cy_Ppc_ConfigAttrib(ppcPtr, &ppcAttribute);
    if (ppcStatus != CY_PPC_SUCCESS) {
        while(1);
    }

    //ppcAttribute.startRegion = (0x0FFFFFFFu) & PROT_SMIF0_CORE0_HSIOM_AMUX;
    ppcAttribute.endRegion = (0x0FFFFFFFu) & PROT_MXU550_ROT;
    ppcStatus = Cy_Ppc_ConfigAttrib(ppcPtr, &ppcAttribute);
    if (ppcStatus != CY_PPC_SUCCESS) {
        while(1);
    }

    ppcAttribute.startRegion = (0x0FFFFFFFu) & PROT_PERI1_MAIN;
    ppcAttribute.endRegion = (0x0FFFFFFFu) & PROT_PERI1_MAIN;
    ppcStatus = Cy_Ppc_ConfigAttrib(ppcPtr, &ppcAttribute);
    if (ppcStatus != CY_PPC_SUCCESS) {
        while(1);
    }

    //ppcAttribute.startRegion = (0x0FFFFFFFu) & PROT_PERI1_TR;
    //ppcAttribute.endRegion = (0x0FFFFFFFu) & PROT_PERI1_TR;
    ppcStatus = Cy_Ppc_ConfigAttrib(ppcPtr, &ppcAttribute);
    if (ppcStatus != CY_PPC_SUCCESS) {
        while(1);
    }

    // Set PC Mask
    pcMaskConfig.startRegion = 0;
    pcMaskConfig.endRegion = 1023;
    pcMaskConfig.pcMask = 0xFFFFFFFF;
    ppcStatus = Cy_Ppc_SetPcMask(ppcPtr, &pcMaskConfig);
    if (ppcStatus != CY_PPC_SUCCESS) {
        while(1);
    }

#if 0
    // Set PC Lock mask
    pcLockMask = 0x00000000;
    ppcStatus = Cy_Ppc_SetLockMask(ppcPtr, pcLockMask);
    if (ppcStatus != CY_PPC_SUCCESS) {
        while(1);
    }

    // Read PC Lock mask
    if (pcLockMask != Cy_Ppc_GetLockMask(ppcPtr))
    {
       while(1);
    }
#endif
}

