/***************************************************************************//**
* \file system_cm0plus.c
* \version 1.0
*
* The device system-source file.
*
********************************************************************************
* \copyright
* Copyright 2021 Cypress Semiconductor Corporation
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

/*******************************************************************************
* Function Name: SystemInit
****************************************************************************//**
*
* Initializes the system:
* - Unlocks and disables WDT.
* - Calls the Cy_SystemInit() function, if compiled from PSoC Creator.
* - Calls \ref SystemCoreClockUpdate().
*
*******************************************************************************/

#include <stdbool.h>
#include "cy_device.h"
#include "cy_device_headers.h"
#include "cy_syslib.h"
#include "cy_sysclk.h"
#include "cy_wdt.h"
#include "cmsis_compiler.h"
#include "system_cat1d.h"

#if defined(__cplusplus)
extern "C" {
#endif
/*----------------------------------------------------------------------------
  Define clocks
 *----------------------------------------------------------------------------*/
#define  XTAL            (80000000UL)     /* dummy value */

#define  SYSTEM_CLOCK    (XTAL / 2U)      /* dummy value */


/*******************************************************************************
* SystemCoreClockUpdate()
*******************************************************************************/

/** Default HFClk frequency in Hz */
#define CY_CLK_HFCLK0_FREQ_HZ_DEFAULT       (50000000UL)

/** Default PeriClk frequency in Hz */
#define CY_CLK_PERICLK_FREQ_HZ_DEFAULT      (50000000UL)

/** Default system core frequency in Hz */
#define CY_CLK_SYSTEM_FREQ_HZ_DEFAULT       (50000000UL)

uint32_t SystemCoreClock = CY_CLK_SYSTEM_FREQ_HZ_DEFAULT;

/** Holds the HFClk0 clock frequency. Updated by \ref SystemCoreClockUpdate(). */
uint32_t cy_Hfclk0FreqHz  = CY_CLK_HFCLK0_FREQ_HZ_DEFAULT;

/** Holds the PeriClk clock frequency. Updated by \ref SystemCoreClockUpdate(). */
uint32_t cy_PeriClkFreqHz = CY_CLK_PERICLK_FREQ_HZ_DEFAULT;

/** Holds the AHB frequency. Updated by \ref SystemCoreClockUpdate(). */
uint32_t cy_AhbFreqHz = CY_CLK_SYSTEM_FREQ_HZ_DEFAULT;

/* Do not use these definitions directly in your application */
#define CY_DELAY_MS_OVERFLOW_THRESHOLD  (0x8000u)
#define CY_DELAY_1K_THRESHOLD           (1000u)
#define CY_DELAY_1K_MINUS_1_THRESHOLD   (CY_DELAY_1K_THRESHOLD - 1u)
#define CY_DELAY_1M_THRESHOLD           (1000000u)
#define CY_DELAY_1M_MINUS_1_THRESHOLD   (CY_DELAY_1M_THRESHOLD - 1u)

uint32_t cy_delayFreqHz   = CY_CLK_SYSTEM_FREQ_HZ_DEFAULT;

uint32_t cy_delayFreqKhz  = (CY_CLK_SYSTEM_FREQ_HZ_DEFAULT + CY_DELAY_1K_MINUS_1_THRESHOLD) /
                            CY_DELAY_1K_THRESHOLD;

uint32_t cy_delayFreqMhz  = (uint32_t)((CY_CLK_SYSTEM_FREQ_HZ_DEFAULT + CY_DELAY_1M_MINUS_1_THRESHOLD) /
                            CY_DELAY_1M_THRESHOLD);


/*----------------------------------------------------------------------------
  System initialization function
 *----------------------------------------------------------------------------*/
void SystemInit (void)
{
    SystemCoreClockUpdate();
}



/*******************************************************************************
* Function Name: SystemCoreClockUpdate
****************************************************************************//**
*
* Gets core clock frequency and updates \ref SystemCoreClock.
*
* Updates global variables used by the \ref Cy_SysLib_Delay(), \ref
* Cy_SysLib_DelayUs(), and \ref Cy_SysLib_DelayCycles().
*
*******************************************************************************/
void SystemCoreClockUpdate (void)
{
    uint32_t pathFreqHz;
    uint32_t clkHfPath;

    /* Get frequency for the high-frequency clock # 0 */
    clkHfPath = CY_SYSCLK_CLK_CORE_HF_PATH_NUM;

    pathFreqHz = Cy_SysClk_ClkHfGetFrequency(clkHfPath);

    SystemCoreClock = pathFreqHz;

    cy_Hfclk0FreqHz = SystemCoreClock;

    /* Get frequency for the high-frequency clock # 2 , whcih is used for PERI PCLK*/
    clkHfPath = CY_SYSCLK_CLK_PERI_HF_PATH_NUM;

    pathFreqHz = Cy_SysClk_ClkHfGetFrequency(clkHfPath);

    cy_PeriClkFreqHz = pathFreqHz;

    /* Sets clock frequency for Delay API */
    cy_delayFreqHz = SystemCoreClock;
    cy_delayFreqMhz = (uint32_t)((cy_delayFreqHz + CY_DELAY_1M_MINUS_1_THRESHOLD) / CY_DELAY_1M_THRESHOLD);
    cy_delayFreqKhz = (cy_delayFreqHz + CY_DELAY_1K_MINUS_1_THRESHOLD) / CY_DELAY_1K_THRESHOLD;

    /* Get the frequency of AHB source, Slow clk is the source for AHB*/
    cy_AhbFreqHz = Cy_SysClk_ClkHfGetFrequency(0);
}

#if defined(__cplusplus)
}
#endif
