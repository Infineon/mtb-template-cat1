/***************************************************************************//**
* \file system_explorer.h
* \version 1.0
*
* \brief Device system header file.
*
********************************************************************************
* \copyright
* Copyright 2016-2020 Cypress Semiconductor Corporation
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


#ifndef _SYSTEM_EXPLORER_H_
#define _SYSTEM_EXPLORER_H_

/** \cond */

/**
* \addtogroup group_system_config_cm33
* \{
* Provides device startup, system configuration, and linker script files.
* The system startup provides the followings features:
* - \ref group_system_config_device_memory_definition_cm33
* - \ref group_system_config_device_initialization_cm33
* - \ref group_system_config_heap_stack_config_cm33
* - \ref group_system_config_default_handlers_cm33
* - \ref group_system_config_device_vector_table_cm33
*
* \section group_system_config_configuration_cm33 Configuration Considerations
*
* \subsection group_system_config_device_memory_definition_cm33 Device Memory Definition
* Allocation of different types of memory such as the ROM, flash, RRAM, SOCMEM and RAM etc. for the CPU is defined by the linker scripts.
*
* \note The linker files provided with the PDL are generic and handle all common
* use cases. Your project may not use every section defined in the linker files.
* In that case you may see warnings during the build process. To eliminate build
* warnings in your project, you can simply comment out or remove the relevant
* code in the linker file.
*
* <b>For Explorer</b>
*
* <b>ARM GCC</b>\n
* The ROM, flash and RAM sections for the CPU are defined in the linker files:
* 'xx_yy.ld', where 'xx' is the device group, and 'yy' represents whether the
* linker script file is for secure or non-secure image; for example,
* 'explorer_s.ld' is the linker file for Explorer device secure image.
* 'explorer_ns.ld' is the linker file for Explorer device non-secure image.
* For devices without security extension, there will be only one linker file;
* for examample, 'cyw20829_ns.ld' is the linker file for cyw20829 device and it
* is always non-secure.
*
* Memory sections are for the GNU GCC ARM tool set for the secure application are defined in the linker file
* \<device\>_s.ld. Following are the important memory sections for the User/Application image.
*
* \code
* CODE_VMA  0x14000000, Starting address of the Secure Application code
* CODE_SIZE 0x0001FF00, Size of the Secure Application code
* DATA_VMA  0x34020000, Starting address of the Secure Application data
* DATA_SIZE 0x0001FFE0, Size of the Secure Application data
* \endcode
*
* Memory sections are for the GNU GCC ARM tool set for the non-secure application are defined in the linker file
* \<device\>_ns.ld. Following are the important memory sections for the User/Application image.
*
* \code
* CODE_VMA  0x04040000, Starting address of the Non-secure Application code
* CODE_SIZE 0x00020000, Size of the Non-secure Application code
* DATA_VMA  0x24060000, Starting address of the Non-secure Application data
* DATA_SIZE 0x00020000, Size of the Non-secure Application data
* \endcode
*
*
* <b>For CYW20829</b>
*
* <b>ARM GCC</b>\n
* The ROM, flash and RAM sections for the CPU are defined in the linker files:
* 'xx_yy.ld', where 'xx' is the device group, and 'yy' represents whether the
* linker script file is for secure or non-secure image; for example,
* 'explorer_s.ld' is the linker file for Explorer device secure image.
* 'explorer_ns.ld' is the linker file for Explorer device non-secure image.
* For devices without security extension, there will be only one linker file;
* for examample, 'cyw20829_ns.ld' is the linker file for cyw20829 device and it
* is always non-secure.
*
* Memory sections are for the GNU GCC ARM tool set is defined in the linker file
* \<device\>_ns.ld. Following are the important memory sections for the User/Application image.
*
* Memory sections are for the GNU GCC ARM tool set is defined in the linker file
* \<device\>_ns.sct. Following are the important memory sections for the User/Application image.
* \code
* code        (rx)  : ORIGIN = CODE_VMA,      LENGTH = CODE_BS_SIZE  Starting address and the size of Non-secure bootstrap code
* bsData      (rwx) : ORIGIN = DATA_BS_VMA,   LENGTH = DATA_BS_SIZE  Starting address and the size of Non-secure bootstrap data
* appCodeRam  (rx)  : ORIGIN = DATA_CBUS_VMA, LENGTH = DATA_SIZE     Starting address and the size of Non-secure application ram funcions
* data        (rwx) : ORIGIN = DATA_VMA,      LENGTH = DATA_SIZE     Starting address and the size of Non-secure applicaiton data
* xip         (rx)  : ORIGIN = XIP_VMA,       LENGTH = XIP_SIZE      Starting address and the size of Non-secure applicaiton code
* \endcode
*
* <b>ARM Compiler</b>\n
* The ROM, flash and RAM sections for the CPU are defined in the linker files:
* 'xx_yy.sct', where 'xx' is the device group, and 'yy' represents whether the
* linker script file is for secure or non-secure image; for example,
* 'explorer_s.sct' is the linker file for Explorer device secure image.
* 'explorer_ns.ld' is the linker file for Explorer device non-secure image.
* For devices without security extension, there will be only one linker file;
* for examample, 'cyw20829_ns.sct' is the linker file for cyw20829 device and it
* is always non-secure.
*
* Memory sections are for the ARM tool set is defined in the linker file
* \<device\>_ns.sct. Following are the important memory sections for the User/Application image.
* \code
* bootstrapText_vma   Starting address of bootstrap code
* bootstrapText_size  Size of memory reserved for Bootstrap code
* bootstrapData_vma   Starting address of Bootstrap data
* appText_vma         Stating address of application code
* appData_vma         Stating address of application data
* \endcode
*
* <b>IAR</b>\n
* The ROM, flash and RAM sections for the CPU are defined in the linker files:
* 'xx_yy.icf', where 'xx' is the device group, and 'yy' represents whether the
* linker script file is for secure or non-secure image; for example,
* 'explorer_s.icf' is the linker file for Explorer device secure image.
* 'explorer_ns.icf' is the linker file for Explorer device non-secure image.
* For devices without security extension, there will be only one linker file;
* for examample, 'cyw20829_ns.icf' is the linker file for cyw20829 device and it
* is always non-secure.
*
* Memory sections are for the IAR tool set is defined in the linker file
* \<device\>_ns.icf. Following are the important memory sections for the User/Application image.
* \code
* define region CODE_region     = mem:[from CODE_VMA size CODE_BS_SIZE];      Bootstrap code region and size
* define region DATA_BS_region  = mem:[from DATA_BS_VMA size DATA_BS_SIZE];   Bootstrap data region and size
* define region DATA_region     = mem:[from DATA_VMA size DATA_SIZE];         Applicatoin data region and size
* define region XIP_region      = mem:[from XIP_VMA size XIP_SIZE];           Application code (xip) region and size
* \endcode
*
* \subsection group_system_config_device_initialization_cm33 Device Initialization
* <b>CM33 With ARM TrustZone Support:</b><br/>
* As soon as the power on reset is released, the execution starts from the secure Enclave. The secure enclave
* contains two secure CM0 cores. The CM0 ROM code initializes the basic clock needed to access and configure
* MMIO registers and then sets up debug port so that the debugger can be attached to CM33 core.
* The secure application is loaded to SRAM at address pointed by CODE_LMA specific to the secure image build.
* Then the non-secure application image is also loaded to SRAM at address pointed by CODE_LMA specific to the
* non--secure image build. This memory address is set at the memory location pointed by NS_LOAD_ADDR in the SRAM.
* Once loading of both secure and non-secure images is done, the address of the secure vector table is set to S_VTOR
* registers of CM33 and the CM33_CTL.CPUWAIT is set to LOW to allow execution of reset handler of secure image.
* Secure application reads the non secure application image vector table address from the secure SRAM memory at
* address pointed by NS_LOAD_ADDR. From this address it reads the non-secure image stack pointer value and
* initializes the MSP_NS register, reads the next entry in the vector table to find the address of the non-secure
* image reset handler. The secure application switches execution form secure to non-secure mode by calling the
* non-secure application's reset handler.
* In the reset handler of the application's bootstrap code, the vector table is copied to RAM area and then the
* address of the vector table is set to VTOR registers. This calls SystemInit function which internally calls
* Cy_PDL_Init, Cy_SystemInit and SystemCoreClockUpdate functions. Then it calls C runtime initialization function
* which calls main function of the secure CM33 application code. In the main function, all security attributes and
* access rights for the system are configured. It is done by programming the SAU, MPC, PPC and MSC using corresponding
* PDL drivers. NVIC is also programmed to route non-secure interrupts to non-secure world. After secure code finishes
* all the necessary initialization, it then reads the address where the non-secure application image is loaded from
* the location pointed by NS_LOAD_ADDR.
* Before switching execution to the non-secure code, the CM33 secure code needs to initialize the stack pointer
* MSP_NS for the non-secure code. This value is picked form the first entry in the non-secure vector table __ns_vector_table.
* Once the non-secure stack is initialized, the CM33 secure code will call the non-secure code entry point which is
* nothing but the Reset_Handler. Address of this function is picked form the second entry in the non-secure vector
* table __ns_vector_table and type casting it to function pointer with the attribute __attribute__((cmse_nonsecure_call))
* supported by ARM CMSE to indicate this as a call to non-secure function from secure world.
*
* In the non-secure Reset_Handler, the vector table is copied to RAM area and then the address of the vector table is set
* to VTOR register. This calls SystemInit function which internally calls Cy_PDL_Init, Cy_SystemInit and SystemCoreClockUpdate
* functions. Then it calls C runtime initialization function which calls main function of the application code.
*
* Incase of Expedition series of devices, CM33 secure startup code could optionally start CM55 core using the CM55 Driver
*
* Below sequence diagram captures the initialization process in the startup code.
* ![](explorer_s_startup.png)
*
* <b>CM33 Without ARM TrustZone Support:</b><br/>
* Below MSC describes the simplified startup sequence starting from reset release of the core. As soon as the reset
* is released, the execution starts form the ROM interrupt vector table reset vector. The ROM code initializes the basic
* clock needed to access and configure MMIO registers and then sets up debug port so that the debugger can be attached.
* After it finishes all the necessary initialization, it reads the bootstrap (part of non secure application image)
* location, size from TOC2 header and loads the bootstrap code into SRAM.
*
* Before switching execution to the non-secure application code, the ROM code needs to initialize the stack pointer
* MSP_NS for the non-secure code. This value is picked form the first entry in the non-secure bootstrap's vector
* table __ns_vector_table. Once the non-secure stack is initialized, the ROM code will call the non-secure code entry
* point which is nothing but the Reset_Handler. Address of this function is picked form the second entry in the non-secure
* vector table __ns_vector_table and type casting it to function pointer.
*
* In the non-secure Reset_Handler, the vector table is copied to RAM area and then the address of the vector table is set
* to VTOR register. This calls SystemInit function which internally calls Cy_PDL_Init, Cy_SystemInit and SystemCoreClockUpdate
* functions. Then it calls C runtime initialization function which calls main function of the application code.
*
* Incase of Expedition series of devices, CM33 non-secure startup code could optionally start CM55 core using the CM55 Driver.
*
* Below sequence diagram captures the initialization process in the startup code.
* ![](explorer_ns_startup.png)
*
* \subsection group_system_config_heap_stack_config_cm33 Heap and Stack Configuration
* By default, the stack size is set to 0x00000800 and the entire remaining ram is used for the heap
*
* \subsubsection group_system_config_heap_stack_config_gcc_cm33 ARM GCC
* - <b>Editing source code files for secure image</b>\n
* The stack and heap sizes are defined in the linker script files: 'xx_yy.ld',
* where 'xx' is the device family, and 'yy' indicates secure or non-secure; for example,
* explorer_s.ld is for Explorer device secure image and explorer_ns.ld is for non-secure image.
* Change the stack size by modifying the following line:\n
* \code MSP_STACK_SIZE = 0x0000800; \endcode
* - <b>Editing source code files for non-secure image</b>\n
* The heap size is defined in the linker file.
* Change the stack size by modifying the following line:\n
* \code NS_MSP_STACK_SIZE = 0x0000800; \endcode
*
* \subsubsection group_system_config_heap_stack_config_arm_cm33 ARM Compiler
* - <b>Editing source code files for secure image</b>\n
* The stack sizes is defined in the linker script files: 'xx_yy.sct', where 'xx' is the device family, and 'yy'
* indicates secure or non-secure; for example,
* explorer_s.sct is for Explorer device secure image and explorer_ns.sct is for non-secure image.
* Change the stack by modifying the following lines:\n
* \code #define MSP_STACK_SIZE  0x0000800 \endcode
* - <b>Editing source code files for non-secure image</b>\n
* The stack size is defined in the linker script files: 'xx_yy.sct', where 'xx' is the device family, and 'yy'
* indicates secure or non-secure; for example,
* explorer_s.sct is for Explorer device secure image and explorer_ns.sct is for non-secure image.
* Change the stack sizes by modifying the following line:\n
* \code #define NS_MSP_STACK_SIZE  0x0000800 \endcode
*
* \subsubsection group_system_config_heap_stack_config_iar_cm33 IAR
* - <b>Editing source code files for secure image</b>\n
* The stack size is defined in the linker script files: 'xx_yy.icf',
* where 'xx' is the device family, and 'yy' indicates secure or non-secure; for example,
* explorer_s.icf is for Explorer device secure image and explorer_ns.icf is for non-secure image.
* Change the stack size by modifying the following line:\n
* \code define symbol NS_MSP_STACK_SIZE = 0x0000800; \endcode
* - <b>Editing source code files for non-secure image</b>\n
* The heap size is defined in the linker file.
* Change the stack size by modifying the following line:\n
* \code define symbol MSP_STACK_SIZE = 0x0000800; \endcode
*
* \subsection group_system_config_default_handlers_cm33 Default Interrupt Handlers Definition
* The default interrupt handler functions are dummy handler in the startup file.\n
* Below is the default hanlder for the secure interrupts:\n
* \code interrupt_type void S_InterruptHandler(void) {
*    while(1);
* } \endcode
* Below is the default hanlder for the non-secure interrupts:\n
* \code interrupt_type void InterruptHandler(void) {
*    while(1);
* } \endcode
*
* \subsection group_system_config_device_vector_table_cm33 Vectors Table Copy from ROM/Flash to RAM
* This process uses memory sections defined in the linker script. The startup code copies the
* default vector table contents to the non-secure SRAM region specified by the linker script.
* APIs are provided in the sysint driver to hook user implemented handler replacing the default
* handler for the corresponding interrupt.
*
* Following tables provide the address of the default and non-secure SRAM interrupt vector
* table for different supported compilers.
* \subsubsection group_system_config_device_vector_table_gcc_cm33 ARM GCC
* The linker script file is 'xx_yy.ld', where 'xx' is the device family, and 'yy' indicates
* secure or non-secure; for example, explorer_s.ld is for Explorer device secure image and
* explorer_ns.ld is for non-secure image.
* For secure world, it uses the following variables.\n
*       Copy interrupt vectors from ROM/flash to RAM: \n
*       From: \code __s_vector_table \endcode
*       To:   \code __s_vector_table_rw \endcode
* For non-secure world, it uses the following variable.\n
*       Copy interrupt vectors from ROM/flash to RAM: \n
*       From: \code __ns_vector_table \endcode
*       To:   \code __ns_vector_table_rw \endcode
* The vector table address (and the vector table itself) are defined in the
* s_start_<device>.c and ns_start_<device>.c startup files corresponding to secure and non-secure
* worlds respectively.
* The code in these files copies the vector table from ROM/Flash to RAM.
* \subsubsection group_system_config_device_vector_table_mdk_cm33 ARM Compiler
* The linker script file is 'xx_yy.sct', where 'xx' is the device family, and 'yy' indicates
* secure or non-secure; for example, explorer_s.sct is for Explorer device secure image and
* explorer_ns.sct is for non-secure image.
* For secure world, it uses the following variables.\n
*       Copy interrupt vectors from ROM/flash to RAM: \n
*       From: \code __s_vector_table \endcode
*       To:   \code __s_vector_table_rw \endcode
* For non-secure world, it uses the following variable.\n
*       Copy interrupt vectors from ROM/flash to RAM: \n
*       From: \code __ns_vector_table \endcode
*       To:   \code __ns_vector_table_rw \endcode
* The vector table address (and the vector table itself) are defined in the
* s_start_<device>.c and ns_start_<device>.c startup files corresponding to secure and non-secure
* worlds respectively.
* The code in these files copies the vector table from ROM/Flash to RAM.
*
* \subsubsection group_system_config_device_vector_table_iar_cm33 IAR
* The linker script file is 'xx_yy.icf', where 'xx' is the device family, and 'yy' indicates
* secure or non-secure; for example, explorer_s.icf is for Explorer device secure image and
* explorer_ns.icf is for non-secure image.
* For secure world, it uses the following variables.\n
*       Copy interrupt vectors from ROM/flash to RAM: \n
*       From: \code __s_vector_table \endcode
*       To:   \code __s_vector_table_rw \endcode
* For non-secure world, it uses the following variable.\n
*       Copy interrupt vectors from ROM/flash to RAM: \n
*       From: \code __ns_vector_table \endcode
*       To:   \code __ns_vector_table_rw \endcode
* The vector table address (and the vector table itself) are defined in the
* s_start_<device>.c and ns_start_<device>.c startup files corresponding to secure and non-secure
* worlds respectively.
* The code in these files copies the vector table from ROM/Flash to RAM.
*
* \section group_system_config_MISRA_cm33 MISRA Compliance
*
* <table class="doxtable">
*   <tr>
*     <th>MISRA Rule</th>
*     <th>Rule Class (Required/Advisory)</th>
*     <th>Rule Description</th>
*     <th>Description of Deviation(s)</th>
*   </tr>
*   <tr>
*     <td> TBD </td>
*     <td> TBD </td>
*     <td> TBD </td>
*     <td> TBD </td>
*   </tr>
* </table>
*
* \section group_system_config_changelog_cm33 Changelog
*   <table class="doxtable">
*   <tr>
*       <th>Version</th>
*       <th>Changes</th>
*       <th>Reason for Change</th>
*   </tr>
*   <tr>
*       <td>1.0</td>
*       <td>Initial version</td>
*       <td></td>
*   </tr>
* </table>
*
*
* \defgroup group_system_config_macro_cm33 Macro
* \{
*   \defgroup group_system_config_system_macro_cm33            System
* \}
* \defgroup group_system_config_functions_cm33 Functions
* \{
*   \defgroup group_system_config_system_functions_cm33_explorer        System
* \}
* \defgroup group_system_config_globals_cm33 Global Variables
*
* \}
*/

/**
* \addtogroup group_system_config_system_functions_cm33_explorer
* \{
*   \details
*   The following system functions implement CMSIS Core functions.
*   Refer to the [CMSIS documentation]
*   (http://www.keil.com/pack/doc/CMSIS/Core/html/group__system__init__gr.html "System and Clock Configuration")
*   for more details.
* \}
*/

/**
* \addtogroup group_system_config_cm55
* \{
* Provides device startup, system configuration, and linker script files.
* The system startup provides the followings features:
* - \ref group_system_config_device_memory_definition_cm55
* - \ref group_system_config_device_initialization_cm55
* - \ref group_system_config_heap_stack_config_cm55
* - \ref group_system_config_default_handlers_cm55
* - \ref group_system_config_device_vector_table_cm55
*
* \section group_system_config_configuration_cm55 Configuration Considerations
*
* \subsection group_system_config_device_memory_definition_cm55 Device Memory Definition
* Allocation of different memory types such as ROM, flash, RRAM, SOCMEM,TCM and RAM for
* the CPU is defined by the linker scripts.
*
* \note The linker files provided with the PDL are generic and handle all common
* use cases. Your project may not use every section defined in the linker files.
* In that case you may see warnings during the build process. To eliminate build
* warnings in your project, you can simply comment out or remove the relevant
* code in the linker file.
*
* <b>ARM GCC</b>\n
* Different memory sections for the CPU are defined in the linker files:
* 'xx_yy.ld', where 'xx' is the device group, and 'yy' represents whether the 
* linker script file is for secure or non-secure image.
* For CPUs without security extension, there will be only one linker file;
* for examample, 'explorer_ns.ld' is the linker file for the CM55 CPU in Explorer device which
* is always non-secure.
*
* Memory sections are for the GNU GCC ARM tool set is defined in the linker file
* \<device\>_ns.ld. Following are the important memory sections for the User/Application image.
* \code
* code       (rx)     : ORIGIN = ITCM_NS_START,    LENGTH = CODE_SIZE
* data       (rwx)    : ORIGIN = DATA_VMA,         LENGTH = DATA_SIZE
* sram       (rwx)    : ORIGIN = SRAM_VMA,         LENGTH = SRAM_SIZE
* \endcode
*
* <b>ARM Compiler</b>\n
* Not supported.
*
* <b>IAR</b>\n
* Not supported.
*
* \subsection group_system_config_device_initialization_cm55 Device Initialization
* <b>CM55 With ARM TrustZone Support:</b><br/>
* CM55 on Explorer does not support ARM TrustZone.
*
* <b>CM55 Without ARM TrustZone Support:</b><br/>
* To boot CM55, it requires appropriate CM55 image to be loaded to the memory that supports CM55 boot.
* Once the right image is loaded, the VTOR registers of CM55 is set with the vector table address
* and set the execution to begin.
* In the current design, CM55 image is built to run from the ITCM. So, it requires the image to be loaded
* to the ITCM of CM55. Once the image is loaded to ITCM the VTOR registers is set with the vector
* table address present in the ITCM and the CPUWAIT is set to LOW. Setting CPUWAIT to LOW releases
* the CM55 core out of reset and starts to execute the reset handler picked from the vector table
* pointed by the VTOR register. The reset handler starts to  execute the startup code.
* The load address in ITCM is pointed by CODE_VMA.
*
* Below sequence diagram captures the initialization process in the startup code.
* ![](Explorer_Boot_CM55.png)
*
* \subsection group_system_config_heap_stack_config_cm55 Heap and Stack Configuration
* By default, the stack size is set to 0x00000800 and the entire remaining ram is used for the heap
*
* \subsubsection group_system_config_heap_stack_config_gcc_cm55 ARM GCC
* - <b>Editing source code files for secure image</b>\n
* Not supported.
* - <b>Editing source code files for non-secure image</b>\n
* The stack size is defined in the linker file.
* Change the stack size by modifying the following line:\n
* \code MSP_NS_STACK_SIZE = 0x0000800; \endcode
*
* \subsubsection group_system_config_heap_stack_config_arm_cm55 ARM Compiler
* Not supported.
*
* \subsubsection group_system_config_heap_stack_config_iar_cm55 IAR
* Not supported.
*
* \subsection group_system_config_default_handlers_cm55 Default Interrupt Handlers Definition
* The default interrupt handler functions are dummy handler in the startup file.\n
* Below is the default hanlder for the non-secure interrupts:\n
* \code interrupt_type void InterruptHandler(void) {
*    while(1);
* } \endcode
*
* \subsection group_system_config_device_vector_table_cm55 Vectors Table Copy from ROM/Flash to RAM
* This process uses memory sections defined in the linker script. The startup code copies the
* default vector table contents to the non-secure SRAM region specified by the linker script.
* APIs are provided in the sysint driver to hook user implemented handler replacing the default
* handler for the corresponding interrupt.
*
* Following tables provide the address of the default and non-secure SRAM interrupt vector
* table for different supported compilers.
* \subsubsection group_system_config_device_vector_table_gcc_cm55 ARM GCC
* The linker script file is 'xx_yy.ld', where 'xx' is the device family, and 'yy' indicates
* secure or non-secure; for example, explorer_ns.ld is for non-secure image.
* For non-secure world, it uses the following variable.\n
*       Copy interrupt vectors from ROM/flash to RAM: \n
*       From: \code __ns_vector_table \endcode
*       To:   \code __ns_vector_table_rw \endcode
* The vector table address (and the vector table itself) are defined in the
* ns_start_<device>.c startup files corresponding to non-secure world.
* The startup code copies the vector table from ROM/Flash to RAM.
*
* \subsubsection group_system_config_device_vector_table_mdk_cm55 ARM Compiler
* Not supported.
*
* \subsubsection group_system_config_device_vector_table_iar_cm55 IAR
* Not supported.
*
* \section group_system_config_MISRA_cm55 MISRA Compliance
*
* <table class="doxtable">
*   <tr>
*     <th>MISRA Rule</th>
*     <th>Rule Class (Required/Advisory)</th>
*     <th>Rule Description</th>
*     <th>Description of Deviation(s)</th>
*   </tr>
*   <tr>
*     <td> TBD </td>
*     <td> TBD </td>
*     <td> TBD </td>
*     <td> TBD </td>
*   </tr>
* </table>
*
* \section group_system_config_changelog_cm55 Changelog
*   <table class="doxtable">
*   <tr>
*       <th>Version</th>
*       <th>Changes</th>
*       <th>Reason for Change</th>
*   </tr>
*   <tr>
*       <td>1.0</td>
*       <td>Initial version</td>
*       <td></td>
*   </tr>
* </table>
*
*
* \defgroup group_system_config_macro_cm55 Macro
* \{
*   \defgroup group_system_config_system_macro_cm55            System
* \}
* \defgroup group_system_config_functions_cm55 Functions
* \{
*   \defgroup group_system_config_system_functions_cm55        System
* \}
* \defgroup group_system_config_globals_cm55 Global Variables
*
* \}
*/

/**
* \addtogroup group_system_config_system_functions_cm55
* \{
*   \details
*   The following system functions implement CMSIS Core functions.
*   Refer to the [CMSIS documentation]
*   (http://www.keil.com/pack/doc/CMSIS/Core/html/group__system__init__gr.html "System and Clock Configuration")
*   for more details.
* \}
*/

#ifdef __cplusplus
extern "C" {
#endif


/*******************************************************************************
* Include files
*******************************************************************************/
#include <stdint.h>


/*******************************************************************************
* Global preprocessor symbols/macros ('define')
*******************************************************************************/

/*******************************************************************************
*
*                      START OF USER SETTINGS HERE
*                      ===========================
*
*                 All lines with '<<<' can be set by user.
*
*******************************************************************************/

/**
* \addtogroup group_system_config_system_macro_cm33
* \{
*/
#if (CY_SYSTEM_CPU_CM33 == 1UL) || defined(CY_DOXYGEN)
    /** The Cortex-M33 startup driver identifier */
    #define CY_STARTUP_M33_ID               ((uint32_t)((uint32_t)((0x10U) & 0x3FFFU) << 18U))
#endif /* (CY_SYSTEM_CPU_CM33 == 1UL) */
/** \} group_system_config_system_macro_cm33 */

/**
* \addtogroup group_system_config_system_macro_cm55
* \{
*/
#if (CY_SYSTEM_CPU_CM55 == 1UL) || defined(CY_DOXYGEN)
    /** The Cortex-M55 startup driver identifier */
    #define CY_STARTUP_M55_ID               ((uint32_t)((uint32_t)((0x11U) & 0x3FFFU) << 18U))
#endif /* (CY_SYSTEM_CPU_CM55 == 1UL) */
/** \} group_system_config_system_macro_cm55 */


#if (CY_SYSTEM_CPU_CM33 == 1UL) ||  defined(CY_DOXYGEN)
/**
* \addtogroup group_system_config_system_functions_cm33_explorer
* \{
*/
#elif (CY_SYSTEM_CPU_CM55 == 1UL) || defined(CY_DOXYGEN)
/**
* \addtogroup group_system_config_system_functions_cm55
* \{
*/
#endif
extern void SystemInit(void);

extern void SystemCoreClockUpdate(void);
/** \} group_system_config_system_functions */

extern void     Cy_SystemInit(void);

extern uint32_t cy_delayFreqHz;
extern uint32_t cy_delayFreqKhz;
extern uint32_t  cy_delayFreqMhz;


#if (CY_SYSTEM_CPU_CM33 == 1UL) || defined(CY_DOXYGEN)
/** \addtogroup group_system_config_globals_cm33
* \{
*/
#elif (CY_SYSTEM_CPU_CM55 == 1UL) || defined(CY_DOXYGEN)
/** \addtogroup group_system_config_globals_cm55
* \{
*/
#endif
extern uint32_t SystemCoreClock;
extern uint32_t cy_Hfclk0FreqHz;
extern uint32_t cy_PeriClkFreqHz;
extern uint32_t cy_AhbFreqHz;

#if (CY_SYSTEM_CPU_CM33 == 1UL) || defined(CY_DOXYGEN)
/** \} group_system_config_globals_cm33 */
#elif (CY_SYSTEM_CPU_CM55 == 1UL) || defined(CY_DOXYGEN)
/** \} group_system_config_globals_cm55 */
#endif

#ifdef __cplusplus
}
#endif

/** \endcond */

#endif /* _SYSTEM_EXPLORER_H_ */


/* [] END OF FILE */



