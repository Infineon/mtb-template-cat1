# ModusToolbox CAT1 BSP Template Library Release Notes

### What Changed?
#### v1.11.0
* Added CY_DISABLE_WARM_BOOT option to disable warm-boot related code when this feature is not used for PSOC Control C3 and CYW20829 devices.
* Added SystemCoreClockSetup() function for PSOC Control C3 and CYW20829 devices.
#### v1.10.0
* Production support for TRAVEO&trade; T2G CYT6BJ devices.
#### v1.9.0
* Add support for XMC5000 devices.
#### v1.8.1
* Updated linker script paths for TRAVEO&trade; T2G CYT3DL and TRAVEO&trade; T2G CYT4DN devices.
#### v1.8.0
* Production support for TRAVEO&trade; T2G CYT3DL devices.
* Add support for HAL interface version 3 on TRAVEO&trade; T2G CYT3DL devices.
#### v1.7.6
* Fixed the compatibility issues encountered when building projects in EWARM using the "C++ only" option.
#### v1.7.5
* Fixed an issue related to timeout error during erase flash operation on XMC7x00.
#### v1.7.4
* Fixed CM7 startup code to remove ARM compiler error.
#### v1.7.3
* Fixed CM0+ startup code to remove GCC14 warning.
#### v1.7.2
* Fixed MTB-HAL dependency for TRAVEO&trade; T2G CYT4DN devices.
#### v1.7.1
* Fixed compilation error in cybsp_init when HAL is not present for CAT1A/CAT1C devices.
#### v1.7.0
* Added support for PSOC Control C3 devices.
* Removed incorrect HAL dependency for TRAVEO&trade; T2G CYT4DN devices.
* Increment BSP generation number to 5 for CAT1A and CAT1B to
  reflect that BSPs created using this template version provide
  equivalent functionality to the generation 5 kit BSPs.
#### v1.6.1
* Production support for FX2G3,FX3G2 devices. Bugfix for PSOC6 Frequencies.
#### v1.6.0
* Production support for TRAVEO&trade; T2G CYT4DN devices.
#### v1.5.0
* Production support for CYT2B6 and CYT2B9 devices.
* Fixed missing file issue when using CYT2B7 devices with BSP assistant.
* Removed redundant CM0P component selections for PSoC™ 64 devices.
* Updated linker scripts and startup code to align with PDL 3.10.0 release.
* Apply changes from 1.3.1 on top of 1.4.0
#### v1.4.0
* Production support for Traveo II Body Entry devices.
#### v1.3.1
* Enabled Configuration of FLL,PLL,Systick,Timer clocks by default for CAT1A devices.
#### v1.3.0
* Fixed the issue where an CM0P prebuilt image could enable both CM7 cores on its devices.
* Added options for setting configuration of WL companion radio REG_ON and HOST_WAKE pins.
#### v1.2.2
* Production support for CYW20829 devices.
* Updated linker scripts and startup code to align with mtb-pdl-cat1 v3.6.0.
#### v1.2.1
* Added explicit include for `cycfg_pins.h` in `cybsp_hw_config.h`.
* Updated linker scripts and startup code to align with mtb-pdl-cat1 v3.4.0.
#### v1.2.0
* Added support for BSP Assistant chip flow.
* Updated MTBX dependencies to use MTB 3.0 Query APIs.
#### v1.2.0
* Added support for BSP Assistant chip flow.
* Updated MTBX dependencies to use MTB 3.0 Query APIs.
#### v1.1.0
* Fixed ifdef for CM0P core in cybsp.c.
* Added `cybsp_hw_config.h` to fileset.
* Added pre-production support for DeepSleep-RAM warm-boot on CAT1B devices.
* Updated linker scripts and startup code to align with mtb-pdl-cat1 v3.3.0.
#### v1.0.0
Initial release.

### Supported Software and Tools
This version of mtb-template-cat1 was validated for compatibility with the following Software and Tools:

| Software and Tools                        | Version |
| :---                                      | :----:  |
| ModusToolbox&trade; Software Environment  | 3.7.0   |
| GCC Compiler                              | 14.2.1  |
| IAR Compiler                              | 9.50.2  |
| ARM Compiler                              | 6.22    |

Minimum required ModusToolbox&trade; Software Environment: v3.0.0

### More information
* [Development Board Documentation](https://www.infineon.com/design-resources/finder-selection-tools/evaluation-board)
* [Infineon](http://www.infineon.com)
* [Infineon GitHub](https://github.com/infineon)
* [ModusToolbox&trade;](https://www.infineon.com/cms/en/design-support/tools/sdk/modustoolbox-software/)

---
© Infineon Technologies AG or an affiliate of Infineon Technologies AG, 2022-2026.
