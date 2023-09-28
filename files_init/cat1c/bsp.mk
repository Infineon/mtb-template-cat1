################################################################################
# \file bsp.mk
#
################################################################################
# \copyright
# Copyright 2019-2023 Cypress Semiconductor Corporation (an Infineon company) or
# an affiliate of Cypress Semiconductor Corporation
#
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

ifeq ($(WHICHFILE),true)
$(info Processing $(lastword $(MAKEFILE_LIST)))
endif

ifneq (,$(filter XMC7x_CM0P_SLEEP, $(BSP_COMPONENTS)))
BSP_LINKER_CORE:=linker
else
BSP_LINKER_CORE:=linker_d
endif
ifeq ($(TOOLCHAIN),GCC_ARM)
	BSP_LINKER_SCRIPT_EXT:=ld
else ifeq ($(TOOLCHAIN),ARM)
	BSP_LINKER_SCRIPT_EXT:=sct
else ifeq ($(TOOLCHAIN),IAR)
	BSP_LINKER_SCRIPT_EXT:=icf
endif
MTB_BSP__LINKER_SCRIPT:=$(MTB_TOOLS__TARGET_DIR)/TOOLCHAIN_$(TOOLCHAIN)/$(BSP_LINKER_CORE).$(BSP_LINKER_SCRIPT_EXT)
$(info "Using linker $(MTB_BSP__LINKER_SCRIPT)")
# Any additional components to apply when using this board.
BSP_COMPONENTS:=

# Any additional defines to apply when using this board.
BSP_DEFINES:=

# Path to the flash loaders to patch for this board
CY_QSPI_FLM_DIR=$(MTB_TOOLS__TARGET_DIR)/config/FlashLoaders/

# Path to the patched flash loaders for this board
CY_QSPI_FLM_DIR_OUTPUT=$(MTB_TOOLS__TARGET_DIR)/config/GeneratedSource/

################################################################################
# ALL ITEMS BELOW THIS POINT ARE AUTO GENERATED BY THE BSP ASSISTANT TOOL.
# DO NOT MODIFY DIRECTLY. CHANGES SHOULD BE MADE THROUGH THE BSP ASSISTANT.
################################################################################

# Board device selection. MPN_LIST tracks what was selected in the BSP Assistant
# All other variables are derived by BSP Assistant based on the MPN_LIST.
MPN_LIST:=
