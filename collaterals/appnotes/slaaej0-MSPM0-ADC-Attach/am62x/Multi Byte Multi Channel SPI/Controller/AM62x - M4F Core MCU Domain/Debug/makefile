################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := C:/ti/ti-cgt-armllvm_2.1.2.LTS

GEN_OPTS__FLAG := 
GEN_CMDS__FLAG := 

ORDERED_OBJS += \
"./syscfg/ti_dpl_config.o" \
"./syscfg/ti_drivers_config.o" \
"./syscfg/ti_drivers_open_close.o" \
"./syscfg/ti_pinmux_config.o" \
"./syscfg/ti_power_clock_config.o" \
"./syscfg/ti_board_config.o" \
"./syscfg/ti_board_open_close.o" \
"./main.o" \
"./mcspi_loopback.o" \
"../linker.cmd" \
$(GEN_CMDS__FLAG) \
-Wl,-lfreertos.am62x.m4f.ti-arm-clang.debug.lib \
-Wl,-ldrivers.am62x.m4f.ti-arm-clang.debug.lib \
-Wl,-lboard.am62x.m4f.ti-arm-clang.debug.lib \
-Wl,-llibc.a \
-Wl,-llibsysbm.a \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(SV7A_DEPS)),)
-include $(SV7A_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
MCSPI_AM62x_multibyte_multichannel.out \

EXE_OUTPUTS__QUOTED += \
"MCSPI_AM62x_multibyte_multichannel.out" \


# All Target
all: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@$(MAKE) --no-print-directory -Onone "MCSPI_AM62x_multibyte_multichannel.out"

# Tool invocations
MCSPI_AM62x_multibyte_multichannel.out: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@echo 'Building target: "$@"'
	@echo 'Invoking: Arm Linker'
	"C:/ti/ti-cgt-armllvm_2.1.2.LTS/bin/tiarmclang.exe" -mcpu=cortex-m4 -mfloat-abi=hard -mlittle-endian -mthumb -DSOC_AM62X -D_DEBUG_=1 -g -Wall -Wno-gnu-variable-sized-type-not-at-end -Wno-unused-function -Wl,-m"mcspi_loopback.Debug.map" -Wl,-i"C:/ti/mcu_plus_sdk_am62x_08_06_00_18/source/kernel/freertos/lib" -Wl,-i"C:/ti/mcu_plus_sdk_am62x_08_06_00_18/source/drivers/lib" -Wl,-i"C:/ti/mcu_plus_sdk_am62x_08_06_00_18/source/board/lib" -Wl,-i"C:/ti/ti-cgt-armllvm_2.1.2.LTS/lib" -Wl,--reread_libs -Wl,--diag_wrap=off -Wl,--display_error_number -Wl,--warn_sections -Wl,--xml_link_info="MCSPI_AM62x_multibyte_multichannel_linkInfo.xml" -Wl,--ram_model -o "MCSPI_AM62x_multibyte_multichannel.out" $(ORDERED_OBJS)
	@echo 'Finished building target: "$@"'
	@echo ' '
	@$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(GEN_MISC_FILES__QUOTED)$(GEN_FILES__QUOTED)$(EXE_OUTPUTS__QUOTED)
	-$(RM) "syscfg\ti_dpl_config.o" "syscfg\ti_drivers_config.o" "syscfg\ti_drivers_open_close.o" "syscfg\ti_pinmux_config.o" "syscfg\ti_power_clock_config.o" "syscfg\ti_board_config.o" "syscfg\ti_board_open_close.o" "main.o" "mcspi_loopback.o" 
	-$(RM) "syscfg\ti_dpl_config.d" "syscfg\ti_drivers_config.d" "syscfg\ti_drivers_open_close.d" "syscfg\ti_pinmux_config.d" "syscfg\ti_power_clock_config.d" "syscfg\ti_board_config.d" "syscfg\ti_board_open_close.d" "main.d" "mcspi_loopback.d" 
	-$(RMDIR) $(GEN_MISC_DIRS__QUOTED)
	-@echo 'Finished clean'
	-@echo ' '

post-build:
	-$(MAKE) -C C:\Users\a0507040\Desktop\SPI_interface\Multi_Byte_Multi_Channel\Multi_Byte_Multi_Channel_MCSPI_AM62x -f makefile_ccs_bootimage_gen OUTNAME=MCSPI_AM62x_multibyte_multichannel PROFILE=Debug MCU_PLUS_SDK_PATH=C:/ti/mcu_plus_sdk_am62x_08_06_00_18 CG_TOOL_ROOT=C:/ti/ti-cgt-armllvm_2.1.2.LTS CCS_INSTALL_DIR=C:\ti\ccs1230\ccs CCS_IDE_MODE=desktop
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

