################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
build-1420901521: ../example.syscfg
	@echo 'Building file: "$<"'
	@echo 'Invoking: SysConfig'
	"C:/ti/sysconfig_1_13_0/sysconfig_cli.bat" -s "C:/ti/mcu_plus_sdk_am62x_08_06_00_18/.metadata/product.json" --script "C:/Users/a0507040/Desktop/SPI_interface/Single_Byte_Multi_Channel/Single_Byte_Multi_Channel_MCSPI_AM62x/example.syscfg" --context "m4fss0-0" -o "syscfg" --part Default --package ALW --compiler ticlang
	@echo 'Finished building: "$<"'
	@echo ' '

syscfg/ti_dpl_config.c: build-1420901521 ../example.syscfg
syscfg/ti_dpl_config.h: build-1420901521
syscfg/ti_drivers_config.c: build-1420901521
syscfg/ti_drivers_config.h: build-1420901521
syscfg/ti_drivers_open_close.c: build-1420901521
syscfg/ti_drivers_open_close.h: build-1420901521
syscfg/ti_pinmux_config.c: build-1420901521
syscfg/ti_power_clock_config.c: build-1420901521
syscfg/ti_board_config.c: build-1420901521
syscfg/ti_board_config.h: build-1420901521
syscfg/ti_board_open_close.c: build-1420901521
syscfg/ti_board_open_close.h: build-1420901521
syscfg/: build-1420901521

syscfg/%.o: ./syscfg/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"C:/ti/ti-cgt-armllvm_2.1.2.LTS/bin/tiarmclang.exe" -c -mcpu=cortex-m4 -mfloat-abi=hard -mlittle-endian -mthumb -I"C:/ti/ti-cgt-armllvm_2.1.2.LTS/include/c" -I"C:/ti/mcu_plus_sdk_am62x_08_06_00_18/source" -I"C:/ti/mcu_plus_sdk_am62x_08_06_00_18/source/kernel/freertos/FreeRTOS-Kernel/include" -I"C:/ti/mcu_plus_sdk_am62x_08_06_00_18/source/kernel/freertos/portable/TI_ARM_CLANG/ARM_CM4F" -I"C:/ti/mcu_plus_sdk_am62x_08_06_00_18/source/kernel/freertos/config/am62x/m4f" -DSOC_AM62X -D_DEBUG_=1 -g -Wall -Wno-gnu-variable-sized-type-not-at-end -Wno-unused-function -MMD -MP -MF"syscfg/$(basename $(<F)).d_raw" -MT"$(@)" -I"C:/Users/a0507040/Desktop/SPI_interface/Single_Byte_Multi_Channel/Single_Byte_Multi_Channel_MCSPI_AM62x/Debug/syscfg"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

%.o: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"C:/ti/ti-cgt-armllvm_2.1.2.LTS/bin/tiarmclang.exe" -c -mcpu=cortex-m4 -mfloat-abi=hard -mlittle-endian -mthumb -I"C:/ti/ti-cgt-armllvm_2.1.2.LTS/include/c" -I"C:/ti/mcu_plus_sdk_am62x_08_06_00_18/source" -I"C:/ti/mcu_plus_sdk_am62x_08_06_00_18/source/kernel/freertos/FreeRTOS-Kernel/include" -I"C:/ti/mcu_plus_sdk_am62x_08_06_00_18/source/kernel/freertos/portable/TI_ARM_CLANG/ARM_CM4F" -I"C:/ti/mcu_plus_sdk_am62x_08_06_00_18/source/kernel/freertos/config/am62x/m4f" -DSOC_AM62X -D_DEBUG_=1 -g -Wall -Wno-gnu-variable-sized-type-not-at-end -Wno-unused-function -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)" -I"C:/Users/a0507040/Desktop/SPI_interface/Single_Byte_Multi_Channel/Single_Byte_Multi_Channel_MCSPI_AM62x/Debug/syscfg"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


