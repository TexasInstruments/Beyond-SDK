################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"/home/yashraj/ti/ccs1281/ccs/tools/compiler/ti-cgt-armllvm_3.2.2.LTS/bin/tiarmclang" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -I"/home/yashraj/Untitled/single_byte_single_channel_LP_MSPM0L1306_nortos_ticlang" -I"/home/yashraj/Untitled/single_byte_single_channel_LP_MSPM0L1306_nortos_ticlang/Debug" -I"/home/yashraj/ti/mspm0_sdk_2_06_00_05/source/third_party/CMSIS/Core/Include" -I"/home/yashraj/ti/mspm0_sdk_2_06_00_05/source" -gdwarf-3 -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$(shell echo $<)"
	@echo 'Finished building: "$<"'
	@echo ' '

build-1612704353: ../single_byte_single_channel.syscfg
	@echo 'Building file: "$<"'
	@echo 'Invoking: SysConfig'
	"/home/yashraj/ti/sysconfig_1.23.0/sysconfig_cli.sh" --script "/home/yashraj/Untitled/single_byte_single_channel_LP_MSPM0L1306_nortos_ticlang/single_byte_single_channel.syscfg" -o "." -s "/home/yashraj/ti/mspm0_sdk_2_06_00_05/.metadata/product.json" --compiler ticlang
	@echo 'Finished building: "$<"'
	@echo ' '

device_linker.cmd: build-1612704353 ../single_byte_single_channel.syscfg
device.opt: build-1612704353
device.cmd.genlibs: build-1612704353
ti_msp_dl_config.c: build-1612704353
ti_msp_dl_config.h: build-1612704353
Event.dot: build-1612704353

%.o: ./%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"/home/yashraj/ti/ccs1281/ccs/tools/compiler/ti-cgt-armllvm_3.2.2.LTS/bin/tiarmclang" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -I"/home/yashraj/Untitled/single_byte_single_channel_LP_MSPM0L1306_nortos_ticlang" -I"/home/yashraj/Untitled/single_byte_single_channel_LP_MSPM0L1306_nortos_ticlang/Debug" -I"/home/yashraj/ti/mspm0_sdk_2_06_00_05/source/third_party/CMSIS/Core/Include" -I"/home/yashraj/ti/mspm0_sdk_2_06_00_05/source" -gdwarf-3 -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$(shell echo $<)"
	@echo 'Finished building: "$<"'
	@echo ' '

startup_mspm0l130x_ticlang.o: /home/yashraj/ti/mspm0_sdk_2_06_00_05/source/ti/devices/msp/m0p/startup_system_files/ticlang/startup_mspm0l130x_ticlang.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"/home/yashraj/ti/ccs1281/ccs/tools/compiler/ti-cgt-armllvm_3.2.2.LTS/bin/tiarmclang" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -I"/home/yashraj/Untitled/single_byte_single_channel_LP_MSPM0L1306_nortos_ticlang" -I"/home/yashraj/Untitled/single_byte_single_channel_LP_MSPM0L1306_nortos_ticlang/Debug" -I"/home/yashraj/ti/mspm0_sdk_2_06_00_05/source/third_party/CMSIS/Core/Include" -I"/home/yashraj/ti/mspm0_sdk_2_06_00_05/source" -gdwarf-3 -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$(shell echo $<)"
	@echo 'Finished building: "$<"'
	@echo ' '


