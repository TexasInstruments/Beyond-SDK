################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
ticlang/%.o: ../ticlang/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"C:/ti/ccs1230/ccs/tools/compiler/ti-cgt-armllvm_2.1.3.LTS/bin/tiarmclang.exe" -c -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -I"C:/Users/a0507040/Desktop/SPI_interface/Single_Byte_Multi_Channel/MSPM0_ADC_SPI" -I"C:/Users/a0507040/Desktop/SPI_interface/Single_Byte_Multi_Channel/MSPM0_ADC_SPI/Debug" -I"C:/ti/mspm0_sdk_1_00_01_03/source/third_party/CMSIS/Core/Include" -I"C:/ti/mspm0_sdk_1_00_01_03/source" -D__MSPM0L1306__ -gdwarf-3 -MMD -MP -MF"ticlang/$(basename $(<F)).d_raw" -MT"$(@)" -I"C:/Users/a0507040/Desktop/SPI_interface/Single_Byte_Multi_Channel/MSPM0_ADC_SPI/Debug/syscfg"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


