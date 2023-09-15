################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"C:/ti/ccs1230/ccs/tools/compiler/ti-cgt-armllvm_2.1.3.LTS/bin/tiarmclang.exe" -c -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -I"C:/Users/a0507040/Desktop/SPI_interface/Multi_Byte_Multi_Channel/MSPM0_ADC_SPI" -I"C:/Users/a0507040/Desktop/SPI_interface/Multi_Byte_Multi_Channel/MSPM0_ADC_SPI/Debug" -I"C:/ti/mspm0_sdk_1_00_01_03/source/third_party/CMSIS/Core/Include" -I"C:/ti/mspm0_sdk_1_00_01_03/source" -D__MSPM0L1306__ -gdwarf-3 -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)" -I"C:/Users/a0507040/Desktop/SPI_interface/Multi_Byte_Multi_Channel/MSPM0_ADC_SPI/Debug/syscfg"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

build-1489417521: ../spi_peripheral_repeated_multibyte_fifo_dma_interrupts.syscfg
	@echo 'Building file: "$<"'
	@echo 'Invoking: SysConfig'
	"C:/ti/ccs1230/ccs/utils/sysconfig_1.16.1/sysconfig_cli.bat" -s "C:/ti/mspm0_sdk_1_00_01_03/.metadata/product.json" --script "C:/Users/a0507040/Desktop/SPI_interface/Multi_Byte_Multi_Channel/MSPM0_ADC_SPI/spi_peripheral_repeated_multibyte_fifo_dma_interrupts.syscfg" -o "syscfg" --compiler ticlang
	@echo 'Finished building: "$<"'
	@echo ' '

syscfg/ti_msp_dl_config.c: build-1489417521 ../spi_peripheral_repeated_multibyte_fifo_dma_interrupts.syscfg
syscfg/ti_msp_dl_config.h: build-1489417521
syscfg/Event.dot: build-1489417521
syscfg/: build-1489417521

syscfg/%.o: ./syscfg/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"C:/ti/ccs1230/ccs/tools/compiler/ti-cgt-armllvm_2.1.3.LTS/bin/tiarmclang.exe" -c -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -I"C:/Users/a0507040/Desktop/SPI_interface/Multi_Byte_Multi_Channel/MSPM0_ADC_SPI" -I"C:/Users/a0507040/Desktop/SPI_interface/Multi_Byte_Multi_Channel/MSPM0_ADC_SPI/Debug" -I"C:/ti/mspm0_sdk_1_00_01_03/source/third_party/CMSIS/Core/Include" -I"C:/ti/mspm0_sdk_1_00_01_03/source" -D__MSPM0L1306__ -gdwarf-3 -MMD -MP -MF"syscfg/$(basename $(<F)).d_raw" -MT"$(@)" -I"C:/Users/a0507040/Desktop/SPI_interface/Multi_Byte_Multi_Channel/MSPM0_ADC_SPI/Debug/syscfg"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


