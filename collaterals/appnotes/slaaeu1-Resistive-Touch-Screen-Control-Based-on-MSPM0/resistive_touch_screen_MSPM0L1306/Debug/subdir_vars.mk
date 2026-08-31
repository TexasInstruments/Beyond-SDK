################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
SYSCFG_SRCS += \
../LowLevelSettings.syscfg 

C_SRCS += \
./ti_msp_dl_config.c \
/home/yashraj/ti/mspm0_sdk_2_06_00_05/source/ti/devices/msp/m0p/startup_system_files/ticlang/startup_mspm0l130x_ticlang.c \
../UART.c \
../main.c \
../resistive_detection.c 

GEN_CMDS += \
./device_linker.cmd 

GEN_FILES += \
./device_linker.cmd \
./device.opt \
./ti_msp_dl_config.c 

C_DEPS += \
./ti_msp_dl_config.d \
./startup_mspm0l130x_ticlang.d \
./UART.d \
./main.d \
./resistive_detection.d 

GEN_OPTS += \
./device.opt 

OBJS += \
./ti_msp_dl_config.o \
./startup_mspm0l130x_ticlang.o \
./UART.o \
./main.o \
./resistive_detection.o 

GEN_MISC_FILES += \
./device.cmd.genlibs \
./ti_msp_dl_config.h \
./Event.dot 

OBJS__QUOTED += \
"ti_msp_dl_config.o" \
"startup_mspm0l130x_ticlang.o" \
"UART.o" \
"main.o" \
"resistive_detection.o" 

GEN_MISC_FILES__QUOTED += \
"device.cmd.genlibs" \
"ti_msp_dl_config.h" \
"Event.dot" 

C_DEPS__QUOTED += \
"ti_msp_dl_config.d" \
"startup_mspm0l130x_ticlang.d" \
"UART.d" \
"main.d" \
"resistive_detection.d" 

GEN_FILES__QUOTED += \
"device_linker.cmd" \
"device.opt" \
"ti_msp_dl_config.c" 

SYSCFG_SRCS__QUOTED += \
"../LowLevelSettings.syscfg" 

C_SRCS__QUOTED += \
"./ti_msp_dl_config.c" \
"/home/yashraj/ti/mspm0_sdk_2_06_00_05/source/ti/devices/msp/m0p/startup_system_files/ticlang/startup_mspm0l130x_ticlang.c" \
"../UART.c" \
"../main.c" \
"../resistive_detection.c" 


