################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/deca_device.c \
../src/deca_led.c \
../src/deca_params_init.c \
../src/deca_spi.c \
../src/main.c \
../src/syscalls.c \
../src/system_stm32f4xx.c 

OBJS += \
./src/deca_device.o \
./src/deca_led.o \
./src/deca_params_init.o \
./src/deca_spi.o \
./src/main.o \
./src/syscalls.o \
./src/system_stm32f4xx.o 

C_DEPS += \
./src/deca_device.d \
./src/deca_led.d \
./src/deca_params_init.d \
./src/deca_spi.d \
./src/main.d \
./src/syscalls.d \
./src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DSTM32F411VETx -DSTM32F4 -DSTM32 -DSTM32F411E_DISCO -DDEBUG -DUSE_STDPERIPH_DRIVER -DSTM32F411xE -I"C:/Users/vedagupt/workspace/NewBoard/inc" -I"C:/Users/vedagupt/workspace/NewBoard/CMSIS/core" -I"C:/Users/vedagupt/workspace/NewBoard/CMSIS/device" -I"C:/Users/vedagupt/workspace/NewBoard/StdPeriph_Driver/inc" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


