################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/ModBusRTU.c \
../Core/Src/Qubic.c \
../Core/Src/Qubic_emxAPI.c \
../Core/Src/Qubic_emxutil.c \
../Core/Src/Qubic_initialize.c \
../Core/Src/Qubic_terminate.c \
../Core/Src/main.c \
../Core/Src/rtGetInf.c \
../Core/Src/rtGetNaN.c \
../Core/Src/rt_nonfinite.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c 

OBJS += \
./Core/Src/ModBusRTU.o \
./Core/Src/Qubic.o \
./Core/Src/Qubic_emxAPI.o \
./Core/Src/Qubic_emxutil.o \
./Core/Src/Qubic_initialize.o \
./Core/Src/Qubic_terminate.o \
./Core/Src/main.o \
./Core/Src/rtGetInf.o \
./Core/Src/rtGetNaN.o \
./Core/Src/rt_nonfinite.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o 

C_DEPS += \
./Core/Src/ModBusRTU.d \
./Core/Src/Qubic.d \
./Core/Src/Qubic_emxAPI.d \
./Core/Src/Qubic_emxutil.d \
./Core/Src/Qubic_initialize.d \
./Core/Src/Qubic_terminate.d \
./Core/Src/main.d \
./Core/Src/rtGetInf.d \
./Core/Src/rtGetNaN.d \
./Core/Src/rt_nonfinite.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/Core/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/Core_A/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/ModBusRTU.cyclo ./Core/Src/ModBusRTU.d ./Core/Src/ModBusRTU.o ./Core/Src/ModBusRTU.su ./Core/Src/Qubic.cyclo ./Core/Src/Qubic.d ./Core/Src/Qubic.o ./Core/Src/Qubic.su ./Core/Src/Qubic_emxAPI.cyclo ./Core/Src/Qubic_emxAPI.d ./Core/Src/Qubic_emxAPI.o ./Core/Src/Qubic_emxAPI.su ./Core/Src/Qubic_emxutil.cyclo ./Core/Src/Qubic_emxutil.d ./Core/Src/Qubic_emxutil.o ./Core/Src/Qubic_emxutil.su ./Core/Src/Qubic_initialize.cyclo ./Core/Src/Qubic_initialize.d ./Core/Src/Qubic_initialize.o ./Core/Src/Qubic_initialize.su ./Core/Src/Qubic_terminate.cyclo ./Core/Src/Qubic_terminate.d ./Core/Src/Qubic_terminate.o ./Core/Src/Qubic_terminate.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/rtGetInf.cyclo ./Core/Src/rtGetInf.d ./Core/Src/rtGetInf.o ./Core/Src/rtGetInf.su ./Core/Src/rtGetNaN.cyclo ./Core/Src/rtGetNaN.d ./Core/Src/rtGetNaN.o ./Core/Src/rtGetNaN.su ./Core/Src/rt_nonfinite.cyclo ./Core/Src/rt_nonfinite.d ./Core/Src/rt_nonfinite.o ./Core/Src/rt_nonfinite.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su

.PHONY: clean-Core-2f-Src

