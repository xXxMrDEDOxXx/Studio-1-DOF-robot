################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/comm/base_system.c 

OBJS += \
./Core/Src/comm/base_system.o 

C_DEPS += \
./Core/Src/comm/base_system.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/comm/%.o Core/Src/comm/%.su Core/Src/comm/%.cyclo: ../Core/Src/comm/%.c Core/Src/comm/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-comm

clean-Core-2f-Src-2f-comm:
	-$(RM) ./Core/Src/comm/base_system.cyclo ./Core/Src/comm/base_system.d ./Core/Src/comm/base_system.o ./Core/Src/comm/base_system.su

.PHONY: clean-Core-2f-Src-2f-comm

