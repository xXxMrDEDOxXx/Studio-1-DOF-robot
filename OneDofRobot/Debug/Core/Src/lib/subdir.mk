################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/lib/cascade_control.c \
../Core/Src/lib/encoder.c \
../Core/Src/lib/kalman_filter.c \
../Core/Src/lib/trajectory.c 

OBJS += \
./Core/Src/lib/cascade_control.o \
./Core/Src/lib/encoder.o \
./Core/Src/lib/kalman_filter.o \
./Core/Src/lib/trajectory.o 

C_DEPS += \
./Core/Src/lib/cascade_control.d \
./Core/Src/lib/encoder.d \
./Core/Src/lib/kalman_filter.d \
./Core/Src/lib/trajectory.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/lib/%.o Core/Src/lib/%.su Core/Src/lib/%.cyclo: ../Core/Src/lib/%.c Core/Src/lib/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-lib

clean-Core-2f-Src-2f-lib:
	-$(RM) ./Core/Src/lib/cascade_control.cyclo ./Core/Src/lib/cascade_control.d ./Core/Src/lib/cascade_control.o ./Core/Src/lib/cascade_control.su ./Core/Src/lib/encoder.cyclo ./Core/Src/lib/encoder.d ./Core/Src/lib/encoder.o ./Core/Src/lib/encoder.su ./Core/Src/lib/kalman_filter.cyclo ./Core/Src/lib/kalman_filter.d ./Core/Src/lib/kalman_filter.o ./Core/Src/lib/kalman_filter.su ./Core/Src/lib/trajectory.cyclo ./Core/Src/lib/trajectory.d ./Core/Src/lib/trajectory.o ./Core/Src/lib/trajectory.su

.PHONY: clean-Core-2f-Src-2f-lib

