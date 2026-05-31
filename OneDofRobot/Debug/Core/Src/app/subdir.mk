################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/app/auto_mission.c \
../Core/Src/app/dashboard.c \
../Core/Src/app/gripper.c \
../Core/Src/app/homing.c \
../Core/Src/app/joystick.c \
../Core/Src/app/test_mode.c

OBJS += \
./Core/Src/app/auto_mission.o \
./Core/Src/app/dashboard.o \
./Core/Src/app/gripper.o \
./Core/Src/app/homing.o \
./Core/Src/app/joystick.o \
./Core/Src/app/test_mode.o

C_DEPS += \
./Core/Src/app/auto_mission.d \
./Core/Src/app/dashboard.d \
./Core/Src/app/gripper.d \
./Core/Src/app/homing.d \
./Core/Src/app/joystick.d \
./Core/Src/app/test_mode.d


# Each subdirectory must supply rules for building sources it contributes
Core/Src/app/%.o Core/Src/app/%.su Core/Src/app/%.cyclo: ../Core/Src/app/%.c Core/Src/app/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-app

clean-Core-2f-Src-2f-app:
	-$(RM) ./Core/Src/app/auto_mission.cyclo ./Core/Src/app/auto_mission.d ./Core/Src/app/auto_mission.o ./Core/Src/app/auto_mission.su ./Core/Src/app/dashboard.cyclo ./Core/Src/app/dashboard.d ./Core/Src/app/dashboard.o ./Core/Src/app/dashboard.su ./Core/Src/app/gripper.cyclo ./Core/Src/app/gripper.d ./Core/Src/app/gripper.o ./Core/Src/app/gripper.su ./Core/Src/app/homing.cyclo ./Core/Src/app/homing.d ./Core/Src/app/homing.o ./Core/Src/app/homing.su ./Core/Src/app/joystick.cyclo ./Core/Src/app/joystick.d ./Core/Src/app/joystick.o ./Core/Src/app/joystick.su ./Core/Src/app/test_mode.cyclo ./Core/Src/app/test_mode.d ./Core/Src/app/test_mode.o ./Core/Src/app/test_mode.su

.PHONY: clean-Core-2f-Src-2f-app

