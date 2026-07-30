################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/ext_drivers/accumulator.c \
../Core/Src/ext_drivers/adbms2950.c \
../Core/Src/ext_drivers/adbms6830.c \
../Core/Src/ext_drivers/adbms_shared.c \
../Core/Src/ext_drivers/canbus.c \
../Core/Src/ext_drivers/cli.c \
../Core/Src/ext_drivers/current_sensor.c \
../Core/Src/ext_drivers/fans.c \
../Core/Src/ext_drivers/imd.c \
../Core/Src/ext_drivers/stm32f767z.c 

OBJS += \
./Core/Src/ext_drivers/accumulator.o \
./Core/Src/ext_drivers/adbms2950.o \
./Core/Src/ext_drivers/adbms6830.o \
./Core/Src/ext_drivers/adbms_shared.o \
./Core/Src/ext_drivers/canbus.o \
./Core/Src/ext_drivers/cli.o \
./Core/Src/ext_drivers/current_sensor.o \
./Core/Src/ext_drivers/fans.o \
./Core/Src/ext_drivers/imd.o \
./Core/Src/ext_drivers/stm32f767z.o 

C_DEPS += \
./Core/Src/ext_drivers/accumulator.d \
./Core/Src/ext_drivers/adbms2950.d \
./Core/Src/ext_drivers/adbms6830.d \
./Core/Src/ext_drivers/adbms_shared.d \
./Core/Src/ext_drivers/canbus.d \
./Core/Src/ext_drivers/cli.d \
./Core/Src/ext_drivers/current_sensor.d \
./Core/Src/ext_drivers/fans.d \
./Core/Src/ext_drivers/imd.d \
./Core/Src/ext_drivers/stm32f767z.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/ext_drivers/%.o Core/Src/ext_drivers/%.su Core/Src/ext_drivers/%.cyclo: ../Core/Src/ext_drivers/%.c Core/Src/ext_drivers/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F767xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-ext_drivers

clean-Core-2f-Src-2f-ext_drivers:
	-$(RM) ./Core/Src/ext_drivers/accumulator.cyclo ./Core/Src/ext_drivers/accumulator.d ./Core/Src/ext_drivers/accumulator.o ./Core/Src/ext_drivers/accumulator.su ./Core/Src/ext_drivers/adbms2950.cyclo ./Core/Src/ext_drivers/adbms2950.d ./Core/Src/ext_drivers/adbms2950.o ./Core/Src/ext_drivers/adbms2950.su ./Core/Src/ext_drivers/adbms6830.cyclo ./Core/Src/ext_drivers/adbms6830.d ./Core/Src/ext_drivers/adbms6830.o ./Core/Src/ext_drivers/adbms6830.su ./Core/Src/ext_drivers/adbms_shared.cyclo ./Core/Src/ext_drivers/adbms_shared.d ./Core/Src/ext_drivers/adbms_shared.o ./Core/Src/ext_drivers/adbms_shared.su ./Core/Src/ext_drivers/canbus.cyclo ./Core/Src/ext_drivers/canbus.d ./Core/Src/ext_drivers/canbus.o ./Core/Src/ext_drivers/canbus.su ./Core/Src/ext_drivers/cli.cyclo ./Core/Src/ext_drivers/cli.d ./Core/Src/ext_drivers/cli.o ./Core/Src/ext_drivers/cli.su ./Core/Src/ext_drivers/current_sensor.cyclo ./Core/Src/ext_drivers/current_sensor.d ./Core/Src/ext_drivers/current_sensor.o ./Core/Src/ext_drivers/current_sensor.su ./Core/Src/ext_drivers/fans.cyclo ./Core/Src/ext_drivers/fans.d ./Core/Src/ext_drivers/fans.o ./Core/Src/ext_drivers/fans.su ./Core/Src/ext_drivers/imd.cyclo ./Core/Src/ext_drivers/imd.d ./Core/Src/ext_drivers/imd.o ./Core/Src/ext_drivers/imd.su ./Core/Src/ext_drivers/stm32f767z.cyclo ./Core/Src/ext_drivers/stm32f767z.d ./Core/Src/ext_drivers/stm32f767z.o ./Core/Src/ext_drivers/stm32f767z.su

.PHONY: clean-Core-2f-Src-2f-ext_drivers

