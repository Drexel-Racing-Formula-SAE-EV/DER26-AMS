################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/tasks/adbms_task.c \
../Core/Src/tasks/air_task.c \
../Core/Src/tasks/canbus_task.c \
../Core/Src/tasks/cli_task.c \
../Core/Src/tasks/current_task.c \
../Core/Src/tasks/error_task.c \
../Core/Src/tasks/fan_task.c \
../Core/Src/tasks/imd_task.c 

OBJS += \
./Core/Src/tasks/adbms_task.o \
./Core/Src/tasks/air_task.o \
./Core/Src/tasks/canbus_task.o \
./Core/Src/tasks/cli_task.o \
./Core/Src/tasks/current_task.o \
./Core/Src/tasks/error_task.o \
./Core/Src/tasks/fan_task.o \
./Core/Src/tasks/imd_task.o 

C_DEPS += \
./Core/Src/tasks/adbms_task.d \
./Core/Src/tasks/air_task.d \
./Core/Src/tasks/canbus_task.d \
./Core/Src/tasks/cli_task.d \
./Core/Src/tasks/current_task.d \
./Core/Src/tasks/error_task.d \
./Core/Src/tasks/fan_task.d \
./Core/Src/tasks/imd_task.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/tasks/%.o Core/Src/tasks/%.su Core/Src/tasks/%.cyclo: ../Core/Src/tasks/%.c Core/Src/tasks/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F767xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-tasks

clean-Core-2f-Src-2f-tasks:
	-$(RM) ./Core/Src/tasks/adbms_task.cyclo ./Core/Src/tasks/adbms_task.d ./Core/Src/tasks/adbms_task.o ./Core/Src/tasks/adbms_task.su ./Core/Src/tasks/air_task.cyclo ./Core/Src/tasks/air_task.d ./Core/Src/tasks/air_task.o ./Core/Src/tasks/air_task.su ./Core/Src/tasks/canbus_task.cyclo ./Core/Src/tasks/canbus_task.d ./Core/Src/tasks/canbus_task.o ./Core/Src/tasks/canbus_task.su ./Core/Src/tasks/cli_task.cyclo ./Core/Src/tasks/cli_task.d ./Core/Src/tasks/cli_task.o ./Core/Src/tasks/cli_task.su ./Core/Src/tasks/current_task.cyclo ./Core/Src/tasks/current_task.d ./Core/Src/tasks/current_task.o ./Core/Src/tasks/current_task.su ./Core/Src/tasks/error_task.cyclo ./Core/Src/tasks/error_task.d ./Core/Src/tasks/error_task.o ./Core/Src/tasks/error_task.su ./Core/Src/tasks/fan_task.cyclo ./Core/Src/tasks/fan_task.d ./Core/Src/tasks/fan_task.o ./Core/Src/tasks/fan_task.su ./Core/Src/tasks/imd_task.cyclo ./Core/Src/tasks/imd_task.d ./Core/Src/tasks/imd_task.o ./Core/Src/tasks/imd_task.su

.PHONY: clean-Core-2f-Src-2f-tasks

