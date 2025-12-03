################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Library/Acceleration/mpu6050.c 

OBJS += \
./Core/Library/Acceleration/mpu6050.o 

C_DEPS += \
./Core/Library/Acceleration/mpu6050.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Library/Acceleration/%.o Core/Library/Acceleration/%.su Core/Library/Acceleration/%.cyclo: ../Core/Library/Acceleration/%.c Core/Library/Acceleration/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Library-2f-Acceleration

clean-Core-2f-Library-2f-Acceleration:
	-$(RM) ./Core/Library/Acceleration/mpu6050.cyclo ./Core/Library/Acceleration/mpu6050.d ./Core/Library/Acceleration/mpu6050.o ./Core/Library/Acceleration/mpu6050.su

.PHONY: clean-Core-2f-Library-2f-Acceleration

