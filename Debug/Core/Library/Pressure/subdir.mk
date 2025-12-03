################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Library/Pressure/bmp280.c 

OBJS += \
./Core/Library/Pressure/bmp280.o 

C_DEPS += \
./Core/Library/Pressure/bmp280.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Library/Pressure/%.o Core/Library/Pressure/%.su Core/Library/Pressure/%.cyclo: ../Core/Library/Pressure/%.c Core/Library/Pressure/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Library-2f-Pressure

clean-Core-2f-Library-2f-Pressure:
	-$(RM) ./Core/Library/Pressure/bmp280.cyclo ./Core/Library/Pressure/bmp280.d ./Core/Library/Pressure/bmp280.o ./Core/Library/Pressure/bmp280.su

.PHONY: clean-Core-2f-Library-2f-Pressure

