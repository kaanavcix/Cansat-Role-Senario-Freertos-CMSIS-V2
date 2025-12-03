################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Library/Utility/utility.c 

OBJS += \
./Core/Library/Utility/utility.o 

C_DEPS += \
./Core/Library/Utility/utility.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Library/Utility/%.o Core/Library/Utility/%.su Core/Library/Utility/%.cyclo: ../Core/Library/Utility/%.c Core/Library/Utility/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Library-2f-Utility

clean-Core-2f-Library-2f-Utility:
	-$(RM) ./Core/Library/Utility/utility.cyclo ./Core/Library/Utility/utility.d ./Core/Library/Utility/utility.o ./Core/Library/Utility/utility.su

.PHONY: clean-Core-2f-Library-2f-Utility

