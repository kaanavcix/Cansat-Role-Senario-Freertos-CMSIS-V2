################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Library/Communication/lora_e22.c \
../Core/Library/Communication/telemetry_packet.c 

OBJS += \
./Core/Library/Communication/lora_e22.o \
./Core/Library/Communication/telemetry_packet.o 

C_DEPS += \
./Core/Library/Communication/lora_e22.d \
./Core/Library/Communication/telemetry_packet.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Library/Communication/%.o Core/Library/Communication/%.su Core/Library/Communication/%.cyclo: ../Core/Library/Communication/%.c Core/Library/Communication/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Library-2f-Communication

clean-Core-2f-Library-2f-Communication:
	-$(RM) ./Core/Library/Communication/lora_e22.cyclo ./Core/Library/Communication/lora_e22.d ./Core/Library/Communication/lora_e22.o ./Core/Library/Communication/lora_e22.su ./Core/Library/Communication/telemetry_packet.cyclo ./Core/Library/Communication/telemetry_packet.d ./Core/Library/Communication/telemetry_packet.o ./Core/Library/Communication/telemetry_packet.su

.PHONY: clean-Core-2f-Library-2f-Communication

