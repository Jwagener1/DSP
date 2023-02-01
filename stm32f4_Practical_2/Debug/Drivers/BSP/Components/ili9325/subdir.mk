################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/Components/ili9325/ili9325.c 

OBJS += \
./Drivers/BSP/Components/ili9325/ili9325.o 

C_DEPS += \
./Drivers/BSP/Components/ili9325/ili9325.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/Components/ili9325/%.o Drivers/BSP/Components/ili9325/%.su: ../Drivers/BSP/Components/ili9325/%.c Drivers/BSP/Components/ili9325/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F429xx -c -I../Core/Inc -I"/Users/jonathanwagener/STM32CubeIDE/workspace_1.8.0/stm32f4_ADC_40kHz/Drivers/CMSIS/DSP/Include" -I../USB_HOST/App -I../USB_HOST/Target -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/Users/jonathanwagener/STM32CubeIDE/workspace_1.8.0/stm32f4_ADC_40kHz/Drivers/BSP/STM32F429I-Discovery" -I"/Users/jonathanwagener/STM32CubeIDE/workspace_1.8.0/stm32f4_ADC_40kHz/Utilities/Pictures" -I"/Users/jonathanwagener/STM32CubeIDE/workspace_1.8.0/stm32f4_ADC_40kHz/Drivers/CMSIS/DSP/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-Components-2f-ili9325

clean-Drivers-2f-BSP-2f-Components-2f-ili9325:
	-$(RM) ./Drivers/BSP/Components/ili9325/ili9325.d ./Drivers/BSP/Components/ili9325/ili9325.o ./Drivers/BSP/Components/ili9325/ili9325.su

.PHONY: clean-Drivers-2f-BSP-2f-Components-2f-ili9325

