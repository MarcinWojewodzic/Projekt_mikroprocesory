################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/GFX_Color.c \
../Core/Src/GFX_Lepsze.c \
../Core/Src/Lepsze.c \
../Core/Src/MFRC522.c \
../Core/Src/adc.c \
../Core/Src/dma.c \
../Core/Src/gpio.c \
../Core/Src/keypad.c \
../Core/Src/main.c \
../Core/Src/spi.c \
../Core/Src/ssd1331.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/tim.c \
../Core/Src/ws2812b.c 

OBJS += \
./Core/Src/GFX_Color.o \
./Core/Src/GFX_Lepsze.o \
./Core/Src/Lepsze.o \
./Core/Src/MFRC522.o \
./Core/Src/adc.o \
./Core/Src/dma.o \
./Core/Src/gpio.o \
./Core/Src/keypad.o \
./Core/Src/main.o \
./Core/Src/spi.o \
./Core/Src/ssd1331.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/tim.o \
./Core/Src/ws2812b.o 

C_DEPS += \
./Core/Src/GFX_Color.d \
./Core/Src/GFX_Lepsze.d \
./Core/Src/Lepsze.d \
./Core/Src/MFRC522.d \
./Core/Src/adc.d \
./Core/Src/dma.d \
./Core/Src/gpio.d \
./Core/Src/keypad.d \
./Core/Src/main.d \
./Core/Src/spi.d \
./Core/Src/ssd1331.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/tim.d \
./Core/Src/ws2812b.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -DARM_MATH_CM4 -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/GFX_Color.d ./Core/Src/GFX_Color.o ./Core/Src/GFX_Lepsze.d ./Core/Src/GFX_Lepsze.o ./Core/Src/Lepsze.d ./Core/Src/Lepsze.o ./Core/Src/MFRC522.d ./Core/Src/MFRC522.o ./Core/Src/adc.d ./Core/Src/adc.o ./Core/Src/dma.d ./Core/Src/dma.o ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/keypad.d ./Core/Src/keypad.o ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/spi.d ./Core/Src/spi.o ./Core/Src/ssd1331.d ./Core/Src/ssd1331.o ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/ws2812b.d ./Core/Src/ws2812b.o

.PHONY: clean-Core-2f-Src

