################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/LCD.c \
../Core/Src/RPi35B_480x320.c \
../Core/Src/W25Q128JV.c \
../Core/Src/dotyk.c \
../Core/Src/flash_konfig.c \
../Core/Src/flash_nor.c \
../Core/Src/font.c \
../Core/Src/freertos.c \
../Core/Src/kamera.c \
../Core/Src/main.c \
../Core/Src/moduly_SPI.c \
../Core/Src/protokol_kom.c \
../Core/Src/stm32h7xx_hal_msp.c \
../Core/Src/stm32h7xx_hal_timebase_tim.c \
../Core/Src/stm32h7xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/wymiana_CM7.c 

OBJS += \
./Core/Src/LCD.o \
./Core/Src/RPi35B_480x320.o \
./Core/Src/W25Q128JV.o \
./Core/Src/dotyk.o \
./Core/Src/flash_konfig.o \
./Core/Src/flash_nor.o \
./Core/Src/font.o \
./Core/Src/freertos.o \
./Core/Src/kamera.o \
./Core/Src/main.o \
./Core/Src/moduly_SPI.o \
./Core/Src/protokol_kom.o \
./Core/Src/stm32h7xx_hal_msp.o \
./Core/Src/stm32h7xx_hal_timebase_tim.o \
./Core/Src/stm32h7xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/wymiana_CM7.o 

C_DEPS += \
./Core/Src/LCD.d \
./Core/Src/RPi35B_480x320.d \
./Core/Src/W25Q128JV.d \
./Core/Src/dotyk.d \
./Core/Src/flash_konfig.d \
./Core/Src/flash_nor.d \
./Core/Src/font.d \
./Core/Src/freertos.d \
./Core/Src/kamera.d \
./Core/Src/main.d \
./Core/Src/moduly_SPI.d \
./Core/Src/protokol_kom.d \
./Core/Src/stm32h7xx_hal_msp.d \
./Core/Src/stm32h7xx_hal_timebase_tim.d \
./Core/Src/stm32h7xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/wymiana_CM7.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -DCORE_CM7 -DUSE_HAL_DRIVER -DSTM32H755xx -DUSE_PWR_SMPS_2V5_SUPPLIES_LDO -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -I../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../LWIP/App -I../LWIP/Target -I../../Middlewares/Third_Party/LwIP/src/include -I../../Middlewares/Third_Party/LwIP/system -I../../Drivers/BSP/Components/lan8742 -I../../Middlewares/Third_Party/LwIP/src/include/netif/ppp -I../../Middlewares/Third_Party/LwIP/src/include/lwip -I../../Middlewares/Third_Party/LwIP/src/include/lwip/apps -I../../Middlewares/Third_Party/LwIP/src/include/lwip/priv -I../../Middlewares/Third_Party/LwIP/src/include/lwip/prot -I../../Middlewares/Third_Party/LwIP/src/include/netif -I../../Middlewares/Third_Party/LwIP/src/include/compat/posix -I../../Middlewares/Third_Party/LwIP/src/include/compat/posix/arpa -I../../Middlewares/Third_Party/LwIP/src/include/compat/posix/net -I../../Middlewares/Third_Party/LwIP/src/include/compat/posix/sys -I../../Middlewares/Third_Party/LwIP/src/include/compat/stdc -I../../Middlewares/Third_Party/LwIP/system/arch -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/LCD.cyclo ./Core/Src/LCD.d ./Core/Src/LCD.o ./Core/Src/LCD.su ./Core/Src/RPi35B_480x320.cyclo ./Core/Src/RPi35B_480x320.d ./Core/Src/RPi35B_480x320.o ./Core/Src/RPi35B_480x320.su ./Core/Src/W25Q128JV.cyclo ./Core/Src/W25Q128JV.d ./Core/Src/W25Q128JV.o ./Core/Src/W25Q128JV.su ./Core/Src/dotyk.cyclo ./Core/Src/dotyk.d ./Core/Src/dotyk.o ./Core/Src/dotyk.su ./Core/Src/flash_konfig.cyclo ./Core/Src/flash_konfig.d ./Core/Src/flash_konfig.o ./Core/Src/flash_konfig.su ./Core/Src/flash_nor.cyclo ./Core/Src/flash_nor.d ./Core/Src/flash_nor.o ./Core/Src/flash_nor.su ./Core/Src/font.cyclo ./Core/Src/font.d ./Core/Src/font.o ./Core/Src/font.su ./Core/Src/freertos.cyclo ./Core/Src/freertos.d ./Core/Src/freertos.o ./Core/Src/freertos.su ./Core/Src/kamera.cyclo ./Core/Src/kamera.d ./Core/Src/kamera.o ./Core/Src/kamera.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/moduly_SPI.cyclo ./Core/Src/moduly_SPI.d ./Core/Src/moduly_SPI.o ./Core/Src/moduly_SPI.su ./Core/Src/protokol_kom.cyclo ./Core/Src/protokol_kom.d ./Core/Src/protokol_kom.o ./Core/Src/protokol_kom.su ./Core/Src/stm32h7xx_hal_msp.cyclo ./Core/Src/stm32h7xx_hal_msp.d ./Core/Src/stm32h7xx_hal_msp.o ./Core/Src/stm32h7xx_hal_msp.su ./Core/Src/stm32h7xx_hal_timebase_tim.cyclo ./Core/Src/stm32h7xx_hal_timebase_tim.d ./Core/Src/stm32h7xx_hal_timebase_tim.o ./Core/Src/stm32h7xx_hal_timebase_tim.su ./Core/Src/stm32h7xx_it.cyclo ./Core/Src/stm32h7xx_it.d ./Core/Src/stm32h7xx_it.o ./Core/Src/stm32h7xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/wymiana_CM7.cyclo ./Core/Src/wymiana_CM7.d ./Core/Src/wymiana_CM7.o ./Core/Src/wymiana_CM7.su

.PHONY: clean-Core-2f-Src

