################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/img/calibration.c \
../Core/Src/img/mtest.c \
../Core/Src/img/multimetr.c \
../Core/Src/img/multitool.c \
../Core/Src/img/oscyloskop.c \
../Core/Src/img/pitlab_logo18.c \
../Core/Src/img/plogo165x80.c \
../Core/Src/img/ppm.c \
../Core/Src/img/sbus.c 

OBJS += \
./Core/Src/img/calibration.o \
./Core/Src/img/mtest.o \
./Core/Src/img/multimetr.o \
./Core/Src/img/multitool.o \
./Core/Src/img/oscyloskop.o \
./Core/Src/img/pitlab_logo18.o \
./Core/Src/img/plogo165x80.o \
./Core/Src/img/ppm.o \
./Core/Src/img/sbus.o 

C_DEPS += \
./Core/Src/img/calibration.d \
./Core/Src/img/mtest.d \
./Core/Src/img/multimetr.d \
./Core/Src/img/multitool.d \
./Core/Src/img/oscyloskop.d \
./Core/Src/img/pitlab_logo18.d \
./Core/Src/img/plogo165x80.d \
./Core/Src/img/ppm.d \
./Core/Src/img/sbus.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/img/%.o Core/Src/img/%.su Core/Src/img/%.cyclo: ../Core/Src/img/%.c Core/Src/img/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -DCORE_CM7 -DUSE_HAL_DRIVER -DSTM32H755xx -DUSE_PWR_SMPS_2V5_SUPPLIES_LDO -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -I../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../LWIP/App -I../LWIP/Target -I../../Middlewares/Third_Party/LwIP/src/include -I../../Middlewares/Third_Party/LwIP/system -I../../Drivers/BSP/Components/lan8742 -I../../Middlewares/Third_Party/LwIP/src/include/netif/ppp -I../../Middlewares/Third_Party/LwIP/src/include/lwip -I../../Middlewares/Third_Party/LwIP/src/include/lwip/apps -I../../Middlewares/Third_Party/LwIP/src/include/lwip/priv -I../../Middlewares/Third_Party/LwIP/src/include/lwip/prot -I../../Middlewares/Third_Party/LwIP/src/include/netif -I../../Middlewares/Third_Party/LwIP/src/include/compat/posix -I../../Middlewares/Third_Party/LwIP/src/include/compat/posix/arpa -I../../Middlewares/Third_Party/LwIP/src/include/compat/posix/net -I../../Middlewares/Third_Party/LwIP/src/include/compat/posix/sys -I../../Middlewares/Third_Party/LwIP/src/include/compat/stdc -I../../Middlewares/Third_Party/LwIP/system/arch -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-img

clean-Core-2f-Src-2f-img:
	-$(RM) ./Core/Src/img/calibration.cyclo ./Core/Src/img/calibration.d ./Core/Src/img/calibration.o ./Core/Src/img/calibration.su ./Core/Src/img/mtest.cyclo ./Core/Src/img/mtest.d ./Core/Src/img/mtest.o ./Core/Src/img/mtest.su ./Core/Src/img/multimetr.cyclo ./Core/Src/img/multimetr.d ./Core/Src/img/multimetr.o ./Core/Src/img/multimetr.su ./Core/Src/img/multitool.cyclo ./Core/Src/img/multitool.d ./Core/Src/img/multitool.o ./Core/Src/img/multitool.su ./Core/Src/img/oscyloskop.cyclo ./Core/Src/img/oscyloskop.d ./Core/Src/img/oscyloskop.o ./Core/Src/img/oscyloskop.su ./Core/Src/img/pitlab_logo18.cyclo ./Core/Src/img/pitlab_logo18.d ./Core/Src/img/pitlab_logo18.o ./Core/Src/img/pitlab_logo18.su ./Core/Src/img/plogo165x80.cyclo ./Core/Src/img/plogo165x80.d ./Core/Src/img/plogo165x80.o ./Core/Src/img/plogo165x80.su ./Core/Src/img/ppm.cyclo ./Core/Src/img/ppm.d ./Core/Src/img/ppm.o ./Core/Src/img/ppm.su ./Core/Src/img/sbus.cyclo ./Core/Src/img/sbus.d ./Core/Src/img/sbus.o ./Core/Src/img/sbus.su

.PHONY: clean-Core-2f-Src-2f-img

