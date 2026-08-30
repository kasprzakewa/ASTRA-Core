################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/coast_sil.c \
../Core/Src/control.c \
../Core/Src/flight_log.c \
../Core/Src/gpio.c \
../Core/Src/main.c \
../Core/Src/pd.c \
../Core/Src/physics.c \
../Core/Src/predictor.c \
../Core/Src/stm32h7xx_hal_msp.c \
../Core/Src/stm32h7xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32h7xx.c \
../Core/Src/telem_playback.c \
../Core/Src/telem_playback_data.c \
../Core/Src/telemetry.c \
../Core/Src/tim.c \
../Core/Src/usart.c 

OBJS += \
./Core/Src/coast_sil.o \
./Core/Src/control.o \
./Core/Src/flight_log.o \
./Core/Src/gpio.o \
./Core/Src/main.o \
./Core/Src/pd.o \
./Core/Src/physics.o \
./Core/Src/predictor.o \
./Core/Src/stm32h7xx_hal_msp.o \
./Core/Src/stm32h7xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32h7xx.o \
./Core/Src/telem_playback.o \
./Core/Src/telem_playback_data.o \
./Core/Src/telemetry.o \
./Core/Src/tim.o \
./Core/Src/usart.o 

C_DEPS += \
./Core/Src/coast_sil.d \
./Core/Src/control.d \
./Core/Src/flight_log.d \
./Core/Src/gpio.d \
./Core/Src/main.d \
./Core/Src/pd.d \
./Core/Src/physics.d \
./Core/Src/predictor.d \
./Core/Src/stm32h7xx_hal_msp.d \
./Core/Src/stm32h7xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32h7xx.d \
./Core/Src/telem_playback.d \
./Core/Src/telem_playback_data.d \
./Core/Src/telemetry.d \
./Core/Src/tim.d \
./Core/Src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H723xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/BSP/STM32H7xx_Nucleo -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/coast_sil.cyclo ./Core/Src/coast_sil.d ./Core/Src/coast_sil.o ./Core/Src/coast_sil.su ./Core/Src/control.cyclo ./Core/Src/control.d ./Core/Src/control.o ./Core/Src/control.su ./Core/Src/flight_log.cyclo ./Core/Src/flight_log.d ./Core/Src/flight_log.o ./Core/Src/flight_log.su ./Core/Src/gpio.cyclo ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/pd.cyclo ./Core/Src/pd.d ./Core/Src/pd.o ./Core/Src/pd.su ./Core/Src/physics.cyclo ./Core/Src/physics.d ./Core/Src/physics.o ./Core/Src/physics.su ./Core/Src/predictor.cyclo ./Core/Src/predictor.d ./Core/Src/predictor.o ./Core/Src/predictor.su ./Core/Src/stm32h7xx_hal_msp.cyclo ./Core/Src/stm32h7xx_hal_msp.d ./Core/Src/stm32h7xx_hal_msp.o ./Core/Src/stm32h7xx_hal_msp.su ./Core/Src/stm32h7xx_it.cyclo ./Core/Src/stm32h7xx_it.d ./Core/Src/stm32h7xx_it.o ./Core/Src/stm32h7xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32h7xx.cyclo ./Core/Src/system_stm32h7xx.d ./Core/Src/system_stm32h7xx.o ./Core/Src/system_stm32h7xx.su ./Core/Src/telem_playback.cyclo ./Core/Src/telem_playback.d ./Core/Src/telem_playback.o ./Core/Src/telem_playback.su ./Core/Src/telem_playback_data.cyclo ./Core/Src/telem_playback_data.d ./Core/Src/telem_playback_data.o ./Core/Src/telem_playback_data.su ./Core/Src/telemetry.cyclo ./Core/Src/telemetry.d ./Core/Src/telemetry.o ./Core/Src/telemetry.su ./Core/Src/tim.cyclo ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/tim.su ./Core/Src/usart.cyclo ./Core/Src/usart.d ./Core/Src/usart.o ./Core/Src/usart.su

.PHONY: clean-Core-2f-Src

