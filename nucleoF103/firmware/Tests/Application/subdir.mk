################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Application/FLASH_PAGE_F1.c \
../Application/app.c \
../Application/calibrate.c \
../Application/can_communication.c \
../Application/control.c \
../Application/measures.c \
../Application/power.c \
../Application/steering.c \
../Application/tests.c \
../Application/ultrasound.c \
../Application/wheels.c 

OBJS += \
./Application/FLASH_PAGE_F1.o \
./Application/app.o \
./Application/calibrate.o \
./Application/can_communication.o \
./Application/control.o \
./Application/measures.o \
./Application/power.o \
./Application/steering.o \
./Application/tests.o \
./Application/ultrasound.o \
./Application/wheels.o 

C_DEPS += \
./Application/FLASH_PAGE_F1.d \
./Application/app.d \
./Application/calibrate.d \
./Application/can_communication.d \
./Application/control.d \
./Application/measures.d \
./Application/power.d \
./Application/steering.d \
./Application/tests.d \
./Application/ultrasound.d \
./Application/wheels.d 


# Each subdirectory must supply rules for building sources it contributes
Application/%.o Application/%.su Application/%.cyclo: ../Application/%.c Application/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -D__TESTS__ -DSTM32F103xB -DUSE_HAL_DRIVER -DSTM32F103RBTx -DSTM32 -DSTM32F1 -DNUCLEO_F103RB -c -I../Drivers/CMSIS/Include -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Inc -I../Application -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Application

clean-Application:
	-$(RM) ./Application/FLASH_PAGE_F1.cyclo ./Application/FLASH_PAGE_F1.d ./Application/FLASH_PAGE_F1.o ./Application/FLASH_PAGE_F1.su ./Application/app.cyclo ./Application/app.d ./Application/app.o ./Application/app.su ./Application/calibrate.cyclo ./Application/calibrate.d ./Application/calibrate.o ./Application/calibrate.su ./Application/can_communication.cyclo ./Application/can_communication.d ./Application/can_communication.o ./Application/can_communication.su ./Application/control.cyclo ./Application/control.d ./Application/control.o ./Application/control.su ./Application/measures.cyclo ./Application/measures.d ./Application/measures.o ./Application/measures.su ./Application/power.cyclo ./Application/power.d ./Application/power.o ./Application/power.su ./Application/steering.cyclo ./Application/steering.d ./Application/steering.o ./Application/steering.su ./Application/tests.cyclo ./Application/tests.d ./Application/tests.o ./Application/tests.su ./Application/ultrasound.cyclo ./Application/ultrasound.d ./Application/ultrasound.o ./Application/ultrasound.su ./Application/wheels.cyclo ./Application/wheels.d ./Application/wheels.o ./Application/wheels.su

.PHONY: clean-Application

