################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

MCSDK_ANY = ../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Src
MCSDK_G4  = ../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/G4xx/Src

# Add inputs and outputs from these tool invocations to the build variables
C_SRCS += \
$(MCSDK_ANY)/bus_voltage_sensor.c \
$(MCSDK_ANY)/circle_limitation.c \
$(MCSDK_ANY)/digital_output.c \
$(MCSDK_ANY)/mcpa.c \
$(MCSDK_ANY)/ntc_temperature_sensor.c \
$(MCSDK_ANY)/open_loop.c \
$(MCSDK_ANY)/pid_regulator.c \
$(MCSDK_ANY)/pqd_motor_power_measurement.c \
$(MCSDK_G4)/r3_2_g4xx_pwm_curr_fdbk.c \
$(MCSDK_ANY)/r_divider_bus_voltage_sensor.c \
$(MCSDK_ANY)/ramp_ext_mngr.c \
$(MCSDK_ANY)/revup_ctrl.c \
$(MCSDK_ANY)/speed_pos_fdbk.c \
$(MCSDK_ANY)/sto_pll_speed_pos_fdbk.c \
$(MCSDK_ANY)/virtual_speed_sensor.c

OBJS += \
./Middlewares/MotorControl/bus_voltage_sensor.o \
./Middlewares/MotorControl/circle_limitation.o \
./Middlewares/MotorControl/digital_output.o \
./Middlewares/MotorControl/mcpa.o \
./Middlewares/MotorControl/ntc_temperature_sensor.o \
./Middlewares/MotorControl/open_loop.o \
./Middlewares/MotorControl/pid_regulator.o \
./Middlewares/MotorControl/pqd_motor_power_measurement.o \
./Middlewares/MotorControl/r3_2_g4xx_pwm_curr_fdbk.o \
./Middlewares/MotorControl/r_divider_bus_voltage_sensor.o \
./Middlewares/MotorControl/ramp_ext_mngr.o \
./Middlewares/MotorControl/revup_ctrl.o \
./Middlewares/MotorControl/speed_pos_fdbk.o \
./Middlewares/MotorControl/sto_pll_speed_pos_fdbk.o \
./Middlewares/MotorControl/virtual_speed_sensor.o

C_DEPS += \
./Middlewares/MotorControl/bus_voltage_sensor.d \
./Middlewares/MotorControl/circle_limitation.d \
./Middlewares/MotorControl/digital_output.d \
./Middlewares/MotorControl/mcpa.d \
./Middlewares/MotorControl/ntc_temperature_sensor.d \
./Middlewares/MotorControl/open_loop.d \
./Middlewares/MotorControl/pid_regulator.d \
./Middlewares/MotorControl/pqd_motor_power_measurement.d \
./Middlewares/MotorControl/r3_2_g4xx_pwm_curr_fdbk.d \
./Middlewares/MotorControl/r_divider_bus_voltage_sensor.d \
./Middlewares/MotorControl/ramp_ext_mngr.d \
./Middlewares/MotorControl/revup_ctrl.d \
./Middlewares/MotorControl/speed_pos_fdbk.d \
./Middlewares/MotorControl/sto_pll_speed_pos_fdbk.d \
./Middlewares/MotorControl/virtual_speed_sensor.d


CFLAGS = -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../../Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/G4xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../../Drivers/CMSIS/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb

# Each subdirectory must supply rules for building sources it contributes
Middlewares/MotorControl/%.o: $(MCSDK_ANY)/%.c Middlewares/MotorControl/subdir.mk
	arm-none-eabi-gcc "$<" $(CFLAGS) -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@"

Middlewares/MotorControl/%.o: $(MCSDK_G4)/%.c Middlewares/MotorControl/subdir.mk
	arm-none-eabi-gcc "$<" $(CFLAGS) -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@"

clean: clean-Middlewares-2f-MotorControl

clean-Middlewares-2f-MotorControl:
	-$(RM) ./Middlewares/MotorControl/bus_voltage_sensor.cyclo ./Middlewares/MotorControl/bus_voltage_sensor.d ./Middlewares/MotorControl/bus_voltage_sensor.o ./Middlewares/MotorControl/bus_voltage_sensor.su ./Middlewares/MotorControl/circle_limitation.cyclo ./Middlewares/MotorControl/circle_limitation.d ./Middlewares/MotorControl/circle_limitation.o ./Middlewares/MotorControl/circle_limitation.su ./Middlewares/MotorControl/digital_output.cyclo ./Middlewares/MotorControl/digital_output.d ./Middlewares/MotorControl/digital_output.o ./Middlewares/MotorControl/digital_output.su ./Middlewares/MotorControl/mcpa.cyclo ./Middlewares/MotorControl/mcpa.d ./Middlewares/MotorControl/mcpa.o ./Middlewares/MotorControl/mcpa.su ./Middlewares/MotorControl/ntc_temperature_sensor.cyclo ./Middlewares/MotorControl/ntc_temperature_sensor.d ./Middlewares/MotorControl/ntc_temperature_sensor.o ./Middlewares/MotorControl/ntc_temperature_sensor.su ./Middlewares/MotorControl/open_loop.cyclo ./Middlewares/MotorControl/open_loop.d ./Middlewares/MotorControl/open_loop.o ./Middlewares/MotorControl/open_loop.su ./Middlewares/MotorControl/pid_regulator.cyclo ./Middlewares/MotorControl/pid_regulator.d ./Middlewares/MotorControl/pid_regulator.o ./Middlewares/MotorControl/pid_regulator.su ./Middlewares/MotorControl/pqd_motor_power_measurement.cyclo ./Middlewares/MotorControl/pqd_motor_power_measurement.d ./Middlewares/MotorControl/pqd_motor_power_measurement.o ./Middlewares/MotorControl/pqd_motor_power_measurement.su ./Middlewares/MotorControl/r3_2_g4xx_pwm_curr_fdbk.cyclo ./Middlewares/MotorControl/r3_2_g4xx_pwm_curr_fdbk.d ./Middlewares/MotorControl/r3_2_g4xx_pwm_curr_fdbk.o ./Middlewares/MotorControl/r3_2_g4xx_pwm_curr_fdbk.su ./Middlewares/MotorControl/r_divider_bus_voltage_sensor.cyclo ./Middlewares/MotorControl/r_divider_bus_voltage_sensor.d ./Middlewares/MotorControl/r_divider_bus_voltage_sensor.o ./Middlewares/MotorControl/r_divider_bus_voltage_sensor.su ./Middlewares/MotorControl/ramp_ext_mngr.cyclo ./Middlewares/MotorControl/ramp_ext_mngr.d ./Middlewares/MotorControl/ramp_ext_mngr.o ./Middlewares/MotorControl/ramp_ext_mngr.su ./Middlewares/MotorControl/revup_ctrl.cyclo ./Middlewares/MotorControl/revup_ctrl.d ./Middlewares/MotorControl/revup_ctrl.o ./Middlewares/MotorControl/revup_ctrl.su ./Middlewares/MotorControl/speed_pos_fdbk.cyclo ./Middlewares/MotorControl/speed_pos_fdbk.d ./Middlewares/MotorControl/speed_pos_fdbk.o ./Middlewares/MotorControl/speed_pos_fdbk.su ./Middlewares/MotorControl/sto_pll_speed_pos_fdbk.cyclo ./Middlewares/MotorControl/sto_pll_speed_pos_fdbk.d ./Middlewares/MotorControl/sto_pll_speed_pos_fdbk.o ./Middlewares/MotorControl/sto_pll_speed_pos_fdbk.su ./Middlewares/MotorControl/virtual_speed_sensor.cyclo ./Middlewares/MotorControl/virtual_speed_sensor.d ./Middlewares/MotorControl/virtual_speed_sensor.o ./Middlewares/MotorControl/virtual_speed_sensor.su

.PHONY: clean-Middlewares-2f-MotorControl

