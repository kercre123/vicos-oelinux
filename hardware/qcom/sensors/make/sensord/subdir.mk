################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../sensord/bstsimple_list.cpp \
../sensord/sensord.cpp \
../sensord/sensord_algo.cpp \
../sensord/sensord_cfg.cpp \
../sensord/sensord_hwcntl.cpp \
../sensord/sensord_hwcntl_implement.cpp \

C_SRCS += \
../sensord/axis_remap.c \
../sensord/sensord_pltf.c \
../sensord/util_misc.c 

OBJS += \
./sensord/axis_remap.o \
./sensord/bstsimple_list.o \
./sensord/sensord.o \
./sensord/sensord_algo.o \
./sensord/sensord_cfg.o \
./sensord/sensord_hwcntl.o \
./sensord/sensord_hwcntl_implement.o \
./sensord/sensord_pltf.o \
./sensord/util_misc.o 

C_DEPS += \
./sensord/axis_remap.d \
./sensord/sensord_pltf.d \
./sensord/util_misc.d 

CPP_DEPS += \
./sensord/bstsimple_list.d \
./sensord/sensord.d \
./sensord/sensord_algo.d \
./sensord/sensord_cfg.d \
./sensord/sensord_hwcntl.d \
./sensord/sensord_hwcntl_implement.d \


# Each subdirectory must supply rules for building sources it contributes
sensord/%.o: ../sensord/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc $(CFLAGS) -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

sensord/%.o: ../sensord/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ $(CFLAGS) -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


