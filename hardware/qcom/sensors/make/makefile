################################################################################
# Automatically-generated file. Do not edit!
################################################################################


RM := rm -rf
PROJ_HOME := \"$(shell pwd)\"

INCLUDES = -I./ -I../sensord/inc/ -I../sensord/bsx/inc -I../hal
PREDEFINE = -DPLTF_LINUX_ENABLED -DSENSORD_STOR=$(PROJ_HOME)

CFLAGS := -Wall -g -pthread $(INCLUDES) $(PREDEFINE)

LIBS := -lrt

-include sensord/subdir.mk
-include hal/subdir.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
endif

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: HAL_BSX4

# Tool invocations
HAL_BSX4: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: Cross G++ Linker'
	g++  -o "HAL_BSX4" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(C++_DEPS)$(OBJS)$(C_DEPS)$(CC_DEPS)$(CPP_DEPS)$(EXECUTABLES)$(CXX_DEPS)$(C_UPPER_DEPS) HAL_BSX4
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

