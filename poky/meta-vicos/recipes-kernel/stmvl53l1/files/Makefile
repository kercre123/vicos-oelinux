ccflags-y	+= -I$(src)/inc -I$(src)/ipp -I$(src)

#ccflags-y	+= -DVL53L1_LOG_ENABLE

obj-m	+= stmvl53l1.o
stmvl53l1-objs		:= stmvl53l1_module.o stmvl53l1_module-i2c.o
stmvl53l1-objs		+= stmvl53l1_i2c.o stmvl53l1_ipp_nl.o stmvl53l1_log.o
stmvl53l1-objs		+= src/vl53l1_api.o src/vl53l1_api_core.o
stmvl53l1-objs		+= src/vl53l1_api_strings.o src/vl53l1_error_strings.o
stmvl53l1-objs		+= src/vl53l1_core.o src/vl53l1_register_funcs.o
stmvl53l1-objs		+= src/vl53l1_api_preset_modes.o
stmvl53l1-objs		+= src/vl53l1_api_calibration.o
stmvl53l1-objs		+= src/vl53l1_fpga_core.o src/vl53l1_silicon_core.o
stmvl53l1-objs		+= src/vl53l1_zone_presets.o src/vl53l1_nvm.o
stmvl53l1-objs		+= src/vl53l1_api_debug.o src/vl53l1_core_support.o
stmvl53l1-objs		+= src/vl53l1_wait.o ipp/ipp_linux.o
stmvl53l1-objs		+= src/vl53l1_nvm_debug.o

SRC := $(shell pwd)

default:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules
modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules_install
clean:
	CONFIG_STMVL53L1=m $(MAKE) -C $(KERNEL_SRC) M=$(SRC) clean
	rm -f *.o *~ core .depend .*.cmd *.ko *.mod.c
	rm -f src/*.o src/*~ core src/.depend src/.*.cmd src/*.ko src/*.mod.c
	rm -f Module.markers Module.symvers modules.order
	rm -rf .tmp_versions Modules.symvers
