AM_CPPFLAGS = 	-O2 \
		-D_GNU_SOURCE \
		-std=gnu++14 \
		-I${WORKSPACE}/vendor/qcom/opensource/bluetooth/hal/include/ \
		-I${WORKSPACE}/vendor/qcom/opensource/bluetooth/vhal/include \
		-I${WORKSPACE}/system/core/include

AM_CFLAGS = 	-Wall \
		-Wundef \
		-Wstrict-prototypes \
		-Wno-trigraphs \
		-DLINUX \
		-std=c11 \
		-I${WORKSPACE}/vendor/qcom/opensource/bluetooth/hal/include/ \
		-I${WORKSPACE}/vendor/qcom/opensource/bluetooth/vhal/include \
		-I${WORKSPACE}/system/core/include

ACLOCAL_AMFLAGS = -I m4

AM_LDFLAGS := $(LDFLAFGS) -lpthread -lhardware

bin_PROGRAMS = ankibluetoothd
ankibluetoothd_CC = @CC@
ankibluetoothd_SOURCES = 	main.cpp \
				btstack.cpp \
				btutils.cpp \
				peripheral.cpp \
				device_info.cpp \
				exec_command.cpp \
				fork_and_exec.cpp \
				taskExecutor.cpp \
				log.cpp

pkgconfigdir = $(libdir)/pkgconfig

