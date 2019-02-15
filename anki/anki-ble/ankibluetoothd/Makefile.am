COMMON_DIR=$(srcdir)/../common

AM_CPPFLAGS = 	-O2 \
		-D_GNU_SOURCE \
		-std=gnu++14 \
		-I$(COMMON_DIR) \
		-I${WORKSPACE}/vendor/qcom/opensource/bluetooth/hal/include/ \
		-I${WORKSPACE}/vendor/qcom/opensource/bluetooth/vhal/include \
		-I${WORKSPACE}/system/core/include

AM_CFLAGS = 	-Wall \
		-Wundef \
		-Wstrict-prototypes \
		-Wno-trigraphs \
		-DLINUX \
		-std=c11 \
		-I$(COMMON_DIR) \
		-I${WORKSPACE}/vendor/qcom/opensource/bluetooth/hal/include/ \
		-I${WORKSPACE}/vendor/qcom/opensource/bluetooth/vhal/include \
		-I${WORKSPACE}/system/core/include

ACLOCAL_AMFLAGS = -I m4

AM_LDFLAGS := $(LDFLAGS) -lpthread -lhardware -lcutils

bin_PROGRAMS = ankibluetoothd
ankibluetoothd_CC = @CC@
ankibluetoothd_SOURCES = 	main.cpp \
				btstack.cpp \
				btutils.cpp \
				agent.cpp \
				device_info.cpp \
				$(COMMON_DIR)/fileutils.cpp \
				$(COMMON_DIR)/taskExecutor.cpp \
				$(COMMON_DIR)/log.cpp \
				$(COMMON_DIR)/static_ev.c \
				$(COMMON_DIR)/ipc-server.cpp \
				$(COMMON_DIR)/ipc.cpp \
				$(COMMON_DIR)/memutils.cpp \
				$(COMMON_DIR)/stringutils.cpp \
				$(COMMON_DIR)/strlcpy.c \
				$(COMMON_DIR)/properties_compat.c

pkgconfigdir = $(libdir)/pkgconfig
