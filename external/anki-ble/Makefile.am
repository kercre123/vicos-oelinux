AM_CPPFLAGS = 	-O2 \
		-D_GNU_SOURCE \
		-DUSE_GLIB \
		@GLIB_CFLAGS@ \
		-std=gnu++14 \
		-I${WORKSPACE}/vendor/qcom/opensource/bluetooth/hal/include/ \
		-I${WORKSPACE}/vendor/qcom/opensource/bluetooth/vhal/include \
		-I${WORKSPACE}/system/core/include

AM_CFLAGS = 	-Wall \
		-Wundef \
		-Wstrict-prototypes \
		-Wno-trigraphs \
		-DLINUX \
		-DUSE_GLIB \
		@GLIB_CFLAGS@ \
		-std=c11 \
		-I${WORKSPACE}/vendor/qcom/opensource/bluetooth/hal/include/ \
		-I${WORKSPACE}/vendor/qcom/opensource/bluetooth/vhal/include \
		-I${WORKSPACE}/system/core/include

ACLOCAL_AMFLAGS = -I m4

AM_LDFLAGS := $(LDFLAGS) -lpthread -lhardware @GLIB_LIBS@

bin_PROGRAMS = ankibluetoothd
ankibluetoothd_CC = @CC@
ankibluetoothd_SOURCES = 	main.cpp \
				btstack.cpp \
				btutils.cpp \
				peripheral.cpp \
				device_info.cpp \
				exec_command.cpp \
				fileutils.cpp \
				fork_and_exec.cpp \
				taskExecutor.cpp \
				log.cpp \
				ssh.cpp \
				wifi.cpp \
				static_ev.c \
				connmanbus.c

EXTRA_ankibluetoothd_DEPENDENCIES = connmanbus.c connmanbus.h

wifi.$(OBJEXT): connmanbus.h

pkgconfigdir = $(libdir)/pkgconfig

connmanbus.c connmanbus.h: net.connman.xml
	gdbus-codegen 	--generate-c-code connmanbus \
			--c-namespace ConnManBus \
			--interface-prefix=net.connman \
			$(srcdir)/net.connman.xml

