COMMON_DIR=$(srcdir)/../common

AM_CPPFLAGS = 	-O2 \
		-D_GNU_SOURCE \
		-DUSE_GLIB \
		@GLIB_CFLAGS@ \
		-std=gnu++14 \
		-I$(COMMON_DIR)

AM_CFLAGS = 	-Wall \
		-Wundef \
		-Wstrict-prototypes \
		-Wno-trigraphs \
		-DLINUX \
		-DUSE_GLIB \
		@GLIB_CFLAGS@ \
		-std=c11 \
		-I$(COMMON_DIR)

ACLOCAL_AMFLAGS = -I m4

AM_LDFLAGS := $(LDFLAGS) -lpthread -lcutils @GLIB_LIBS@

bin_PROGRAMS = vicdevsetup

vicdevsetup_CC = @CC@
vicdevsetup_SOURCES =	vicdevsetup.cpp \
			connmanbus.c \
			wifi.cpp \
			ssh.cpp \
			exec_command.cpp \
			fork_and_exec.cpp \
			$(COMMON_DIR)/log.cpp \
			$(COMMON_DIR)/static_ev.c \
			$(COMMON_DIR)/ipc-client.cpp \
			$(COMMON_DIR)/ipc.cpp \
			$(COMMON_DIR)/memutils.cpp \
			$(COMMON_DIR)/taskExecutor.cpp \
			$(COMMON_DIR)/fileutils.cpp \
			$(COMMON_DIR)/stringutils.cpp \
			$(COMMON_DIR)/strlcpy.c \
			$(COMMON_DIR)/properties_compat.c \
			$(COMMON_DIR)/robot_name.cpp

EXTRA_vicdevsetup_DEPENDENCIES = connmanbus.c connmanbus.h

wifi.$(OBJEXT): connmanbus.h

pkgconfigdir = $(libdir)/pkgconfig

connmanbus.c connmanbus.h: net.connman.xml
	gdbus-codegen 	--generate-c-code connmanbus \
			--c-namespace ConnManBus \
			--interface-prefix=net.connman \
			$(srcdir)/net.connman.xml
