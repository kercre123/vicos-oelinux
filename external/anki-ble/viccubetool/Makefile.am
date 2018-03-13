COMMON_DIR=$(srcdir)/../common

AM_CPPFLAGS = 	-O2 \
		-D_GNU_SOURCE \
		-std=gnu++14 \
		-I$(COMMON_DIR)

AM_CFLAGS = 	-Wall \
		-Wundef \
		-Wstrict-prototypes \
		-Wno-trigraphs \
		-DLINUX \
		-std=c11 \
		-I$(COMMON_DIR)

ACLOCAL_AMFLAGS = -I m4

AM_LDFLAGS := $(LDFLAGS) -lpthread

bin_PROGRAMS = viccubetool
viccubetool_CC = @CC@
viccubetool_SOURCES = 	main.cpp \
			viccubetool.cpp \
			$(COMMON_DIR)/crc32.cpp \
			$(COMMON_DIR)/fileutils.cpp \
			$(COMMON_DIR)/stringutils.cpp \
			$(COMMON_DIR)/taskExecutor.cpp \
			$(COMMON_DIR)/log.cpp \
			$(COMMON_DIR)/static_ev.c \
			$(COMMON_DIR)/ipc-client.cpp \
			$(COMMON_DIR)/ipc.cpp \
			$(COMMON_DIR)/memutils.cpp \
			$(COMMON_DIR)/strlcpy.c \
			$(COMMON_DIR)/byte_vector.cpp

pkgconfigdir = $(libdir)/pkgconfig
