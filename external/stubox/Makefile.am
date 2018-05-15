AM_CPPFLAGS = 	-O2 \
		-D_GNU_SOURCE \
		-std=gnu++14

AM_CFLAGS = 	-Wall \
		-Wundef \
		-Wstrict-prototypes \
		-Wno-trigraphs \
		-DLINUX \
		-std=c11

ACLOCAL_AMFLAGS = -I m4

AM_LDFLAGS := $(LDFLAGS) -lpthread -lcutils

bin_PROGRAMS = stubox
stubox_CC = @CC@
stubox_SOURCES = main.cpp

pkgconfigdir = $(libdir)/pkgconfig
