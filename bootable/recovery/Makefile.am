ACLOCAL_AMFLAGS := -I m4

SUBDIRS := minzip mtdutils edify applypatch updater minadbd

AM_CFLAGS := -Wundef \
             -Wstrict-prototypes \
             -Wno-trigraphs \
             -g -O0 \
             -fno-inline \
             -fno-short-enums \
             -fpic \
             -D__reboot=reboot \
             -Dstrlcat=strncat \
             $(LIBSELINUX_CFLAGS) \
             -D_GNU_SOURCE \
             -D_LARGEFILE64_SOURCE

AM_CPPFLAGS := \
              -I. \
              -I$(WORKSPACE)/system/core/include \
              -I$(WORKSPACE)/system/core/fs_mgr/include \
              -I$(WORKSPACE)/system/core/adb \
              -I$(WORKSPACE)/system/extras/ext4_utils \
              $(LIBSELINUX_CFLAGS) \
              -std=c++11 \
              -Dstrlcat=strncat

#AM_LDFLAGS := -shared -version-info 1:0:0

fuse_sideload_sources := fuse_sideload.c

library_includedir := $(pkgincludedir)

lib_LTLIBRARIES := libfusesideload.la
libfusesideload_la_SOURCES := $(fuse_sideload_sources)

if USE_GLIB
libfusesideload_la_CFLAGS := $(AM_CFLAGS) -DUSE_GLIB @GLIB_CFLAGS@
libfusesideload_la_CPPFLAGS := $(AM_CPPFLAGS) -DUSE_GLIB @GLIB_CFLAGS@
libfusesideload_la_LIBADD := -lm -lpthread @GLIB_LIBS@
libfusesideload_la_LDFLAGS := -all-static
else
libfusesideload_la_CFLAGS := $(AM_CFLAGS)
libfusesideload_la_CPPFLAGS := $(AM_CPPFLAGS)
libfusesideload_la_LIBADD := -lm -lpthread
libfusesideload_la_LDFLAGS := -all-static
endif


cpp_sources := adb_install.cpp \
    asn1_decoder.cpp \
    bootloader.cpp \
    device.cpp \
    default_device.cpp \
    fuse_sdcard_provider.c \
    install.cpp \
    recovery.cpp \
    roots.cpp \
    screen_ui.cpp \
    ui.cpp \
    verifier.cpp

requiredlibs := -lm \
               -lpthread \
               -lz \
               -lcutils \
               -lmincrypt \
               -lselinux \
               -lext4_utils \
               ./mtdutils/libmtdutils.la \
               ./minadbd/libminadbd.la \
               ./libfusesideload.la \
               ./edify/libedify.la \
               ./minzip/libminzip.la \
               ./applypatch/libapplypatch.la


RECOVERY_API_VERSION := 3

recovery_SOURCES := $(cpp_sources)
recovery_CFLAGS := $(AM_CFLAGS) -DRECOVERY_API_VERSION=$(RECOVERY_API_VERSION)
recovery_CPPFLAGS := $(AM_CPPFLAGS) -DRECOVERY_API_VERSION=$(RECOVERY_API_VERSION)
recovery_LDADD := $(requiredlibs)


verifier_test_libs := -lmincrypt ./minzip/libminzip.la -lm -lpthread -lcutils
verifier_test_CFLAGS := $(AM_CFLAGS)

verifier_test_CPPFLAGS := $(AM_CPPFLAGS)
verifier_test_LDADD := $(verifier_test_libs)

verifier_test_SOURCES := verifier_test.cpp asn1_decoder.cpp verifier.cpp ui.cpp

bin_PROGRAMS := verifier_test recovery

pkgconfigdir := $(libdir)/pkgconfig
pkgconfig_DATA := recovery.pc
EXTRA_DIST := $(pkgconfig_DATA)
