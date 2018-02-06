AM_CFLAGS = -Wall \
        -Wundef \
        -Wstrict-prototypes \
        -g -O0  \
        -fno-inline \
        -fno-short-enums \
        -fpic \
        $(QMI_CFLAGS) \
        $(QMIFRAMEWORK_CFLAGS) \
        -I .

ACLOCAL_AMFLAGS = -I m4
AM_CPPFLAGS = -D__packed__= -DLE_BUILD
AM_CPPFLAGS += -DLIBPDNOTIFIER_ENABLED
pd_notifier_lib_c_sources = ./libpdnotifier.c \
                            ../pd-notifier-idl/service_registry_notifier_v01.c
pd_notifier_lib_h_sources = ./libpdnotifier.h \
                            ../pd-notifier-idl/service_registry_notifier_v01.h
requiredlibs = -lm \
               -lpthread \
               -lcutils \
               -lutils
lib_LTLIBRARIES = libpdnotifier.la
#PD mapper library
libpdnotifier_la_SOURCES = $(pd_notifier_lib_c_sources) \
                           $(pd_notifier_lib_h_sources)
libpdnotifier_la_LIBADD = $(requiredlibs) \
                          $(QMIF_LIBS) \
                          $(QMI_LIBS)
pkgconfigdir = $(libdir)/pkgconfig
