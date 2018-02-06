AM_CFLAGS = -Wall \
        -Wundef \
        -Wstrict-prototypes \
        -g -O0  \
        -fno-inline \
        -fno-short-enums \
        -fpic \
        $(QMI_CFLAGS) \
        $(QMIFRAMEWORK_CFLAGS) \
        $(LIBJSON_CFLAGS) \
        -I .

ACLOCAL_AMFLAGS = -I m4
AM_CPPFLAGS = -D__packed__= -DLE_BUILD
AM_CPPFLAGS += -DLIBPDMAPPER_SUPPORTED

pd_mapper_lib_c_sources = libpdmapper.c \
                          ../pd-mapper-idl/service_registry_locator_v01.c
pd_mapper_lib_h_sources = libpdmapper.h \
                          ../pd-mapper-idl/service_registry_locator_v01.h

requiredlibs = -lm \
               -lpthread \
               -lcutils \
               -lutils \
               $(LIBJSON_LIBS)

lib_LTLIBRARIES = libpdmapper.la

#PD mapper library
libpdmapper_la_SOURCES = $(pd_mapper_lib_c_sources) \
                         $(pd_mapper_lib_h_sources)
libpdmapper_la_LIBADD = $(requiredlibs) \
                        $(QMIF_LIBS) \
                        $(QMI_LIBS)
include_HEADERS = libpdmapper.h
pkgconfigdir = $(libdir)/pkgconfig
