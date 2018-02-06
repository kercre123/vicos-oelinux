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

pd_mapper_bin_c_sources = ./pd-mapper.cpp \
                          ../pd-mapper-idl/service_registry_locator_v01.c
pd_mapper_bin_h_sources = ./pd-mapper.h \
                          ../pd-mapper-idl/service_registry_locator_v01.h

requiredlibs = -lm \
               -lpthread \
               -lcutils \
               -lutils \
               $(LIBJSON_LIBS)

bin_PROGRAMS = pdmappersvc

pdmappersvc_CC = @CC@
pkgconfigdir = $(libdir)/pkgconfig
#PD mapper binary
pdmappersvc_SOURCES = $(pd_mapper_bin_c_sources) \
                      $(pd_mapper_bin_h_sources)
pdmappersvc_LDADD = $(requiredlibs) \
                    $(QMIF_LIBS) \
                    $(QMI_LIBS)
