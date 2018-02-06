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
AM_CPPFLAGS += -DLIBPDMAPPER_SUPPORTED

pd_api_test_c_sources = ./pd-api-test.c

requiredlibs = -lm \
               -lpthread \
               -lcutils \
               -lutils

bin_PROGRAMS = pdapitest

pdapitest_CC = @CC@
pkgconfigdir = $(libdir)/pkgconfig
pdapitest_SOURCES = $(pd_api_test_c_sources)

pdapitest_LDADD = $(requiredlibs) \
                    $(QMIF_LIBS) \
                    $(QMI_LIBS) \
                    ../pd-mapper/libpdmapper/libpdmapper.la \
                    ../pd-notifier/libpdnotifier/libpdnotifier.la
