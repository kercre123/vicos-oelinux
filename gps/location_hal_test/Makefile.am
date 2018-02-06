# Makefile.am - Automake script for location_hal_test test app
#

ACLOCAL_AMFLAGS = -I m4

AM_CFLAGS = \
    -I$(WORKSPACE)/hardware/qcom/gps/utils \
    -I$(WORKSPACE)/hardware/qcom/gps/core \
    -I$(WORKSPACE)/gps/garden-app/GARDEn/ \
     $(GPSUTILS_CFLAGS) \
     $(IZATCORE_CFLAGS) \
     $(LOCFLP_CFLAGS) \
     $(LOCGEOFENCE_CFLAGS) \
     $(LOCPLA_CFLAGS)

requiredlibs = ${LOCFLP_LIBS} ${LOCGEOFENCE_LIBS} $(LOCHAL_LIBS) $(LOCPLA_LIBS) $(GPSUTILS_LIBS)

location_hal_sources = location_hal_test.cpp

location_hal_tests_SOURCES = $(location_hal_sources)

if USE_GLIB
location_hal_tests_CFLAGS = -DUSE_GLIB $(AM_CFLAGS) @GLIB_CFLAGS@
location_hal_tests_LDFLAGS = -lstdc++ -Wl,-z,defs -lpthread @GLIB_LIBS@ -shared -version-info 1:0:0
location_hal_tests_CPPFLAGS = -DUSE_GLIB $(AM_CFLAGS) $(AM_CPPFLAGS) @GLIB_CFLAGS@
else
location_hal_tests_CFLAGS = $(AM_CFLAGS)
location_hal_tests_LDFLAGS = -Wl,-z,defs -lpthread -shared -version-info 1:0:0
location_hal_tests_CPPFLAGS = $(AM_CFLAGS) $(AM_CPPFLAGS)
endif

location_hal_tests_LDADD = -lcutils -ldl $(requiredlibs)

bin_PROGRAMS = location_hal_tests

pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = location-hal-tests.pc
EXTRA_DIST = $(pkgconfig_DATA)
