# Makefile.am - Automake script for gps location-service-glue
#

AM_CFLAGS = \
     -I./jni/inc \
     -I./common/inc/internal \
     $(LOCPLA_CFLAGS) \
     $(LOC_CFLAGS) \
     $(GPSUTILS_CFLAGS) \
     $(LOCFLP_CFLAGS) \
     $(LOCGF_CFLAGS) \
     $(DATAITEMS_CFLAGS) \
     $(ULP2_CFLAGS) \
     $(LOCSERVICE_CFLAGS) \
     $(LOCBASEUTIL_CFLAGS) \
     $(LOCNETIFACE_CFLAGS) \
     $(QMIF_CFLAGS) \
     $(QMI_CFLAGS) \
     $(DATA_CFLAGS) \
     $(DSUTILS_CFLAGS) \
     $(DATAITEMS_CFLAGS) \
     -D__func__=__PRETTY_FUNCTION__ \
     -DON_TARGET_TEST

ACLOCAL_AMFLAGS = -I m4

AM_CPPFLAGS = -std=c++11

liblocationservice_glue_la_SOURCES = \
        common/src/RawCommandInjector.cpp \
        common/src/OSFrameworkCleaner.cpp \
        common/src/FusedLocationProviderGlue.cpp \
        common/src/NetworkLocationProviderGlue.cpp \
        common/src/LocationProviderGlue.cpp \
        common/src/OSNPGlue.cpp \
        common/src/Subscription.cpp \
        common/src/FrameworkActionRequest.cpp \
        common/src/OSFramework.cpp


if USE_GLIB
liblocationservice_glue_la_CFLAGS = -DUSE_GLIB $(AM_CFLAGS) @GLIB_CFLAGS@
liblocationservice_glue_la_LDFLAGS = -lstdc++ -Wl,-z,defs -lpthread @GLIB_LIBS@ -shared -version-info 1:0:0
liblocationservice_glue_la_CPPFLAGS = -DUSE_GLIB $(AM_CFLAGS) $(AM_CPPFLAGS) @GLIB_CFLAGS@
else
liblocationservice_glue_la_CFLAGS = $(AM_CFLAGS)
liblocationservice_glue_la_LDFLAGS = -Wl,-z,defs -lpthread -shared -version-info 1:0:0
liblocationservice_glue_la_CPPFLAGS = $(AM_CFLAGS) $(AM_CPPFLAGS)
endif

liblocationservice_glue_la_LIBADD = -lstdc++ $(LOCPLA_LIBS) $(LOC_LIBS) $(LOCFLP_LIBS) $(LOCFGF_LIBS) \
$(LBSCORE_LIBS) $(ULP2_LIBS) $(LOCSERVICE_LIBS) $(GPSUTILS_LIBS) $(LOCNETIFACE_LIBS) $(DATAITEMS_LIBS)

library_include_HEADERS = \
         common/inc/internal/FusedLocationProviderGlue.h \
         common/inc/internal/LocationProviderGlue.h \
         common/inc/internal/NetworkLocationProviderGlue.h \
         common/inc/internal/OSFrameworkCleaner.h \
         common/inc/internal/OSFramework.h \
         common/inc/internal/OSNPGlue.h \
         common/inc/internal/RawCommandInjector.h \
         common/inc/internal/Subscription.h \
         common/inc/internal/FrameworkActionRequest.h

library_includedir = $(pkgincludedir)

#Create and Install libraries
lib_LTLIBRARIES = liblocationservice_glue.la

pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = loc-glue.pc
EXTRA_DIST = $(pkgconfig_DATA)
