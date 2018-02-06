# Makefile.am - Automake script for gps loc-vzw
#

AM_CFLAGS = \
     $(LOCPLA_CFLAGS) \
     $(LOC_CFLAGS) \
     $(QMIF_CFLAGS) \
     $(GPSUTILS_CFLAGS)

ACLOCAL_AMFLAGS = -I m4

libloc_ext_la_SOURCES = loc_ext.cpp

if USE_GLIB
libloc_ext_la_CFLAGS = -DUSE_GLIB $(AM_CFLAGS) @GLIB_CFLAGS@
libloc_ext_la_LDFLAGS = -lstdc++ -Wl,-z,defs -lpthread @GLIB_LIBS@ -shared -version-info 1:0:0
libloc_ext_la_CPPFLAGS = -DUSE_GLIB $(AM_CFLAGS) $(AM_CPPFLAGS) @GLIB_CFLAGS@
else
libloc_ext_la_CFLAGS = $(AM_CFLAGS)
libloc_ext_la_LDFLAGS = -Wl,-z,defs -lpthread -shared -version-info 1:0:0
libloc_ext_la_CPPFLAGS = $(AM_CFLAGS) $(AM_CPPFLAGS)
endif

library_includedir = $(includedir)/loc-vzw/vzwGpsLocationProvider/loc_ext

library_include_HEADERS = loc_vzw.h

libloc_ext_la_LIBADD = -lcutils $(LOCPLA_LIBS) $(GPSUTILS_LIBS) $(LOC_LIBS) $(QMIF_LIBS)

#Create and Install libraries
lib_LTLIBRARIES = libloc_ext.la

pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = loc-vzw.pc
EXTRA_DIST = $(pkgconfig_DATA)
