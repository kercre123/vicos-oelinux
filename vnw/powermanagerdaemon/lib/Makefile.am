AM_CFLAGS = \
     -I./ \
     -D__func__=__PRETTY_FUNCTION__ \
     -DLINUX_AUTO_MAKE

ACLOCAL_AMFLAGS = -I m4

libpowermanager_la_SOURCES = \
    power_state.cpp

if USE_GLIB
libpowermanager_la_CFLAGS = -DUSE_GLIB $(AM_CFLAGS) @GLIB_CFLAGS@
libpowermanager_la_LDFLAGS = -lrt -lstdc++ -lpthread @GLIB_LIBS@ -shared -version-info 1:0:0
libpowermanager_la_CPPFLAGS = -DUSE_GLIB $(AM_CFLAGS) $(AM_CPPFLAGS) @GLIB_CFLAGS@
else
libpowermanager_la_CFLAGS = $(AM_CFLAGS)
libpowermanager_la_LDFLAGS = -lrt -lpthread -shared -version-info 1:0:0
libpowermanager_la_CPPFLAGS = $(AM_CFLAGS) $(AM_CPPFLAGS)
endif

libpowermanager_la_LIBADD = -lcutils

library_includedir = $(pkgincludedir)

#Create and Install libraries
lib_LTLIBRARIES = libpowermanager.la

pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = powermanager.pc
EXTRA_DIST = $(pkgconfig_DATA)

