AM_CFLAGS = \
    -DDEBUG \
    -DLINUX_AUTO_MAKE

ALOCAL_AMFLAGS = -I m4

power_manager_daemon_c_sources = \
     power_manager_daemon.cpp

power_manager_daemon_SOURCES = $(power_manager_daemon_c_sources)

if USE_GLIB
power_manager_daemon_CFLAGS = -DUSE_GLIB $(AM_CFLAGS) @GLIB_CFLAGS@
power_manager_daemon_LDFLAGS = -lstdc++ -g -lpthread @GLIB_LIBS@ -shared -version-info 1:0:0
power_manager_daemon_CPPFLAGS = -DUSE_GLIB $(AM_CFLAGS) @GLIB_CFLAGS@
else
power_manager_daemon_CFLAGS = $(AM_CFLAGS)
power_manager_daemon_LDFLAGS = -lpthread -shared -version-info 1:0:0
power_manager_daemon_CPPFLAGS = $(AM_CFLAGS)
endif

power_manager_daemon_LDADD = -lcanwrapper

bin_PROGRAMS = power_manager_daemon

library_includedir = $(pkgincludedir)

pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = power_manager_daemon.pc
sysconf_DATA = $(WORKSPACE)/vnw/powermanagerdaemon/src/power_state.conf

EXTRA_DIST = $(pkgconfig_DATA)
