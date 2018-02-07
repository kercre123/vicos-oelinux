# Build shared lib.
lib_LTLIBRARIES := libsync.la

libsync_la_SOURCES := sync.c
libsync_la_CFLAGS  := -I$(top_srcdir)/include
libsync_la_CFLAGS  += -I$(top_srcdir)/libsync/include
libsync_la_LDFLAGS := -llog

# Build sync_test
bin_PROGRAMS := sync_test

sync_test_SOURCES := sync.c sync_test.c
sync_test_CFLAGS  := -I$(top_srcdir)/include
sync_test_CFLAGS  += -I$(top_srcdir)/libsync/include
sync_test_LDADD   := libsync.la
sync_test_LDADD   += -lcutils -llog -lpthread

# Export headers
sync_includedir=$(includedir)/sync
sync_include_HEADERS := $(top_srcdir)/libsync/include/sync/sync.h

pkgconfigdir := $(libdir)/pkgconfig
pkgconfig_DATA := libsync.pc
EXTRA_DIST := $(pkgconfig_DATA)

