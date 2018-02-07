# Build liboverlay.so
lib_LTLIBRARIES = liboverlay.la

liboverlay_la_SOURCES = qmmf_overlay.cc

liboverlay_la_CPPFLAGS = -I$(top_srcdir)/include
liboverlay_la_CPPFLAGS += -I$(top_srcdir)/
liboverlay_la_CPPFLAGS += -DUSE_SKIA=0
liboverlay_la_CPPFLAGS += -DUSE_CAIRO=1
liboverlay_la_LDFLAGS  = -lutils -lcutils -lpthread -ldl -llog -lC2D2 -lcairo

# Codec Adaptor sample test app
bin_PROGRAMS = overlay_test

overlay_test_SOURCES  = test/qmmf_overlay_test.cc
overlay_test_CPPFLAGS = $(liboverlay_la_CPPFLAGS) -I$(top_srcdir)
overlay_test_LDADD    = liboverlay.la

root_etcdir   = "/etc"
root_etc_DATA = test/raw_image/overlay_test.rgba
