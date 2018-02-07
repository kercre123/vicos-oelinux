# Build libav_queue.so
lib_LTLIBRARIES = libav_queue.la

libav_queue_la_SOURCES = qmmf_queue.cc

libav_queue_la_CPPFLAGS = -I$(top_srcdir)
libav_queue_la_CPPFLAGS += -I$(top_srcdir)/include

Export qmmf-sdk headers:
sdkincdir=$(includedir)/qmmf-sdk
sdkinc_HEADERS = $(top_srcdir)/include/qmmf-sdk/qmmf_queue.h
