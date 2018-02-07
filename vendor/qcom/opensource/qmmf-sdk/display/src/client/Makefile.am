source_files = qmmf_display.cc
source_files += qmmf_display_client.cc

# Build qmmf display client library
# libqmmf_display_client.so
lib_LTLIBRARIES = libqmmf_display_client.la

libqmmf_display_client_la_CPPFLAGS = -I$(top_srcdir)/include
libqmmf_display_client_la_CPPFLAGS += -I$(top_srcdir)/
libqmmf_display_client_la_CPPFLAGS += -I$(MMCORE_CFLAGS)
libqmmf_display_client_la_LDFLAGS  = -llog -lbinder -lutils -lcutils

libqmmf_display_client_la_SOURCES = $(source_files)

Export qmmf-sdk headers:
sdkincdir=$(includedir)/qmmf-sdk
sdkinc_HEADERS = $(top_srcdir)/include/qmmf-sdk/qmmf_recorder.h
sdkinc_HEADERS += $(top_srcdir)/include/qmmf-sdk/qmmf_recorder_params.h
sdkinc_HEADERS += $(top_srcdir)/include/qmmf-sdk/qmmf_display.h
sdkinc_HEADERS += $(top_srcdir)/include/qmmf-sdk/qmmf_display_params.h
sdkinc_HEADERS += $(top_srcdir)/include/qmmf-sdk/qmmf_system.h
sdkinc_HEADERS += $(top_srcdir)/include/qmmf-sdk/qmmf_system_params.h
sdkinc_HEADERS += $(top_srcdir)/include/qmmf-sdk/qmmf_player.h
sdkinc_HEADERS += $(top_srcdir)/include/qmmf-sdk/qmmf_player_params.h
sdkinc_HEADERS += $(top_srcdir)/include/qmmf-sdk/qmmf_device.h
sdkinc_HEADERS += $(top_srcdir)/include/qmmf-sdk/qmmf_codec.h

