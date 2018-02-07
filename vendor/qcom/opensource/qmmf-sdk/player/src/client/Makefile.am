source_files = qmmf_player.cc
source_files += qmmf_player_client.cc

# Build qmmf player client library
# libqmmf_player_client.so
lib_LTLIBRARIES = libqmmf_player_client.la

libqmmf_player_client_la_CPPFLAGS = -I$(top_srcdir)/include
libqmmf_player_client_la_CPPFLAGS += -I$(top_srcdir)/
libqmmf_player_client_la_CPPFLAGS += -I$(MMCORE_CFLAGS)
libqmmf_player_client_la_LDFLAGS  = -llog -lbinder -lutils -lcutils

libqmmf_player_client_la_SOURCES = $(source_files)

Export qmmf-sdk headers:
sdkincdir=$(includedir)/qmmf-sdk
sdkinc_HEADERS = $(top_srcdir)/include/qmmf-sdk/qmmf_player.h
sdkinc_HEADERS += $(top_srcdir)/include/qmmf-sdk/qmmf_player_params.h
sdkinc_HEADERS += $(top_srcdir)/include/qmmf-sdk/qmmf_display.h
sdkinc_HEADERS += $(top_srcdir)/include/qmmf-sdk/qmmf_display_params.h
sdkinc_HEADERS += $(top_srcdir)/include/qmmf-sdk/qmmf_system.h
sdkinc_HEADERS += $(top_srcdir)/include/qmmf-sdk/qmmf_system_params.h
