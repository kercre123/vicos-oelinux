source_files = qmmf_recorder.cc
source_files += qmmf_recorder_client.cc
source_files += qmmf_recorder_client_ion.cc
source_files += qmmf_recorder_extra_param.cc

# Build qmmf recorder client library
# libqmmf_recorder_client.so
lib_LTLIBRARIES = libqmmf_recorder_client.la

libqmmf_recorder_client_la_CPPFLAGS = -I$(top_srcdir)/include
libqmmf_recorder_client_la_CPPFLAGS += -I$(top_srcdir)/
libqmmf_recorder_client_la_CPPFLAGS += -I$(MMCORE_CFLAGS)
libqmmf_recorder_client_la_LDFLAGS  = -llog -lbinder -lutils -lcutils
libqmmf_recorder_client_la_LDFLAGS  += -lcamera_client -lcamera_metadata
libqmmf_recorder_client_la_LIBADD  = ../../../common/cameraadaptor/libcamera_adaptor.la

libqmmf_recorder_client_la_SOURCES = $(source_files)

Export qmmf-sdk headers:
sdkincdir=$(includedir)/qmmf-sdk
sdkinc_HEADERS = $(top_srcdir)/include/qmmf-sdk/qmmf_recorder.h
sdkinc_HEADERS += $(top_srcdir)/include/qmmf-sdk/qmmf_recorder_params.h
sdkinc_HEADERS += $(top_srcdir)/include/qmmf-sdk/qmmf_recorder_extra_param.h
sdkinc_HEADERS += $(top_srcdir)/include/qmmf-sdk/qmmf_recorder_extra_param_tags.h
sdkinc_HEADERS += $(top_srcdir)/include/qmmf-sdk/qmmf_display.h
sdkinc_HEADERS += $(top_srcdir)/include/qmmf-sdk/qmmf_display_params.h
sdkinc_HEADERS += $(top_srcdir)/include/qmmf-sdk/qmmf_system.h
sdkinc_HEADERS += $(top_srcdir)/include/qmmf-sdk/qmmf_system_params.h
sdkinc_HEADERS += $(top_srcdir)/include/qmmf-sdk/qmmf_player.h
sdkinc_HEADERS += $(top_srcdir)/include/qmmf-sdk/qmmf_player_params.h
sdkinc_HEADERS += $(top_srcdir)/include/qmmf-sdk/qmmf_device.h
sdkinc_HEADERS += $(top_srcdir)/include/qmmf-sdk/qmmf_codec.h
sdkinc_HEADERS += $(top_srcdir)/include/qmmf-sdk/qmmf_overlay.h
