source_files = qmmf_player_service.cc
source_files += qmmf_player_impl.cc
source_files += qmmf_player_common.cc
source_files += qmmf_player_remote_cb.cc
source_files += qmmf_player_audio_decoder_core.cc
source_files += qmmf_player_video_decoder_core.cc
source_files += qmmf_player_audio_sink.cc
source_files += qmmf_player_video_sink.cc

# Build qmmf player service library
# libqmmf_player_service.so
lib_LTLIBRARIES = libqmmf_player_service.la

libqmmf_player_service_la_SOURCES = $(source_files)

libqmmf_player_service_la_CPPFLAGS = -I$(top_srcdir)/include
libqmmf_player_service_la_CPPFLAGS += -I$(top_srcdir)/
libqmmf_player_service_la_CPPFLAGS += -I$(LIBGRALLOC)
libqmmf_player_service_la_CPPFLAGS += -I$(MMCORE_CFLAGS)
libqmmf_player_service_la_LDFLAGS  = -llog -lbinder -lutils -lcutils -lfastcvopt

libqmmf_player_service_la_LIBADD   = ../client/libqmmf_player_client.la
libqmmf_player_service_la_LIBADD   += ../../../common/codecadaptor/libcodec_adaptor.la
libqmmf_player_service_la_LIBADD   += ../../../common/audio/src/client/libqmmf_audio_client.la
libqmmf_player_service_la_LIBADD   += ../../../display/src/client/libqmmf_display_client.la

