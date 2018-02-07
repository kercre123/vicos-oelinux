# Build transcode application binary
bin_PROGRAMS = transcode_app

transcode_app_SOURCES  = qmmf_transcode_utils.cc
transcode_app_SOURCES  += qmmf_transcode_pipe.cc
transcode_app_SOURCES  += qmmf_transcode_source.cc
transcode_app_SOURCES  += qmmf_transcode_sink.cc
transcode_app_SOURCES  += qmmf_transcode_track.cc
transcode_app_SOURCES  += qmmf_transcode_test.cc
transcode_app_CPPFLAGS = $(CFLAGS)
transcode_app_CPPFLAGS += -I$(top_srcdir)/include
transcode_app_CPPFLAGS += -I$(top_srcdir)/
transcode_app_LDFLAGS  = -lutils -lcutils -llog
transcode_app_LDADD    = ../demuxer/libdemuxer.la
transcode_app_LDADD    += ../../../avcodec/libav_codec.la