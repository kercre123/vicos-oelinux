# Build libavcodec.so
lib_LTLIBRARIES = libav_codec.la

libav_codec_la_SOURCES = src/qmmf_avcodec.cc

libav_codec_la_CPPFLAGS = -I$(top_srcdir)/include
libav_codec_la_CPPFLAGS += -I$(top_srcdir)/
libav_codec_la_CPPFLAGS += -I$(MMCORE_CFLAGS)
libav_codec_la_LDFLAGS  = -lutils -lcutils -lpthread -ldl -llog
libav_codec_la_LIBADD   = ../common/codecadaptor/libcodec_adaptor.la
libav_codec_la_LIBADD   += ../common/utils/libqmmf_utils.la

# AVCodec sample test app
bin_PROGRAMS = av_codec_test

av_codec_test_SOURCES  = test/sample/qmmf_avcodec_test.cc
av_codec_test_CPPFLAGS = $(libav_codec_la_CPPFLAGS)
av_codec_test_LDADD    = libav_codec.la

# AVCodec gtest app
bin_PROGRAMS += av_codec_gtest

av_codec_gtest_SOURCES  = test/gtest/qmmf_avcodec_gtest.cc
av_codec_gtest_CPPFLAGS = $(libav_codec_la_CPPFLAGS)
av_codec_gtest_LDFLAGS  = -lgtest -lgtest_main
av_codec_gtest_LDADD    = libav_codec.la

# Codec Adaptor sample audio decode test app
bin_PROGRAMS += av_codec_audio_decode_test

av_codec_audio_decode_test_SOURCES  = test/sample/qmmf_audio_decode_test.cc
av_codec_audio_decode_test_CPPFLAGS = $(libav_codec_la_CPPFLAGS)
av_codec_audio_decode_test_LDADD    = libav_codec.la

if ENABLE_JPEG_TURBO
# Bin Program for Standalone Jpeg turbo decode test
bin_PROGRAMS += jpeg_turbo_decode_test
jpeg_turbo_decode_test_SOURCES = test/sample/jpeg-turbo/jpeg_turbo_decode_test.cc
jpeg_turbo_decode_test_CPPFLAGS = $(libav_codec_la_CPPFLAGS)
jpeg_turbo_decode_test_LDFLAGS  = -llog -ljpeg -lturbojpeg
endif

Export qmmf-sdk headers:
sdkincdir=$(includedir)/qmmf-sdk
sdkinc_HEADERS = $(top_srcdir)/include/qmmf-sdk/qmmf_avcodec.h
sdkinc_HEADERS += $(top_srcdir)/include/qmmf-sdk/qmmf_avcodec_params.h
sdkinc_HEADERS += $(top_srcdir)/include/qmmf-sdk/qmmf_buffer.h
sdkinc_HEADERS += $(top_srcdir)/include/qmmf-sdk/qmmf_codec.h
