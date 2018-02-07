# Build libhttp_interface
lib_LTLIBRARIES = libqmmf_jpeg_encoder.la

libqmmf_jpeg_encoder_la_SOURCES := src/qmmf_jpeg_encoder.cpp

libqmmf_jpeg_encoder_la_CPPFLAGS := -I$(top_srcdir)/jpegencoder/inc
libqmmf_jpeg_encoder_la_CPPFLAGS += -I$(CAMERASTACKCOMMON_CFLAGS)
libqmmf_jpeg_encoder_la_CPPFLAGS += -I$(CAMERAINTERFACE_CFLAGS)
libqmmf_jpeg_encoder_la_CPPFLAGS += -I$(EXIF_CFLAGS)
libqmmf_jpeg_encoder_la_CPPFLAGS += -I$(OMXCORE_CFLAGS)
libqmmf_jpeg_encoder_la_CPPFLAGS += -I$(OPENMAX_CFLAGS)
libqmmf_jpeg_encoder_la_CPPFLAGS += -std=c++14 -std=gnu++0x -std=c++0x -std=gnu++11 -Wall -Wextra -Werror
libqmmf_jpeg_encoder_la_LDFLAGS  := -lutils -lpthread -ldl -llog
