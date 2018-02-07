# Build libqmmf_jpeg
lib_LTLIBRARIES = libqmmf_jpeg.la

libqmmf_jpeg_la_SOURCES = qmmf_jpeg_core.cc
libqmmf_jpeg_la_SOURCES += qmmf_jpeg.cc

libqmmf_jpeg_la_CPPFLAGS = -I$(top_srcdir)/include
libqmmf_jpeg_la_CPPFLAGS += -I$(top_srcdir)/
libqmmf_jpeg_la_CPPFLAGS += -I$(top_srcdir)/jpegencoder
libqmmf_jpeg_la_CPPFLAGS += -I$(LIBGRALLOC)
libqmmf_jpeg_la_CPPFLAGS += -I$(CAMERASTACKCOMMON_CFLAGS)
libqmmf_jpeg_la_CPPFLAGS += -I$(CAMERAINTERFACE_CFLAGS)
libqmmf_jpeg_la_CPPFLAGS += -I$(EXIF_CFLAGS)
libqmmf_jpeg_la_CPPFLAGS += -I$(OMXCORE_CFLAGS)
libqmmf_jpeg_la_CPPFLAGS += -I$(OPENMAX_CFLAGS)
libqmmf_jpeg_la_LDFLAGS  = -lutils -lpthread -ldl -llog -lbinder -lcutils
