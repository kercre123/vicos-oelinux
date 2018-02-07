# Build libvam_interface
lib_LTLIBRARIES = libvam_interface.la

libvam_interface_la_SOURCES  = src/qmmf_vam_config_parser.cc
libvam_interface_la_SOURCES  += src/qmmf_vam_interface.cc

libvam_interface_la_CPPFLAGS = -I$(top_srcdir)/vaminterface/inc
libvam_interface_la_CPPFLAGS += -I$(top_srcdir)/httpinterface/inc
libvam_interface_la_CPPFLAGS += -I$(top_srcdir)/jpegencoder/inc
libvam_interface_la_CPPFLAGS += -I$(top_srcdir)/database/inc
libvam_interface_la_LDFLAGS  = -lutils -lpthread -ldl -llog -ljson_apis
libvam_interface_la_LIBADD  = $(top_builddir)/jpegencoder/libqmmf_jpeg_encoder.la
libvam_interface_la_LIBADD   += $(top_builddir)/database/libqmmf_database.la
