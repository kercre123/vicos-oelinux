# Build libhttp_interface
lib_LTLIBRARIES = libhttp_interface.la

libhttp_interface_la_SOURCES = src/qmmf_http_interface.cc
libhttp_interface_la_SOURCES += src/qmmf_camera_configuration.cc

libhttp_interface_la_CPPFLAGS = -I$(top_srcdir)/httpinterface/inc
libhttp_interface_la_CPPFLAGS += -I$(top_srcdir)/rtspinterface/
libhttp_interface_la_CPPFLAGS += -I$(top_srcdir)/muxinterface/inc
libhttp_interface_la_CPPFLAGS += -I$(top_srcdir)/vaminterface/inc
libhttp_interface_la_CPPFLAGS += -I$(CAMERAHAL_CFLAGS) -DQMMF_LE_BUILD
libhttp_interface_la_LDFLAGS  = -lutils -lcutils -lcamera_client -lcamera_metadata -lpthread -ldl -llog
libhttp_interface_la_LDFLAGS  += -lqmmf_recorder_client -lVAManager -ljson_apis -lav_queue -lfpv_rave -lqmmf_display_client
libhttp_interface_la_LIBADD   = $(top_builddir)/rtspinterface/liblive555rtsp_in.la
libhttp_interface_la_LIBADD   += $(top_builddir)/muxinterface/libmux_interface.la
libhttp_interface_la_LIBADD   += $(top_builddir)/vaminterface/libvam_interface.la

# gtest app
bin_PROGRAMS = http_interface_gtest

http_interface_gtest_SOURCES  = gtest/qmmf_http_interface_gtest.cc
http_interface_gtest_CPPFLAGS = $(libhttp_interface_la_CPPFLAGS) -I$(top_srcdir)
http_interface_gtest_LDADD    = libhttp_interface.la -lgtest -lgtest_main
http_interface_gtest_LDFLAGS  = $(libhttp_interface_la_LDFLAGS)
