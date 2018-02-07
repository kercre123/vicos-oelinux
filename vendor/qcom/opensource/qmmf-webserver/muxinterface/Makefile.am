# Build libhmux_interface
lib_LTLIBRARIES = libmux_interface.la

libmux_interface_la_SOURCES  = src/qmmf_mux_interface.cc

libmux_interface_la_CPPFLAGS = -I$(top_srcdir)/muxinterface/inc
libmux_interface_la_CPPFLAGS += -I$(top_srcdir)/httpinterface/inc
libmux_interface_la_LDFLAGS  = -lutils -lpthread -ldl -llog -lFileMux
