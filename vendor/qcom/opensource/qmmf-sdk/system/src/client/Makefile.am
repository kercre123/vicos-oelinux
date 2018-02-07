# Build qmmf system client library
# libqmmf_system_client.so
lib_LTLIBRARIES = libqmmf_system_client.la

source_files = qmmf_system.cc
source_files += qmmf_system_client.cc

cpp_flags = -I$(top_srcdir)
cpp_flags += -I$(top_srcdir)/include

ld_flags = -llog
ld_flags += -lbinder
ld_flags += -lutils
ld_flags += -lcutils

libqmmf_system_client_la_SOURCES  = $(source_files)
libqmmf_system_client_la_CPPFLAGS = $(cpp_flags)
libqmmf_system_client_la_LDFLAGS  = $(ld_flags)

Export qmmf-sdk headers:
sdkincdir=$(includedir)/qmmf-sdk
sdkinc_HEADERS = $(top_srcdir)/include/qmmf-sdk/qmmf_system.h
sdkinc_HEADERS += $(top_srcdir)/include/qmmf-sdk/qmmf_system_params.h
sdkinc_HEADERS += $(top_srcdir)/include/qmmf-sdk/qmmf_device.h
sdkinc_HEADERS += $(top_srcdir)/include/qmmf-sdk/qmmf_codec.h
