# Build qmmf system service library
# libqmmf_system_service.so
lib_LTLIBRARIES = libqmmf_system_service.la

source_files = qmmf_system_service.cc
source_files += qmmf_system_implementation.cc
source_files += qmmf_system_ion.cc
source_files += qmmf_system_keytone.cc
source_files += qmmf_system_devices.cc
source_files += qmmf_system_trigger.cc

cpp_flags = -I$(top_srcdir)
cpp_flags += -I$(top_srcdir)/include

ld_flags = -llog
ld_flags += -lbinder
ld_flags += -lutils
ld_flags += -lcutils

libqmmf_system_service_la_SOURCES  = $(source_files)
libqmmf_system_service_la_CPPFLAGS = $(cpp_flags)
libqmmf_system_service_la_LDFLAGS  = $(ld_flags)

libqmmf_system_service_la_LIBADD = ../client/libqmmf_system_client.la
