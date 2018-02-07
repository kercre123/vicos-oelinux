# Build system test application binary
bin_PROGRAMS = system_test

source_files = qmmf_system_test.cc
source_files += qmmf_system_test_wav.cc

cpp_flags = -I$(top_srcdir)
cpp_flags += -I$(top_srcdir)/include

ld_flags = -llog
ld_flags += -lutils
ld_flags += -lcutils

system_test_SOURCES  = $(source_files)
system_test_CPPFLAGS = $(cpp_flags)
system_test_LDFLAGS  = $(ld_flags)

system_test_LDADD    = ../../src/client/libqmmf_system_client.la

