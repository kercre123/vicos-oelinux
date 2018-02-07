# Build audio test application binary
bin_PROGRAMS = audio_test

source_files = qmmf_audio_test.cc
source_files += qmmf_audio_test_ion.cc
source_files += qmmf_audio_test_wav.cc

cpp_flags = -I$(top_srcdir)
cpp_flags += -I$(top_srcdir)/include

ld_flags = -llog
ld_flags += -lutils
ld_flags += -lcutils

audio_test_SOURCES  = $(source_files)
audio_test_CPPFLAGS = $(cpp_flags)
audio_test_LDFLAGS  = $(ld_flags)

audio_test_LDADD    = ../../src/client/libqmmf_audio_client.la

