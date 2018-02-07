# Build display test application binary
bin_PROGRAMS = display_test

display_test_SOURCES  = qmmf_display_test.cc
display_test_CPPFLAGS = -I$(top_srcdir)/include
display_test_CPPFLAGS += -I$(top_srcdir)/
display_test_LDFLAGS  = -lutils -lcutils -llog
display_test_LDADD    = ../../src/client/libqmmf_display_client.la

