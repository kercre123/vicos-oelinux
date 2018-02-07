# Build recorder test application binary
bin_PROGRAMS = recorder_test

recorder_test_SOURCES  = qmmf_recorder_test.cc
recorder_test_SOURCES  += qmmf_recorder_test_wav.cc
recorder_test_SOURCES  += qmmf_recorder_test_aac.cc
recorder_test_SOURCES  += qmmf_recorder_test_amr.cc
recorder_test_CPPFLAGS = -I$(top_srcdir)/include
recorder_test_CPPFLAGS += -I$(top_srcdir)/
recorder_test_CPPFLAGS += -I$(CAMERAHAL_CFLAGS)
recorder_test_CPPFLAGS += -DUSE_SKIA=0
recorder_test_CPPFLAGS += -DUSE_CAIRO=1
recorder_test_LDFLAGS  = -lutils -lcutils -llog -lcairo $(LIBGTEST)
recorder_test_LDADD    = ../../src/client/libqmmf_recorder_client.la
recorder_test_LDADD    += ../../../display/src/client/libqmmf_display_client.la
