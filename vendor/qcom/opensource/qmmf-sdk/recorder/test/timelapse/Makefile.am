# Build recorder time lapse application binary
bin_PROGRAMS = recorder_timelapse

recorder_timelapse_SOURCES  = qmmf_time_lapse.cc
recorder_timelapse_SOURCES  += qmmf_time_lapse_main.cc
recorder_timelapse_CPPFLAGS = -I$(top_srcdir)/include
recorder_timelapse_CPPFLAGS += -I$(top_srcdir)/
recorder_timelapse_LDFLAGS  = -lutils -lcutils -llog
recorder_timelapse_LDFLAGS  += -lcamera_client -lcamera_metadata
recorder_timelapse_LDADD    = ../../src/client/libqmmf_recorder_client.la
recorder_timelapse_LDADD    += ../../../display/src/client/libqmmf_display_client.la
