# Build recorder time lapse application binary
bin_PROGRAMS = recorder_video_timelapse

recorder_video_timelapse_SOURCES  = qmmf_video_time_lapse.cc
recorder_video_timelapse_SOURCES  += qmmf_video_time_lapse_main.cc

recorder_video_timelapse_CPPFLAGS = -I$(top_srcdir)/include
recorder_video_timelapse_CPPFLAGS += -I$(top_srcdir)/
recorder_video_timelapse_CPPFLAGS += -I$(CAMERAHAL_CFLAGS)

recorder_video_timelapse_LDFLAGS  = -lutils -lcutils -llog
recorder_video_timelapse_LDFLAGS  += -lcamera_client -lcamera_metadata

recorder_video_timelapse_LDADD    = ../../src/client/libqmmf_recorder_client.la
recorder_video_timelapse_LDADD    += ../../../avcodec/libav_codec.la
