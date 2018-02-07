LOCAL_PATH := $(call my-dir)

QMMF_WEBSERVER_TOP_SRCDIR := $(LOCAL_PATH)/..

include $(QMMF_WEBSERVER_TOP_SRCDIR)/build.mk

ifneq (,$(BUILD_QMMMF))

# Build liblive555rtsp_in

include $(CLEAR_VARS)

include $(QMMF_WEBSERVER_TOP_SRCDIR)/common.mk

LOCAL_C_INCLUDES += $(QMMF_SDK_TOP_SRCDIR)/common/avqueue
LOCAL_C_INCLUDES += $(TOP)/external/live555/liveMedia/include
LOCAL_C_INCLUDES += $(TOP)/external/live555/groupsock/include
LOCAL_C_INCLUDES += $(TOP)/external/live555/UsageEnvironment/include
LOCAL_C_INCLUDES += $(TOP)/external/live555/BasicUsageEnvironment/include

LOCAL_SRC_FILES := qmmf_adts_rtime_framed_source.cc
LOCAL_SRC_FILES += qmmf_amr_rtime_framed_source.cc
LOCAL_SRC_FILES += qmmf_audio_rtime_server_media_subsession.cc
LOCAL_SRC_FILES += qmmf_mp2ts_rtime_framed_source.cc
LOCAL_SRC_FILES += qmmf_mp2ts_rtime_server_media_subsession.cc
LOCAL_SRC_FILES += qmmf_mp2ts_es_rtime_server_media_subsession.cc
LOCAL_SRC_FILES += qmmf_metadata_server_media_subsession.cc
LOCAL_SRC_FILES += qmmf_metadata_framed_source.cc
LOCAL_SRC_FILES += qmmf_realtime_rtsp_server.cc
LOCAL_SRC_FILES += qmmf_video_rtime_framed_source.cc
LOCAL_SRC_FILES += qmmf_video_rtime_server_media_subsession.cc
LOCAL_SRC_FILES += qmmf_rtsp_server_interface.cc

LOCAL_SHARED_LIBRARIES += libqmmf_av_queue liblive555Media
LOCAL_SHARED_LIBRARIES += liblive555BasicUsageEnvironment liblive555groupsock

LOCAL_MODULE = libqmmf_live555rtsp_in

include $(BUILD_SHARED_LIBRARY)

endif # BUILD_QMMMF
