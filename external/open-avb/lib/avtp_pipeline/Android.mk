LOCAL_PATH := $(call my-dir)

#######################################
# openavb core library
#######################################

include $(CLEAR_VARS)

LOCAL_CFLAGS := -Wall -Wextra -Wno-parentheses -ggdb -D_GNU_SOURCE -Wunreachable-code -DAVB_FEATURE_FQTSS

LOCAL_LDFLAGS += -Wl,--export-dynamic

LOCAL_C_INCLUDES := $(LOCAL_PATH) \
            $(LOCAL_PATH)/avtp \
            $(LOCAL_PATH)/endpoint \
            $(LOCAL_PATH)/include \
            $(LOCAL_PATH)/inih \
            $(LOCAL_PATH)/maap \
            $(LOCAL_PATH)/mcr \
            $(LOCAL_PATH)/mediaq \
            $(LOCAL_PATH)/openavb_common \
            $(LOCAL_PATH)/platform/generic \
            $(LOCAL_PATH)/platform/generic/include/linux \
            $(LOCAL_PATH)/platform/Linux \
            $(LOCAL_PATH)/platform/Linux/endpoint \
            $(LOCAL_PATH)/platform/Linux/rawsock \
            $(LOCAL_PATH)/tl \
            $(LOCAL_PATH)/util \
            $(LOCAL_PATH)/qmgr \
            $(LOCAL_PATH)/srp \
            $(LOCAL_PATH)/rawsock \
            $(LOCAL_PATH)/../../daemons/mrpd \
            $(LOCAL_PATH)/../../daemons/common


LOCAL_SRC_FILES := \
            avtp/openavb_avtp_time.c \
            avtp/openavb_avtp.c \
            endpoint/openavb_endpoint.c \
            endpoint/openavb_endpoint_client.c \
            endpoint/openavb_endpoint_server.c \
            inih/ini.c \
            mcr/openavb_reference_clock.c \
            mediaq/openavb_mediaq.c \
            openavb_common/avb.c \
            openavb_common/mrp_client.c \
            platform/Linux/tl/openavb_tl_osal.c \
            platform/Linux/endpoint/openavb_endpoint_osal.c \
            platform/Linux/openavb_osal_endpoint.c \
            platform/Linux/openavb_time_osal.c \
            platform/Linux/endpoint/openavb_endpoint_cfg.c \
            qmgr/openavb_qmgr.c \
            tl/openavb_listener.c \
            tl/openavb_listener_endpoint.c \
            tl/openavb_talker.c \
            tl/openavb_talker_endpoint.c \
            tl/openavb_tl.c \
            tl/openavb_tl_endpoint.c

# raw socket
LOCAL_SRC_FILES += \
            rawsock/rawsock_impl.c \
            platform/Linux/rawsock/openavb_rawsock.c \
            platform/Linux/rawsock/simple_rawsock.c \
            platform/Linux/rawsock/ring_rawsock.c \
            platform/Linux/rawsock/pcap_rawsock.c

# utils
LOCAL_SRC_FILES +=  \
            util/openavb_log.c \
            util/openavb_queue.c \
            util/openavb_array.c \
            util/openavb_list.c \
            util/openavb_debug.c \
            util/openavb_printbuf.c \
            util/openavb_time.c \
            util/openavb_timestamp.c \
            util/openavb_plugin.c \
            util/openavb_result_codes.c

LOCAL_EXPORT_C_INCLUDE_DIRS := $(LOCAL_C_INCLUDES)

LOCAL_MODULE := libopenavb

LOCAL_STATIC_LIBRARIES := libpcap

LOCAL_SHARED_LIBRARIES += liblog

include $(BUILD_SHARED_LIBRARY)

#######################################
# openavb executable
#######################################
include $(CLEAR_VARS)

LOCAL_C_INCLUDES := $(LOCAL_PATH)

LOCAL_SRC_FILES := platform/Linux/avb_host/openavb_host.c

LOCAL_SHARED_LIBRARIES += \
            libopenavb \
            libopenavb_intf_clk_ref \
            libopenavb_intf_ctrl \
            libopenavb_intf_echo \
            libopenavb_intf_h264_file \
            libopenavb_intf_h264_stream \
            libopenavb_intf_logger \
            libopenavb_intf_mjpeg_file \
            libopenavb_intf_mjpeg_opengl \
            libopenavb_intf_mpeg2ts_file \
            libopenavb_intf_mpeg2ts_stream \
            libopenavb_intf_null \
            libopenavb_intf_tinyalsa \
            libopenavb_intf_tonegen \
            libopenavb_intf_viewer \
            libopenavb_intf_wav_file \
            libopenavb_map_aaf_audio \
            libopenavb_map_clk_ref \
            libopenavb_map_ctrl \
            libopenavb_map_h264 \
            libopenavb_map_mjpeg \
            libopenavb_map_mpeg2ts \
            libopenavb_map_null \
            libopenavb_map_pipe \
            libopenavb_map_uncmp_audio

LOCAL_MODULE := openavb_harness

include $(BUILD_EXECUTABLE)

###########################################
# include all mapping and interface modules
###########################################
include $(call first-makefiles-under,$(LOCAL_PATH))
