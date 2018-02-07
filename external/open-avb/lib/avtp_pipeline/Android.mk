LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_CFLAGS := -Wall -Wextra -Wno-parentheses -ggdb -D_GNU_SOURCE -Wunreachable-code -DAVB_FEATURE_FQTSS

LOCAL_LDFLAGS += -Wl,--export-dynamic

LOCAL_C_INCLUDES := $(LOCAL_PATH) \
					$(LOCAL_PATH)/avtp \
					$(LOCAL_PATH)/endpoint \
					$(LOCAL_PATH)/include \
					$(LOCAL_PATH)/inih \
					$(LOCAL_PATH)/intf_clk_ref \
					$(LOCAL_PATH)/intf_ctrl \
					$(LOCAL_PATH)/intf_echo \
					$(LOCAL_PATH)/intf_logger \
					$(LOCAL_PATH)/intf_null \
					$(LOCAL_PATH)/intf_tonegen \
					$(LOCAL_PATH)/intf_viewer \
					$(LOCAL_PATH)/maap \
					$(LOCAL_PATH)/map_aaf_audio \
					$(LOCAL_PATH)/map_ctrl \
					$(LOCAL_PATH)/map_clk_ref \
					$(LOCAL_PATH)/map_h264 \
					$(LOCAL_PATH)/map_mjpeg \
					$(LOCAL_PATH)/map_null \
					$(LOCAL_PATH)/map_pipe \
					$(LOCAL_PATH)/map_uncmp_audio \
					$(LOCAL_PATH)/mcr \
					$(LOCAL_PATH)/mediaq \
					$(LOCAL_PATH)/openavb_common \
					$(LOCAL_PATH)/platform/generic \
					$(LOCAL_PATH)/platform/generic/include/linux \
					$(LOCAL_PATH)/platform/Linux \
					$(LOCAL_PATH)/platform/Linux/endpoint \
					$(LOCAL_PATH)/platform/Linux/rawsock \
					$(LOCAL_PATH)/tl \
					$(LOCAL_PATH)/platform/platTCAL/GNU \
					$(LOCAL_PATH)/util \
					$(LOCAL_PATH)/../igb \
					$(LOCAL_PATH)/../libmjpegavbsink \
					$(LOCAL_PATH)/platform/x86_i210 \
					$(LOCAL_PATH)/qmgr \
					$(LOCAL_PATH)/srp \
					$(LOCAL_PATH)/rawsock \
					$(LOCAL_PATH)/../../daemons/mrpd \
					$(LOCAL_PATH)/../../daemons/common \
					$(LOCAL_PATH)/../libmpeg2ts \
					$(LOCAL_PATH)/../libh264sink \
					external/tinyalsa/include/


LOCAL_SRC_FILES :=  platform/Linux/avb_host/openavb_host.c \
					platform/Linux/openavb_osal_endpoint.c \
					platform/Linux/rawsock/openavb_rawsock.c \
					platform/Linux/rawsock/simple_rawsock.c \
					platform/Linux/rawsock/ring_rawsock.c \
					platform/Linux/rawsock/igb_rawsock.c \
					platform/Linux/openavb_time_osal.c \
					platform/x86_i210/openavb_time_hal.c \
					platform/x86_i210/mcr/openavb_mcr_hal.c \
					openavb_common/avb.c \
					openavb_common/mrp_client.c \
					platform/Linux/rawsock/pcap_rawsock.c \
					util/openavb_log.c \
					util/openavb_queue.c \
					tl/openavb_tl.c \
					util/openavb_plugin.c \
					map_pipe/openavb_map_pipe.c \
					map_clk_ref/openavb_map_clk_ref.c \
					map_ctrl/openavb_map_ctrl.c \
					mediaq/openavb_mediaq.c \
					avtp/openavb_avtp_time.c \
					util/openavb_result_codes.c \
					tl/openavb_tl_endpoint.c \
					platform/Linux/tl/openavb_tl_osal.c \
					util/openavb_array.c \
					platform/Linux/endpoint/openavb_endpoint_osal.c \
					endpoint/openavb_endpoint.c \
					util/openavb_list.c \
					util/openavb_debug.c \
					util/openavb_printbuf.c \
					util/openavb_time.c \
					util/openavb_timestamp.c \
					tl/openavb_listener.c \
					tl/openavb_listener_endpoint.c \
					tl/openavb_talker.c \
					tl/openavb_talker_endpoint.c \
					avtp/openavb_avtp.c \
					rawsock/rawsock_impl.c \
					endpoint/openavb_endpoint_client.c \
					inih/ini.c \
					platform/Linux/endpoint/openavb_endpoint_cfg.c \
					endpoint/openavb_endpoint_server.c \
					intf_viewer/openavb_intf_viewer.c \
					intf_clk_ref/openavb_intf_clk_ref.c \
					intf_ctrl/openavb_intf_ctrl.c \
					intf_echo/openavb_intf_echo.c \
					intf_logger/openavb_intf_logger.c \
					intf_null/openavb_intf_null.c \
					intf_tonegen/openavb_intf_tonegen.c \
					platform/Linux/intf_mpeg2ts_file/openavb_intf_mpeg2ts_file.c \
					platform/Linux/intf_wav_file/openavb_intf_wav_file.c \
					map_aaf_audio/openavb_map_aaf_audio.c \
					map_h264/openavb_map_h264.c \
					map_mjpeg/openavb_map_mjpeg.c \
					map_mpeg2ts/openavb_map_mpeg2ts.c \
					map_null/openavb_map_null.c \
					map_uncmp_audio/openavb_map_uncmp_audio.c \
					qmgr/openavb_qmgr.c \
					platform/x86_i210/openavb_ether_hal.c \
					../igb/igb.c  \
					platform/Linux/intf_mjpeg_file/openavb_intf_mjpeg_file.c \
					platform/Linux/intf_h264_file/openavb_intf_h264_file.c \
					platform/Linux/intf_h264_stream/openavb_intf_h264_stream.c \
					platform/Linux/intf_alsa/openavb_intf_tinyalsa.c \
					mcr/openavb_reference_clock.c \
					platform/Linux/intf_mjpeg_opengl/openavb_intf_mjpeg_opengl.c \
					platform/Linux/intf_mpeg2ts_stream/openavb_intf_mpeg2ts_stream.c

LOCAL_STATIC_LIBRARIES += 	libpcap \

LOCAL_SHARED_LIBRARIES += 	libtinyalsa \
				liblog \
				libmjpegavbsink \
                                libmpeg2ts\
				libh264sink

LOCAL_MODULE := openavb_harness


include $(BUILD_EXECUTABLE)
