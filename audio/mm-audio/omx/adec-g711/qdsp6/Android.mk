ifeq ($(call is-board-platform-in-list,msm8660 msm8960 msm8974 msm8226 msm8610 copper apq8084 msm8994 msm8992 msm8996 msm8952 msm8937 thorium msm8953 msmgold msm8998 sdm660),true)
ifneq ($(BUILD_TINY_ANDROID),true)

LOCAL_PATH:= $(call my-dir)
include $(CLEAR_VARS)

# ---------------------------------------------------------------------------------
#                 Common definitons
# ---------------------------------------------------------------------------------

libOmxG711Dec-def := -g -O3
libOmxG711Dec-def += -DQC_MODIFIED
libOmxG711Dec-def += -D_ANDROID_
libOmxG711Dec-def += -D_ENABLE_QC_MSG_LOG_
libOmxG711Dec-def += -DVERBOSE
libOmxG711Dec-def += -D_DEBUG
libOmxG711Dec-def += -Wconversion
ifeq ($(call is-board-platform-in-list,msm8610 apq8084 msm8996 msm8952 msm8937 thorium msm8953 msmgold msm8998 sdm660),true)
libOmxG711Dec-def += -DQCOM_AUDIO_USE_SYSTEM_HEAP_ID
endif

# ---------------------------------------------------------------------------------
#             Make the Shared library (libOmxG711Dec)
# ---------------------------------------------------------------------------------

include $(CLEAR_VARS)

libOmxG711Dec-inc        := $(LOCAL_PATH)/inc
libOmxG711Dec-inc        += $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr/include
libOmxG711Dec-inc        += $(TARGET_OUT_HEADERS)/mm-core/omxcore
libOmxG711Dec-inc        += $(TARGET_OUT_HEADERS)/common/inc

LOCAL_ADDITIONAL_DEPENDENCIES  := $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr

LOCAL_MODULE            := libOmxG711Dec
LOCAL_MODULE_TAGS       := optional
LOCAL_CFLAGS            := $(libOmxG711Dec-def)
LOCAL_C_INCLUDES        := $(libOmxG711Dec-inc)
LOCAL_PRELINK_MODULE    := false
LOCAL_SHARED_LIBRARIES  := libutils liblog

LOCAL_SRC_FILES         := src/adec_svr.c
LOCAL_SRC_FILES         += src/omx_g711_adec.cpp

LOCAL_MODULE_OWNER := qti
LOCAL_PROPRIETARY_MODULE := true

include $(BUILD_SHARED_LIBRARY)

endif #BUILD_TINY_ANDROID
endif #TARGET_BOARD_PLATFORM
# ---------------------------------------------------------------------------------
#                     END
# ---------------------------------------------------------------------------------

