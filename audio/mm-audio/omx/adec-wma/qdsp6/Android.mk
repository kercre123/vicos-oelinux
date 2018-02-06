ifeq ($(call is-board-platform-in-list,msm8660 msm8960 msm8974 msm8226 msm8610 copper apq8084 msm8994 msm8992 msm8996 msm8952 msm8937 thorium msm8953 msmgold msm8998 sdm660),true)
ifneq ($(BUILD_TINY_ANDROID),true)

LOCAL_PATH:= $(call my-dir)
include $(CLEAR_VARS)

# ---------------------------------------------------------------------------------
#                 Common definitons
# ---------------------------------------------------------------------------------

libOmxWmaDec-def := -g -O3
libOmxWmaDec-def += -DQC_MODIFIED
libOmxWmaDec-def += -D_ANDROID_
libOmxWmaDec-def += -D_ENABLE_QC_MSG_LOG_
libOmxWmaDec-def += -DVERBOSE
libOmxWmaDec-def += -D_DEBUG
libOmxWmaDec-def += -DAUDIOV2
libOmxWmaDec-def += -Wconversion
ifeq ($(call is-board-platform-in-list,msm8610 apq8084 msm8996 msm8952 msm8937 thorium msm8953 msmgold msm8998 sdm660),true)
libOmxWmaDec-def += -DQCOM_AUDIO_USE_SYSTEM_HEAP_ID
endif
ifeq ($(strip $(AUDIO_FEATURE_ENABLED_NT_PAUSE_TIMEOUT)),true)
libOmxWmaDec-def += -DNT_PAUSE_TIMEOUT_ENABLED
endif

# ---------------------------------------------------------------------------------
#             Make the Shared library (libOmxWmaDec)
# ---------------------------------------------------------------------------------

include $(CLEAR_VARS)

libOmxWmaDec-inc        := $(LOCAL_PATH)/inc
libOmxWmaDec-inc        += $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr/include
libOmxWmaDec-inc        += $(TARGET_OUT_HEADERS)/mm-core/omxcore
libOmxWmaDec-inc        += $(TARGET_OUT_HEADERS)/common/inc

LOCAL_ADDITIONAL_DEPENDENCIES  := $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr

LOCAL_MODULE            := libOmxWmaDec
LOCAL_MODULE_TAGS       := optional
LOCAL_CFLAGS            := $(libOmxWmaDec-def)
LOCAL_C_INCLUDES        := $(libOmxWmaDec-inc)
LOCAL_PRELINK_MODULE    := false
LOCAL_SHARED_LIBRARIES  := libutils liblog

LOCAL_SRC_FILES         := src/adec_svr.c
LOCAL_SRC_FILES         += src/omx_wma_adec.cpp

LOCAL_MODULE_OWNER := qcom
LOCAL_PROPRIETARY_MODULE := true

include $(BUILD_SHARED_LIBRARY)

endif #BUILD_TINY_ANDROID
endif #TARGET_BOARD_PLATFORM
# ---------------------------------------------------------------------------------
#                     END
# ---------------------------------------------------------------------------------

