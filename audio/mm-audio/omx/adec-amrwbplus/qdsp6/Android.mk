ifeq ($(call is-board-platform-in-list,msm8660 msm8960 msm8974 msm8226 copper apq8084 msm8994 msm8992 msm8996 msm8952 msm8937 thorium msm8953 msmgold msm8998 sdm660),true)
ifneq ($(BUILD_TINY_ANDROID),true)

LOCAL_PATH:= $(call my-dir)
include $(CLEAR_VARS)

# ---------------------------------------------------------------------------------
#                 Common definitons
# ---------------------------------------------------------------------------------

libOmxAmrwbplusDec-def := -g -O3
libOmxAmrwbplusDec-def += -DQC_MODIFIED
libOmxAmrwbplusDec-def += -D_ANDROID_
libOmxAmrwbplusDec-def += -D_ENABLE_QC_MSG_LOG_
libOmxAmrwbplusDec-def += -DVERBOSE
libOmxAmrwbplusDec-def += -D_DEBUG
libOmxAmrwbplusDec-def += -DAUDIOV2
libOmxAmrwbplusDec-def += -Wconversion
ifeq ($(call is-board-platform-in-list,msm8610 apq8084 msm8996 msm8952 msm8937 thorium msm8953 msmgold msm8998 sdm660),true)
libOmxAmrwbplusDec-def += -DQCOM_AUDIO_USE_SYSTEM_HEAP_ID
endif
# ---------------------------------------------------------------------------------
#             Make the Shared library (libOmxAmrwbplusDec)
# ---------------------------------------------------------------------------------

include $(CLEAR_VARS)

libOmxAmrwbplus-inc        := $(LOCAL_PATH)/inc
libOmxAmrwbplus-inc        += $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr/include
libOmxAmrwbplus-inc        += $(TARGET_OUT_HEADERS)/mm-core/omxcore
libOmxAmrwbplus-inc        += $(TARGET_OUT_HEADERS)/common/inc

LOCAL_ADDITIONAL_DEPENDENCIES  := $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr

LOCAL_MODULE            := libOmxAmrwbplusDec
LOCAL_MODULE_TAGS       := optional
LOCAL_CFLAGS            := $(libOmxAmrwbplusDec-def)
LOCAL_C_INCLUDES        := $(libOmxAmrwbplus-inc)
LOCAL_PRELINK_MODULE    := false
LOCAL_SHARED_LIBRARIES  := libutils liblog

LOCAL_SRC_FILES         := src/adec_svr.c
LOCAL_SRC_FILES         += src/omx_amrwbplus_adec.cpp

LOCAL_MODULE_OWNER := qcom
LOCAL_PROPRIETARY_MODULE := true

include $(BUILD_SHARED_LIBRARY)

endif #BUILD_TINY_ANDROID
endif #TARGET_BOARD_PLATFORM
# ---------------------------------------------------------------------------------
#                     END
# ---------------------------------------------------------------------------------

