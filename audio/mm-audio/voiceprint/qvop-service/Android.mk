##########################################################
#
# Makefile for VoicePrint modules - Android
#
##########################################################
# Qvop lib
##########################################################
LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)
LOCAL_CFLAGS += -Werror -Wno-unused-parameter -Wno-ignored-attributes
# debug flag
LOCAL_CFLAGS +=-DQVOP_DEBUG
LOCAL_C_INCLUDES := $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr/include \
                    $(LOCAL_PATH)/../../../securemsm/QSEEComAPI \
                    $(TARGET_OUT_HEADERS)/common/inc \
                    $(LOCAL_PATH)/..          \
                    $(LOCAL_PATH)/../../../diag/include \
                    external/connectivity/stlport/stlport
LOCAL_ADDITIONAL_DEPENDENCIES := $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr
LOCAL_SRC_FILES := \
    BpQvopService.cpp \
    BnQvopService.cpp \
    IQvopServiceAndroid.cpp \
    logfile.cpp \
    QvopCallbackAndroid.cpp \
    QvopCommandService.cpp \
    QvopServiceAndroid.cpp \
    QvopServiceJni.cpp \
    qvop_interface.c \
    qvop_qsee_interface.c \
    qvop_qsee.c


LOCAL_SHARED_LIBRARIES += libbinder libandroid_runtime libc libcutils libdiag libutils libQSEEComAPI
LOCAL_LDLIBS :=
LOCAL_PRELINK_MODULE := false
LOCAL_MODULE:= libqvop-service
LOCAL_MODULE_TAGS := eng
LOCAL_MODULE_OWNER := qti
LOCAL_PROPRIETARY_MODULE := true
LOCAL_ADDITIONAL_DEPENDENCIES += $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr
include $(BUILD_SHARED_LIBRARY)

##########################################################
#
# Makefile for VoicePrint modules - Linux
#
##########################################################
# Qvop lib
##########################################################
# checks equivalent to /hardware/qcom/audio
ifneq ($(filter mpq8092 msm8960 msm8226 msm8x26 msm8610 msm8974 msm8x74 apq8084 msm8916 msm8994 msm8992 msm8909 msm8996 msm8952 msm8937 thorium msm8953 msmgold msm8998 sdm660,$(TARGET_BOARD_PLATFORM)),)
ifeq ($(BOARD_USES_LEGACY_ALSA_AUDIO),false)
# BOARD_SUPPORTS_QAHW - check equivalent to
# /hardware/qcom/audio/qahw_api
ifeq ($(strip $(BOARD_SUPPORTS_QAHW)),true)
include $(CLEAR_VARS)
LOCAL_CFLAGS += -Werror -Wno-unused-parameter -Wno-ignored-attributes
# debug flag
LOCAL_CFLAGS +=-DQVOP_DEBUG
LOCAL_CFLAGS +=-DQVOP_LNX_ONLY
LOCAL_C_INCLUDES := $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr/include \
                    $(LOCAL_PATH)/../../../securemsm/QSEEComAPI \
                    $(TARGET_OUT_HEADERS)/common/inc \
                    $(LOCAL_PATH)/..          \
                    $(LOCAL_PATH)/../../../diag/include \
                    external/connectivity/stlport/stlport
LOCAL_ADDITIONAL_DEPENDENCIES := $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr
LOCAL_SRC_FILES := \
    logfile.cpp \
    QvopCommandService.cpp \
    QvopService.cpp \
    qvop_interface.c \
    qvop_qsee_interface.c \
    qvop_qsee.c

LOCAL_SHARED_LIBRARIES += libc libcutils libdiag libutils libQSEEComAPI
LOCAL_LDLIBS :=
LOCAL_PRELINK_MODULE := false
LOCAL_MODULE:= libqvop-lnx-service
LOCAL_MODULE_TAGS := eng
LOCAL_MODULE_OWNER := qti
LOCAL_PROPRIETARY_MODULE := true
include $(BUILD_SHARED_LIBRARY)
endif
endif
endif
##########################################################
# Qvop Daemon
##########################################################
include $(CLEAR_VARS)
LOCAL_CFLAGS += -Werror -Wno-unused-parameter -Wno-error=date-time
LOCAL_CFLAGS +=-DQVOP_DEBUG
LOCAL_C_INCLUDES := $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr/include \
                    $(LOCAL_PATH)/../../../securemsm/QSEEComAPI \
                    $(TARGET_OUT_HEADERS)/common/inc \
                    $(LOCAL_PATH)/..          \
                    external/connectivity/stlport/stlport
LOCAL_ADDITIONAL_DEPENDENCIES := $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr
LOCAL_SRC_FILES:= QvopDaemon.cpp logfile.cpp
LOCAL_SHARED_LIBRARIES := libcutils libutils libbinder libqvop-service liblog
LOCAL_ADDITIONAL_DEPENDENCIES += $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr
LOCAL_MODULE:= qvop-daemon
LOCAL_MODULE_TAGS := eng
include $(BUILD_EXECUTABLE)

##########################################################
# Files required by Algorithm
##########################################################
include $(CLEAR_VARS)
LOCAL_MODULE := cmudict.bin
LOCAL_MODULE_TAGS := eng
LOCAL_MODULE_CLASS := ETC
LOCAL_MODULE_PATH := $(TARGET_OUT_ETC)/qvop/
LOCAL_SRC_FILES := calib/cmudict.bin
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_MODULE := poc_64_hmm.gmm
LOCAL_MODULE_TAGS := eng
LOCAL_MODULE_CLASS := ETC
LOCAL_MODULE_PATH := $(TARGET_OUT_ETC)/qvop/
LOCAL_SRC_FILES := calib/poc_64_hmm.gmm
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_MODULE := noisesample.bin
LOCAL_MODULE_TAGS := eng
LOCAL_MODULE_CLASS := ETC
LOCAL_MODULE_PATH := $(TARGET_OUT_ETC)/qvop/
LOCAL_SRC_FILES := calib/noisesample.bin
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_MODULE := antispoofing.bin
LOCAL_MODULE_TAGS := eng
LOCAL_MODULE_CLASS := ETC
LOCAL_MODULE_PATH := $(TARGET_OUT_ETC)/qvop/
LOCAL_SRC_FILES := calib/antispoofing.bin
include $(BUILD_PREBUILT)


##########################################################
# Test app for linux.
##########################################################
# checks equivalent to /hardware/qcom/audio
ifneq ($(filter mpq8092 msm8960 msm8226 msm8x26 msm8610 msm8974 msm8x74 apq8084 msm8916 msm8994 msm8992 msm8909 msm8996 msm8952 msm8937 thorium msm8953 msmgold msm8998 sdm660,$(TARGET_BOARD_PLATFORM)),)
ifeq ($(BOARD_USES_LEGACY_ALSA_AUDIO),false)
# BOARD_SUPPORTS_QAHW - check equivalent to
# /hardware/qcom/audio/qahw_api
ifeq ($(strip $(BOARD_SUPPORTS_QAHW)),true)
include $(LOCAL_PATH)/test-app-lnx/Android.mk
endif
endif
endif
