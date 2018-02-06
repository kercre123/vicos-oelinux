ifeq ($(call is-board-platform-in-list,msm8974 msm8960 msm8660 msm8226 msm8610 msm8916 msm8916_32 msm8916_32_k64 msm8916_64 apq8084 msm8939 msm8994 msm8909 msm8996 msm8992 msm8952 msm8937 msm8953 msm8998 sdm660),true)

##########################################################
# QSEE Connector Service
LOCAL_PATH:= $(call my-dir)

SECUREMSM_SHIP_PATH   := vendor/qcom/proprietary/securemsm
SSE_SHIP_PATH         := $(SECUREMSM_SHIP_PATH)/sse
PROXY_SHIP_PATH       := $(SECUREMSM_SHIP_PATH)/proxydaemon

COMMON_INCLUDES     := $(PROXY_SHIP_PATH) \
                       $(SECUREMSM_SHIP_PATH)/QSEEComAPI \
                       $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr/include \
                       external/connectivity/stlport/stlport \

SERVER_SRC_FILES := BnQSEEConnectorService.cpp \
                    QSEEConnectorService.cpp \
                    IQSEEConnectorService.cpp \

DAEMON_SRC_FILES := daemon.cpp \
                    TAInterface.cpp \

NAMELIB_SRC_FILES := proxyname.cpp


##########################################################
##########################################################
# QSEE Proxy Service
##########################################################
##########################################################

#----------------------------
# QSEE Proxy Sample Daemon
#----------------------------

include $(CLEAR_VARS)

LOCAL_C_INCLUDES += $(COMMON_INCLUDES)

LOCAL_SRC_FILES := $(DAEMON_SRC_FILES)
LOCAL_SRC_FILES += $(SERVER_SRC_FILES)
LOCAL_SRC_FILES += modules/sampleauth.cpp \
                   modules/sampleextauth.cpp \

LOCAL_SHARED_LIBRARIES += \
        libbinder \
        libandroid_runtime \
        liblog \
        libutils \
        libQSEEComAPI \

LOCAL_CFLAGS    := -g -Wno-missing-field-initializers -Wno-format -Wno-unused-parameter -std=c++11

LOCAL_CFLAGS += -DQSEE_CONNECTOR_SERVICE_NAME=com.qualcomm.qti.qseeproxysample
LOCAL_CFLAGS += -DTA_SAMPLEAUTH
LOCAL_CFLAGS += -DTA_SAMPLEEXTAUTH

LOCAL_PRELINK_MODULE := false
LOCAL_MODULE:= qseeproxysampledaemon
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_OWNER := qti
LOCAL_PROPRIETARY_MODULE := true
LOCAL_ADDITIONAL_DEPENDENCIES += $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr
include $(BUILD_EXECUTABLE)

#----------------------------
# QSEE Proxy Daemon - test client
#----------------------------

include $(CLEAR_VARS)

LOCAL_C_INCLUDES += $(COMMON_INCLUDES) \
                    $(LOCAL_PATH) \

LOCAL_SRC_FILES :=  client.cpp

LOCAL_SHARED_LIBRARIES += \
        libbinder \
        libandroid_runtime \
        liblog \
        libutils

LOCAL_CFLAGS    := -g -Wno-missing-field-initializers -Wno-format -Wno-unused-parameter -std=c++11
LOCAL_CFLAGS += -DQSEE_CONNECTOR_SERVICE_NAME=com.qualcomm.qti.qseeproxysample

LOCAL_PRELINK_MODULE := false
LOCAL_MODULE:= qsee_proxy_client_test
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_OWNER := qti
LOCAL_PROPRIETARY_MODULE := true
LOCAL_ADDITIONAL_DEPENDENCIES += $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr
include $(BUILD_EXECUTABLE)

endif # end filter
