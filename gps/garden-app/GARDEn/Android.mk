ifeq ($(TARGET_NO_RPC),true)

LOCAL_PATH:=$(call my-dir)
include $(CLEAR_VARS)

LOC_API_APP_PATH:=$(LOCAL_PATH)

LOC_API_DEBUG:=false

LOCAL_SRC_FILES:= \
    garden_app_session_tracker.cpp \
    test_android_gps.cpp

LOCAL_CFLAGS:= \
    -DDEBUG \
    -D_ANDROID_ \
    -DTEST_POSITION \
    -DTEST_TIME_INJECTION \
    -DTEST_DELETE_ASSISTANCE \
    -DTEST_POSITION \
    -DTEST_ULP \
#    -DTEST_RIL_CB \
#    -DTEST_ATL_CB \
#    -DTEST_XTRA_INJECTION \
#    -DTEST_ATL_FAILURE \
#
# LOC_SVC_FEATURE_FLAGS = \
#    -DTEST_ATL_BACK_TO_BACK \
#    -DTEST_ATL_CB_MULTIPLE \
#    -DTEST_ATL_WRONG_CLOSE

LOCAL_C_INCLUDES:= \
    $(TARGET_OUT_HEADERS)/gps.utils \
    $(TARGET_OUT_HEADERS)/libloc_eng \
    $(TARGET_OUT_HEADERS)/libloc_core \
    $(TARGET_OUT_HEADERS)/libizat_core \
    $(TARGET_OUT_HEADERS)/qmi-framework/inc \
    $(TARGET_OUT_HEADERS)/qmi/inc \
    $(TARGET_OUT_HEADERS)/libevent_observer \
    $(LOCAL_PATH) \
    $(TOP)/vendor/qcom/proprietary/gps/vzwGpsLocationProvider/loc_ext \
    $(TOP)/vendor/qcom/proprietary/gps/location-xtra/pubinc \
    $(TOP)/vendor/qcom/proprietary/gps/framework/native/service \
    $(TARGET_OUT_HEADERS)/libloc_pla \
    $(TARGET_OUT_HEADERS)/libflp

LOCAL_SHARED_LIBRARIES := \
    libgps.utils \
    libcutils \
    libloc_ext \
    libloc_eng \
    libqmi_cci \
    libqmi_csi \
    liblocationservice \
    libulp2 \
    liblog \
    libevent_observer \
    libloc_pla \
    libflp

LOCAL_PRELINK_MODULE:=false

LOCAL_MODULE:=garden_app
LOCAL_MODULE_TAGS := optional
include $(BUILD_EXECUTABLE)

endif # TARGET_NO_RPC
