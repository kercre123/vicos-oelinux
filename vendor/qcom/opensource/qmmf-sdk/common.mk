LOCAL_CPP_EXTENSION := .cc

LOCAL_CFLAGS := -Wall -Wextra -Werror -std=c++14 -fexceptions
# TODO functions have unused input parameters
LOCAL_CFLAGS += -Wno-unused-parameter
# Suppress unused variable caused by assert only for release variant
ifeq (userdebug,$(TARGET_BUILD_VARIANT))
LOCAL_CFLAGS += -UNDEBUG
else
LOCAL_CFLAGS += -Wno-unused-variable
endif

LOCAL_C_INCLUDES := $(QMMF_SDK_TOP_SRCDIR)/include
LOCAL_C_INCLUDES += $(QMMF_SDK_TOP_SRCDIR)
LOCAL_C_INCLUDES += $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr/include
LOCAL_ADDITIONAL_DEPENDENCIES += $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr

LOCAL_SHARED_LIBRARIES := libcutils libutils libdl liblog

LOCAL_EXPORT_C_INCLUDE_DIRS := $(QMMF_SDK_TOP_SRCDIR)/include

LOCAL_32_BIT_ONLY := true

# ANDROID version check
ANDROID_MAJOR_VERSION :=$(shell echo $(PLATFORM_VERSION) | cut -f1 -d.)
IS_ANDROID_O_OR_ABOVE :=$(shell test $(ANDROID_MAJOR_VERSION) -gt 8 -o $(ANDROID_MAJOR_VERSION) -eq 8 && echo true)
ifeq ($(IS_ANDROID_O_OR_ABOVE),true)
LOCAL_CFLAGS += -DANDROID_O_OR_ABOVE
endif #ANDROID version check

# Enable libs/bins installation into vendor
ifeq ($(IS_ANDROID_O_OR_ABOVE),true)
LOCAL_VENDOR_MODULE := true
endif #LOCAL_VENDOR_MODULE

# Disable jpeg postproc
ifeq ($(TARGET_BOARD_PLATFORM),qcs605)
DISABLE_PP_JPEG := 1
LOCAL_CFLAGS += -DDISABLE_PP_JPEG
endif #DISABLE_PP_JPEG

# Disable Video LPM
ifeq ($(TARGET_BOARD_PLATFORM),qcs605)
LOCAL_CFLAGS += -DDISABLE_VID_LPM
endif #DISABLE_VID_LPM

# Disable Video QP Range
ifeq ($(TARGET_BOARD_PLATFORM),qcs605)
LOCAL_CFLAGS += -DDISABLE_VID_QP_RANGE
endif #DISABLE_VID_QP_RANGE

# Disable Op Modes
ifeq ($(TARGET_BOARD_PLATFORM),qcs605)
LOCAL_CFLAGS += -DDISABLE_OP_MODES
endif #DISABLE_OP_MODES

# Disable display service
ifeq ($(TARGET_BOARD_PLATFORM),qcs605)
DISABLE_DISPLAY := 1
LOCAL_CFLAGS += -DDISABLE_DISPLAY
endif #DISABLE_DISPLAY

# Enable Gralloc1 support
ifeq ($(TARGET_USES_GRALLOC1),true)
LOCAL_CFLAGS += -DTARGET_USES_GRALLOC1
endif #TARGET_USES_GRALLOC1

# Enable Vendor Tag Descriptor
ifeq ($(TARGET_BOARD_PLATFORM),qcs605)
LOCAL_CFLAGS += -DUSE_VENDOR_TAG_DESC
endif #VENDOR_TAG_DESC
