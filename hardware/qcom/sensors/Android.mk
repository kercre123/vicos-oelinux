# Copyright (C) 2008 The Android Open Source Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


LOCAL_PATH := $(call my-dir)

ifneq ($(TARGET_SIMULATOR),true)

include $(CLEAR_VARS)

ifeq ($(origin TARGET_BOARD_PLATFORM), undefined)
	LOCAL_MODULE := sensors.default
else
	LOCAL_MODULE := sensors.$(TARGET_BOARD_PLATFORM)
endif

LOCAL_MODULE_TAGS := eng

LOCAL_MODULE_PATH := $(TARGET_OUT_SHARED_LIBRARIES)/hw

LOCAL_MULTILIB := first

LOCAL_SRC_FILES :=\
	sensord/axis_remap.c\
	sensord/sensord_hwcntl.cpp\
	sensord/sensord_hwcntl_implement.cpp\
	sensord/util_misc.c\
	sensord/sensord_pltf.c\
	sensord/sensord_cfg.cpp\
	sensord/sensord_algo.cpp\
	sensord/sensord.cpp\
	sensord/bstsimple_list.cpp\
	hal/sensors.cpp\
	hal/BstSensor.cpp


LOCAL_C_INCLUDES := $(LOCAL_PATH)/hal\
		$(LOCAL_PATH)/sensord/bsx/inc\
		$(LOCAL_PATH)/sensord/inc

LOCAL_CFLAGS := -pthread

LOCAL_CPPFLAGS := -pthread\
                  -Wno-error=date-time

LOCAL_LDLIBS += -lm -llog -lutils
#LOCAL_SHARED_LIBRARIES := libm
#LOCAL_LDFLAGS :=
#LOCAL_LDFLAGS_arm :=
#LOCAL_LDFLAGS_arm64 :=
LOCAL_PRELINK_MODULE := false



include $(BUILD_SHARED_LIBRARY)


endif  # TARGET_SIMULATOR != true
