ifeq ($(call is-board-platform-in-list, msm8937 msm8953 msm8996 msm8998 sdm660),true)
  include $(call all-subdir-makefiles)
endif

