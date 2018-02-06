ifeq ($(call is-board-platform-in-list,msmcobalt msm8998 sdm660),true)

include $(call all-subdir-makefiles)

endif # end filter
