ifneq ($(call is-board-platform-in-list,msmskunk),true)

include $(call all-subdir-makefiles)

endif #end filter
