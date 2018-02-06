ifeq ($(strip $(TARGET_USES_QPAY)),true)

include $(call all-subdir-makefiles)

endif # end filter
