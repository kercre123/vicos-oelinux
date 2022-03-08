ifeq ($(TARGET_ARCH),$(filter $(TARGET_ARCH),arm arm64))
  ifeq ($(call is-board-platform-in-list, msm8909),true)
    include $(call all-named-subdir-makefiles, ov8856_f8v05a) $(call all-named-subdir-makefiles, bf2253L)
  else
    include $(call all-subdir-makefiles)
  endif
endif
