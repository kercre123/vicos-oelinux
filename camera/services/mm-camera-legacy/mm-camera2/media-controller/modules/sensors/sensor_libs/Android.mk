ifeq ($(TARGET_ARCH),$(filter $(TARGET_ARCH),arm arm64))
  ifeq ($(call is-board-platform-in-list, msm8909),true)
    #include $(call all-named-subdir-makefiles, ov5648_q5v22e)
    include $(call all-named-subdir-makefiles, bf2253L)
  else
    include $(call all-subdir-makefiles)
  endif
endif
