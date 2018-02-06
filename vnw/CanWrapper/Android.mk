#/*
# * Copyright (c) 2014 Qualcomm Technologies, Inc.  All Rights Reserved.
# * Qualcomm Technologies Proprietary and Confidential.
# */
#--------------------------------------------------------------------
# makefile to copy canwrapper header files to common include location
#--------------------------------------------------------------------
LOCAL_PATH:= $(call my-dir)
include $(CLEAR_VARS)

LOCAL_COPY_HEADERS_TO := CanWrapper
LOCAL_COPY_HEADERS += CanWrapper.h CwBase.h CwFrame.h CwSim.h CwNode.h CwBlockingQueue.h CwNodeSet.h CwLock.h
include $(BUILD_COPY_HEADERS)
